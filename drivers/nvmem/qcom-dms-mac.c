// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm DMS MAC address NVMEM provider
 *
 * Qualcomm modems may expose factory-provisioned WLAN and Bluetooth MAC
 * addresses through the Device Management Service (DMS) over QRTR/QMI.
 *
 * This driver keeps a DMS service lookup alive, fetches the WLAN address
 * when the service becomes available and then exposes the immutable address
 * as a small read-only NVMEM provider. Consumers such as ath10k can use the
 * standard "mac-address" NVMEM cell without knowing anything about QMI.
 */

#include <linux/etherdevice.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/net.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/qrtr.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/workqueue.h>
#include <net/sock.h>

#define QCOM_DMS_SERVICE_ID			0x02
#define QCOM_DMS_SERVICE_VERSION		0x01

#define QMI_DMS_GET_MAC_ADDRESS_REQ		0x005c
#define QMI_DMS_GET_MAC_ADDRESS_REQ_MAX_LEN	7
#define QMI_DMS_GET_MAC_ADDRESS_RESP_MAX_LEN	19
#define QMI_DMS_MAC_ADDR_MAX			8

#define QCOM_DMS_QMI_MAX_MSG_LEN		QMI_DMS_GET_MAC_ADDRESS_RESP_MAX_LEN
#define QCOM_DMS_FETCH_RETRY_MS			2000
#define QCOM_DMS_QMI_TIMEOUT_MS			5000

enum qcom_dms_device_mac {
	QCOM_DMS_DEVICE_MAC_WLAN = 0,
	QCOM_DMS_DEVICE_MAC_BT = 1,
};

struct qcom_dms_get_mac_req {
	enum qcom_dms_device_mac device;
};

struct qcom_dms_get_mac_resp {
	struct qmi_response_type_v01 resp;
	u8 mac_address_valid;
	u32 mac_address_len;
	u8 mac_address[QMI_DMS_MAC_ADDR_MAX];
};

struct qcom_dms_mac {
	struct device *dev;
	struct qmi_handle qmi;
	struct delayed_work fetch_work;
	struct mutex lock;
	struct nvmem_device *nvmem;
	u8 mac[ETH_ALEN];
	bool connected;
	bool mac_valid;
	bool removing;
};

static const struct qmi_elem_info qcom_dms_get_mac_req_ei[] = {
	{
		.data_type = QMI_SIGNED_4_BYTE_ENUM,
		.elem_len = 1,
		.elem_size = sizeof(enum qcom_dms_device_mac),
		.array_type = NO_ARRAY,
		.tlv_type = 0x01,
		.offset = offsetof(struct qcom_dms_get_mac_req, device),
	},
	{
		.data_type = QMI_EOTI,
		.array_type = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

static const struct qmi_elem_info qcom_dms_get_mac_resp_ei[] = {
	{
		.data_type = QMI_STRUCT,
		.elem_len = 1,
		.elem_size = sizeof(struct qmi_response_type_v01),
		.array_type = NO_ARRAY,
		.tlv_type = 0x02,
		.offset = offsetof(struct qcom_dms_get_mac_resp, resp),
		.ei_array = qmi_response_type_v01_ei,
	},
	{
		.data_type = QMI_OPT_FLAG,
		.elem_len = 1,
		.elem_size = sizeof(u8),
		.array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qcom_dms_get_mac_resp,
				   mac_address_valid),
	},
	{
		.data_type = QMI_DATA_LEN,
		.elem_len = 1,
		.elem_size = sizeof(u8),
		.array_type = NO_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qcom_dms_get_mac_resp,
				   mac_address_len),
	},
	{
		.data_type = QMI_UNSIGNED_1_BYTE,
		.elem_len = QMI_DMS_MAC_ADDR_MAX,
		.elem_size = sizeof(u8),
		.array_type = VAR_LEN_ARRAY,
		.tlv_type = 0x10,
		.offset = offsetof(struct qcom_dms_get_mac_resp, mac_address),
	},
	{
		.data_type = QMI_EOTI,
		.array_type = NO_ARRAY,
		.tlv_type = QMI_COMMON_TLV_TYPE,
	},
};

static int qcom_dms_mac_nvmem_read(void *priv, unsigned int offset,
				   void *val, size_t bytes)
{
	struct qcom_dms_mac *dms = priv;

	if (offset > ETH_ALEN || bytes > ETH_ALEN - offset)
		return -EINVAL;

	mutex_lock(&dms->lock);
	if (!dms->mac_valid) {
		mutex_unlock(&dms->lock);
		return -EPROBE_DEFER;
	}

	memcpy(val, dms->mac + offset, bytes);
	mutex_unlock(&dms->lock);

	return 0;
}

static void qcom_dms_mac_retry(struct qcom_dms_mac *dms)
{
	bool retry;

	mutex_lock(&dms->lock);
	retry = dms->connected && !dms->mac_valid && !dms->removing;
	mutex_unlock(&dms->lock);

	if (retry)
		mod_delayed_work(system_wq, &dms->fetch_work,
				 msecs_to_jiffies(QCOM_DMS_FETCH_RETRY_MS));
}

static int qcom_dms_mac_fetch(struct qcom_dms_mac *dms, u8 *mac)
{
	struct qcom_dms_get_mac_req req = {
		.device = QCOM_DMS_DEVICE_MAC_WLAN,
	};
	struct qcom_dms_get_mac_resp resp = {};
	struct qmi_txn txn;
	int ret;

	ret = qmi_txn_init(&dms->qmi, &txn, qcom_dms_get_mac_resp_ei,
			   &resp);
	if (ret < 0)
		return ret;

	ret = qmi_send_request(&dms->qmi, NULL, &txn,
			       QMI_DMS_GET_MAC_ADDRESS_REQ,
			       QMI_DMS_GET_MAC_ADDRESS_REQ_MAX_LEN,
			       qcom_dms_get_mac_req_ei, &req);
	if (ret < 0) {
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn,
			   msecs_to_jiffies(QCOM_DMS_QMI_TIMEOUT_MS));
	if (ret < 0)
		return ret;

	if (resp.resp.result != QMI_RESULT_SUCCESS_V01)
		return -EIO;

	if (!resp.mac_address_valid || resp.mac_address_len != ETH_ALEN)
		return -ENODATA;

	if (!is_valid_ether_addr(resp.mac_address))
		return -EINVAL;

	ether_addr_copy(mac, resp.mac_address);

	return 0;
}

static void qcom_dms_mac_fetch_work(struct work_struct *work)
{
	struct qcom_dms_mac *dms =
		container_of(to_delayed_work(work), struct qcom_dms_mac,
			     fetch_work);
	u8 mac[ETH_ALEN];
	bool fetch;
	int ret;

	mutex_lock(&dms->lock);
	fetch = dms->connected && !dms->mac_valid && !dms->removing;
	mutex_unlock(&dms->lock);

	if (!fetch)
		return;

	ret = qcom_dms_mac_fetch(dms, mac);
	if (ret) {
		dev_dbg(dms->dev, "failed to fetch WLAN MAC from DMS: %d\n",
			ret);
		qcom_dms_mac_retry(dms);
		return;
	}

	mutex_lock(&dms->lock);
	if (dms->removing) {
		mutex_unlock(&dms->lock);
		return;
	}

	ether_addr_copy(dms->mac, mac);
	dms->mac_valid = true;
	mutex_unlock(&dms->lock);

	dev_info(dms->dev, "factory WLAN MAC available from DMS: %pM\n",
		 mac);
}

static int qcom_dms_mac_connect(struct qcom_dms_mac *dms,
				struct qmi_service *service)
{
	struct sockaddr_qrtr sq = {
		.sq_family = AF_QIPCRTR,
		.sq_node = service->node,
		.sq_port = service->port,
	};
	int ret;

	mutex_lock(&dms->qmi.sock_lock);
	if (!dms->qmi.sock)
		ret = -ENODEV;
	else
		ret = kernel_connect(dms->qmi.sock, (struct sockaddr *)&sq,
				     sizeof(sq), 0);
	mutex_unlock(&dms->qmi.sock_lock);

	return ret;
}

static int qcom_dms_mac_new_server(struct qmi_handle *qmi,
				   struct qmi_service *service)
{
	struct qcom_dms_mac *dms =
		container_of(qmi, struct qcom_dms_mac, qmi);
	bool schedule;
	int ret;

	ret = qcom_dms_mac_connect(dms, service);
	if (ret)
		return ret;

	mutex_lock(&dms->lock);
	schedule = !dms->removing && !dms->mac_valid;
	if (!dms->removing)
		dms->connected = true;
	mutex_unlock(&dms->lock);

	if (schedule)
		mod_delayed_work(system_wq, &dms->fetch_work, 0);

	return 0;
}

static void qcom_dms_mac_del_server(struct qmi_handle *qmi,
				    struct qmi_service *service)
{
	struct qcom_dms_mac *dms =
		container_of(qmi, struct qcom_dms_mac, qmi);

	mutex_lock(&dms->lock);
	dms->connected = false;
	mutex_unlock(&dms->lock);

	cancel_delayed_work(&dms->fetch_work);
}

static void qcom_dms_mac_net_reset(struct qmi_handle *qmi)
{
	struct qcom_dms_mac *dms =
		container_of(qmi, struct qcom_dms_mac, qmi);

	mutex_lock(&dms->lock);
	dms->connected = false;
	mutex_unlock(&dms->lock);

	cancel_delayed_work(&dms->fetch_work);
}

static const struct qmi_ops qcom_dms_mac_qmi_ops = {
	.new_server = qcom_dms_mac_new_server,
	.del_server = qcom_dms_mac_del_server,
	.net_reset = qcom_dms_mac_net_reset,
};

static int qcom_dms_mac_probe(struct platform_device *pdev)
{
	struct nvmem_config config = {
		.dev = &pdev->dev,
		.name = "qcom-dms-mac",
		.id = NVMEM_DEVID_AUTO,
		.owner = THIS_MODULE,
		.add_legacy_fixed_of_cells = true,
		.read_only = true,
		.root_only = false,
		.of_node = pdev->dev.of_node,
		.reg_read = qcom_dms_mac_nvmem_read,
		.size = ETH_ALEN,
		.word_size = 1,
		.stride = 1,
	};
	struct qcom_dms_mac *dms;
	int ret;

	dms = devm_kzalloc(&pdev->dev, sizeof(*dms), GFP_KERNEL);
	if (!dms)
		return -ENOMEM;

	dms->dev = &pdev->dev;
	mutex_init(&dms->lock);
	INIT_DELAYED_WORK(&dms->fetch_work, qcom_dms_mac_fetch_work);
	platform_set_drvdata(pdev, dms);

	config.priv = dms;
	dms->nvmem = devm_nvmem_register(&pdev->dev, &config);
	if (IS_ERR(dms->nvmem))
		return dev_err_probe(&pdev->dev, PTR_ERR(dms->nvmem),
				     "failed to register NVMEM provider\n");

	ret = qmi_handle_init(&dms->qmi, QCOM_DMS_QMI_MAX_MSG_LEN,
			      &qcom_dms_mac_qmi_ops, NULL);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to initialize QMI client\n");

	ret = qmi_add_lookup(&dms->qmi, QCOM_DMS_SERVICE_ID,
			     QCOM_DMS_SERVICE_VERSION, 0);
	if (ret) {
		qmi_handle_release(&dms->qmi);
		return dev_err_probe(&pdev->dev, ret,
				     "failed to add DMS service lookup\n");
	}

	return 0;
}

static void qcom_dms_mac_remove(struct platform_device *pdev)
{
	struct qcom_dms_mac *dms = platform_get_drvdata(pdev);

	mutex_lock(&dms->lock);
	dms->removing = true;
	mutex_unlock(&dms->lock);

	cancel_delayed_work_sync(&dms->fetch_work);
	qmi_handle_release(&dms->qmi);
}

static const struct of_device_id qcom_dms_mac_of_match[] = {
	{ .compatible = "qcom,dms-mac-address" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_dms_mac_of_match);

static struct platform_driver qcom_dms_mac_driver = {
	.probe = qcom_dms_mac_probe,
	.remove = qcom_dms_mac_remove,
	.driver = {
		.name = "qcom-dms-mac",
		.of_match_table = qcom_dms_mac_of_match,
	},
};
module_platform_driver(qcom_dms_mac_driver);

MODULE_DESCRIPTION("Qualcomm DMS MAC address NVMEM provider");
MODULE_LICENSE("GPL");
