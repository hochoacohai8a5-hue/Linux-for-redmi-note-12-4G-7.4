// SPDX-License-Identifier: GPL-2.0
/*
 * Lion Semiconductor LN8000 direct-charge pump driver
 *
 * This is a C port of the safe-standby driver written for the
 * Xiaomi Redmi Note 10S / POCO M5s mainline effort. The register map,
 * ADC formulas and protection semantics ultimately come from Xiaomi's
 * GPL-2.0 rosemary-r-oss LN8000 driver.
 *
 * This first-stage driver is deliberately conservative:
 *   - identify the chip and reject unexpected silicon;
 *   - force standby before touching configuration;
 *   - keep hardware protections and regulation loops enabled;
 *   - enable ADC telemetry;
 *   - expose read-only standard power_supply properties;
 *   - leave the pump in standby on remove and shutdown.
 *
 * It deliberately does not implement fast-charge policy, interrupt
 * handling, NTC monitors, watchdog management, dual-pump operation or
 * undocumented vendor reset/trim sequences. In particular, userspace
 * cannot take the pump out of standby through this driver yet.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#define LN8000_REG_DEVICE_ID			0x00
#define LN8000_REG_SYS_STS			0x03
#define LN8000_REG_SAFETY_STS			0x04
#define LN8000_REG_FAULT1_STS			0x05
#define LN8000_REG_FAULT2_STS			0x06
#define LN8000_REG_ADC_IIN			0x09
#define LN8000_REG_ADC_VIN			0x0b
#define LN8000_REG_ADC_DIE_TEMP			0x0f
#define LN8000_REG_IIN_CTRL			0x1b
#define LN8000_REG_REGULATION_CTRL		0x1c
#define LN8000_REG_SYS_CTRL			0x1e
#define LN8000_REG_GLITCH_CTRL			0x20
#define LN8000_REG_FAULT_CTRL			0x21
#define LN8000_REG_ADC_CTRL			0x23
#define LN8000_REG_ADC_CFG			0x24
#define LN8000_REG_TIMER_CTRL			0x26
#define LN8000_REG_V_FLOAT_CTRL			0x28

#define LN8000_DEVICE_ID			0x42

#define LN8000_SYS_STS_BYPASS			BIT(3)
#define LN8000_SYS_STS_SWITCHING		BIT(2)

#define LN8000_SAFETY_TEMP_MAX			BIT(6)

#define LN8000_FAULT1_WATCHDOG			BIT(7)
#define LN8000_FAULT1_VBAT_OV			BIT(6)
#define LN8000_FAULT1_VAC_UNPLUG		BIT(4)
#define LN8000_FAULT1_VAC_OV			BIT(3)
#define LN8000_FAULT1_VIN_OV			BIT(1)

#define LN8000_FAULT2_IIN_OC			BIT(7)

#define LN8000_SYS_CTRL_STANDBY_EN		BIT(3)
#define LN8000_SYS_CTRL_REV_IIN_DET		BIT(2)

#define LN8000_REG_CTRL_DISABLE_VFLOAT_LOOP	BIT(5)
#define LN8000_REG_CTRL_DISABLE_IIN_LOOP	BIT(4)
#define LN8000_REG_CTRL_TEMP_MAX_EN		BIT(2)
#define LN8000_REG_CTRL_TEMP_REG_EN		BIT(1)

#define LN8000_FAULT_CTRL_DISABLE_IIN_OCP	BIT(6)
#define LN8000_FAULT_CTRL_DISABLE_VBAT_OV	BIT(5)

#define LN8000_TIMER_CTRL_PAUSE_ADC		BIT(1)

#define LN8000_ADC_CTRL_MODE_SHIFT		5
#define LN8000_ADC_CTRL_MODE_MASK		GENMASK(7, 5)
#define LN8000_ADC_CTRL_DELAY_MASK		GENMASK(4, 3)
#define LN8000_ADC_MODE_AUTO_HIBERNATE		0
#define LN8000_ADC_MODE_SHUTDOWN		2
#define LN8000_ADC_HIBERNATE_4S			(3 << 3)

#define LN8000_GLITCH_CTRL_VAC_OVP_MASK	GENMASK(3, 2)

#define LN8000_VFLOAT_MIN_UV			3725000
#define LN8000_VFLOAT_STEP_UV			5000

#define LN8000_IIN_MIN_UA			500000
#define LN8000_IIN_STEP_UA			50000

#define LN8000_ADC_VIN_STEP_UV			16000
#define LN8000_ADC_IIN_STEP_UA			4890

struct ln8000 {
	struct device *dev;
	struct i2c_client *client;
	struct mutex lock;
	struct power_supply *psy;
};

struct ln8000_state {
	u8 sys;
	u8 safety;
	u8 fault1;
	u8 fault2;
};

static const u32 ln8000_vac_ovp_levels_uv[] = {
	6500000,
	11000000,
	12000000,
	13000000,
};

static int ln8000_read(struct ln8000 *ln, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(ln->client, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int ln8000_write(struct ln8000 *ln, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(ln->client, reg, val);
}

static int ln8000_update_bits(struct ln8000 *ln, u8 reg, u8 mask, u8 val)
{
	u8 old;
	int ret;

	ret = ln8000_read(ln, reg, &old);
	if (ret)
		return ret;

	old &= ~mask;
	old |= val & mask;

	return ln8000_write(ln, reg, old);
}

static int ln8000_set_standby(struct ln8000 *ln, bool standby)
{
	return ln8000_update_bits(ln, LN8000_REG_SYS_CTRL,
				  LN8000_SYS_CTRL_STANDBY_EN,
				  standby ? LN8000_SYS_CTRL_STANDBY_EN : 0);
}

static int ln8000_float_voltage_uv(struct ln8000 *ln, int *uv)
{
	u8 reg;
	int ret;

	ret = ln8000_read(ln, LN8000_REG_V_FLOAT_CTRL, &reg);
	if (ret)
		return ret;

	*uv = LN8000_VFLOAT_MIN_UV + reg * LN8000_VFLOAT_STEP_UV;
	return 0;
}

static int ln8000_input_current_limit_ua(struct ln8000 *ln, int *ua)
{
	u8 reg;
	int ret;

	ret = ln8000_read(ln, LN8000_REG_IIN_CTRL, &reg);
	if (ret)
		return ret;

	*ua = max_t(int, (reg & 0x7f) * LN8000_IIN_STEP_UA,
		    LN8000_IIN_MIN_UA);
	return 0;
}

static int ln8000_input_overvoltage_uv(struct ln8000 *ln, int *uv)
{
	u8 reg;
	u8 field;
	int ret;

	ret = ln8000_read(ln, LN8000_REG_GLITCH_CTRL, &reg);
	if (ret)
		return ret;

	field = FIELD_GET(LN8000_GLITCH_CTRL_VAC_OVP_MASK, reg);
	if (field >= ARRAY_SIZE(ln8000_vac_ovp_levels_uv))
		return -EINVAL;

	*uv = ln8000_vac_ovp_levels_uv[field];
	return 0;
}

static int ln8000_adc_read_pair(struct ln8000 *ln, u8 reg, u8 *lo, u8 *hi)
{
	int ret;
	int resume_ret;

	ret = ln8000_update_bits(ln, LN8000_REG_TIMER_CTRL,
				 LN8000_TIMER_CTRL_PAUSE_ADC,
				 LN8000_TIMER_CTRL_PAUSE_ADC);
	if (ret)
		return ret;

	ret = ln8000_read(ln, reg, lo);
	if (!ret)
		ret = ln8000_read(ln, reg + 1, hi);

	resume_ret = ln8000_update_bits(ln, LN8000_REG_TIMER_CTRL,
					LN8000_TIMER_CTRL_PAUSE_ADC, 0);

	if (ret)
		return ret;

	return resume_ret;
}

static int ln8000_adc_vin_uv(struct ln8000 *ln, int *uv)
{
	u8 lo, hi;
	int code;
	int ret;

	ret = ln8000_adc_read_pair(ln, LN8000_REG_ADC_VIN, &lo, &hi);
	if (ret)
		return ret;

	code = ((hi & 0x3f) << 4) | (lo >> 4);
	*uv = code * LN8000_ADC_VIN_STEP_UV;
	return 0;
}

static int ln8000_adc_iin_ua(struct ln8000 *ln, int *ua)
{
	u8 lo, hi;
	int code;
	int ret;

	ret = ln8000_adc_read_pair(ln, LN8000_REG_ADC_IIN, &lo, &hi);
	if (ret)
		return ret;

	code = ((hi & 0x03) << 8) | lo;
	*ua = code * LN8000_ADC_IIN_STEP_UA;
	return 0;
}

static int ln8000_adc_die_temp_decic(struct ln8000 *ln, int *decic)
{
	u8 lo, hi;
	int code;
	int temp;
	int ret;

	ret = ln8000_adc_read_pair(ln, LN8000_REG_ADC_DIE_TEMP, &lo, &hi);
	if (ret)
		return ret;

	code = ((hi & 0x0f) << 6) | (lo >> 2);

	/*
	 * Vendor formula: (935 - code) * 1000 / 2300.
	 * The raw result behaves as degrees Celsius, despite downstream naming
	 * suggesting deci-degrees, so scale to tenths for power_supply.
	 */
	temp = (935 - code) * 10000 / 2300;
	*decic = clamp_t(int, temp, -250, 1600);

	return 0;
}

static int ln8000_read_state(struct ln8000 *ln, struct ln8000_state *state)
{
	int ret;

	ret = ln8000_read(ln, LN8000_REG_SYS_STS, &state->sys);
	if (ret)
		return ret;

	ret = ln8000_read(ln, LN8000_REG_SAFETY_STS, &state->safety);
	if (ret)
		return ret;

	ret = ln8000_read(ln, LN8000_REG_FAULT1_STS, &state->fault1);
	if (ret)
		return ret;

	return ln8000_read(ln, LN8000_REG_FAULT2_STS, &state->fault2);
}

static int ln8000_state_health(const struct ln8000_state *state)
{
	if (state->safety & LN8000_SAFETY_TEMP_MAX)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (state->fault1 & (LN8000_FAULT1_VBAT_OV |
			     LN8000_FAULT1_VAC_OV |
			     LN8000_FAULT1_VIN_OV))
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	if (state->fault2 & LN8000_FAULT2_IIN_OC)
		return POWER_SUPPLY_HEALTH_OVERCURRENT;

	if (state->fault1 & LN8000_FAULT1_WATCHDOG)
		return POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static bool ln8000_state_charging(const struct ln8000_state *state)
{
	return state->sys & (LN8000_SYS_STS_SWITCHING |
			     LN8000_SYS_STS_BYPASS);
}

static int ln8000_init_safe(struct ln8000 *ln)
{
	int ret;

	/* First action after identification: force the pump off. */
	ret = ln8000_set_standby(ln, true);
	if (ret)
		return ret;

	/* Keep input over-current and battery over-voltage protection enabled. */
	ret = ln8000_update_bits(ln, LN8000_REG_FAULT_CTRL,
				 LN8000_FAULT_CTRL_DISABLE_IIN_OCP |
				 LN8000_FAULT_CTRL_DISABLE_VBAT_OV,
				 0);
	if (ret)
		return ret;

	/*
	 * Keep the float-voltage and input-current loops enabled, and enable
	 * die-temperature protection/regulation.
	 */
	ret = ln8000_update_bits(ln, LN8000_REG_REGULATION_CTRL,
				 LN8000_REG_CTRL_DISABLE_VFLOAT_LOOP |
				 LN8000_REG_CTRL_DISABLE_IIN_LOOP |
				 LN8000_REG_CTRL_TEMP_MAX_EN |
				 LN8000_REG_CTRL_TEMP_REG_EN,
				 LN8000_REG_CTRL_TEMP_MAX_EN |
				 LN8000_REG_CTRL_TEMP_REG_EN);
	if (ret)
		return ret;

	ret = ln8000_update_bits(ln, LN8000_REG_SYS_CTRL,
				 LN8000_SYS_CTRL_REV_IIN_DET,
				 LN8000_SYS_CTRL_REV_IIN_DET);
	if (ret)
		return ret;

	/*
	 * ADC mode changes are made while the ADC is shut down. Enable all
	 * channels, set a 4-second auto-hibernate delay, then return to the
	 * automatic mode used by the rosemary implementation.
	 */
	ret = ln8000_update_bits(ln, LN8000_REG_ADC_CTRL,
				 LN8000_ADC_CTRL_MODE_MASK,
				 LN8000_ADC_MODE_SHUTDOWN <<
				 LN8000_ADC_CTRL_MODE_SHIFT);
	if (ret)
		return ret;

	ret = ln8000_update_bits(ln, LN8000_REG_ADC_CTRL,
				 LN8000_ADC_CTRL_DELAY_MASK,
				 LN8000_ADC_HIBERNATE_4S);
	if (ret)
		return ret;

	ret = ln8000_write(ln, LN8000_REG_ADC_CFG, 0xff);
	if (ret)
		return ret;

	return ln8000_update_bits(ln, LN8000_REG_ADC_CTRL,
				  LN8000_ADC_CTRL_MODE_MASK,
				  LN8000_ADC_MODE_AUTO_HIBERNATE <<
				  LN8000_ADC_CTRL_MODE_SHIFT);
}

static enum power_supply_property ln8000_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
};

static int ln8000_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct ln8000 *ln = power_supply_get_drvdata(psy);
	struct ln8000_state state;
	int ret;
	int value;

	switch (prop) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Lion Semiconductor";
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "LN8000";
		return 0;
	default:
		break;
	}

	mutex_lock(&ln->lock);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = ln8000_read_state(ln, &state);
		if (!ret)
			val->intval = ln8000_state_charging(&state) ?
				POWER_SUPPLY_STATUS_CHARGING :
				POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = ln8000_read_state(ln, &state);
		if (!ret)
			val->intval =
				!(state.fault1 & LN8000_FAULT1_VAC_UNPLUG);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = ln8000_read_state(ln, &state);
		if (!ret)
			val->intval = ln8000_state_health(&state);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = ln8000_read_state(ln, &state);
		if (!ret) {
			if (!ln8000_state_charging(&state))
				val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			else if (state.sys & LN8000_SYS_STS_BYPASS)
				val->intval = POWER_SUPPLY_CHARGE_TYPE_BYPASS;
			else
				val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = ln8000_adc_vin_uv(ln, &value);
		if (!ret)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = ln8000_adc_iin_ua(ln, &value);
		if (!ret)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = ln8000_adc_die_temp_decic(ln, &value);
		if (!ret)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = ln8000_float_voltage_uv(ln, &value);
		if (!ret)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = ln8000_input_current_limit_ua(ln, &value);
		if (!ret)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = ln8000_input_overvoltage_uv(ln, &value);
		if (!ret)
			val->intval = value;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&ln->lock);

	return ret;
}

static const struct power_supply_desc ln8000_psy_desc = {
	.name = "ln8000",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ln8000_properties,
	.num_properties = ARRAY_SIZE(ln8000_properties),
	.get_property = ln8000_get_property,
};

static int ln8000_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &client->dev;
	struct ln8000 *ln;
	u8 id;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	ln = devm_kzalloc(dev, sizeof(*ln), GFP_KERNEL);
	if (!ln)
		return -ENOMEM;

	ln->dev = dev;
	ln->client = client;
	mutex_init(&ln->lock);
	i2c_set_clientdata(client, ln);

	mutex_lock(&ln->lock);

	ret = ln8000_read(ln, LN8000_REG_DEVICE_ID, &id);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to read device ID\n");
		goto out_unlock;
	}

	if (id != LN8000_DEVICE_ID) {
		ret = dev_err_probe(dev, -ENODEV,
				    "unexpected device ID %#x, expected %#x\n",
				    id, LN8000_DEVICE_ID);
		goto out_unlock;
	}

	ret = ln8000_init_safe(ln);
	if (ret) {
		ret = dev_err_probe(dev, ret,
				    "failed to put charge pump into safe state\n");
		goto out_standby;
	}

	mutex_unlock(&ln->lock);

	psy_cfg.drv_data = ln;
	psy_cfg.fwnode = dev_fwnode(dev);

	ln->psy = devm_power_supply_register(dev, &ln8000_psy_desc, &psy_cfg);
	if (IS_ERR(ln->psy)) {
		ret = dev_err_probe(dev, PTR_ERR(ln->psy),
				    "failed to register power supply\n");

		mutex_lock(&ln->lock);
		ln8000_set_standby(ln, true);
		mutex_unlock(&ln->lock);

		return ret;
	}

	dev_info(dev, "LN8000 charge pump found, safe standby enabled\n");

	return 0;

out_standby:
	ln8000_set_standby(ln, true);
out_unlock:
	mutex_unlock(&ln->lock);

	return ret;
}

static void ln8000_remove(struct i2c_client *client)
{
	struct ln8000 *ln = i2c_get_clientdata(client);

	mutex_lock(&ln->lock);
	if (ln8000_set_standby(ln, true))
		dev_warn(ln->dev, "failed to enter standby on remove\n");
	mutex_unlock(&ln->lock);
}

static void ln8000_shutdown(struct i2c_client *client)
{
	struct ln8000 *ln = i2c_get_clientdata(client);

	mutex_lock(&ln->lock);
	if (ln8000_set_standby(ln, true))
		dev_warn(ln->dev, "failed to enter standby on shutdown\n");
	mutex_unlock(&ln->lock);
}

static const struct i2c_device_id ln8000_i2c_id[] = {
	{ "ln8000", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ln8000_i2c_id);

static const struct of_device_id ln8000_of_match[] = {
	{ .compatible = "lionsemi,ln8000" },
	{ }
};
MODULE_DEVICE_TABLE(of, ln8000_of_match);

static struct i2c_driver ln8000_driver = {
	.driver = {
		.name = "ln8000-charger",
		.of_match_table = ln8000_of_match,
	},
	.probe = ln8000_probe,
	.remove = ln8000_remove,
	.shutdown = ln8000_shutdown,
	.id_table = ln8000_i2c_id,
};
module_i2c_driver(ln8000_driver);

MODULE_DESCRIPTION("Lion Semiconductor LN8000 direct-charge pump driver");
MODULE_LICENSE("GPL");
