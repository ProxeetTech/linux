/*
 * MAX5980 IEEE 802.3at/af Quad Port Power-over-Ethernet PSE Controller
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "max5980.h"

static int max5980_read_reg(struct max5980_data *ddata, u8 reg)
{
	int res;
	res  = i2c_smbus_read_byte_data(ddata->client, reg);
	if (res < 0)
		dev_err(&ddata->client->dev, "read byte error %d\n", res);
	return res;
}

static int max5980_write_reg(struct max5980_data *ddata, u8 reg, u8 val)
{
	int res;
	res  = i2c_smbus_write_byte_data(ddata->client, reg, val);
	if (res < 0)
		dev_err(&ddata->client->dev, "write byte error %d\n", res);
	return res;
}

static int max5980_read(struct max5980_data *ddata,
			u8 reg, int len, void * values) {
	int res;
	u8 l;
	u8 *data = (char *)values;
	for (l = 0; l < len; l++) {
		res = i2c_smbus_read_byte_data(ddata->client, reg + l);
		if (res < 0) {
			dev_err(&ddata->client->dev, "read error %d\n", res);
			return res;
		}
		data[l] = (u8)res;
	}
	return l;
}

static int max5980_irq_enable(struct max5980_data *ddata, u8 interrupts) {
	return max5980_write_reg(ddata, MAX5980_INT_MASK_REG, interrupts);
}

static int max5980_irq_clear(struct max5980_data *ddata) {
	int val = max5980_read_reg(ddata, MAX5980_GLOBAL_PUSHBTN_REG);
	if (val >=0) {
		return max5980_write_reg(ddata, MAX5980_GLOBAL_PUSHBTN_REG, ((u8)val) | MAX5980_INT_CLR);
	} else {
		return val;
	}
}

static irqreturn_t max5980_irq_handler(int irq, void *device_data)
{
	int i;
	u8 ports;
	u8 irq_flags;
	struct max5980_data *ddata = (struct max5980_data *)device_data;
	irq_flags = max5980_read_reg(ddata, MAX5980_INT_REG);
	/* Critical faults */
	if (MAX5980_TCUT_INT & irq_flags) {
		/* Icut */
		ports = max5980_read_reg(ddata, MAX5980_FAULT_EVENT_COR_REG);
		for (i = 0; i < MAX5980_PORTS_NUM; i++) {
			if (ports & (1 << i))
				dev_info(&ddata->client->dev,
				    "port%d: overload occured", i);
		}
		/* Ilim */
		ports = max5980_read_reg(ddata, MAX5980_STARTUP_EVENT_COR_REG);
		for (i = 0; i < MAX5980_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: exceeded current limit", i);
		}
	}
	if (MAX5980_SUP_MASK & irq_flags)
		dev_info(&ddata->client->dev, "power supply fault\n");
	/* Deferrable events: PEC, PGC, DISF, DETC, CLASC, STRTF */
	if (MAX5980_TST_MASK & irq_flags) {
		ports = max5980_read_reg(ddata, MAX5980_STARTUP_EVENT_COR_REG);
		for (i = 0; i < MAX5980_PORTS_NUM; i++) {
			if (ports & (1 << i))
				dev_info(&ddata->client->dev,
				    "port%d: start fault", i);
		}
	}
	if (MAX5980_DIS_INT & irq_flags) {
		ports = max5980_read_reg(ddata, MAX5980_FAULT_EVENT_COR_REG);
		for (i = 0; i < MAX5980_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: powered device disconnected", i);
		}
	}
	if (MAX5980_CLS_INT & irq_flags) {
		ports = max5980_read_reg(ddata, MAX5980_DET_EVENT_COR_REG);
		for (i = 0; i < MAX5980_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: powered device connected", i);
		}
	}
	max5980_irq_clear(ddata); // clear all interrupts
	return IRQ_HANDLED;
}

int pt_is_valid(struct device *dev, int pt_number) {
	struct max5980_data *ddata = dev_get_drvdata(dev);
	if (ddata->pdata.pt_df[pt_number].enable == 1)
		return 1;
	else
		return 0;
}

static u8 append_pt_mode_reg(u8 old_mode_reg, int pt_number, int mode) {
	u8 new_mode_reg;
	u8 pt_mask;
	pt_mask = ~(MAX5980_PT_MODE_MASK << (pt_number * MAX5980_PT_MODE_FIELD_WIDTH));
	new_mode_reg = mode << (pt_number * MAX5980_PT_MODE_FIELD_WIDTH);
	new_mode_reg = new_mode_reg | (pt_mask & old_mode_reg);
	return new_mode_reg;
}

static int max5980_get_pt_mode(struct device *dev, int porti) {
	u8 mode;
	struct max5980_data *ddata = dev_get_drvdata(dev);
	mode = max5980_read_reg(ddata, MAX5980_OPER_MODE_REG);
	mode = (mode >> porti * MAX5980_PT_MODE_FIELD_WIDTH) & MAX5980_PT_MODE_MASK;
	return (int)mode;
}

static int max5980_set_pt_mode(struct device *dev, int porti, int mode) {
	u8 mcfg;
	struct max5980_data *ddata = dev_get_drvdata(dev);

	mcfg = max5980_read_reg(ddata, MAX5980_OPER_MODE_REG);
	mcfg = append_pt_mode_reg(mcfg, porti, mode);
	max5980_write_reg(ddata, MAX5980_OPER_MODE_REG, mcfg);

	if (mode == MAX5980_OPER_MODE_MANUAL) {
		/* Disable connection monitoring */
		mcfg = max5980_read_reg(ddata, MAX5980_DET_AND_CLASS_EN_REG);
		mcfg &= ~(MAX5980_DET_CLAS_EN << porti);
		max5980_write_reg(ddata, MAX5980_DET_AND_CLASS_EN_REG, mcfg);
		mcfg = max5980_read_reg(ddata, MAX5980_DISCONN_EN_REG);
		mcfg &= ~(MAX5980_DIS_EN << porti);
		max5980_write_reg(ddata, MAX5980_DISCONN_EN_REG, mcfg);
	} else if (mode == MAX5980_OPER_MODE_AUTO) {
		/* Enable connect detection */
		mcfg = max5980_read_reg(ddata, MAX5980_DET_AND_CLASS_EN_REG);
		mcfg |= MAX5980_DET_CLAS_EN << porti;
		max5980_write_reg(ddata, MAX5980_DET_AND_CLASS_EN_REG, mcfg);
		mcfg = max5980_read_reg(ddata, MAX5980_DISCONN_EN_REG);
		mcfg |= MAX5980_DIS_EN << porti;
		max5980_write_reg(ddata, MAX5980_DISCONN_EN_REG, mcfg);
	}
	return 0;
}

static int max5980_switch_pt_power(struct device *dev, int porti, int state) {
	int res;
	struct max5980_data *ddata = dev_get_drvdata(dev);
	if (porti >= 0 && porti < MAX5980_PORTS_NUM) {
		if (MAX5980_OPER_MODE_AUTO == max5980_get_pt_mode(dev, porti))
			return -EINVAL;
		if (state == 1)
			state = MAX5980_PORT_SELECT(porti) << MAX5980_PWR_ON_SHIFT;
		else
			state = MAX5980_PORT_SELECT(porti) << MAX5980_PWR_OFF_SHIFT;
	}
	else
		return (res = -EINVAL);
	return max5980_write_reg(ddata, MAX5980_PWR_EN_PUSHBTN_REG, state);
}


static int max5980_apply_dt_defaults(struct device *dev) {
	int i;
	struct max5980_data *ddata = dev_get_drvdata(dev);
	for (i = 0; i < MAX5980_PORTS_NUM; i++) {
		if (!pt_is_valid(dev, i))
			max5980_set_pt_mode(dev, i, MAX5980_OPER_MODE_SHUTDOWN);
		else {
			max5980_set_pt_mode(dev, i, ddata->pdata.pt_df[i].mode);
			if (ddata->pdata.pt_df[i].mode == MAX5980_OPER_MODE_MANUAL)
				max5980_switch_pt_power(dev, i,
				    ddata->pdata.pt_df[i].pwr);
		}
	}
	return 0;
}

/* 
 * SYSFS block
 * ********************************
 */
static ssize_t max5980_store_pt_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	u8 mode;
	int port_idx = ASCII_TO_DIGIT(buf[0]);
	if (!pt_is_valid(dev, port_idx))
		return -EINVAL;
	if (!strncmp(buf + 1, "off", 3))
		mode = MAX5980_OPER_MODE_SHUTDOWN;
	else if (!strncmp(buf + 1, "manual", 6))
		mode = MAX5980_OPER_MODE_MANUAL;
	else if (!strncmp(buf + 1, "semiauto", 8))
		mode = MAX5980_OPER_MODE_SEMIAUTO;
	else if (!strncmp(buf + 1, "auto", 4))
		mode = MAX5980_OPER_MODE_AUTO;
	else
		return -EINVAL;
	max5980_set_pt_mode(dev, port_idx, mode);
	return count;
}

static ssize_t max5980_store_pt_power(struct device *dev,
		const char *buf, size_t count, int state) {
	int res;
	long port_idx;
	res = kstrtol(buf, 10, &port_idx);
	if (res != 0)
		return res;
	if (!pt_is_valid(dev, port_idx))
		return -EINVAL;
	res = max5980_switch_pt_power(dev, port_idx, state);
	if (res == 0)
		return count;
	else
		return res;
}

static ssize_t max5980_store_pt_power_on(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	return max5980_store_pt_power(dev, buf, count, MAX5980_P_ON);
}

static ssize_t max5980_store_pt_power_off(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	return max5980_store_pt_power(dev, buf, count, MAX5980_P_OFF);
}

static ssize_t max5980_show_pt_info(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int i;
	int len;
	unsigned long val = 0;
	struct max5980_data *ddata = dev_get_drvdata(dev);
	max5980_read(ddata, MAX5980_DC_REGS, MAX5980_DC_INFO_SZ, ddata->pt_dc);
	len = sprintf(buf, "# mode voltage current\n"); /* column names */
	for (i = 0; i < MAX5980_PORTS_NUM; i++) {
		/* Port number */
		len += sprintf(buf+len, "%d ", i);
		/* Port mode of operation */
		val = max5980_get_pt_mode(dev, i);
		if (val == MAX5980_OPER_MODE_SHUTDOWN)
			len += sprintf(buf+len, "off ");
		else if (val == MAX5980_OPER_MODE_MANUAL)
			len += sprintf(buf+len, "manual ");
		else if (val == MAX5980_OPER_MODE_SEMIAUTO)
			len += sprintf(buf+len, "semiauto ");
		else if (val == MAX5980_OPER_MODE_AUTO)
			len += sprintf(buf+len, "auto ");
		else
			len += sprintf(buf+len, "unknown ");
		/* Port voltage */
		if (!pt_is_valid(dev, i))
			len += sprintf(buf+len,"* ");
		else {
			val = VOLT_LSB * __le16_to_cpu(ddata->pt_dc[i].v);
			len += sprintf(buf+len,"%lu.%01lu ",
					val/1000000, (val%1000000)/100000);
		}
		/* Port current intensity */
		if (!pt_is_valid(dev, i))
			len += sprintf(buf+len,"*\n");
		else {
			val = CURR_LSB * __le16_to_cpu(ddata->pt_dc[i].i);
			len += sprintf(buf+len, "%lu.%03lu\n",
					val/1000000000, val/1000000);
		}
	}
	return len;
}

static DEVICE_ATTR(port_mode, S_IWUSR|S_IWGRP,
		NULL, max5980_store_pt_mode);
static DEVICE_ATTR(port_power_on, S_IWUSR|S_IWUSR,
		NULL, max5980_store_pt_power_on);
static DEVICE_ATTR(port_power_off, S_IWUSR|S_IWUSR,
		NULL, max5980_store_pt_power_off);
static DEVICE_ATTR(port_info, S_IRUGO,
		max5980_show_pt_info, NULL);

static struct attribute *max5980_attributes[] = {
	&dev_attr_port_mode.attr,
	&dev_attr_port_power_on.attr,
	&dev_attr_port_power_off.attr,
	&dev_attr_port_info.attr,
	NULL,
};

static const struct attribute_group max5980_attr_group = {
	.attrs = max5980_attributes,
};

/* 
 * end of SYSFS block
 * ********************************
 */

static const struct of_device_id max5980_dt_id[] = {
	{ .compatible = "maxim,max5980"},
	{ }
};

static int max5980_of_probe(struct device *dev,
		struct max5980_platform_data *pdata) {
	int i;
	const char *pmode;
	struct device_node *np = dev->of_node;
	struct device_node *child;

	if (!of_match_device(max5980_dt_id, dev))
		return -ENODEV;

	if ((!np) || !of_get_next_child(np, NULL))
		return -EINVAL;
	
	of_property_read_u32(np, "irq-gpio", &pdata->irq);
	i = 0;
	for_each_child_of_node(np, child) {
		of_property_read_u32(child, "enable", &pdata->pt_df[i].enable);

		if (0 == of_property_read_string(child, "mode", &pmode)) {
			if (!strncmp(pmode, "manual", 6))
				pdata->pt_df[i].mode = MAX5980_OPER_MODE_MANUAL;
			else if (!strncmp(pmode, "semiauto", 8))
				pdata->pt_df[i].mode = MAX5980_OPER_MODE_SEMIAUTO;
			else if (!strncmp(pmode, "auto", 4))
				pdata->pt_df[i].mode = MAX5980_OPER_MODE_AUTO;
			else
				pdata->pt_df[i].mode = MAX5980_OPER_MODE_SHUTDOWN;
		}
		of_property_read_u32(child, "power", &pdata->pt_df[i].pwr);
		i++;
	}
	return 0;
}

static int max5980_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	int res;
	static struct max5980_data *ddata;
	struct max5980_platform_data *pdata;
	struct device *dev = &client->dev;

	ddata = devm_kzalloc(dev, sizeof(struct max5980_data), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;
	res = sysfs_create_group(&dev->kobj, &max5980_attr_group);
	if (res) {
		dev_err(dev, "failed to create sysfs attributes\n");
		devm_kfree(&client->dev, ddata);
		return res;	
	}
	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(dev, ddata);
	pdata = &ddata->pdata;
	res = max5980_of_probe(dev, pdata);
	max5980_apply_dt_defaults(dev);

	dev_info(dev, "MAX5980 ID 0x%x, rev. number %u, fw %u",
			max5980_read_reg(ddata, MAX5980_DEV_ID_REV_NUM_REG) >> 5 & 0x7,
			max5980_read_reg(ddata, MAX5980_DEV_ID_REV_NUM_REG) & 0x7,
			max5980_read_reg(ddata, MAX5980_FW_REG));

	max5980_irq_enable(ddata, MAX5980_SUP_INT | MAX5980_TST_INT | MAX5980_TCUT_INT | MAX5980_CLS_INT | MAX5980_DET_INT);
	res = request_threaded_irq(gpio_to_irq(pdata->irq), NULL,
			max5980_irq_handler,
			IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
			"max5980", ddata);
	if (res) {
		dev_err(dev, "failed to register interrupt\n");
		sysfs_remove_group(&client->dev.kobj, &max5980_attr_group);
		devm_kfree(&client->dev, ddata);
		return res;
	}
	max5980_irq_clear(ddata);
	max5980_irq_handler(gpio_to_irq(pdata->irq), ddata);
	return 0;
}

static void max5980_remove(struct i2c_client *client)
{
	struct max5980_data *ddata = i2c_get_clientdata(client);
	free_irq(gpio_to_irq(ddata->pdata.irq), ddata);
	sysfs_remove_group(&client->dev.kobj, &max5980_attr_group);
	devm_kfree(&client->dev, ddata);
}

static const struct i2c_device_id max5980_id[] = {
	{ "max5980", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max5980_id);

static struct i2c_driver max5980_driver = {
	.driver = {
		.name   = "max5980",
		.owner  = THIS_MODULE,
		.of_match_table = max5980_dt_id,
	},
	.probe          = max5980_probe,
	.remove         = max5980_remove,
	.id_table	= max5980_id,
};

static int __init max5980_init(void) {
	return i2c_add_driver(&max5980_driver);
}

static void __exit max5980_exit(void) {
	i2c_del_driver(&max5980_driver);
}

module_init(max5980_init);
module_exit(max5980_exit);

MODULE_AUTHOR("Evgenii Tsybra <zcjack81@gmail.com>");
MODULE_DESCRIPTION("MAX5980 Quad Port POE PSE Controller Driver");
MODULE_LICENSE("GPL v2");
