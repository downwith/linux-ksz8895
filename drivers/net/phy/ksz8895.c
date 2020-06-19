/*
 * drivers/net/phy/ksz8895.c
 *
 * Driver for the PHY-mode Micrel/Microchip KSZ8895 5-port switch using SMI
 *
 * Author: Nathan L. Conrad <nathan@noreply.alt-teknik.com>
 *         Greg Malysa <greg.malysa@timesys.com>
 *
 * Copyright (c) 2013 - 2017 Nathan L. Conrad
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/errno.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#define KSZ8895_PHY_ID			0x00221450
#define KSZ8895_PHY_ID_MASK		0xFFFFFFFF
#define KSZ8895_PORT_MASK		0x1F
#define KSZ8895_PORTS			5

/* The address for the PHY-mode switch on the pseudo MII bus.  Must be different
 * than KSZ8895_PHY_5_ADDR to simplify the differentiating of PHY devices in the
 * PHY driver.  See ksz8895_switch_match_phy_device.
 */
#define KSZ8895_SWITCH_ADDR		0x00

#define KSZ8895_PHY_1_ADDR		0x01
#define KSZ8895_PHY_5_ADDR		0x05

/* The MII_BMSR bits to indicate the switch link is up */
#define KSZ8895_SWITCH_LINK_UP		(BMSR_LSTATUS | BMSR_ANEGCOMPLETE)

/* SMI registers */
#define KSZ8895_CHIP_ID_0		0x00
#define KSZ8895_CHIP_ID_1		0x01
#define KSZ8895_GLOBAL_CONTROL_1	0x03
#define KSZ8895_GLOBAL_CONTROL_2	0x04
#define KSZ8895_GLOBAL_CONTROL_3	0x05
#define KSZ8895_GLOBAL_CONTROL_4	0x06
#define KSZ8895_GLOBAL_CONTROL_9	0x0B
#define KSZ8895_POWER_MGMT_CONTROL_1	0x0E
#define KSZ8895_PORT_CONTROL_OFFSET	0x10
#define KSZ8895_PORT_CONTROL_0		0x10
#define KSZ8895_PORT_CONTROL_2		0x12
#define KSZ8895_PORT_CONTROL_4		0x14
#define KSZ8895_INDIRECT_CONTROL_0	0x6E
#define KSZ8895_INDIRECT_CONTROL_1	0x6F
#define KSZ8895_INDIRECT_DATA_0		0x78
#define KSZ8895_CACHE_START		KSZ8895_GLOBAL_CONTROL_4
#define KSZ8895_CACHE_SIZE		1

/* KSZ8895_CHIP_ID_0 */
#define KSZ8895_FAMILY_ID		0x95

/* KSZ8895_CHIP_ID_1 */
#define KSZ8895_START_SWITCH		0x01
#define KSZ8895_REVISION_ID_MASK	0x0E
#define KSZ8895MQ_CHIP_ID		0x40
#define KSZ8895RQ_CHIP_ID		0x60
#define KSZ8895FMQ_CHIP_ID		0xC0
#define KSZ8895_CHIP_ID_MASK		0xF0
#define KSZ8895_REVISION_ID_SHIFT	1

/* KSZ8895_GLOBAL_CONTROL_1 */
#define KSZ8895_AGGRESSIVE_BACK_OFF	0x01
#define KSZ8895_AGING_ENABLE		0x04
#define KSZ8895_RX_FC_DISABLE		0x10
#define KSZ8895_TX_FC_DISABLE		0x20

/* KSZ8895_GLOBAL_CONTROL_2 */
#define KSZ8895_PACKET_SIZE_CHK_DISABLE	0x02
#define KSZ8895_NO_EXCESS_COL_DROP	0x08

/* KSZ8895_GLOBAL_CONTROL_3 */
#define KSZ8895_VLAN_ENABLE		0x80

/* KSZ8895_GLOBAL_CONTROL_4 */
#define KSZ8895_SW5_MII_10MBPS_SPEED	0x10
#define KSZ8895_SW5_MII_FC_ENABLE	0x20
#define KSZ8895_SW5_MII_HALF_DUPLEX	0x40

/* KSZ8895_GLOBAL_CONTROL_9 */
#define KSZ8895_LED_MODE		0x02

/* KSZ8895_POWER_MGMT_CONTROL_1 */
#define KSZ8895_ENERGY_DETECTION_MODE	0x08

/* KSZ8895_PORT_CONTROL_0 */
#define KSZ8895_TAG_REMOVAL		0x02
#define KSZ8895_TAG_INSERTION		0x04

/* KSZ8895_PORT_CONTROL_2 */
#define KSZ8895_BACK_PRESSURE_ENABLE	0x08
#define KSZ8895_INGRESS_VLAN_FILTERING	0x40

/* KSZ8895_PORT_CONTROL_3 */
#define KSZ8895_DEFAULT_TAG_VID_MASK	0x0F

/* KSZ8895_INDIRECT_CONTROL_0 */
#define KSZ8895_INDIRECT_SELECT_MASK	0x1F
#define KSZ8895_INDIRECT_READ_SELECT	0x10
#define KSZ8895_VLAN_TABLE_SELECT	0x04

/* VLAN table */
#define KSZ8895_MAXIMUM_VLAN_ID		4095
#define KSZ8895_INDIRECT_VLAN_TABLES	4

/* VLAN table entries */
#define KSZ8895_VLAN_ENTRY_MASK		0x1FFF
#define KSZ8895_VLAN_ENTRY_MEMBER_MASK	0x0F80
#define KSZ8895_VLAN_ENTRY_VALID	0x1000
#define KSZ8895_VLAN_ENTRY_PROPERTIES	3
#define KSZ8895_VLAN_ENTRY_BITS		13
#define KSZ8895_MAXIMUM_VLAN_FID	127
#define KSZ8895_VLAN_ENTRY_MEMBER_SHIFT	7

struct ksz8895_vlan_entry {
	u16 id;
	bool valid;
	u8 fid;
	u8 membership;
};

struct ksz8895_data {
	struct device *dev;

	/* The actual MII/SMI bus */
	struct mii_bus *bus;

	/* The psuedo MII bus for the switch */
	struct mii_bus *switch_bus;

	/* If true, a link is detected by scanning the switch PHYs.  If false,
	 * the link is always up whenever the switch is enabled.
	 */
	bool detect_switch_link;

	/* If not null, caches configuration registers in an array so they don't
	 * need to be read repeatedly
	 */
	u8 *cache;

	/* vlan table is a kset of entries */
	struct kset *vlan_entries;

	struct kset *ports;

	struct gpio_desc *reset;
	struct work_struct reset_work;

};

struct ksz8895_smi_override {
	const char *name;
	u8 regnum;
	u8 mask;

	/* True if there is a register for each port */
	bool port;
};

/* Forward declarations of all functions in this module for included files to see them */
static ssize_t vlan_export_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t vlan_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t vlan_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t vlan_unexport_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);

static int ksz8895_smi_regnum_to_mii_addr(u8 regnum);
static u32 ksz8895_smi_regnum_to_mii_regnum(u8 regnum);
static int ksz8895_smi_read(struct ksz8895_data *data, u8 regnum);
static int ksz8895_smi_cached_read(struct ksz8895_data *data, u8 regnum);
static int ksz8895_smi_write(struct ksz8895_data *data, u8 regnum, u8 val);
static int ksz8895_smi_modify(struct ksz8895_data *data, u8 regnum, u8 mask, u8 val);
static int ksz8895_mii_read(struct ksz8895_data *data, int addr, u32 regnum);
static int ksz8895_read(struct mii_bus *bus, int phy_id, int regnum);
static int ksz8895_write(struct mii_bus *bus, int phy_id, int regnum, u16 val);

static int ksz8895_read_vlan_entry(struct ksz8895_data *data, struct ksz8895_vlan_entry *vlan);
static int ksz8895_trigger_vlan_table_access(struct ksz8895_data *data, u16 addr, bool read);
static int ksz8895_write_vlan_entry(struct ksz8895_data *data,
	struct ksz8895_vlan_entry *entry, bool valid);

static int ksz8895_probe_vlan_table(struct ksz8895_data *data);
static int ksz8895_probe_default_tags(struct ksz8895_data *data);
static int ksz8895_probe_smi_overrides(struct ksz8895_data *data);
static int ksz8895_probe_cache(struct ksz8895_data *data);
static int ksz8895_probe_device_tree(struct ksz8895_data *data);
static void ksz8895_free_data(struct ksz8895_data *data);
static int ksz8895_probe(struct platform_device *pdev);
static int ksz8895_remove(struct platform_device *pdev);
static void ksz8895_soft_reset_work(struct work_struct *work);

static int ksz8895_set_supported(struct phy_device *phydev);
static int ksz8895_switch_config_init(struct phy_device *phydev);
static int ksz8895_phy_5_config_init(struct phy_device *phydev);

#include "ksz8895_vlan.c"
#include "ksz8895_port.c"

static ssize_t vlan_export_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	char vlan_name[32];
	struct kobject *kobj;
	struct ksz8895_sysfs_vlan_entry *entry;
	struct ksz8895_data *ksz8895;
	unsigned long vid;
	int ret;

	ret = kstrtoul(buf, 10, &vid);
	if (ret)
		return ret;

	if (vid > KSZ8895_MAXIMUM_VLAN_ID)
		return -EINVAL;

 	scnprintf(vlan_name, sizeof(vlan_name), "vlan%lu", vid);

	dev_info(dev, "ksz vlan_export request for %lu\n", vid);
	ksz8895 = dev_get_drvdata(dev);

	kobj = kset_find_obj(ksz8895->vlan_entries, vlan_name);
	if (kobj) {
		dev_dbg(dev, "found matching vlan entry, reusing it.\n");
		kobject_put(kobj);
		goto done;
	}

	dev_dbg(dev, "creating vlan entry named %s\n", vlan_name);

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->ksz8895 = ksz8895;
	entry->entry.id = (uint16_t) vid;
	ret = ksz8895_read_vlan_entry(ksz8895, &entry->entry);
	if (ret) {
		kfree(entry);
		return -EIO;
	}

	entry->kobj.kset = ksz8895->vlan_entries;
	ret = kobject_init_and_add(&entry->kobj, &ksz8895_vlan_type, NULL, "vlan%lu", vid);
	if (ret) {
		kobject_put(&entry->kobj);
		return -EIO;
	}

	kobject_uevent(&entry->kobj, KOBJ_ADD);

done:
	return count;
}

static ssize_t vlan_enable_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct ksz8895_data *data;
	int ret;

	data = dev_get_drvdata(dev);
	ret = ksz8895_smi_read(data, KSZ8895_GLOBAL_CONTROL_3);
	if (ret < 0)
		return ret;

	ret = (ret & KSZ8895_VLAN_ENABLE) ? 1 : 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t vlan_enable_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	bool enable;
	int ret;
	struct ksz8895_data *data = dev_get_drvdata(dev);

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	ret = ksz8895_smi_modify(data,
	                   KSZ8895_GLOBAL_CONTROL_3,
					   KSZ8895_VLAN_ENABLE,
					   enable ? KSZ8895_VLAN_ENABLE : 0);
	if (ret)
		return ret;

	return count;
}

static ssize_t vlan_unexport_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	char vlan_name[32];
	struct kobject *kobj;
	struct ksz8895_data *ksz8895;
	unsigned long vid;
	int ret;

	ret = kstrtoul(buf, 10, &vid);
	if (ret)
		return ret;

 	scnprintf(vlan_name, sizeof(vlan_name), "vlan%lu", vid);

	dev_info(dev, "ksz vlan_unexport request for %lu\n", vid);
	ksz8895 = dev_get_drvdata(dev);

	kobj = kset_find_obj(ksz8895->vlan_entries, vlan_name);
	if (kobj) {
		dev_dbg(dev, "found matching vlan entry, removing it.\n");
		kobject_put(kobj);
		kobject_put(kobj);
		return count;
	}

	return -EINVAL;
}

static ssize_t soft_reset_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct ksz8895_data *ksz8895;

	ksz8895 = dev_get_drvdata(dev);

	schedule_work(&ksz8895->reset_work);

	return count;
}

struct ksz8895_control_attribute {
	struct device_attribute attr;
	u8 regnum;
};

#define to_ksz8895_control_attribute(c) container_of(c, struct ksz8895_control_attribute, attr)

static ssize_t ksz8895_control_show(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct ksz8895_data *ksz8895;
	struct ksz8895_control_attribute *control;
	int ret;

	control = to_ksz8895_control_attribute(attr);
	ksz8895 = dev_get_drvdata(dev);

	ret = ksz8895_smi_read(ksz8895, control->regnum);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ret);
}

static ssize_t ksz8895_control_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct ksz8895_data *ksz8895;
	struct ksz8895_control_attribute *control;
	int ret;
	int value;

	control = to_ksz8895_control_attribute(attr);
	ksz8895 = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &value);
	if (ret)
		return ret;

	if (value > 0xff)
		return -EINVAL;

	ret = ksz8895_smi_write(ksz8895, control->regnum, (u8) value);
	if (ret)
		return ret;

	return count;
}

#define KSZ8895_CONTROL_ATTR(name, num) \
static struct ksz8895_control_attribute control_attr_##name = { \
	.attr = __ATTR( \
		name, \
		(S_IWUSR | S_IRUGO), \
		ksz8895_control_show, \
		ksz8895_control_store \
	), \
	.regnum = num \
}

static DEVICE_ATTR_WO(vlan_export);
static DEVICE_ATTR_RW(vlan_enable);
static DEVICE_ATTR_WO(vlan_unexport);
static DEVICE_ATTR_WO(soft_reset);
KSZ8895_CONTROL_ATTR(chipid0, 0);
KSZ8895_CONTROL_ATTR(chipid1, 1);
KSZ8895_CONTROL_ATTR(control0, 2);
KSZ8895_CONTROL_ATTR(control1, 3);
KSZ8895_CONTROL_ATTR(control2, 4);
KSZ8895_CONTROL_ATTR(control3, 5);
KSZ8895_CONTROL_ATTR(control4, 6);
KSZ8895_CONTROL_ATTR(control5, 7);
/* control6-8 are factory test regs and should not be modified */
KSZ8895_CONTROL_ATTR(control9, 11);
KSZ8895_CONTROL_ATTR(control10, 12);
/* control11 is a factory test register */
KSZ8895_CONTROL_ATTR(power_control1, 14);
KSZ8895_CONTROL_ATTR(power_control2, 15);

KSZ8895_CONTROL_ATTR(control19, 135);

/* @todo add "advanced control registers" here */

static struct attribute *ksz8895_attrs[] = {
	&dev_attr_vlan_export.attr,
	&dev_attr_vlan_enable.attr,
	&dev_attr_vlan_unexport.attr,
	&dev_attr_soft_reset.attr,
	&control_attr_chipid0.attr.attr,
	&control_attr_chipid1.attr.attr,
	&control_attr_control0.attr.attr,
	&control_attr_control1.attr.attr,
	&control_attr_control2.attr.attr,
	&control_attr_control3.attr.attr,
	&control_attr_control4.attr.attr,
	&control_attr_control5.attr.attr,
	&control_attr_control9.attr.attr,
	&control_attr_control10.attr.attr,
	&control_attr_power_control1.attr.attr,
	&control_attr_power_control2.attr.attr,
	&control_attr_control19.attr.attr,
	NULL
};

static struct attribute_group ksz_device_attrs = {
	.attrs = ksz8895_attrs
};

static const struct attribute_group *ksz_attr_groups[] = {
	&ksz_device_attrs,
	NULL
};

static int ksz8895_smi_regnum_to_mii_addr(u8 regnum)
{
	return (regnum >> 5 & 0x01) | 0x06 | (regnum >> 3 & 0x18);
}

static u32 ksz8895_smi_regnum_to_mii_regnum(u8 regnum)
{
	return regnum & 0x1F;
}

static int ksz8895_smi_read(struct ksz8895_data *data, u8 regnum)
{
	int val = mdiobus_read(data->bus,
			       ksz8895_smi_regnum_to_mii_addr(regnum),
			       ksz8895_smi_regnum_to_mii_regnum(regnum));

	if (val < 0)
		dev_err(data->dev,
			"Read from SMI register 0x%x failed with %i\n", regnum,
			val);
	else if (val & 0xFFFFFF00) {
		dev_err(data->dev,
			"Invalid read of 0x%x from SMI register 0x%x\n", val,
			regnum);
		return -EINVAL;
	}
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(data->dev, "Read 0x%x from SMI register 0x%x\n", val, regnum);
#endif
	return val;
}

static int ksz8895_smi_cached_read(struct ksz8895_data *data, u8 regnum)
{
	int val;

	if (data->cache) {
		val = data->cache[regnum - KSZ8895_CACHE_START];
#ifdef CONFIG_KSZ8895_DEBUG
		dev_info(data->dev, "Read 0x%x from SMI register 0x%x cache\n",
			 val, regnum);
#endif
	} else
		val = ksz8895_smi_read(data, regnum);
	return val;
}

static int ksz8895_smi_write(struct ksz8895_data *data, u8 regnum, u8 val)
{
	int ret = mdiobus_write(data->bus,
			       ksz8895_smi_regnum_to_mii_addr(regnum),
			       ksz8895_smi_regnum_to_mii_regnum(regnum), val);

	if (ret < 0)
		dev_err(data->dev,
			"Write of 0x%x to SMI register 0x%x failed with %i\n",
			val, regnum, ret);
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(data->dev, "Wrote 0x%x to SMI register 0x%x\n", val, regnum);
#endif
	return ret;
}

static int ksz8895_smi_modify(struct ksz8895_data *data, u8 regnum, u8 mask,
			      u8 val)
{
	int ret = ksz8895_smi_read(data, regnum);
	u8 write;

	if (ret < 0)
		return ret;
	write = ((u8)ret & ~mask) | (val & mask);
	return ret == write ? 0 : ksz8895_smi_write(data, regnum, write);
}

static int ksz8895_mii_read(struct ksz8895_data *data, int addr, u32 regnum)
{
	int val = mdiobus_read(data->bus, addr, regnum);

	if (val < 0)
		dev_err(data->dev,
			"Read from MII register 0x%x at address 0x%x failed with %i\n",
			regnum, addr, val);
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(data->dev,
		 "Read 0x%x from MII register 0x%x at address 0x%x\n", val,
		 regnum, addr);
#endif
	return val;
}

static int ksz8895_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct ksz8895_data *data = bus->priv;
	int ret, val, addr;

	if (phy_id != KSZ8895_SWITCH_ADDR || regnum > MII_NCONFIG) {
		dev_err(data->dev,
			"Invalid read from register 0x%x at address 0x%x\n",
			regnum, phy_id);
		return -EINVAL;
	}
	switch (regnum) {
	case MII_BMCR:
		ret = BMCR_FULLDPLX | BMCR_ANENABLE | BMCR_SPEED100;
		break;
	case MII_BMSR:
		ret = BMSR_ANEGCAPABLE | BMSR_100FULL;
		if (!data->detect_switch_link) {
			ret |= KSZ8895_SWITCH_LINK_UP;
			break;
		}
		for (addr = KSZ8895_PHY_1_ADDR; addr < KSZ8895_PHY_5_ADDR;
		     ++addr) {
			val = ksz8895_mii_read(data, addr, (u32)regnum);
			if (val < 0)
				return val;
			val &= KSZ8895_SWITCH_LINK_UP;
			if (val == KSZ8895_SWITCH_LINK_UP) {
				ret |= KSZ8895_SWITCH_LINK_UP;
				break;
			}
		}
		break;
	case MII_PHYSID1:
		ret = KSZ8895_PHY_ID >> 16;
		break;
	case MII_PHYSID2:
		ret = (u16)KSZ8895_PHY_ID;
		break;
	case MII_ADVERTISE:
	case MII_LPA:
		val = ksz8895_smi_cached_read(data, KSZ8895_GLOBAL_CONTROL_4);
		if (val < 0)
			return val;
		ret = ADVERTISE_CSMA | ADVERTISE_100FULL;
		if (val & KSZ8895_SW5_MII_FC_ENABLE)
			ret |= ADVERTISE_PAUSE_CAP;
		break;
	default:
		ret = 0;
	}
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(data->dev, "Read 0x%x from register 0x%x\n", ret, regnum);
#endif
	return ret;
}

static int ksz8895_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
	struct ksz8895_data *data = bus->priv;

	if (phy_id != KSZ8895_SWITCH_ADDR || regnum > MII_NCONFIG) {
		dev_err(data->dev,
			"Invalid write of 0x%x to register 0x%x at address 0x%x\n",
			val, regnum, phy_id);
		return -EINVAL;
	}
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(data->dev, "Wrote 0x%x to register 0x%x\n", val, regnum);
#endif
	return 0;
}

#define VLAN_BIT_READ(data, out, reg, ret, mask, shift) \
	do { \
		ret = ksz8895_smi_read(data, reg); \
		if (ret < 0) \
			return ret; \
		dev_dbg(data->dev, "vlan read: reg %d = 0x%02x\n", reg, ret); \
		out |= (uint16_t) ((ret & mask) << shift); \
	} while (0)

#define VLAN_BIT_READ_RIGHT(data, out, reg, ret, mask, shift) \
	do { \
		ret = ksz8895_smi_read(data, reg); \
		if (ret < 0) \
			return ret; \
		dev_dbg(data->dev, "vlan read: reg %d = 0x%02x\n", reg, ret); \
		out |= (uint16_t) ((ret & mask) >> shift); \
	} while (0)

/**
 * Read the vlan table entry for the given vlan id
 * @vlan->id must be set to the id to read
 */
static int ksz8895_read_vlan_entry(
	struct ksz8895_data *data,
	struct ksz8895_vlan_entry *vlan
) {
	uint16_t offset = vlan->id / KSZ8895_INDIRECT_VLAN_TABLES;
	uint16_t which = vlan->id % KSZ8895_INDIRECT_VLAN_TABLES;
	uint16_t vlan_bits = 0;
	int ret;

	/* load all four vlan entries to indirect access registers */
	ret = ksz8895_trigger_vlan_table_access(data, offset, true);
	if (ret < 0)
		return ret;

	dev_dbg(data->dev, "which = %d\n", which);

	/* read out 13 bit record based on which vlan entry we're looking for */
	switch (which) {
	case 0:
		VLAN_BIT_READ(data, vlan_bits, 120, ret, 0xff, 0);
		VLAN_BIT_READ(data, vlan_bits, 119, ret, 0x1f, 8);
		break;
	case 1:
		VLAN_BIT_READ_RIGHT(data, vlan_bits, 119, ret, 0xe0, 5);
		VLAN_BIT_READ(data, vlan_bits, 118, ret, 0xff, 3);
		VLAN_BIT_READ(data, vlan_bits, 117, ret, 0x03, 11);
		break;
	case 2:
		VLAN_BIT_READ_RIGHT(data, vlan_bits, 117, ret, 0xfc, 2);
		VLAN_BIT_READ(data, vlan_bits, 116, ret, 0x7f, 6);
		break;
	case 3:
	default:
		VLAN_BIT_READ_RIGHT(data, vlan_bits, 116, ret, 0x80, 7);
		VLAN_BIT_READ(data, vlan_bits, 115, ret, 0xff, 1);
		VLAN_BIT_READ(data, vlan_bits, 114, ret, 0x0f, 9);
		break;
	}

	dev_dbg(data->dev, "read vlan entry %d as 0x%04x\n", vlan->id, vlan_bits);

	/* parse vlan table entry into data structure */
	vlan->valid = (vlan_bits >> 12) & 0x01;
	vlan->membership = (vlan_bits >> 7) & 0x1f;
	vlan->fid = vlan_bits & 0x7f;
	return 0;
}

static int ksz8895_trigger_vlan_table_access(struct ksz8895_data *data,
					     u16 addr, bool read)
{
	int ret = ksz8895_smi_modify(data, KSZ8895_INDIRECT_CONTROL_0,
				     KSZ8895_INDIRECT_SELECT_MASK,
				     (u8)(addr >> 8) |
				     KSZ8895_VLAN_TABLE_SELECT |
				     (read ? KSZ8895_INDIRECT_READ_SELECT : 0));

	if (ret < 0)
		return ret;
	ret = ksz8895_smi_write(data, KSZ8895_INDIRECT_CONTROL_1, (u8)addr);
	return ret < 0 ? ret : 0;
}

static int ksz8895_write_vlan_entry(struct ksz8895_data *data,
				    struct ksz8895_vlan_entry *entry,
				    bool valid)
{
	int ret;
	u32 val, mask;
	u16 addr = entry->id / KSZ8895_INDIRECT_VLAN_TABLES;
	u8 bit = (u8)((entry->id % KSZ8895_INDIRECT_VLAN_TABLES) *
		 KSZ8895_VLAN_ENTRY_BITS);
	u8 regnum = KSZ8895_INDIRECT_DATA_0 - (bit / 8);

	bit %= 8;
	val = valid ? (entry->fid | ((u32)entry->membership <<
	      KSZ8895_VLAN_ENTRY_MEMBER_SHIFT &
	      KSZ8895_VLAN_ENTRY_MEMBER_MASK) | KSZ8895_VLAN_ENTRY_VALID) <<
	      bit : 0;
	mask = (u16)KSZ8895_VLAN_ENTRY_MASK << bit;
	ret = ksz8895_trigger_vlan_table_access(data, addr, true);
	if (ret)
		return ret;
	ret = ksz8895_smi_modify(data, regnum--, (u8)mask, (u8)val);
	if (ret < 0)
		return ret;
	ret = ksz8895_smi_modify(data, regnum--, (u8)(mask >> 8),
				 (u8)(val >> 8));
	if (ret < 0)
		return ret;
	if (bit > 16 - KSZ8895_VLAN_ENTRY_BITS) {
		ret = ksz8895_smi_modify(data, regnum--, (u8)(mask >> 16),
					 (u8)(val >> 16));
		if (ret < 0)
			return ret;
	}
	ret = ksz8895_trigger_vlan_table_access(data, addr, false);
	if (ret)
		return ret;
	if (valid)
		dev_info(data->dev,
			 "Assigned VLAN ID %u FID %u and ports 0x%x\n",
			 entry->id, entry->fid, entry->membership);
	return 0;
}

static int ksz8895_probe_vlan_table(struct ksz8895_data *data)
{
	struct property *prop = of_find_property(data->dev->of_node,
						 "vlan-table", NULL);
	struct ksz8895_vlan_entry entry = {
		.fid		= 0,
		.membership	= 0,
	};
	struct ksz8895_vlan_entry *table, *i;
	const __be32 *prop_val;
	int ret, length;
	u32 val;
	bool write_all = of_property_read_bool(data->dev->of_node,
					       "write-entire-vlan-table");
	bool valid;

	if (!prop) {
		if (!write_all)
			return 0;
		ret = 0;
		length = 0;
		table = NULL;
		prop_val = NULL;
	} else if (!prop->value)
		ret = -ENODATA;
	else if (prop->length <= 0 ||
		 prop->length % (sizeof(u32) * KSZ8895_VLAN_ENTRY_PROPERTIES))
		ret = -EINVAL;
	else {
		ret = 0;
		length = prop->length /
			 (sizeof(u32) * KSZ8895_VLAN_ENTRY_PROPERTIES);
		table = kzalloc(sizeof(*table) * length,  GFP_KERNEL);
		if (!table) {
			dev_err(data->dev, "Failed to allocate VLAN table\n");
			return -ENOMEM;
		}
		prop_val = prop->value;
	}
	if (ret) {
		dev_err(data->dev,
			"Invalid device tree 'vlan-table' property\n");
		return ret;
	}
	for (i = table; i < table + length; ++i) {
		val = be32_to_cpup(prop_val++);
		if (val > KSZ8895_MAXIMUM_VLAN_ID) {
			dev_err(data->dev, "Invalid VLAN ID %u\n", val);
			kfree(table);
			return -EINVAL;
		}
		i->id = (u16)val;
		val = be32_to_cpup(prop_val++);
		if (val > KSZ8895_MAXIMUM_VLAN_FID) {
			dev_err(data->dev, "Invalid VLAN FID %u\n", val);
			kfree(table);
			return -EINVAL;
		}
		i->fid = (u8)val;
		val = be32_to_cpup(prop_val++);
		if (val > KSZ8895_PORT_MASK) {
			dev_err(data->dev, "Invalid VLAN port membership %u\n",
				val);
			kfree(table);
			return -EINVAL;
		}
		i->membership = (u8)val;
	}
	for (entry.id = 0; entry.id <= KSZ8895_MAXIMUM_VLAN_ID; ++entry.id) {
		valid = false;
		for (i = table; i < table + length; ++i) {
			if (i->id == entry.id) {
				valid = true;
				break;
			}
		}
		if (!write_all && !valid)
			continue;
		ret = ksz8895_write_vlan_entry(data, valid ? i : &entry, valid);
		if (ret) {
			kfree(table);
			return ret;
		}
	}
	kfree(table);
	return 0;
}

static int ksz8895_probe_default_tags(struct ksz8895_data *data)
{
	u32 tags[KSZ8895_PORTS];
	u32 *tag;
	int ret = of_property_read_u32_array(data->dev->of_node, "default-tags",
					     tags, ARRAY_SIZE(tags));
	u8 i, regnum;

	if (ret) {
		if (ret == -EINVAL)
			return 0;
		dev_err(data->dev,
			"Invalid device tree 'default-tags' property\n");
		return -EINVAL;
	}
	for (i = 0, tag = tags; i < ARRAY_SIZE(tags); ++tag) {
		if (*tag > KSZ8895_MAXIMUM_VLAN_ID) {
			dev_err(data->dev, "Invalid default tag %u\n", *tag);
			return -EINVAL;
		}
		regnum = KSZ8895_PORT_CONTROL_4 +
			 i * KSZ8895_PORT_CONTROL_OFFSET;
		ret = ksz8895_smi_write(data, regnum--, (u8)*tag);
		if (ret < 0)
			return ret;
		ret = ksz8895_smi_modify(data, regnum,
					 KSZ8895_DEFAULT_TAG_VID_MASK,
					 (u8)(*tag >> 8));
		if (ret < 0)
			return ret;
		dev_info(data->dev, "Assigned port %u default tag %u\n", ++i,
			 *tag);
	}
	return 0;
}

static int ksz8895_probe_smi_overrides(struct ksz8895_data *data)
{
	/* Execution order follows array order */
	static const struct ksz8895_smi_override overrides[] = { {
		.name		= "tag-removal",
		.regnum		= KSZ8895_PORT_CONTROL_0,
		.mask		= KSZ8895_TAG_REMOVAL,
		.port		= true,
	}, {
		.name		= "tag-insertion",
		.regnum		= KSZ8895_PORT_CONTROL_0,
		.mask		= KSZ8895_TAG_INSERTION,
		.port		= true,
	}, {
		.name		= "ingress-vlan-filtering",
		.regnum		= KSZ8895_PORT_CONTROL_2,
		.mask		= KSZ8895_INGRESS_VLAN_FILTERING,
		.port		= true,
	}, {
		/* PMRXD0 strap option */
		.name		= "aggressive-back-off",
		.regnum		= KSZ8895_GLOBAL_CONTROL_1,
		.mask		= KSZ8895_AGGRESSIVE_BACK_OFF,
		.port		= false,
	}, {
		/* LED5-2 strap option */
		.name		= "aging",
		.regnum		= KSZ8895_GLOBAL_CONTROL_1,
		.mask		= KSZ8895_AGING_ENABLE,
		.port		= false,
	}, {
		/* PMRXD3 strap option */
		.name		= "disable-rx-flow-control",
		.regnum		= KSZ8895_GLOBAL_CONTROL_1,
		.mask		= KSZ8895_RX_FC_DISABLE,
		.port		= false,
	}, {
		/* PMRXD3 strap option */
		.name		= "disable-tx-flow-control",
		.regnum		= KSZ8895_GLOBAL_CONTROL_1,
		.mask		= KSZ8895_TX_FC_DISABLE,
		.port		= false,
	}, {
		/* PMRXER strap option */
		.name		= "disable-maximum-packet-size-check",
		.regnum		= KSZ8895_GLOBAL_CONTROL_2,
		.mask		= KSZ8895_PACKET_SIZE_CHK_DISABLE,
		.port		= false,
	}, {
		/* PMRXD1 strap option */
		.name		= "disable-excessive-collision-drop",
		.regnum		= KSZ8895_GLOBAL_CONTROL_2,
		.mask		= KSZ8895_NO_EXCESS_COL_DROP,
		.port		= false,
	}, {
		/* SMRXD1 strap option */
		.name		= "sw5-mii-10mbps",
		.regnum		= KSZ8895_GLOBAL_CONTROL_4,
		.mask		= KSZ8895_SW5_MII_10MBPS_SPEED,
		.port		= false,
	}, {
		/* SMRXD3 strap option */
		.name		= "sw5-mii-flow-control",
		.regnum		= KSZ8895_GLOBAL_CONTROL_4,
		.mask		= KSZ8895_SW5_MII_FC_ENABLE,
		.port		= false,
	}, {
		/* SMRXD2 strap option */
		.name		= "sw5-mii-half-duplex",
		.regnum		= KSZ8895_GLOBAL_CONTROL_4,
		.mask		= KSZ8895_SW5_MII_HALF_DUPLEX,
		.port		= false,
	}, {
		/* SMRXD0 strap option */
		.name		= "led-mode",
		.regnum		= KSZ8895_GLOBAL_CONTROL_9,
		.mask		= KSZ8895_LED_MODE,
		.port		= false,
	}, {
		/* LED4-0 strap option */
		.name		= "energy-detection-mode",
		.regnum		= KSZ8895_POWER_MGMT_CONTROL_1,
		.mask		= KSZ8895_ENERGY_DETECTION_MODE,
		.port		= false,
	}, {
		/* PMRXD2 strap option */
		.name		= "back-pressure",
		.regnum		= KSZ8895_PORT_CONTROL_2,
		.mask		= KSZ8895_BACK_PRESSURE_ENABLE,
		.port		= true,
	}, {
		.name		= "vlan",
		.regnum		= KSZ8895_GLOBAL_CONTROL_3,
		.mask		= KSZ8895_VLAN_ENABLE,
		.port		= false,
	}, {
		.name		= "start-switch",
		.regnum		= KSZ8895_CHIP_ID_1,
		.mask		= KSZ8895_START_SWITCH,
		.port		= false,
	}, };

	const struct ksz8895_smi_override *i;
	int ret;
	u32 vals[2];
	u8 mask, regnum;

	for (i = overrides; i < overrides + ARRAY_SIZE(overrides); ++i) {
		ret = of_property_read_u32_array(data->dev->of_node, i->name,
						 vals, i->port ?
						 ARRAY_SIZE(vals) : 1);
		if (ret || (i->port && (vals[0] == 0 ||
		    vals[0] > KSZ8895_PORT_MASK ||
		    vals[1] > KSZ8895_PORT_MASK)) || (!i->port && *vals > 1)) {
			if (ret == -EINVAL)
				continue;
			dev_err(data->dev,
				"Invalid device tree '%s' property\n", i->name);
			return -EINVAL;
		}
		if (i->port) {
			for (mask = 1, regnum = i->regnum;
			     mask < KSZ8895_PORT_MASK; mask <<= 1,
			     regnum += KSZ8895_PORT_CONTROL_OFFSET) {
				if (!(vals[0] & mask))
					continue;
				ret = ksz8895_smi_modify(data, regnum, i->mask,
							 vals[1] & mask ?
							 i->mask : 0);
				if (ret < 0)
					return ret;
			}
			dev_info(data->dev,
				 "Overrode '%s' for ports 0x%x with 0x%x\n",
				 i->name, vals[0], vals[0] & vals[1]);
		} else {
			ret = ksz8895_smi_modify(data, i->regnum, i->mask,
						 *vals ? i->mask : 0);
			if (ret < 0)
				return ret;
			dev_info(data->dev, "Overrode '%s' with %i\n",
				 i->name, (int)(*vals != 0));
		}
	}
	return 0;
}

static int ksz8895_probe_cache(struct ksz8895_data *data)
{
	int val;
	u8 i;

	if (!of_property_read_bool(data->dev->of_node, "cache-config"))
		return 0;
	data->cache = kzalloc(KSZ8895_CACHE_SIZE, GFP_KERNEL);
	if (!data->cache) {
		dev_err(data->dev, "Failed to allocate configuration cache\n");
		return -ENOMEM;
	}
	for (i = 0; i < KSZ8895_CACHE_SIZE; ++i) {
		val = ksz8895_smi_read(data, KSZ8895_CACHE_START + i);
		if (val < 0)
			return val;
		data->cache[i] = (u8)val;
	}
	dev_info(data->dev, "Cached configuration registers\n");
	return 0;
}

static int ksz8895_probe_device_tree(struct ksz8895_data *data)
{
	const __be32 *bus;
	struct device_node *node;
	const char *id;
	int val;

	/* Ensures the device tree node exists */
	if (!data->dev->of_node) {
		dev_err(data->dev, "Missing device tree node\n");
		return -EINVAL;
	}

	/* Determines if switch link detection is enabled */
	if (of_property_read_bool(data->dev->of_node, "detect-switch-link")) {
		data->detect_switch_link = true;
		dev_info(data->dev, "Enabled switch link detection\n");
	}

	/* Finds the MII/SMII bus */
	bus = of_get_property(data->dev->of_node, "bus", &val);
	if (!bus || val != sizeof(void *)) {
		dev_err(data->dev, "Missing device tree 'bus' property\n");
		return -EINVAL;
	}

	node = of_find_node_by_phandle(be32_to_cpup(bus));
	if (!node) {
		dev_err(data->dev,
			"Failed to find the MII/SMI node phandle\n");
		return -EPROBE_DEFER;
	}

	data->bus = of_mdio_find_bus(node);
	if (!data->bus) {
		dev_err(data->dev,
			"Failed to find the MII/SMI platform device\n");
		return -EPROBE_DEFER;
	}
	dev_info(data->dev, "Using '%s' for MII/SMI\n", data->bus->name);

	/* Ensures the family ID is correct */
	val = ksz8895_smi_read(data, KSZ8895_CHIP_ID_0);

	if (val < 0)
		return val;
	if (val != KSZ8895_FAMILY_ID) {
		dev_err(data->dev, "Invalid family ID 0x%x\n", val);
		return -EINVAL;
	}

	/* Notes the chip ID and revision ID */
	val = ksz8895_smi_read(data, KSZ8895_CHIP_ID_1);
	if (val < 0)
		return val;
	switch (val & KSZ8895_CHIP_ID_MASK) {
	case KSZ8895MQ_CHIP_ID:
		id = "KSZ8895MQ";
		break;
	case KSZ8895RQ_CHIP_ID:
		id = "KSZ8895RQ";
		break;
	case KSZ8895FMQ_CHIP_ID:
		id = "KSZ8895FMQ";
		break;
	default:
		id = "an unknown device";
	}
	dev_info(data->dev, "Detected %s revision 0x%x\n", id,
		 (val & KSZ8895_REVISION_ID_MASK) >> KSZ8895_REVISION_ID_SHIFT);

	/* If specified, writes the VLAN table */
	val = ksz8895_probe_vlan_table(data);
	if (val)
		return val;

	/* If specified, sets the default tags */
	val = ksz8895_probe_default_tags(data);
	if (val)
		return val;

	/* If specified, overrides SMI registers */
	val = ksz8895_probe_smi_overrides(data);
	if (val)
		return val;

	/* If specified, caches configuration registers */
	return ksz8895_probe_cache(data);
}

static void ksz8895_free_data(struct ksz8895_data *data)
{
	if (data->cache)
		kfree(data->cache);
	kfree(data);
}

static void ksz8895_reset(struct ksz8895_data *data) {
	if (!data->reset)
		return;
	
	gpiod_set_value(data->reset, 1);
	mdelay(1);
	gpiod_set_value(data->reset, 0);
}

static int ksz8895_probe(struct platform_device *pdev)
{
	/* Allocates the private driver data */
	struct ksz8895_data *data = kzalloc(sizeof(*data), GFP_KERNEL);
	struct phy_device *phydev;
	int ret = 0;
	struct ksz8895_sysfs_port *ports[5] = { 0 };
	int i;

	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate private driver data\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, data);
	data->dev = &pdev->dev;

	INIT_WORK(&data->reset_work, ksz8895_soft_reset_work);
	data->reset = devm_gpiod_get_optional(data->dev, "soft-reset", GPIOD_OUT_LOW);

	ksz8895_reset(data);

	/* Probes the device tree */
	ret = ksz8895_probe_device_tree(data);
	if (ret)
		goto data_free;

	/* Allocates and initializes a pseudo MII bus for the switch */
	data->switch_bus = mdiobus_alloc();
	if (!data->switch_bus) {
		dev_err(data->dev, "Failed to allocate MII bus\n");
		ret = -ENOMEM;
		goto data_free;
	}
	data->switch_bus->name = dev_name(data->dev);
	snprintf(data->switch_bus->id, MII_BUS_ID_SIZE, "%s", pdev->name);
	data->switch_bus->priv = data;
	data->switch_bus->read = ksz8895_read,
	data->switch_bus->write = ksz8895_write,
	data->switch_bus->parent = data->dev;
	data->switch_bus->phy_mask = ~((u32)1 << KSZ8895_SWITCH_ADDR);

	/* Registers the pseudo MII bus */
	ret = of_mdiobus_register(data->switch_bus, data->dev->of_node);
	if (ret) {
		dev_err(data->dev, "Failed to register MII bus\n");
		goto mdio_free;
	}

	/* Ensures a switch PHY driver was found */
	phydev = mdiobus_get_phy(data->switch_bus, KSZ8895_SWITCH_ADDR);
	if (!phydev || !phydev->drv) {
		dev_err(data->dev, "Failed to find a switch PHY driver\n");
		ret = -ENODEV;
		goto mdio_unreg;
	}
	dev_info(data->dev, "Detected PHY driver '%s'\n", phydev->drv->name);

	/* Create sysfs interface */
	ret = device_add_groups(data->dev, ksz_attr_groups);
	if (ret) {
		dev_err(data->dev, "Failed to create sysfs entries: %d\n", ret);
		goto mdio_unreg;
	}

	data->vlan_entries = kset_create_and_add("vlans", NULL, &data->dev->kobj);
	if (!data->vlan_entries) {
		ret = -ENOMEM;
		dev_err(data->dev, "Failed to create vlan kset.\n");
		goto mdio_unreg;
	}

	data->ports = kset_create_and_add("ports", NULL, &data->dev->kobj);
	if (!data->ports) {
		ret = -ENOMEM;
		dev_err(data->dev, "Failed to create port kset.\n");
		goto vlan_cleanup;
	}

	/* Create entries for each port on the switch */
	for (i = 0; i < 5; ++i) {
		ports[i] = kzalloc(sizeof(*ports[i]), GFP_KERNEL);
		if (!ports[i]) {
			ret = -ENOMEM;
			goto port_cleanup;
		}

		ports[i]->ksz8895 = data;
		ports[i]->id = (uint8_t) (i + 1);
		ports[i]->regbase = (u8) (0x10 * (i+1));
		ports[i]->kobj.kset = data->ports;
		ret = kobject_init_and_add(&ports[i]->kobj, &ksz8895_port_type, NULL, "port%d", i+1);
		if (ret) {
			ret = -EIO;
			goto port_cleanup;
		}

		kobject_uevent(&ports[i]->kobj, KOBJ_ADD);
	}

	return 0;

	port_cleanup:
		for (i = 0; i < 5; ++i) {
			if (ports[i]) {
				kobject_put(&ports[i]->kobj);
			}
		}
	vlan_cleanup:
		kset_unregister(data->vlan_entries);
	mdio_unreg:
		mdiobus_unregister(data->switch_bus);
	mdio_free:
		mdiobus_free(data->switch_bus);
	data_free:
		ksz8895_free_data(data);
	return ret;
}

static int ksz8895_remove(struct platform_device *pdev)
{
	struct ksz8895_data *data = platform_get_drvdata(pdev);

	// @todo need to iterate ports and vlan entries ksets and put objects

	kset_unregister(data->ports);
	kset_unregister(data->vlan_entries);
	mdiobus_unregister(data->switch_bus);
	mdiobus_free(data->switch_bus);
	ksz8895_free_data(data);
	return 0;
}

static const struct of_device_id ksz8895_of_match_table[] = { {
	.compatible = "microchip,ksz8895",
}, {
	/* Sentinel */
}, };
MODULE_DEVICE_TABLE(of, ksz8895_of_match_table);

static struct platform_driver ksz8895_platform_driver = {
	.driver = {
		.name = "ksz8895",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ksz8895_of_match_table),
	},
	.probe = ksz8895_probe,
	.remove = ksz8895_remove,
};

static void ksz8895_soft_reset_work(struct work_struct *work) {
	struct ksz8895_data *data = container_of(work, struct ksz8895_data, reset_work);
	int ret = 0;

	ksz8895_reset(data);
	ret = ksz8895_probe_device_tree(data);
	if (ret)
		dev_err(data->dev, "Unexpected error during soft-reset of switch!\n");
}

static int ksz8895_set_supported(struct phy_device *phydev)
{
	int val = phy_read(phydev, MII_BMSR);

	if (val < 0)
		return val;
	phydev->supported = SUPPORTED_TP | SUPPORTED_MII;
	if (val & BMSR_ANEGCAPABLE)
		phydev->supported |= SUPPORTED_Autoneg;
	if (val & BMSR_100FULL)
		phydev->supported |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		phydev->supported |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		phydev->supported |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		phydev->supported |= SUPPORTED_10baseT_Half;
	return 0;
}

static int ksz8895_switch_config_init(struct phy_device *phydev)
{
	int val = ksz8895_set_supported(phydev);
	struct ksz8895_data *data = phydev->mdio.bus->priv;

	if (val)
		return val;
	val = ksz8895_smi_cached_read(data, KSZ8895_GLOBAL_CONTROL_4);
	if (val < 0)
		return val;
	if (val & KSZ8895_SW5_MII_FC_ENABLE)
		phydev->supported |= SUPPORTED_Pause;
	phydev->advertising = phydev->supported;
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(&phydev->dev, "Initialized to support 0x%x\n",
		 phydev->supported);
#endif
	return 0;
}

/* Matches on the PHY-mode switch on the pseudo MII bus.  PHY 1-4 should not
 * match any driver as they are located and controlled by the platform driver if
 * desired.  Matches are performed based on the bus address which is fixed in
 * the platform driver for the PHY-mode switch and fixed in the hardware for
 * PHY 5.  Addresses 0x01-0x1F of the actual bus will be used by the 5 PHYs and
 * SMI registers.
 */
int ksz8895_switch_match_phy_device(struct phy_device *phydev)
{
	return (phydev->phy_id & KSZ8895_PHY_ID_MASK) ==
	       (KSZ8895_PHY_ID & KSZ8895_PHY_ID_MASK) &&
	       phydev->mdio.addr == KSZ8895_SWITCH_ADDR;
}

static int ksz8895_phy_5_config_init(struct phy_device *phydev)
{
	int ret = ksz8895_set_supported(phydev);

	if (ret)
		return ret;
	phydev->supported |= SUPPORTED_Pause;
	phydev->advertising = phydev->supported;
#ifdef CONFIG_KSZ8895_DEBUG
	dev_info(&phydev->dev, "Initialized to support 0x%x\n",
		 phydev->supported);
#endif
	return 0;
}

/* See ksz8895_switch_match_phy_device */
int ksz8895_phy_5_match_phy_device(struct phy_device *phydev)
{
	return (phydev->phy_id & KSZ8895_PHY_ID_MASK) ==
	       (KSZ8895_PHY_ID & KSZ8895_PHY_ID_MASK) &&
	       phydev->mdio.addr == KSZ8895_PHY_5_ADDR;
}

static struct phy_driver ksz8895_phy_drivers[] = { {
	.phy_id			= KSZ8895_PHY_ID,
	.name			= "Microchip KSZ8895 Switch",
	.phy_id_mask		= KSZ8895_PHY_ID_MASK,
	.config_init		= ksz8895_switch_config_init,
	.config_aneg		= genphy_config_aneg,
	.read_status		= genphy_read_status,
	.match_phy_device	= ksz8895_switch_match_phy_device,
}, {
	.phy_id			= KSZ8895_PHY_ID,
	.name			= "Microchip KSZ8895 PHY 5",
	.phy_id_mask		= KSZ8895_PHY_ID_MASK,
	.config_init		= ksz8895_phy_5_config_init,
	.config_aneg		= genphy_config_aneg,
	.read_status		= genphy_read_status,
	.match_phy_device	= ksz8895_phy_5_match_phy_device,
}, };

static int __init ksz8895_init(void)
{
	int ret;

	ret = phy_drivers_register(ksz8895_phy_drivers,
				   ARRAY_SIZE(ksz8895_phy_drivers),
				   THIS_MODULE);
	if (ret)
		return ret;
	ret = platform_driver_register(&ksz8895_platform_driver);
	if (ret)
		phy_drivers_unregister(ksz8895_phy_drivers,
				       ARRAY_SIZE(ksz8895_phy_drivers));
	return ret;
}
module_init(ksz8895_init);

static void __exit ksz8895_exit(void)
{
	platform_driver_unregister(&ksz8895_platform_driver);
	phy_drivers_unregister(ksz8895_phy_drivers,
			       ARRAY_SIZE(ksz8895_phy_drivers));
}
module_exit(ksz8895_exit);

static struct mdio_device_id __maybe_unused ksz8895_device_ids[] = { {
	.phy_id		= KSZ8895_PHY_ID,
	.phy_id_mask	= KSZ8895_PHY_ID_MASK,
}, {
	/* Sentinel */
}, };
MODULE_DEVICE_TABLE(mdio, ksz8895_device_ids);

MODULE_DESCRIPTION("Microchip KSZ8895 driver");
MODULE_AUTHOR("Nathan L. Conrad <nathan@noreply.alt-teknik.com>");
MODULE_LICENSE("GPL");
