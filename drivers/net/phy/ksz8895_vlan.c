/**
 * vlan specific kobject management stuff, include directly in ksz8895.c
 */

struct ksz8895_sysfs_vlan_entry {
	struct kobject kobj;
	struct ksz8895_vlan_entry entry;
	struct ksz8895_data *ksz8895;
};

#define to_ksz8895_sysfs_vlan_entry(vlan) container_of(vlan, struct ksz8895_sysfs_vlan_entry, kobj)

struct ksz8895_vlan_attribute {
	struct attribute attr;
	ssize_t (*show)(struct ksz8895_sysfs_vlan_entry *,
	                struct ksz8895_vlan_attribute *,
					char *);
	ssize_t (*store)(struct ksz8895_sysfs_vlan_entry *,
	                 struct ksz8895_vlan_attribute *,
					 const char *,
					 size_t);
};

#define to_ksz8895_vlan_attribute(vlan) container_of(vlan, struct ksz8895_vlan_attribute, attr)

/* free memory associated with vlan entry */
static void ksz8895_vlan_release(struct kobject *kobj) {
	struct ksz8895_sysfs_vlan_entry *entry = to_ksz8895_sysfs_vlan_entry(kobj);
	kfree(entry);
}

/* show a vlan entry */
static ssize_t ksz8895_vlan_show(struct kobject *kobj, struct attribute *attr, char *buf) {
	struct ksz8895_sysfs_vlan_entry *entry = to_ksz8895_sysfs_vlan_entry(kobj);
	struct ksz8895_vlan_attribute *vlan_attr = to_ksz8895_vlan_attribute(attr);

	if (!vlan_attr->show)
		return -EIO;

	return vlan_attr->show(entry, vlan_attr, buf);
}

/* modify a vlan entry */
static ssize_t ksz8895_vlan_store(
	struct kobject *kobj,
	struct attribute *attr,
	const char *buf,
	size_t count)
{
	struct ksz8895_sysfs_vlan_entry *entry = to_ksz8895_sysfs_vlan_entry(kobj);
	struct ksz8895_vlan_attribute *vlan_attr = to_ksz8895_vlan_attribute(attr);

	if (!vlan_attr->store)
		return -EIO;

	return vlan_attr->store(entry, vlan_attr, buf, count);
}

static const struct sysfs_ops ksz8895_vlan_ops = {
	.show = ksz8895_vlan_show,
	.store = ksz8895_vlan_store,
};

static int ksz8895_sysfs_vlan_write(struct ksz8895_sysfs_vlan_entry *entry) {
	return ksz8895_write_vlan_entry(
		entry->ksz8895,
		&entry->entry,
		entry->entry.valid
	);
}

/**
 * Raw entry access
 */
static ssize_t vlan_entry_show(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "valid: %d\nfid: 0x%02x\nmembers: 0x%02x\n",
		entry->entry.valid,
		entry->entry.fid,
		entry->entry.membership);
}

/**
 * Add a port to the vlan membership list
 */
static ssize_t vlan_add_port_store(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned int port;
	int ret;

	ret = kstrtouint(buf, 10, &port);
	if (ret)
		return ret;

	if (port < 1 || port > 5)
		return -EINVAL;

	dev_info(entry->ksz8895->dev,
		"add port request for %d to vlan %d\n",
		port, entry->entry.id);

	/* logical 1-indexed ports are 0-indexed in hardware */
	port -= 1;

	entry->entry.membership |= (1 << port);

	ret = ksz8895_sysfs_vlan_write(entry);
	if (ret)
		return ret;

	return count;
}

/**
 * View or modify the ports as an integer, use add/remove instead for simplicity
 */
static ssize_t vlan_ports_show(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", entry->entry.membership);
}

static ssize_t vlan_ports_store(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned int ports;
	int ret;

	ret = kstrtouint(buf, 10, &ports);
	if (ret)
		return ret;

	/* 5 bit fields for 5 ports */
	if (ports > 0x1f)
		return -EINVAL;

	entry->entry.membership = ports;
	ret = ksz8895_sysfs_vlan_write(entry);
	if (ret)
		return ret;

	return count;
}

static ssize_t vlan_remove_port_store(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned int port;
	int ret;

	ret = kstrtouint(buf, 10, &port);
	if (ret)
		return ret;

	if (port < 1 || port > 5)
		return -EINVAL;

	dev_info(entry->ksz8895->dev,
		"remove port request for %d to vlan %d\n",
		port, entry->entry.id);

	/* logical 1-indexed ports are 0-indexed in hardware */
	port -= 1;

	entry->entry.membership &= ~(1 << port);

	ret = ksz8895_sysfs_vlan_write(entry);
	if (ret)
		return ret;

	return count;
}

static ssize_t vlan_fid_show(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", entry->entry.fid);
}

static ssize_t vlan_fid_store(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned int fid;
	int ret;

	ret = kstrtouint(buf, 10, &fid);
	if (ret)
		return ret;

	if (fid > KSZ8895_MAXIMUM_VLAN_FID)
		return -EINVAL;

	dev_info(entry->ksz8895->dev, "vlan %d: change fid %d -> %d\n",
		entry->entry.id, entry->entry.fid, fid);

	entry->entry.fid = (u8) fid;

	ret = ksz8895_sysfs_vlan_write(entry);
	if (ret)
		return ret;

	return count;
}

static ssize_t vlan_valid_show(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", entry->entry.valid);
}

static ssize_t vlan_valid_store(
	struct ksz8895_sysfs_vlan_entry *entry,
	struct ksz8895_vlan_attribute *attr,
	const char *buf,
	size_t count)
{
	bool valid;
	int ret;

	ret = kstrtobool(buf, &valid);
	if (ret)
		return ret;

	dev_info(entry->ksz8895->dev, "vlan %d: change valid %d -> %d\n",
		entry->entry.id, entry->entry.valid, valid);

	entry->entry.valid = valid;

	ret = ksz8895_sysfs_vlan_write(entry);
	if (ret)
		return ret;

	return count;
}

static struct ksz8895_vlan_attribute attr_vlan_entry = __ATTR(
	entry,
	S_IRUGO,
	vlan_entry_show,
	NULL
);

static struct ksz8895_vlan_attribute attr_vlan_add_port = __ATTR(
	add_port,
	S_IWUSR,
	NULL,
	vlan_add_port_store
);

static struct ksz8895_vlan_attribute attr_vlan_ports = __ATTR(
	ports,
	(S_IWUSR | S_IRUGO),
	vlan_ports_show,
	vlan_ports_store
);

static struct ksz8895_vlan_attribute attr_vlan_remove_port = __ATTR(
	remove_port,
	S_IWUSR,
	NULL,
	vlan_remove_port_store
);

static struct ksz8895_vlan_attribute attr_vlan_fid = __ATTR(
	fid,
	(S_IWUSR | S_IRUGO),
	vlan_fid_show,
	vlan_fid_store
);

static struct ksz8895_vlan_attribute attr_vlan_valid = __ATTR(
	valid,
	(S_IWUSR | S_IRUGO),
	vlan_valid_show,
	vlan_valid_store
);

static struct attribute *ksz8895_vlan_default_attrs[] = {
	&attr_vlan_entry.attr,
	&attr_vlan_add_port.attr,
	&attr_vlan_ports.attr,
	&attr_vlan_remove_port.attr,
	&attr_vlan_fid.attr,
	&attr_vlan_valid.attr,
	NULL
};

static struct kobj_type ksz8895_vlan_type = {
	.sysfs_ops = &ksz8895_vlan_ops,
	.release = ksz8895_vlan_release,
	.default_attrs = ksz8895_vlan_default_attrs,
};

