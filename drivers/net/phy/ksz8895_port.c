/**
 * port specific kobject management stuff, include directly in ksz8895.c
 */

/**
 * @ksz8895 pointer to data object for ksz8895_* methods
 * @id port id, 1-5
 * @regbase base register address to use for control regs
 */
struct ksz8895_sysfs_port {
	struct kobject kobj;
	struct ksz8895_data *ksz8895;
	uint8_t id;
	u8 regbase;
};

#define to_ksz8895_sysfs_port(port) container_of(port, struct ksz8895_sysfs_port, kobj)

struct ksz8895_port_attribute {
	struct attribute attr;
	ssize_t (*show)(struct ksz8895_sysfs_port *,
	                struct ksz8895_port_attribute *,
					char *);
	ssize_t (*store)(struct ksz8895_sysfs_port *,
	                 struct ksz8895_port_attribute *,
					 const char *,
					 size_t);
};

/* attribute that has a register offset to read/write embedded */
struct ksz8895_port_control_attribute {
	struct ksz8895_port_attribute attr;
	u8 regoffset;
};

#define to_ksz8895_port_attribute(port) container_of(port, struct ksz8895_port_attribute, attr)
#define to_ksz8895_port_control(port) \
	container_of(port, struct ksz8895_port_control_attribute, attr)

/* free memory associated with a port entry */
static void ksz8895_port_release(struct kobject *kobj) {
	struct ksz8895_sysfs_port *port = to_ksz8895_sysfs_port(kobj);
	kfree(port);
}

static ssize_t ksz8895_port_show(struct kobject *kobj, struct attribute *attr, char *buf) {
	struct ksz8895_sysfs_port *port = to_ksz8895_sysfs_port(kobj);
	struct ksz8895_port_attribute *port_attr = to_ksz8895_port_attribute(attr);

	if (!port_attr->show)
		return -EIO;

	return port_attr->show(port, port_attr, buf);
}

static ssize_t ksz8895_port_store(
	struct kobject *kobj,
	struct attribute *attr,
	const char *buf,
	size_t count)
{
	struct ksz8895_sysfs_port *port = to_ksz8895_sysfs_port(kobj);
	struct ksz8895_port_attribute *port_attr = to_ksz8895_port_attribute(attr);

	if (!port_attr->store)
		return -EIO;

	return port_attr->store(port, port_attr, buf, count);
}

static const struct sysfs_ops ksz8895_port_ops = {
	.show = ksz8895_port_show,
	.store = ksz8895_port_store,
};

static ssize_t port_vid_show(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	char *buf)
{
	int result;
	unsigned int vid = 0;

	result = ksz8895_smi_read(port->ksz8895, port->regbase + 3);
	if (result < 0)
		return result;

	vid |= (result & 0x0f) << 8;

	result = ksz8895_smi_read(port->ksz8895, port->regbase + 4);
	if (result < 0)
		return result;

	vid |= (result & 0xff);

	return scnprintf(buf, PAGE_SIZE, "%d\n", vid);
}

static ssize_t port_vid_store(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	const char *buf,
	size_t count)
{
	unsigned int vid;
	int ret;

	ret = kstrtouint(buf, 10, &vid);
	if (ret)
		return ret;

	if (vid > KSZ8895_MAXIMUM_VLAN_ID)
		return -EINVAL;

	/* don't clobber other bits */
	ret = ksz8895_smi_modify(
		port->ksz8895,
		port->regbase + 3,
		0x0f,
		(u8) ((vid >> 8) & 0x0f)
	);
	if (ret)
		return ret;

	/* this is the whole register */
	ret = ksz8895_smi_write(
		port->ksz8895,
		port->regbase + 4,
		(u8) (vid & 0xff)
	);
	if (ret)
		return ret;

	return count;
}

static ssize_t port_ingress_filter_show(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	char *buf)
{
	int result;
	int filter;

	result = ksz8895_smi_read(port->ksz8895, port->regbase + 2);
	if (result < 0)
		return result;

	filter = result & KSZ8895_INGRESS_VLAN_FILTERING;
	return scnprintf(buf, PAGE_SIZE, "%d\n", filter);
}

static ssize_t port_ingress_filter_store(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	const char *buf,
	size_t count)
{
	bool filter;
	int ret;

	ret = kstrtobool(buf, &filter);
	if (ret)
		return ret;

	ret = ksz8895_smi_modify(
		port->ksz8895,
		port->regbase + 2,
		KSZ8895_INGRESS_VLAN_FILTERING,
		filter ? KSZ8895_INGRESS_VLAN_FILTERING : 0
	);
	if (ret)
		return ret;

	return count;
}

static ssize_t port_insert_tag_show(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	char *buf)
{
	bool insert;
	int result;

	result = ksz8895_smi_read(port->ksz8895,
	                          port->regbase + 0);
	if (result < 0)
		return result;

	insert = (result & KSZ8895_TAG_INSERTION) ? 1 : 0;
	return scnprintf(buf, PAGE_SIZE, "%d\n", insert);
}

static ssize_t port_insert_tag_store(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	const char *buf,
	size_t count)
{
	bool insert;
	int ret;

	ret = kstrtobool(buf, &insert);
	if (ret)
		return ret;

	ret = ksz8895_smi_modify(
		port->ksz8895,
		port->regbase + 0,
		KSZ8895_TAG_INSERTION,
		insert ? KSZ8895_TAG_INSERTION : 0
	);
	if (ret)
		return ret;

	return count;
}

static ssize_t port_remove_tag_show(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	char *buf)
{
	bool remove;
	int result;

	result = ksz8895_smi_read(port->ksz8895,
	                          port->regbase + 0);
	if (result < 0)
		return result;

	remove = (result & KSZ8895_TAG_REMOVAL) ? 1 : 0;
	return scnprintf(buf, PAGE_SIZE, "%d\n", remove);
}

static ssize_t port_remove_tag_store(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	const char *buf,
	size_t count)
{
	bool remove;
	int ret;

	ret = kstrtobool(buf, &remove);
	if (ret)
		return ret;

	ret = ksz8895_smi_modify(
		port->ksz8895,
		port->regbase + 0,
		KSZ8895_TAG_REMOVAL,
		remove ? KSZ8895_TAG_REMOVAL : 0
	);
	if (ret)
		return ret;

	return count;
}

static ssize_t port_control_show(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	char *buf)
{
	struct ksz8895_port_control_attribute *control;
	int ret;

	control = to_ksz8895_port_control(attr);

	ret = ksz8895_smi_read(
		port->ksz8895,
		port->regbase + control->regoffset
	);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ret);
}

static ssize_t port_control_store(
	struct ksz8895_sysfs_port *port,
	struct ksz8895_port_attribute *attr,
	const char *buf,
	size_t count)
{
	struct ksz8895_port_control_attribute *control;
	int value;
	int ret;

	ret = kstrtoint(buf, 10, &value);

	if (value > 0xff)
		return -EINVAL;

	control = to_ksz8895_port_control(attr);
	ret = ksz8895_smi_write(
		port->ksz8895,
		port->regbase + control->regoffset,
		(u8) value
	);

	if (ret)
		return ret;

	return count;
}

static struct ksz8895_port_attribute attr_port_vid = __ATTR(
	vid,
	(S_IWUSR | S_IRUGO),
	port_vid_show,
	port_vid_store
);

static struct ksz8895_port_attribute attr_port_ingress_filter = __ATTR(
	ingress_filter,
	(S_IWUSR | S_IRUGO),
	port_ingress_filter_show,
	port_ingress_filter_store
);

static struct ksz8895_port_attribute attr_port_insert_tag = __ATTR(
	insert_tag,
	(S_IWUSR | S_IRUGO),
	port_insert_tag_show,
	port_insert_tag_store
);

static struct ksz8895_port_attribute attr_port_remove_tag = __ATTR(
	remove_tag,
	(S_IWUSR | S_IRUGO),
	port_remove_tag_show,
	port_remove_tag_store
);

#define KSZ8895_PORT_CONTROL_ATTR(name, offset) \
static struct ksz8895_port_control_attribute attr_port_##name = { \
	.attr = __ATTR( \
		name, \
		(S_IWUSR | S_IRUGO), \
		port_control_show, \
		port_control_store \
	), \
	.regoffset = offset \
}

KSZ8895_PORT_CONTROL_ATTR(control0, 0);
KSZ8895_PORT_CONTROL_ATTR(control1, 1);
KSZ8895_PORT_CONTROL_ATTR(control2, 2);
KSZ8895_PORT_CONTROL_ATTR(control3, 3);
KSZ8895_PORT_CONTROL_ATTR(control4, 4);
KSZ8895_PORT_CONTROL_ATTR(status0, 9);
KSZ8895_PORT_CONTROL_ATTR(phy_special, 10);
KSZ8895_PORT_CONTROL_ATTR(linkmd, 11);
KSZ8895_PORT_CONTROL_ATTR(control5, 12);
KSZ8895_PORT_CONTROL_ATTR(control6, 13);
KSZ8895_PORT_CONTROL_ATTR(status1, 14);
KSZ8895_PORT_CONTROL_ATTR(status2, 15);		/* same register as control 7 */
KSZ8895_PORT_CONTROL_ATTR(control7, 15);

/* Additional port control registers in the advanced section */
KSZ8895_PORT_CONTROL_ATTR(control8, 0xa0);

static struct attribute *ksz8895_port_default_attrs[] = {
	&attr_port_vid.attr,
	&attr_port_ingress_filter.attr,
	&attr_port_insert_tag.attr,
	&attr_port_remove_tag.attr,
	&attr_port_control0.attr.attr,
	&attr_port_control1.attr.attr,
	&attr_port_control2.attr.attr,
	&attr_port_control3.attr.attr,
	&attr_port_control4.attr.attr,
	&attr_port_status0.attr.attr,
	&attr_port_phy_special.attr.attr,
	&attr_port_linkmd.attr.attr,
	&attr_port_control5.attr.attr,
	&attr_port_control6.attr.attr,
	&attr_port_status1.attr.attr,
	&attr_port_status2.attr.attr,
	&attr_port_control7.attr.attr,
	&attr_port_control8.attr.attr,
	NULL
};

static struct kobj_type ksz8895_port_type = {
	.sysfs_ops = &ksz8895_port_ops,
	.release = ksz8895_port_release,
	.default_attrs = ksz8895_port_default_attrs,
};
