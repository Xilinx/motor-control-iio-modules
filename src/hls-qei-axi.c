// SPDX-License-Identifier: GPL-2.0
/*
 * HLS QEI driver
 *
 * Copyright (C) 2023, Advanced Micro Devices, Inc.
 *
 * Description:
 * This driver is developed for HLS quadrature encoder. The driver supports INDIO Mode
 * and supports encoder data and direction monitoring via IIO sysfs interface, buffer capable.
 */

#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sched/task.h>

/* register space */
#define AP_CTRL	0x00
#define REG_CPR_I	0x10
#define REG_CPR_O	0x18
#define REG_CTRL	0x20		/* register for direction clockwise or counterclockwise */
#define REG_THETA_RPM	0x28
#define REG_DIR	0x38
#define REG_ERR	0x48			/* clear on read COR */
#define MASK_UPPER	GENMASK(31, 16)
#define MASK_LOWER	GENMASK(15, 0)
#define DEFAULT_CPR	0x3E8
#define DEFAULT_AP_CTRL	0x00
#define SAMPLE_INTERVAL	0xFF
#define SIGNEX(v, sb) (((v) & (1 << ((sb) - 1))) ? ((v) | (~((1 << (sb)) - 1))) : (v))
#define COUNT_CHANNEL(num, _ext)				\
	{							\
	    .type = IIO_ROT,					\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = REG_THETA_RPM,				\
	    .scan_index = (num),				\
	    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				BIT(IIO_CHAN_INFO_SCALE),	\
	    .scan_type = {					\
			.sign = 'u',				\
			.realbits = 32,				\
			.storagebits = 32,			\
			.shift = 0,				\
			},					\
	    .extend_name = _ext,				\
	}							\

struct qei_data {
	void __iomem *base;
	u32 *data;
	u32 interval;
	struct task_struct *task;
};

static const struct iio_chan_spec rot_channels[] = {
	COUNT_CHANNEL(0, "rpm"),
	COUNT_CHANNEL(1, "theta_degree"),
	IIO_CHAN_SOFT_TIMESTAMP(2),

};

static inline void qei_read_reg(struct qei_data *st, unsigned int offset,
				u32 *data)
{
	*data = readl(st->base + offset);
}

static inline void qei_write_reg(struct qei_data *st, unsigned int offset,
				 u32 data)
{
	writel(data, st->base + offset);
}

/* function to convert raw value to degrees */
static int xqei_convert_to_deg(unsigned int *data)
{
	unsigned int tmp;
		tmp = MASK_UPPER & *data;
		tmp = tmp >> 16;
		tmp = (tmp * 360 / 1000);
		*data = tmp;
	return 0;
}

static int qei_read_attribute(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channels, int *val,
			      int *val2, long mask)
{
	struct qei_data *st = iio_priv(indio_dev);
	unsigned int data;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		qei_read_reg(st, channels->address, &data);
		if (!(channels->channel)) {
			/* read RPM for channel 0 */
			*val = SIGNEX(data & MASK_LOWER, 16);
		} else {/* read theta for channel 1 */
			xqei_convert_to_deg(&data);
			*val = data;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int xqei_verify_write_success(struct iio_dev *indio_dev, unsigned int reg_in,
				     unsigned int reg_out, u32 data_in)
{
	struct qei_data *data = iio_priv(indio_dev);
	u32 data_out;

	qei_write_reg(data, reg_in, data_in);
	/* The HW requires 100ms to propagate data_in value to the shadow reg */
	mdelay(100);
	qei_read_reg(data, reg_out, &data_out);
	/* comparing shadow reg with data_in */
	if (data_out != data_in) {
		dev_warn(&indio_dev->dev, "value %u is out of bounds. Register [base + %u] failed to set\n",
			 data_in, reg_out);
		return -EINVAL;
	}
	return 0;
}

static ssize_t qei_ctrl_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct qei_data *st = iio_priv(indio_dev);
	unsigned int val;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case REG_CPR_I:
		qei_read_reg(st, REG_CPR_O, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case REG_CTRL:
		qei_read_reg(st, REG_CTRL, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case REG_DIR:
		qei_read_reg(st, REG_DIR, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case REG_ERR:
		qei_read_reg(st, REG_ERR, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case AP_CTRL:
		qei_read_reg(st, AP_CTRL, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case SAMPLE_INTERVAL:
		val = st->interval;
		ret = sprintf(buf, "%u\n", val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t qei_ctrl_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct qei_data *st = iio_priv(indio_dev);
	unsigned long long data_in;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case REG_CPR_I:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			xqei_verify_write_success(indio_dev, REG_CPR_I, REG_CPR_O, data_in);
		else
			return ret;
		break;
	case REG_CTRL:
		ret = kstrtoull(buf, 10, &data_in);
		if (ret) {
			return ret;
		} else if (data_in == 1 || data_in == 0) {
			qei_write_reg(st, REG_CTRL, data_in);
		} else {
			dev_err(&indio_dev->dev, "1: 'A lead B', 0: 'B lead A'. Value %llu is out of range\n",
				data_in);
			return -EINVAL;
		}
		break;
	case REG_DIR:
		return -EINVAL;
	case REG_ERR:
		return -EINVAL;
	case AP_CTRL:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			qei_write_reg(st, AP_CTRL, data_in);
		else
			return ret;
		break;
	case SAMPLE_INTERVAL:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			st->interval = data_in;
		else
			return ret;
		break;
	default:
		return -EINVAL;
	}
return len;
}

static const struct of_device_id xqei_ids[] = {
	{ .compatible = "xlnx,hls-qei-axi-1.0", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, xqei_ids);

static IIO_DEVICE_ATTR(error_status, 0644,
		       qei_ctrl_show,
		       qei_ctrl_store,
		       REG_ERR);

static IIO_DEVICE_ATTR(direction, 0644,
		       qei_ctrl_show,
		       qei_ctrl_store,
		       REG_DIR);

static IIO_DEVICE_ATTR(mode, 0644,
		       qei_ctrl_show,
		       qei_ctrl_store,
		       REG_CTRL);

static IIO_DEVICE_ATTR(cycles_per_revolution, 0644,
		       qei_ctrl_show,
		       qei_ctrl_store,
		       REG_CPR_I);

static IIO_DEVICE_ATTR(ap_ctrl, 0644,
		       qei_ctrl_show,
		       qei_ctrl_store,
		       AP_CTRL);

static IIO_DEVICE_ATTR(sample_interval_us, 0644,
		       qei_ctrl_show,
		       qei_ctrl_store,
		       SAMPLE_INTERVAL);

static struct attribute *qei_attributes[] = {
	&iio_dev_attr_cycles_per_revolution.dev_attr.attr,
	&iio_dev_attr_sample_interval_us.dev_attr.attr,
	&iio_dev_attr_error_status.dev_attr.attr,
	&iio_dev_attr_direction.dev_attr.attr,
	&iio_dev_attr_ap_ctrl.dev_attr.attr,
	&iio_dev_attr_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group qei_attribute_group = {
	.attrs = qei_attributes,
};

static const struct iio_info xqei_info = {
	.read_raw = &qei_read_attribute,
	.attrs = &qei_attribute_group,
};

static int xqei_fill_buffer(struct iio_dev *indio_dev)
{
	struct qei_data *st = iio_priv(indio_dev);
	/* data buffer for channel and time stamp */
	unsigned int data[1 + sizeof(s64) / sizeof(int)] = {0};
	unsigned int val, chan = 0, i = 0;
	s64 time;

	time = iio_get_time_ns(indio_dev);
	/*
	 * Read data for active channels from device
	 */
	for_each_set_bit(chan, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		qei_read_reg(st, REG_THETA_RPM, &val);
		if (!chan) {
			/* read RPM for channel 0 */
			data[i] = SIGNEX(val & MASK_LOWER, 16);
		} else {
			/* read theta for channel 1 */
			xqei_convert_to_deg(&val);
			data[i] = val;
		}
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data, time);

	return 0;
}

static int xqei_capture_thread(void *data)
{
	struct iio_dev *indio_dev = data;
	struct qei_data *st = iio_priv(indio_dev);
	u32 interval = st->interval;

	do {
		/* Read the data from sensor and push it to buffers */
		int ret = xqei_fill_buffer(indio_dev);

		if (ret < 0)
			return ret;
		usleep_range(interval, interval + 20);
	} while (!kthread_should_stop());

	return 0;
}

static int xqei_buffer_enable(struct iio_dev *indio_dev)
{
	struct qei_data *st = iio_priv(indio_dev);
	struct task_struct *task;

	task = kthread_run(xqei_capture_thread, (void *)indio_dev,
			      "%s", indio_dev->name);

	if (IS_ERR(task))
		return PTR_ERR(task);

	get_task_struct(task);
	wake_up_process(task);
	st->task = task;

	return 0;
}

static int xqei_buffer_disable(struct iio_dev *indio_dev)
{
	struct qei_data *st = iio_priv(indio_dev);

	if (st->task) {
		kthread_stop(st->task);
		put_task_struct(st->task);
		st->task = NULL;
	}
	return 0;
}

static const struct iio_buffer_setup_ops xqei_setup_ops = {
	.predisable = &xqei_buffer_disable,
	.postenable = &xqei_buffer_enable,
};

static int xqei_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct qei_data *data;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(xqei_ids, pdev->dev.of_node);

	if (!id)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));

	if (!indio_dev) {
		dev_err(&pdev->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &xqei_info;
	indio_dev->name = KBUILD_MODNAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = rot_channels;
	indio_dev->num_channels = ARRAY_SIZE(rot_channels);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	data->interval = 500;

	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	qei_write_reg(data, REG_CPR_I, DEFAULT_CPR);
	qei_write_reg(data, AP_CTRL, DEFAULT_AP_CTRL);
	ret = devm_iio_kfifo_buffer_setup(&indio_dev->dev, indio_dev,
					  &xqei_setup_ops);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Successfully registered device XQEI encoder");
	platform_set_drvdata(pdev, indio_dev);

	return 0;
}

static int xqei_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	return 0;
}

static struct platform_driver xqei_drv = {
	.probe = xqei_probe,
	.remove = xqei_remove,
	.driver = {
		.name = "hls-qei-axi",
		.of_match_table = of_match_ptr(xqei_ids),
	},
};
module_platform_driver(xqei_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vivekananda Dayananda <vivekananda.dayananda@amd.com>");
