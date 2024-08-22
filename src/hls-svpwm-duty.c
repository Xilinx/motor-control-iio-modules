// SPDX-License-Identifier: GPL-2.0
/*
 * SVPWM DUTY driver
 *
 * Copyright (C) 2023, Advanced Micro Devices, Inc.
 *
 * Description:
 * This driver is developed for Vitis xf_motorcontrol HLS library, IP SVPWM_DUTY.
 * The driver supports INDIO Mode with control and monitoring of the IP via
 * IIO sysfs interface, also buffer capable.
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
#define AP_CTRL		0x00
#define DC_LINK_REF_DATA	0x10
#define STT_CNT_ITER_DATA	0x18 // debug reg
#define DC_SRC_MODE_DATA	0x28
#define SAMPLE_II_DATA	0x30
#define VA_CMD_DATA	0x38
#define VB_CMD_DATA	0x48
#define VC_CMD_DATA	0x58
/* defaults */
#define DEFAULT_REF	0x180000
#define DEFAULT_MODE	0x0
#define DEFAULT_II	0x3E8
#define DEFAULT_AP_CTRL	0
#define MIN_DELAY_US	1000
#define MAX_DELAY_US	1200
/* channel spec */
#define VOLTAGE_CHANNEL(num, _ext)				\
	{							\
	    .type = IIO_VOLTAGE,				\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = VA_CMD_DATA + (num * 0x10),		\
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

struct private_data {
	void __iomem *base;
	u32 *data;
	struct task_struct *task;
};

static const struct iio_chan_spec volt_cmd_channels[] = {
	VOLTAGE_CHANNEL(0, "Va_cmd"),
	VOLTAGE_CHANNEL(1, "Vb_cmd"),
	VOLTAGE_CHANNEL(2, "Vc_cmd"),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static inline void svpwm_read_reg(struct private_data *st, unsigned int offset,
				  u32 *data)
{
	*data = readl(st->base + offset);
}

static inline void svpwm_write_reg(struct private_data *st, unsigned int offset,
				   u32 data)
{
	writel(data, st->base + offset);
}

static int svpmw_read_attribute(struct iio_dev *indio_dev,
				struct iio_chan_spec const *channels, int *val,
				int *val2, long mask)
{
	struct private_data *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		svpwm_read_reg(st, channels->address, val);
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

/**
 * svpwm_debugfs_reg_access - read or write register value
 * @indio_dev: IIO device structure
 * @reg: register offset
 * @writeval: value to write
 * @readval: value to read
 *
 * To read a value from the register:
 *   echo [svpwm reg offset] > direct_reg_access
 *   cat direct_reg_access
 *
 * To write a value in a svpwm register:
 *   echo [svpwm_reg_offset] [value] > direct_reg_access
 */
static int svpwm_debugfs_reg_access(struct iio_dev *indio_dev,
				    unsigned int reg, unsigned int writeval,
				    unsigned int *readval)
{
	struct private_data *svpwm_data = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;

	if (reg == 1) {
		svpwm_read_reg(svpwm_data, STT_CNT_ITER_DATA, readval);
		dev_info(dev, "Value of debug register count_iter with offset 0x%x is %d\n",
			 STT_CNT_ITER_DATA, *readval);
	} else if (!readval) {
		svpwm_write_reg(svpwm_data, reg, writeval);
	} else {
		svpwm_read_reg(svpwm_data, reg, readval);
	}
	return 0;
}

static ssize_t svpwm_ctrl_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct private_data *st = iio_priv(indio_dev);
	unsigned int val;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case DC_LINK_REF_DATA:
		svpwm_read_reg(st, DC_LINK_REF_DATA, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case DC_SRC_MODE_DATA:
		svpwm_read_reg(st, DC_SRC_MODE_DATA, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case SAMPLE_II_DATA:
		svpwm_read_reg(st, SAMPLE_II_DATA, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case AP_CTRL:
		svpwm_read_reg(st, AP_CTRL, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t svpwm_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct private_data *st = iio_priv(indio_dev);
	unsigned long long data_in;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case DC_LINK_REF_DATA:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			svpwm_write_reg(st, DC_LINK_REF_DATA, data_in);
		else
			return ret;
		break;
	case DC_SRC_MODE_DATA:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			svpwm_write_reg(st, DC_SRC_MODE_DATA, data_in);
		else
			return ret;
		break;
	case SAMPLE_II_DATA:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			svpwm_write_reg(st, SAMPLE_II_DATA, data_in);
		else
			return ret;
		break;
	case AP_CTRL:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			svpwm_write_reg(st, AP_CTRL, data_in);
		else
			return ret;
		break;
	default:
		return -EINVAL;
	}
return len;
}

static IIO_DEVICE_ATTR(dc_link_ref_voltage, 0644,
		       svpwm_ctrl_show,
		       svpwm_ctrl_store,
		       DC_LINK_REF_DATA);

static IIO_DEVICE_ATTR(dc_src_mode, 0644,
		       svpwm_ctrl_show,
		       svpwm_ctrl_store,
		       DC_SRC_MODE_DATA);

static IIO_DEVICE_ATTR(sample_ii, 0644,
		       svpwm_ctrl_show,
		       svpwm_ctrl_store,
		       SAMPLE_II_DATA);

static IIO_DEVICE_ATTR(ap_ctrl, 0644,
		       svpwm_ctrl_show,
		       svpwm_ctrl_store,
		       AP_CTRL);

static struct attribute *svpwm_attributes[] = {
	&iio_dev_attr_dc_link_ref_voltage.dev_attr.attr,
	&iio_dev_attr_dc_src_mode.dev_attr.attr,
	&iio_dev_attr_sample_ii.dev_attr.attr,
	&iio_dev_attr_ap_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group svpwm_attribute_group = {
	.attrs = svpwm_attributes,
};

static const struct iio_info svpwm_info = {
	.debugfs_reg_access = svpwm_debugfs_reg_access,
	.read_raw = &svpmw_read_attribute,
	.attrs = &svpwm_attribute_group,
};

static int svpwm_fill_buffer(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);
	/* data buffer for channel data and timestamp */
	unsigned int data[3 + sizeof(s64) / sizeof(int)] = {0};
	unsigned int val, chan = 0, i = 0;
	s64 time;

	time = iio_get_time_ns(indio_dev);
	/*
	 * Read data for active channels from device
	 */
	for_each_set_bit(chan, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		svpwm_read_reg(st, VA_CMD_DATA + chan * 0x10, &val);
		data[i] = val;
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data, time);

	return 0;
}

static int svpwm_capture_thread(void *data)
{
	struct iio_dev *indio_dev = data;

	do {
		/* Read the data from sensor and push it to buffers */
		int ret = svpwm_fill_buffer(indio_dev);

		if (ret < 0)
			return ret;
		usleep_range(MIN_DELAY_US, MAX_DELAY_US);
	} while (!kthread_should_stop());

	return 0;
}

static int svpwm_buffer_enable(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);
	struct task_struct *task;

	task = kthread_run(svpwm_capture_thread, (void *)indio_dev,
			      "%s", indio_dev->name);

	if (IS_ERR(task))
		return PTR_ERR(task);

	get_task_struct(task);
	wake_up_process(task);
	st->task = task;

	return 0;
}

static int svpwm_buffer_disable(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);

	if (st->task) {
		kthread_stop(st->task);
		put_task_struct(st->task);
		st->task = NULL;
	}
	return 0;
}

static const struct iio_buffer_setup_ops svpwm_setup_ops = {
	.postenable = &svpwm_buffer_enable,
	.predisable = &svpwm_buffer_disable,
};

static const struct of_device_id svpwm_ids[] = {
	{ .compatible = "xlnx,hls-svpwm-duty-1.0", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, svpwm_ids);

static int svpwm_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct private_data *data;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(svpwm_ids, pdev->dev.of_node);

	if (!id)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));

	if (!indio_dev) {
		dev_err(&pdev->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &svpwm_info;
	indio_dev->name = KBUILD_MODNAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = volt_cmd_channels;
	indio_dev->num_channels = ARRAY_SIZE(volt_cmd_channels);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	svpwm_write_reg(data, DC_LINK_REF_DATA, DEFAULT_REF);
	svpwm_write_reg(data, DC_SRC_MODE_DATA, DEFAULT_MODE);
	svpwm_write_reg(data, SAMPLE_II_DATA, DEFAULT_II);
	svpwm_write_reg(data, AP_CTRL, DEFAULT_AP_CTRL);
	ret = devm_iio_kfifo_buffer_setup(&indio_dev->dev, indio_dev,
					  &svpwm_setup_ops);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Successfully registered device SVPWM DUTY ");
	platform_set_drvdata(pdev, indio_dev);

	return 0;
}

static int svpwm_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	return 0;
}

static struct platform_driver svpwm_drv = {
	.probe = svpwm_probe,
	.remove = svpwm_remove,
	.driver = {
		.name = "hls-svpwm-duty",
		.of_match_table = of_match_ptr(svpwm_ids),
	},
};
module_platform_driver(svpwm_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vivekananda Dayananda <vivekananda.dayananda@amd.com>");
