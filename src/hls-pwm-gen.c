// SPDX-License-Identifier: GPL-2.0
/*
 * PWM GEN driver
 *
 * Copyright (C) 2023, Advanced Micro Devices, Inc.
 *
 * Description:
 * This driver is developed for Vitis xf_motorcontrol HLS library, IP PWM_GEN.
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
#define AP_CTRL	0x00
#define PWM_FREQ	0x10
#define DEAD_CYCLES	0x18
#define PHASE_SHIFT	0x20
#define PWM_CYCLE	0x28	// debug reg
#define SAMPLE_II	0x38
#define DUTY_RATIO_A	0x40
#define DUTY_RATIO_B	0x50
#define DUTY_RATIO_C	0x60
/* defaults */
#define DEFAULT_PWM_FREQ	0x4E20
#define DEFAULT_DEAD_CYCLES	0xA
#define DEFAULT_PHASE_SHIFT	0x0
#define DEFAULT_SAMPLE_II	0x1
#define DEFAULT_AP_CTRL	0x1
#define MIN_DELAY_US	1000
#define MAX_DELAY_US	1200
/* channel spec */
#define VOLTAGE_CHANNEL(num, _ext)				\
	{							\
	    .type = IIO_VOLTAGE,				\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = DUTY_RATIO_A + (num * 0x10),		\
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

static const struct iio_chan_spec volt_duty_ratio_channels[] = {
	VOLTAGE_CHANNEL(0, "Va_duty_ratio"),
	VOLTAGE_CHANNEL(1, "Vb_duty_ratio"),
	VOLTAGE_CHANNEL(2, "Vc_duty_ratio"),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static inline void pwmgen_read_reg(struct private_data *st, unsigned int offset,
				   u32 *data)
{
	*data = readl(st->base + offset);
}

static inline void pwmgen_write_reg(struct private_data *st, unsigned int offset,
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
		pwmgen_read_reg(st, channels->address, val);
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 0;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

/**
 * pwmgen_debugfs_reg_access - read or write register value
 * @indio_dev: IIO device structure
 * @reg: register offset
 * @writeval: value to write
 * @readval: value to read
 *
 * To read a value from the register:
 *   echo [pwmgen reg offset] > direct_reg_access
 *   cat direct_reg_access
 *
 * To write a value in a pwmgen register:
 *   echo [pwmgen_reg_offset] [value] > direct_reg_access
 */
static int pwmgen_debugfs_reg_access(struct iio_dev *indio_dev,
				     unsigned int reg, unsigned int writeval,
				     unsigned int *readval)
{
	struct private_data *pwmgen_data = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;

	if (reg == 1) {
		pwmgen_read_reg(pwmgen_data, PWM_CYCLE, readval);
		dev_info(dev, "Value of debug register pwm_cycle with offset 0x%x is %d\n",
			 PWM_CYCLE, *readval);
	} else if (!readval) {
		pwmgen_write_reg(pwmgen_data, reg, writeval);
	} else {
		pwmgen_read_reg(pwmgen_data, reg, readval);
	}
	return 0;
}

static ssize_t pwmgen_ctrl_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct private_data *st = iio_priv(indio_dev);
	unsigned int val;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case PWM_FREQ:
		pwmgen_read_reg(st, PWM_FREQ, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case DEAD_CYCLES:
		pwmgen_read_reg(st, DEAD_CYCLES, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case PHASE_SHIFT:
		pwmgen_read_reg(st, PHASE_SHIFT, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case SAMPLE_II:
		pwmgen_read_reg(st, SAMPLE_II, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case AP_CTRL:
		pwmgen_read_reg(st, AP_CTRL, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t pwmgen_ctrl_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct private_data *st = iio_priv(indio_dev);
	unsigned long long data_in;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case PWM_FREQ:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			pwmgen_write_reg(st, PWM_FREQ, data_in);
		else
			return ret;
		break;
	case DEAD_CYCLES:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			pwmgen_write_reg(st, DEAD_CYCLES, data_in);
		else
			return ret;
		break;
	case PHASE_SHIFT:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			pwmgen_write_reg(st, PHASE_SHIFT, data_in);
		else
			return ret;
		break;
	case SAMPLE_II:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			pwmgen_write_reg(st, SAMPLE_II, data_in);
		else
			return ret;
		break;
	case AP_CTRL:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			pwmgen_write_reg(st, AP_CTRL, data_in);
		else
			return ret;
		break;
	default:
		return -EINVAL;
	}
return len;
}

static IIO_DEVICE_ATTR(pwm_freq, 0644,
		       pwmgen_ctrl_show,
		       pwmgen_ctrl_store,
		       PWM_FREQ);

static IIO_DEVICE_ATTR(dead_cycles, 0644,
		       pwmgen_ctrl_show,
		       pwmgen_ctrl_store,
		       DEAD_CYCLES);

static IIO_DEVICE_ATTR(phase_shift, 0644,
		       pwmgen_ctrl_show,
		       pwmgen_ctrl_store,
		       PHASE_SHIFT);

static IIO_DEVICE_ATTR(sample_ii, 0644,
		       pwmgen_ctrl_show,
		       pwmgen_ctrl_store,
		       SAMPLE_II);

static IIO_DEVICE_ATTR(ap_ctrl, 0644,
		       pwmgen_ctrl_show,
		       pwmgen_ctrl_store,
		       AP_CTRL);

static struct attribute *pwmgen_attributes[] = {
	&iio_dev_attr_phase_shift.dev_attr.attr,
	&iio_dev_attr_dead_cycles.dev_attr.attr,
	&iio_dev_attr_sample_ii.dev_attr.attr,
	&iio_dev_attr_pwm_freq.dev_attr.attr,
	&iio_dev_attr_ap_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group pwmgen_attribute_group = {
	.attrs = pwmgen_attributes,
};

static const struct iio_info pwmgen_info = {
	.debugfs_reg_access = pwmgen_debugfs_reg_access,
	.read_raw = &svpmw_read_attribute,
	.attrs = &pwmgen_attribute_group,
};

static int pwmgen_fill_buffer(struct iio_dev *indio_dev)
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
		pwmgen_read_reg(st, DUTY_RATIO_A + chan * 0x10, &val);
		data[i] = val;
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data, time);

	return 0;
}

static int pwmgen_capture_thread(void *data)
{
	struct iio_dev *indio_dev = data;

	do {
		/* Read the data from sensor and push it to buffers */
		int ret = pwmgen_fill_buffer(indio_dev);

		if (ret < 0)
			return ret;
		usleep_range(MIN_DELAY_US, MAX_DELAY_US);
	} while (!kthread_should_stop());

	return 0;
}

static int pwmgen_buffer_enable(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);
	struct task_struct *task;

	task = kthread_create(pwmgen_capture_thread, (void *)indio_dev,
			      "%s", indio_dev->name);

	if (IS_ERR(task))
		return PTR_ERR(task);

	get_task_struct(task);
	wake_up_process(task);
	st->task = task;

	return 0;
}

static int pwmgen_buffer_disable(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);

	if (st->task) {
		kthread_stop(st->task);
		put_task_struct(st->task);
		st->task = NULL;
	}
	return 0;
}

static const struct iio_buffer_setup_ops pwmgen_setup_ops = {
	.postenable = &pwmgen_buffer_enable,
	.predisable = &pwmgen_buffer_disable,
};

static const struct of_device_id pwmgen_ids[] = {
	{ .compatible = "xlnx,hls-pwm-gen-1.0", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, pwmgen_ids);

static int pwmgen_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct private_data *data;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(pwmgen_ids, pdev->dev.of_node);

	if (!id)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));

	if (!indio_dev) {
		dev_err(&pdev->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &pwmgen_info;
	indio_dev->name = KBUILD_MODNAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = volt_duty_ratio_channels;
	indio_dev->num_channels = ARRAY_SIZE(volt_duty_ratio_channels);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	pwmgen_write_reg(data, PWM_FREQ, DEFAULT_PWM_FREQ);
	pwmgen_write_reg(data, DEAD_CYCLES, DEFAULT_DEAD_CYCLES);
	pwmgen_write_reg(data, PHASE_SHIFT, DEFAULT_PHASE_SHIFT);
	pwmgen_write_reg(data, SAMPLE_II, DEFAULT_SAMPLE_II);
	pwmgen_write_reg(data, AP_CTRL, DEFAULT_AP_CTRL);
	ret = devm_iio_kfifo_buffer_setup(&indio_dev->dev, indio_dev,
					  INDIO_BUFFER_SOFTWARE,
					  &pwmgen_setup_ops);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Successfully registered device PWM-GEN encoder");
	platform_set_drvdata(pdev, indio_dev);

	return 0;
}

static int pwmgen_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	return 0;
}

static struct platform_driver pwmgen_drv = {
	.probe = pwmgen_probe,
	.remove = pwmgen_remove,
	.driver = {
		.name = "hls-pwm-gen",
		.of_match_table = of_match_ptr(pwmgen_ids),
	},
};
module_platform_driver(pwmgen_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vivekananda Dayananda <vivekananda.dayananda@amd.com>");
