// SPDX-License-Identifier: GPL-2.0
/*
 * ADC-HUB driver
 *
 * Copyright (C) 2022 - 2023 Advance Micro Devices, Inc.
 *
 * Description:
 * The driver is developed for ADC-HUB vivado IP. The driver operates in INDIO Mode.
 * The driver supports the following features
 * - Multi voltage and current channel monitoring via IIO sysfs interface,
 * - Event monitoring for over and under range faults,
 * - IRQ support for clearing and registering faults,
 * - Kfifo buffers per channel for continuous data samples,
 * - Custom attributes for channel calibration, scaling factor & Averaging data samples
 * Driver supports the current max HW capability of 16 ADC channels for continuous monitoring
 */

#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sched/task.h>

/* Register offsets per channel */
#define OVERLIMIT_VALUE 0x00
#define UNDERLIMIT_VALUE 0x04
#define SCALE 0x08
#define TAP 0x0C
#define RAWDATA 0x10
#define QDATA 0x14
#define ERROR_STATUS 0x18
#define CLEAR_ERROR_OVERLIMIT 0x1C
#define CLEAR_ERROR_UNDERLIMIT 0x20
#define OVERLIMIT_EN 0x24
#define UNDERLIMIT_EN 0x28
#define INTERRUPT 0x38
#define CHANNEL_OFFSET_VALUE 0x3C
#define SAMPLE_INTERVAL	0xD0

/* generic macros */
#define DEFAULT_VOLTAGE_SCALE_FACTOR 1233
#define DEFAULT_CURRENT_SCALE_FACTOR 327
#define MASK_LOWER GENMASK(31, 16)
#define MASK_UPPER GENMASK(15, 0)
#define UNDERLIMIT_MASK_BIT 0x02
#define OVERLIMIT_MASK_BIT 0x01
#define CLEAR_VAL 0x0BAD00FF
#define NUMERICAL_INDEX 0x01
#define DISABLE_FAULT 0x00
#define ENABLE_FAULT 0x01
#define DEFAULT_LIMIT 0x0
#define DISABLE_IRQ 0x01
#define STORAGE_BITS 32
#define ENABLE_IRQ 0x00
#define NUM_SAMPLES 64
#define SIGNEX(v, sb) (((v) & (1 << ((sb) - 1))) ? ((v) | (~((1 << (sb)) - 1))) : (v))
#define _ADC_EXT_INFO(_name, _shared, _ident)                                  \
	{                                                                      \
		.name = _name, .read = read_hub_ext_attribute,                 \
		.write = write_hub_ext_attribute, .shared = _shared,           \
		.private = _ident,                                             \
	}

static ssize_t read_hub_ext_attribute(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf);

static ssize_t write_hub_ext_attribute(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len);
enum {
	FILTER_TAP,
	UNDER_RANGE_STATUS,
	OVER_RANGE_STATUS,
	FAULT_CLEAR,
	CALIBERATE
};

static const struct iio_event_spec hub_channel_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate =
			BIT(IIO_EV_INFO_ENABLE) | BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate =
			BIT(IIO_EV_INFO_ENABLE) | BIT(IIO_EV_INFO_VALUE),
	},
};

struct hubdata {
	void __iomem *base;
	const struct iio_chan_spec *hub_channels;
	unsigned int irq;
	u32 interval;
	struct task_struct *task;
};

static const struct iio_chan_spec_ext_info adc_ext_info[] = {
	_ADC_EXT_INFO("set_filter_tap", IIO_SEPARATE, FILTER_TAP),
	_ADC_EXT_INFO("under_range_fault_status", IIO_SEPARATE,
		      UNDER_RANGE_STATUS),
	_ADC_EXT_INFO("over_range_fault_status", IIO_SEPARATE,
		      OVER_RANGE_STATUS),
	_ADC_EXT_INFO("fault_clear", IIO_SEPARATE, FAULT_CLEAR),
	_ADC_EXT_INFO("calibrate", IIO_SEPARATE, CALIBERATE),
	{},
};

static inline void read_reg(struct hubdata *st, unsigned int offset, u32 *data)
{
	*data = readl(st->base + offset);
}

static inline void write_reg(struct hubdata *st, unsigned int offset, u32 data)
{
	writel(data, st->base + offset);
}

/* Read callback implementation for the external info attribute */
static ssize_t read_hub_ext_attribute(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *channels,
				      char *buf)
{
	struct hubdata *st = iio_priv(indio_dev);
	int val;

	switch ((u32)private) {
	case FILTER_TAP:
		read_reg(st, channels->address + TAP, &val);
		break;

	case UNDER_RANGE_STATUS:
		read_reg(st, channels->address + ERROR_STATUS, &val);
		val &= UNDERLIMIT_MASK_BIT;
		if (val == 2)
			val = 1;
		break;

	case OVER_RANGE_STATUS:
		read_reg(st, channels->address + ERROR_STATUS, &val);
		val &= OVERLIMIT_MASK_BIT;
		break;

	case FAULT_CLEAR:
		return -EINVAL;

	case CALIBERATE:
		return -EINVAL;

	default:
		return -EINVAL;
	}
	return sysfs_emit(buf, "%d\n", val);
}

/* channel calibration implementation for the device attribute */
static int calibrate_channel(struct hubdata *st,
			     const struct iio_chan_spec *channels,
			     struct iio_dev *indio_dev)
{
	int offsetvalue = 0, val = 0, i, avg;

	for (i = 0; i < NUM_SAMPLES; i++) {
		read_reg(st, channels->address + RAWDATA, &val);
		val = SIGNEX(val, 12);
		offsetvalue += val;
	}
	avg = offsetvalue >> 6;
	avg = ((avg) & (1 << 11)) ? (avg & 0x00000FFF) : avg;
	dev_info(&indio_dev->dev, "Offset Calibration Complete\n");
	return avg;
}

/* Write callback implementation for the external info attribute */
static ssize_t write_hub_ext_attribute(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *channels,
				       const char *buf, size_t len)
{
	struct hubdata *st = iio_priv(indio_dev);
	unsigned long buffer_data;
	bool tapvalid;
	int ret;

	switch ((u32)private) {
	case FILTER_TAP:
		ret = kstrtoul(buf, 10, &buffer_data);
		if (!ret) {
			int i, tsize, valid_taps[] = {1, 2, 4, 8, 16, 32, 64, 128};

			tsize = ARRAY_SIZE(valid_taps);
			tapvalid = false;

			for (i = 0; i < tsize; i++) {
				if (buffer_data == valid_taps[i]) {
					tapvalid = true;
					break;
				}
			}
			if (tapvalid) {
				write_reg(st, channels->address + TAP, buffer_data);
			} else {
				dev_info(&indio_dev->dev,
					 "%lu is an invalid option, choose a valid power of two in 1:128\n",
					 buffer_data);
				return -EINVAL;
			}
		} else {
			return ret;
		}
		break;
	case FAULT_CLEAR:
		ret = kstrtoul(buf, 10, &buffer_data);
		if (ret) {
			return ret;
		} else if (buffer_data == 1) {
			write_reg(st, channels->address + CLEAR_ERROR_OVERLIMIT,
				  CLEAR_VAL);
			write_reg(st,
				  channels->address + CLEAR_ERROR_UNDERLIMIT,
				  CLEAR_VAL);
		}
		break;
	case CALIBERATE:
		ret = kstrtoul(buf, 10, &buffer_data);
		if (ret) {
			return ret;
		} else if (buffer_data == 1) {
			int offset;

			offset = calibrate_channel(st, channels, indio_dev);
			write_reg(st, channels->address + CHANNEL_OFFSET_VALUE,
				  offset);
		}
		break;
	default:
		return -EINVAL;
	}
	return len;
}

/* computing Q value */
static u32 find_qvalue(int val, int val2)
{
	long long magnitude, fraction;
	u64 qval, modnum;

	if (val >= 0 && ((u64)val2 < 99999999)) {
		qval = (((u64)val * 1000000 + (u64)val2) * 65536) / 100000;
		/* rounding up */
		modnum = qval % 10;
		if ((qval % 10) >= 5)
			qval += (10 - modnum);
		qval /= 10;
	} else if (val == 0) {
		magnitude = (65535) << 16;
		fraction = ((long long)val2 * 65535) / 1000000;
		qval = magnitude | fraction;
	} else {
		magnitude = (65535 + (long long)val) << 16;
		fraction = ((1000000 - (long long)val2) * 65535) / 1000000;
		qval = magnitude | fraction;
	}
	return (u32)qval;
}

/* Write callback implementation for the IIO info attribute */
static int hub_write_attribute(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *channels, int val,
			       int val2, long mask)
{
	struct hubdata *st = iio_priv(indio_dev);
	u32 qval;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		qval = find_qvalue(val, val2);
		write_reg(st, channels->address + SCALE, qval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* Read callback implementation for the IIO info attribute */
static int hub_read_attribute(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channels, int *val,
			      int *val2, long mask)
{
	struct hubdata *st = iio_priv(indio_dev);
	u32 offset_val, scale_val;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		read_reg(st, channels->address + RAWDATA, val);
		*val2 = 0;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		read_reg(st, channels->address + SCALE, val);
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_PROCESSED:
		read_reg(st, channels->address + QDATA, val);
		*val = (u32)(*val);
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_OFFSET:
		read_reg(st, channels->address + CHANNEL_OFFSET_VALUE,
			 &offset_val);
		read_reg(st, channels->address + SCALE, &scale_val);
		*val = (offset_val & (1 << 11)) ? ((4096 - offset_val) * scale_val) : (offset_val *
			scale_val);
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

/**
 * hub_read_event_config() - is event enabled?
 * @indio_dev: the device instance data
 * @channels: channel for the event whose state is being queried
 * @type: type of the event whose state is being queried
 * @dir: direction of the event whose state is being queried
 *
 * This function would normally query the relevant registers or a cache to
 * discover if the event generation is enabled on the device.
 */
static int hub_read_event_config(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *channels,
				 enum iio_event_type type,
				 enum iio_event_direction dir)
{
	struct hubdata *st = iio_priv(indio_dev);
	int value;

	if (dir == IIO_EV_DIR_RISING)
		read_reg(st, channels->address + OVERLIMIT_EN, &value);
	else if (dir == IIO_EV_DIR_FALLING)
		read_reg(st, channels->address + UNDERLIMIT_EN, &value);
	else
		return -EINVAL;

	return value;
}

/**
 * hub_write_event_config() - set whether event is enabled
 * @indio_dev: the device instance data
 * @channels: channel for the event whose state is being set
 * @type: type of the event whose state is being set
 * @dir: direction of the vent whose state is being set
 * @state: whether to enable or disable the device.
 *
 * This function would normally set the relevant registers on the devices
 * so that it generates the specified event. Here it just sets up a cached
 * value.
 */
static int hub_write_event_config(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *channels,
				  enum iio_event_type type,
				  enum iio_event_direction dir, int state)
{
	struct hubdata *st = iio_priv(indio_dev);

	switch (channels->type) {
	case IIO_VOLTAGE:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			if (dir == IIO_EV_DIR_RISING) {
				write_reg(st, channels->address + OVERLIMIT_EN,
					  state);
			} else if (dir == IIO_EV_DIR_FALLING) {
				write_reg(st, channels->address + UNDERLIMIT_EN,
					  state);
			}
			if (state)
				write_reg(st, channels->address + INTERRUPT, ENABLE_IRQ);
			break;
		default:
			return -EINVAL;
		}
		break;

	case IIO_CURRENT:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			if (dir == IIO_EV_DIR_RISING) {
				write_reg(st, channels->address + OVERLIMIT_EN,
					  state);
			} else if (dir == IIO_EV_DIR_FALLING) {
				write_reg(st, channels->address + UNDERLIMIT_EN,
					  state);
			}
			if (state)
				write_reg(st, channels->address + INTERRUPT, ENABLE_IRQ);
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hub_read_event_value() - get value associated with event
 * @indio_dev: device instance specific data
 * @channels: channel for the event whose value is being read
 * @type: type of the event whose value is being read
 * @dir: direction of the vent whose value is being read
 * @info: info type of the event whose value is being read
 * @val: value for the event code.
 * @val2: represent fraction value.
 *
 */
static int hub_read_event_value(struct iio_dev *indio_dev,
				const struct iio_chan_spec *channels,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info, int *val, int *val2)
{
	struct hubdata *st = iio_priv(indio_dev);
	*val2 = 16;

	if (dir == IIO_EV_DIR_RISING)
		read_reg(st, channels->address + OVERLIMIT_VALUE, val);
	else if (dir == IIO_EV_DIR_FALLING)
		read_reg(st, channels->address + UNDERLIMIT_VALUE, val);
	else
		return -EINVAL;

	return IIO_VAL_FRACTIONAL_LOG2;
}

/**
 * hub_write_event_value() - set value associate with event
 * @indio_dev: device instance specific data
 * @channels: channel for the event whose value is being set
 * @type: type of the event whose value is being set
 * @dir: direction of the vent whose value is being set
 * @info: info type of the event whose value is being set
 * @val: integer value to be set.
 * @val2: fraction value to be set.
 */
static int hub_write_event_value(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *channels,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 enum iio_event_info info, int val, int val2)
{
	struct hubdata *st = iio_priv(indio_dev);
	u32 qval;

	switch (channels->type) {
	case IIO_VOLTAGE:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			if (dir == IIO_EV_DIR_RISING) {
				qval = find_qvalue(val, val2);
				write_reg(st,
					  channels->address + OVERLIMIT_VALUE,
					  qval);
			} else if (dir == IIO_EV_DIR_FALLING) {
				qval = find_qvalue(val, val2);
				write_reg(st,
					  channels->address + UNDERLIMIT_VALUE,
					  qval);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CURRENT:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			if (dir == IIO_EV_DIR_RISING) {
				qval = find_qvalue(val, val2);
				write_reg(st,
					  channels->address + OVERLIMIT_VALUE,
					  qval);
			} else if (dir == IIO_EV_DIR_FALLING) {
				qval = find_qvalue(val, val2);
				write_reg(st,
					  channels->address + UNDERLIMIT_VALUE,
					  qval);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * hub_event_handler() - identify and pass on event
 * @irq: irq of event line
 * @private: pointer to device instance state.
 *
 * This handler is responsible for querying the device to find out what
 * event occurred and for then pushing that event towards user space.
 */
static irqreturn_t hub_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct hubdata *st = iio_priv(indio_dev);
	unsigned int chan, fault_value, chan_irq;

	st->hub_channels = indio_dev->channels;

	/* find fault occurring channel */
	for (chan = 0; chan < indio_dev->num_channels - 1; chan++) {
		read_reg(st, ERROR_STATUS + chan * 0x100, &fault_value);
		read_reg(st, INTERRUPT + chan * 0x100, &chan_irq);
		if (fault_value > 0 && !chan_irq)
			break;
	}
	write_reg(st, INTERRUPT + chan * 0x100, DISABLE_IRQ);

	switch (fault_value) {
	case 2:
		if (st->hub_channels[chan].type) {
			iio_push_event(indio_dev,
				       IIO_EVENT_CODE(IIO_CURRENT, 0, 0,
						      IIO_EV_DIR_FALLING,
						      IIO_EV_TYPE_THRESH, chan,
						      0, 0),
				       iio_get_time_ns(indio_dev));
			write_reg(st, UNDERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			dev_info(&indio_dev->dev,
				 "channel %d under_current irq event occurred\n",
				 chan);
		} else {
			iio_push_event(indio_dev,
				       IIO_EVENT_CODE(IIO_VOLTAGE, 0, 0,
						      IIO_EV_DIR_FALLING,
						      IIO_EV_TYPE_THRESH, chan,
						      0, 0),
				       iio_get_time_ns(indio_dev));
			write_reg(st, UNDERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			dev_info(&indio_dev->dev,
				 "channel %d under_voltage irq event occurred\n",
				 chan);
		}
		break;
	case 1:
		if (st->hub_channels[chan].type) {
			iio_push_event(indio_dev,
				       IIO_EVENT_CODE(IIO_CURRENT, 0, 0,
						      IIO_EV_DIR_RISING,
						      IIO_EV_TYPE_THRESH, chan,
						      0, 0),
				       iio_get_time_ns(indio_dev));
			write_reg(st, OVERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			dev_info(&indio_dev->dev,
				 "channel %d over_current irq event occurred\n",
				 chan);
		} else {
			iio_push_event(indio_dev,
				       IIO_EVENT_CODE(IIO_VOLTAGE, 0, 0,
						      IIO_EV_DIR_RISING,
						      IIO_EV_TYPE_THRESH, chan,
						      0, 0),
				       iio_get_time_ns(indio_dev));
			write_reg(st, OVERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			dev_info(&indio_dev->dev,
				 "channel %d over_voltage irq event occurred\n",
				 chan);
		}
		break;
	case 3:
		if (st->hub_channels[chan].type) {
			iio_push_event(indio_dev,
				       IIO_EVENT_CODE(IIO_CURRENT, 0, 0,
						      IIO_EV_DIR_RISING,
						      IIO_EV_TYPE_THRESH, chan,
						      0, 0),
				       iio_get_time_ns(indio_dev));
			write_reg(st, OVERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			write_reg(st, UNDERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			dev_info(&indio_dev->dev,
				 "channel %d over_current and under_current irq event detected\n",
				 chan);
		} else {
			iio_push_event(indio_dev,
				       IIO_EVENT_CODE(IIO_VOLTAGE, 0, 0,
						      IIO_EV_DIR_RISING,
						      IIO_EV_TYPE_THRESH, chan,
						      0, 0),
				       iio_get_time_ns(indio_dev));
			write_reg(st, OVERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			write_reg(st, UNDERLIMIT_EN + chan * 0x100,
				  DISABLE_FAULT);
			dev_info(&indio_dev->dev,
				 "channel %d over_voltage and under_voltage irq event detected\n",
				 chan);
		}
		break;
	default:
		dev_info(&indio_dev->dev, "channel irq invalid event\n");
		return -EINVAL;
	}
	return IRQ_HANDLED;
}

static const struct of_device_id hub_ids[] = {
	{
		.compatible = "xlnx,adc-hub-1.0",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, hub_ids);

static ssize_t hub_ctrl_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct hubdata *st = iio_priv(indio_dev);
	unsigned int val;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case SAMPLE_INTERVAL:
		val = st->interval;
		ret = sprintf(buf, "%u\n", val);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static ssize_t hub_ctrl_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct hubdata *st = iio_priv(indio_dev);
	unsigned long data_in;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case SAMPLE_INTERVAL:
		ret = kstrtoul(buf, 10, &data_in);
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

static IIO_DEVICE_ATTR(sample_interval_us, 0644,
		       hub_ctrl_show,
		       hub_ctrl_store,
		       SAMPLE_INTERVAL);

static struct attribute *hub_attributes[] = {
		&iio_dev_attr_sample_interval_us.dev_attr.attr,
		NULL,
};

static const struct attribute_group hub_attribute_group = {
		.attrs = hub_attributes,
};

static const struct iio_info hub_info = {
	.write_event_config = &hub_write_event_config,
	.read_event_config = &hub_read_event_config,
	.write_event_value = &hub_write_event_value,
	.read_event_value = &hub_read_event_value,
	.write_raw = &hub_write_attribute,
	.read_raw = &hub_read_attribute,
	.attrs = &hub_attribute_group,
};

/* Kfifo buffer implementation for enabled channels */
static int hub_fill_buffer(struct iio_dev *indio_dev)
{
	/* data buffer for channel data and timestamp */
	unsigned int data[8 + sizeof(s64) / sizeof(int)] = { 0 };
	struct hubdata *st = iio_priv(indio_dev);
	int chan = 0, i = 0;
	s64 time;

	time = iio_get_time_ns(indio_dev);

	/*
	 * Read raw data for current and voltage channels from device
	 */
	for_each_set_bit(chan, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		unsigned int val;

		read_reg(st, QDATA + (0x100 * chan), &val);
		data[i] = val;
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data, time);

	return 0;
}

static int hub_capture_thread(void *data)
{
	struct iio_dev *indio_dev = data;
	struct hubdata *st = iio_priv(indio_dev);
	u32 interval = st->interval;

	do {
		/* Read the data from sensor and push it to buffers */
		int ret;

		ret = hub_fill_buffer(indio_dev);
		if (ret < 0)
			return ret;
		usleep_range(interval, interval + 20);
	} while (!kthread_should_stop());

	return 0;
}

static int hub_buffer_enable(struct iio_dev *indio_dev)
{
	struct hubdata *st = iio_priv(indio_dev);
	struct task_struct *task;

	task = kthread_run(hub_capture_thread, (void *)indio_dev, "%s:%d",
			      indio_dev->name, iio_device_id(indio_dev));

	if (IS_ERR(task))
		return PTR_ERR(task);

	get_task_struct(task);
	wake_up_process(task);
	st->task = task;

	return 0;
}

static int hub_buffer_disable(struct iio_dev *indio_dev)
{
	struct hubdata *st = iio_priv(indio_dev);

	if (st->task) {
		kthread_stop(st->task);
		put_task_struct(st->task);
		st->task = NULL;
	}
	return 0;
}

static const struct iio_buffer_setup_ops hub_setup_ops = {
	.predisable = &hub_buffer_disable,
	.postenable = &hub_buffer_enable,
};

/* Obtain channel configuration from the device tree */
static int
hub_parse_child_nodes(struct iio_dev *indio_dev, struct platform_device *pdev)
{
	struct device_node *child_node = NULL, *np = pdev->dev.of_node;
	struct iio_chan_spec *hub_channels;

	u32 chan_size = sizeof(struct iio_chan_spec);
	const char *voltage_string = "voltage";
	const char *current_string = "current";
	u8 num_supply_chan = 0;
	u32 bits = 32, reg = 0;
	const char *name;
	int ret, i = 0;

	ret = of_property_read_u8(np, "xlnx,numchannels", &num_supply_chan);
	if (ret < 0)
		return ret;
	hub_channels = devm_kzalloc(&pdev->dev,
				(chan_size * (num_supply_chan + 1)), GFP_KERNEL);

	for_each_child_of_node(np, child_node) {
		int index = num_supply_chan - 1 - i;
		/*
		 * Child nodes are stacked similar to LIFO,
		 * hence populating hub channel array from last element
		 */

		ret = of_property_read_u32(child_node, "reg", &reg);
		if (!(ret < 0)) {
			hub_channels[index].address = reg * 0x100;
			hub_channels[index].channel = reg;
		} else {
			of_node_put(child_node);
			return ret;
		}

		ret = of_property_read_string(child_node, "xlnx,type", &name);
		if (ret < 0) {
			of_node_put(child_node);
			return ret;
		} else if (strcmp(name, voltage_string) == 0) {
			hub_channels[index].type = IIO_VOLTAGE;
		} else if (strcmp(name, current_string) == 0) {
			hub_channels[index].type = IIO_CURRENT;
		} else {
			of_node_put(child_node);
			return ret;
		}

		if (of_property_read_bool(child_node, "xlnx,bipolar"))
			hub_channels[index].scan_type.sign = 's';
		else
			hub_channels[index].scan_type.sign = 'u';

		hub_channels[index].scan_index = index;
		hub_channels[index].ext_info = adc_ext_info;
		hub_channels[index].indexed = NUMERICAL_INDEX;
		hub_channels[index].event_spec = hub_channel_events;
		hub_channels[index].scan_type.realbits = bits;
		hub_channels[index].scan_type.storagebits = STORAGE_BITS;
		hub_channels[index].num_event_specs =
			ARRAY_SIZE(hub_channel_events);
		hub_channels[index].info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET);
		i++;
	}

	// soft time-stamp as last channel
	hub_channels[num_supply_chan].type = IIO_TIMESTAMP;
	hub_channels[num_supply_chan].channel = -1;
	hub_channels[num_supply_chan].scan_index = num_supply_chan;
	hub_channels[num_supply_chan].scan_type.sign = 's';
	hub_channels[num_supply_chan].scan_type.realbits = 64;
	hub_channels[num_supply_chan].scan_type.storagebits = 64;

	indio_dev->num_channels = num_supply_chan + 1;
	indio_dev->channels = hub_channels;

	return 0;
}

/* initializing HW to a clean state */
static int hub_device_init(struct iio_dev *hubdevice)
{
	struct hubdata *st;
	int chan;

	st = iio_priv(hubdevice);

	for (chan = 0; chan < hubdevice->num_channels - 1; chan++) {
		/* disable faults and irq */
		write_reg(st, INTERRUPT + chan * 0x100, DISABLE_IRQ);
		write_reg(st, OVERLIMIT_EN + chan * 0x100, DISABLE_FAULT);
		write_reg(st, UNDERLIMIT_EN + chan * 0x100, DISABLE_FAULT);
		/* clear errors */
		write_reg(st, CLEAR_ERROR_OVERLIMIT + chan * 0x100, CLEAR_VAL);
		write_reg(st, CLEAR_ERROR_UNDERLIMIT + chan * 0x100, CLEAR_VAL);
		/* set vlotage limits and scale value */
		write_reg(st, OVERLIMIT_VALUE + chan * 0x100, DEFAULT_LIMIT);
		write_reg(st, UNDERLIMIT_VALUE + chan * 0x100, DEFAULT_LIMIT);
		if (hubdevice->channels[chan].type == IIO_CURRENT)
			write_reg(st, SCALE + chan * 0x100,
				  DEFAULT_CURRENT_SCALE_FACTOR);
		else
			write_reg(st, SCALE + chan * 0x100,
				  DEFAULT_VOLTAGE_SCALE_FACTOR);
	}
	return 0;
}

/* device probe and registration */
static int adc_hub_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct resource *res;
	struct hubdata *st;
	int ret, chan;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(hub_ids, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&pdev->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = KBUILD_MODNAME;
	indio_dev->info = &hub_info;
	st = iio_priv(indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->base = devm_ioremap_resource(&pdev->dev, res);
	st->interval = 200;
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);

	ret = hub_parse_child_nodes(indio_dev, pdev);
	if (ret)
		return ret;

	ret = devm_iio_kfifo_buffer_setup(&indio_dev->dev, indio_dev,
					  &hub_setup_ops);
	if (ret)
		return ret;

	ret = hub_device_init(indio_dev);
	if (ret)
		return ret;

	st->irq = platform_get_irq_byname(pdev, "adc_hub-irq");
	ret = devm_request_irq(&pdev->dev, st->irq, &hub_event_handler, 0,
			       "adc_hub-irq", indio_dev);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to register interrupt\n");

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Successfully registered ADC-HUB channels");
	platform_set_drvdata(pdev, indio_dev);

	for (chan = 0; chan < indio_dev->num_channels - 1; chan++)
		write_reg(st, INTERRUPT + chan * 0x100, ENABLE_IRQ);

	return 0;
}

static int adc_hub_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver adc_hub = {
	.probe = adc_hub_probe,
	.remove = adc_hub_remove,
	.driver = {
		.name = "adc_hub",
		.of_match_table = of_match_ptr(hub_ids),
	},
};
module_platform_driver(adc_hub);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vivekananda Dayananda <vivekananda.dayananda@amd.com>");
