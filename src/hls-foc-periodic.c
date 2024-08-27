// SPDX-License-Identifier: GPL-2.0
/*
 * FOC PERIODIC driver
 *
 * Copyright (C) 2023, Advanced Micro Devices, Inc.
 *
 * Description:
 * The driver is developed for HLS Field oriented Control IP. The driver supports INDIO Mode.
 * Device configurations and status are handled via IIO sysfs interface, buffer capable.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/kthread.h>
#include <linux/sched/task.h>
#include <linux/delay.h>
#include <linux/ctype.h>

/* register space */
#define AP_CTRL		0x00
#define PPR_ARGS		0x010
#define CONTROL_MODE_ARGS	0x018	// MOD_STOPPED
#define CONTROL_FIXPERIOD_ARGS	0x020	// debug enum // fixed_speed // top_temp
#define FLUX_SP_ARGS		0x028
#define FLUX_KP_ARGS		0x030
#define FLUX_KI_ARGS		0x038
#define FLUX_KD_ARGS		0x040
#define TORQUE_SP_ARGS		0x048
#define TORQUE_KP_ARGS		0x050
#define TORQUE_KI_ARGS		0x058
#define TORQUE_KD_ARGS		0x060
#define SPEED_SP_ARGS		0x068
#define SPEED_KP_ARGS		0x070
#define SPEED_KI_ARGS		0x078
#define SPEED_KD_ARGS		0x080
#define ANGLE_SH_ARGS		0x088
#define VD_ARGS			0x090
#define VQ_ARGS			0x098
#define FW_KP_ARGS		0x0a0
#define FW_KI_ARGS		0x0a8
#define ID_STTS			0x0b0	// channel current out
#define FLUX_ACC_STTS		0x0c0	// read
#define FLUX_ERR_STTS		0x0d0	// read
#define FLUX_OUT_STTS		0x0e0	// channel flux IIO_INTENSITY out
#define IQ_STTS			0x0f0	// channel // current out
#define TORQUE_ACC_STTS		0x100	// read
#define TORQUE_ERR_STTS		0x110	// read
#define TORQUE_OUT_STTS		0x120	// channel torq IIO_ROT out
#define SPEED_STTS		0x130	// channel speed IIO_ROT out
#define SPEED_ACC_STTS		0x140	// debug
#define SPEED_ERR_STTS		0x150	// debug
#define SPEED_OUT_STTS		0x160	// channel IIO_ROT out
#define ANGLE_STTS		0x170	// debug channel angle IIO_ANGL out
#define VA_CMD_STTS		0x180	// debug
#define VB_CMD_STTS		0x190	// debug
#define VC_CMD_STTS		0x1a0	// debug
#define IALPHA_STTS		0x1b0	// channel current out
#define IBETA_STTS		0x1c0	// channel current out
#define IHOMOPOLAR_STTS		0x1d0	// channel current  out
#define FIXED_ANGLE_ARGS	0x1e0
#define SAMPLE_TIME		0x200
/* defaults */
#define DEFAULT_AP_CTRL		0x00
#define DEFAULT_PPR_ARGS		0x02
#define DEFAULT_CONTROL_MODE_ARGS	0x00 //MOD_STOPPED
#define DEFAULT_CONTROL_FIXPERIOD_ARGS	0x00 // debug enum // fixed_speed // top_temp
#define DEFAULT_FLUX_SP_ARGS		0x00
#define DEFAULT_FLUX_KP_ARGS		0x00
#define DEFAULT_FLUX_KI_ARGS		0x00
#define DEFAULT_FLUX_KD_ARGS		0x00
#define DEFAULT_TORQUE_SP_ARGS		0x00
#define DEFAULT_TORQUE_KP_ARGS		0x00
#define DEFAULT_TORQUE_KI_ARGS		0x00
#define DEFAULT_TORQUE_KD_ARGS		0x00
#define DEFAULT_SPEED_SP_ARGS		0x00
#define DEFAULT_SPEED_KP_ARGS		0x00
#define DEFAULT_SPEED_KI_ARGS		0x00
#define DEFAULT_SPEED_KD_ARGS		0x00
#define DEFAULT_ANGLE_SH_ARGS		0x00
#define DEFAULT_VD_ARGS			0x00
#define DEFAULT_VQ_ARGS			0x00
#define DEFAULT_FW_KP_ARGS		0x00
#define DEFAULT_FW_KI_ARGS		0x00
#define DEFAULT_FIXED_ANGLE		0x00
#define MAXFRANGE			16
#define MAXIRANGE			5
/* channel spec */
#define FLUX_CHANNEL(num, addr, _ext)				\
	{							\
	    .type = IIO_INTENSITY,				\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = (addr),					\
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

#define CURRENT_CHANNEL(num, addr, _ext)			\
	{							\
	    .type = IIO_CURRENT,				\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = (addr),					\
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

#define ROT_CHANNEL(num, addr, _ext)				\
	{							\
	    .type = IIO_ROT,					\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = (addr),					\
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

#define ANGLE_CHANNEL(num, addr)				\
	{							\
	    .type = IIO_ANGL,					\
	    .indexed = 1,					\
	    .channel = (num),					\
	    .address = (addr),					\
	    .scan_index = (num),				\
	    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				BIT(IIO_CHAN_INFO_SCALE),	\
	    .scan_type = {					\
			.sign = 'u',				\
			.realbits = 32,				\
			.storagebits = 32,			\
			.shift = 0,				\
			},					\
	}							\

struct private_data {
	void __iomem *base;
	u32 *data;
	u32 interval;
	struct task_struct *task;
};

static const struct iio_chan_spec foc_channels[] = {
	CURRENT_CHANNEL(0, ID_STTS, "Id"),
	CURRENT_CHANNEL(1, IQ_STTS, "Iq"),
	CURRENT_CHANNEL(2, IALPHA_STTS, "I_alpha"),
	CURRENT_CHANNEL(3, IBETA_STTS, "I_beta"),
	CURRENT_CHANNEL(4, IHOMOPOLAR_STTS, "I_homopolar"),
	ROT_CHANNEL(5, SPEED_OUT_STTS, "speed_pi_out"),
	ROT_CHANNEL(6, TORQUE_OUT_STTS, "torque_pi_out"),
	FLUX_CHANNEL(7, FLUX_OUT_STTS, "flux"),
	ROT_CHANNEL(8, SPEED_STTS, "speed"),
	ANGLE_CHANNEL(9, ANGLE_STTS),
	IIO_CHAN_SOFT_TIMESTAMP(10),
};

static inline void foc_read_reg(struct private_data *st, unsigned int offset,
				u32 *data)
{
	*data = readl(st->base + offset);
}

static inline void foc_write_reg(struct private_data *st, unsigned int offset,
				 u32 data)
{
	writel(data, st->base + offset);
}

static int foc_read_attribute(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channels,
			      int *val,
			      int *val2,
			      long mask)
{
	struct private_data *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		foc_read_reg(st, channels->address, val);
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
 * foc_debugfs_reg_access - read or write register value
 * @indio_dev: IIO device structure
 * @reg: register offset
 * @writeval: value to write
 * @readval: value to read
 *
 * To read a value from the register:
 *   echo [foc reg offset] > direct_reg_access
 *   cat direct_reg_access
 *
 * To write a value in a foc register:
 *   echo [foc_reg_offset] [value] > direct_reg_access
 */
static int foc_debugfs_reg_access(struct iio_dev *indio_dev,
				  unsigned int reg, unsigned int writeval,
				  unsigned int *readval)
{
	struct private_data *foc_data = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;

	if (reg == 1) {
		foc_read_reg(foc_data, SPEED_ACC_STTS, readval);
		dev_info(dev, "Value of debug register speed_acc with offest 0x%x is %d\n",
			 SPEED_ACC_STTS, *readval);
		foc_read_reg(foc_data, SPEED_ERR_STTS, readval);
		dev_info(dev, "Value of debug register speed_err with offest 0x%x is %d\n",
			 SPEED_ERR_STTS, *readval);
		foc_read_reg(foc_data, VA_CMD_STTS, readval);
		dev_info(dev, "Value of debug register Va_cmd with offest 0x%x is %d\n",
			 VA_CMD_STTS, *readval);
		foc_read_reg(foc_data, VB_CMD_STTS, readval);
		dev_info(dev, "Value of debug register Vb_cmd with offest 0x%x is %d\n",
			 VB_CMD_STTS, *readval);
		foc_read_reg(foc_data, VC_CMD_STTS, readval);
		dev_info(dev, "Value of debug register Vc_cmd with offest 0x%x is %d\n",
			 VC_CMD_STTS, *readval);
	} else if (!readval) {
		foc_write_reg(foc_data, reg, writeval);
	} else {
		foc_read_reg(foc_data, reg, readval);
	}
	return 0;
}

static inline s64 div_s64_srem(s64 dividend, s64 divisor, s64 *remainder)
{
	*remainder = dividend % divisor;
	return dividend / divisor;
}

/*
 * provide data accurate to 16 decimal places
 */
static ssize_t format_value(char *buf, const int *vals)
{
	s64 tmp0, tmp1, tmp2;
	int i;

	tmp0 = vals[0] / 65536;
	tmp2 = (tmp0 != 0) ? (vals[0] - (65536 * tmp0)) : vals[0];
	for (i = 0; i < vals[1]; i++)
		tmp2 = shift_right(tmp2 * 10, 1);

	div_s64_srem(tmp2, 10000000000000000LL, &tmp1);
	if (tmp0 == 0 && tmp2 < 0)
		return sysfs_emit_at(buf, 0, "-0.%016llu", abs(tmp1));
	else
		return sysfs_emit_at(buf, 0, "%lld.%016llu", tmp0, abs(tmp1));
}

static ssize_t kq16tos(char *buf, int *vals)
{
	ssize_t len;

	len = format_value(buf, vals);
	if (len >= PAGE_SIZE - 1)
		return -EFBIG;

	return len + sysfs_emit_at(buf, len, "\n");
}

static ssize_t foc_ctrl_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct private_data *st = iio_priv(indio_dev);
	unsigned int value[2] = {0, 16};
	unsigned int val;
	int ret = 0;

	switch ((u32)this_attr->address) {
	case AP_CTRL:
		foc_read_reg(st, AP_CTRL, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case FLUX_ACC_STTS:
		foc_read_reg(st, FLUX_ACC_STTS, value);
		ret = kq16tos(buf, value);
		break;
	case FLUX_ERR_STTS:
		foc_read_reg(st, FLUX_ERR_STTS, value);
		ret = kq16tos(buf, value);
		break;
	case TORQUE_ACC_STTS:
		foc_read_reg(st, TORQUE_ACC_STTS, value);
		ret = kq16tos(buf, value);
		break;
	case TORQUE_ERR_STTS:
		foc_read_reg(st, TORQUE_ERR_STTS, value);
		ret = kq16tos(buf, value);
		break;
	case PPR_ARGS:
		foc_read_reg(st, PPR_ARGS, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case CONTROL_MODE_ARGS:
		foc_read_reg(st, CONTROL_MODE_ARGS, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case CONTROL_FIXPERIOD_ARGS:
		foc_read_reg(st, CONTROL_FIXPERIOD_ARGS, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case FLUX_SP_ARGS:
		foc_read_reg(st, FLUX_SP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case FLUX_KP_ARGS:
		foc_read_reg(st, FLUX_KP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case FLUX_KI_ARGS:
		foc_read_reg(st, FLUX_KI_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case FLUX_KD_ARGS:
		foc_read_reg(st, FLUX_KD_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case TORQUE_SP_ARGS:
		foc_read_reg(st, TORQUE_SP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case TORQUE_KP_ARGS:
		foc_read_reg(st, TORQUE_KP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case TORQUE_KI_ARGS:
		foc_read_reg(st, TORQUE_KI_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case TORQUE_KD_ARGS:
		foc_read_reg(st, TORQUE_KD_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case SPEED_SP_ARGS:
		foc_read_reg(st, SPEED_SP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case SPEED_KI_ARGS:
		foc_read_reg(st, SPEED_KI_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case SPEED_KP_ARGS:
		foc_read_reg(st, SPEED_KP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case SPEED_KD_ARGS:
		foc_read_reg(st, SPEED_KD_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case ANGLE_SH_ARGS:
		foc_read_reg(st, ANGLE_SH_ARGS, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case VD_ARGS:
		foc_read_reg(st, VD_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case VQ_ARGS:
		foc_read_reg(st, VQ_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case FW_KP_ARGS:
		foc_read_reg(st, FW_KP_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case FW_KI_ARGS:
		foc_read_reg(st, FW_KI_ARGS, value);
		ret = kq16tos(buf, value);
		break;
	case FIXED_ANGLE_ARGS:
		foc_read_reg(st, FIXED_ANGLE_ARGS, &val);
		ret = sprintf(buf, "%u\n", val);
		break;
	case SAMPLE_TIME:
		val = st->interval;
		ret = sprintf(buf, "%u\n", val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int kstrtoq1516(const char *buf, int *res)
{
	int deccount = 0, i = 0, pos = 0;
	bool negative = false, fraction = false;
	unsigned long long inumber = 0, fnumber = 0, fstrval = 0, istrval = 0,
			   divider = 1;

	while (buf[pos]) {
		if (pos == 0 && (buf[pos] == '+' || buf[pos] == '-')) {
			if (buf[pos] == '-')
				negative = true;
		} else if (buf[pos] == '.' && !fraction) {
			fraction = true;
		} else if (!fraction && isdigit(buf[pos])) {
			inumber = inumber * 10 + buf[pos] - '0';
			if (i > MAXIRANGE)
				return -ERANGE;
			i++;
		} else if (fraction && isdigit(buf[pos])) {
			fnumber = fnumber * 10 + buf[pos] - '0';
			if (deccount > MAXFRANGE) {
				deccount = MAXFRANGE;
				break;
			}
			deccount++;
		} else if (buf[pos] == '\n') {
			break;
		} else {
			return -EINVAL;
		}
		pos++;
	}

	if (inumber >= 32768 && -inumber < -32768)
		return -ERANGE;

	istrval = (inumber) << 16;

	while (deccount) {
		deccount--;
		divider = divider * 10;
	}

		/* avoiding overflow */
	if (!(fnumber > 281474976710656))
		fstrval = (fnumber << 16) / divider;
	else
		fstrval = ((fnumber / 1000) << 16) / (divider / 1000);

	*res = istrval + fstrval;

	if (negative)
		*res *= -1;

	return 0;
}

static ssize_t foc_ctrl_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct private_data *st = iio_priv(indio_dev);
	unsigned long long data_in;
	int ret = 0, qdata_in = 0;

	switch ((u32)this_attr->address) {
	case AP_CTRL:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			foc_write_reg(st, AP_CTRL, data_in);
		else
			return ret;
		break;
	case PPR_ARGS:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			foc_write_reg(st, PPR_ARGS, data_in);
		else
			return ret;
		break;
	case CONTROL_MODE_ARGS:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			foc_write_reg(st, CONTROL_MODE_ARGS, data_in);
		else
			return ret;
		break;
	case CONTROL_FIXPERIOD_ARGS:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			foc_write_reg(st, CONTROL_FIXPERIOD_ARGS, data_in);
		else
			return ret;
		break;
	case FLUX_SP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, FLUX_SP_ARGS, qdata_in);
		else
			return ret;
		break;
	case FLUX_KP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, FLUX_KP_ARGS, qdata_in);
		else
			return ret;
		break;
	case FLUX_KI_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, FLUX_KI_ARGS, qdata_in);
		else
			return ret;
		break;
	case FLUX_KD_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, FLUX_KD_ARGS, qdata_in);
		else
			return ret;
		break;
	case TORQUE_SP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, TORQUE_SP_ARGS, qdata_in);
		else
			return ret;
		break;
	case TORQUE_KP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, TORQUE_KP_ARGS, qdata_in);
		else
			return ret;
		break;
	case TORQUE_KI_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, TORQUE_KI_ARGS, qdata_in);
		else
			return ret;
		break;
	case TORQUE_KD_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, TORQUE_KD_ARGS, qdata_in);
		else
			return ret;
		break;
	case SPEED_SP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, SPEED_SP_ARGS, qdata_in);
		else
			return ret;
		break;
	case SPEED_KI_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, SPEED_KI_ARGS, qdata_in);
		else
			return ret;
		break;
	case SPEED_KP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, SPEED_KP_ARGS, qdata_in);
		else
			return ret;
		break;
	case SPEED_KD_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, SPEED_KD_ARGS, qdata_in);
		else
			return ret;
		break;
	case ANGLE_SH_ARGS:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			foc_write_reg(st, ANGLE_SH_ARGS, data_in);
		else
			return ret;
		break;
	case VD_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, VD_ARGS, qdata_in);
		else
			return ret;
		break;
	case VQ_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, VQ_ARGS, qdata_in);
		else
			return ret;
		break;
	case FW_KP_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, FW_KP_ARGS, qdata_in);
		else
			return ret;
		break;
	case FW_KI_ARGS:
		ret = kstrtoq1516(buf, &qdata_in);
		if (!ret)
			foc_write_reg(st, FW_KI_ARGS, qdata_in);
		else
			return ret;
		break;
	case FIXED_ANGLE_ARGS:
		ret = kstrtoull(buf, 10, &data_in);
		if (!ret)
			foc_write_reg(st, FIXED_ANGLE_ARGS, data_in);
		else
			return ret;
		break;
	case SAMPLE_TIME:
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

static IIO_DEVICE_ATTR(flux_acc, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FLUX_ACC_STTS);

static IIO_DEVICE_ATTR(flux_err, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FLUX_ERR_STTS);

static IIO_DEVICE_ATTR(torque_acc, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       TORQUE_ACC_STTS);

static IIO_DEVICE_ATTR(torque_err, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       TORQUE_ERR_STTS);

/* read write attributes */

static IIO_DEVICE_ATTR(ap_ctrl, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       AP_CTRL);

static IIO_DEVICE_ATTR(PPR, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       PPR_ARGS);

static IIO_DEVICE_ATTR(control_mode, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       CONTROL_MODE_ARGS);

static IIO_DEVICE_ATTR(fixed_period_ctrl, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       CONTROL_FIXPERIOD_ARGS);

static IIO_DEVICE_ATTR(flux_sp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FLUX_SP_ARGS);

static IIO_DEVICE_ATTR(flux_kp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FLUX_KP_ARGS);

static IIO_DEVICE_ATTR(flux_ki, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FLUX_KI_ARGS);

static IIO_DEVICE_ATTR(flux_kd, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FLUX_KD_ARGS);

static IIO_DEVICE_ATTR(torque_sp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       TORQUE_SP_ARGS);

static IIO_DEVICE_ATTR(torque_kp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       TORQUE_KP_ARGS);

static IIO_DEVICE_ATTR(torque_ki, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       TORQUE_KI_ARGS);

static IIO_DEVICE_ATTR(torque_kd, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       TORQUE_KD_ARGS);

static IIO_DEVICE_ATTR(speed_sp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       SPEED_SP_ARGS);

static IIO_DEVICE_ATTR(speed_ki, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       SPEED_KI_ARGS);

static IIO_DEVICE_ATTR(speed_kp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       SPEED_KP_ARGS);

static IIO_DEVICE_ATTR(speed_kd, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       SPEED_KD_ARGS);

static IIO_DEVICE_ATTR(angle_sh, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       ANGLE_SH_ARGS);

static IIO_DEVICE_ATTR(vd, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       VD_ARGS);

static IIO_DEVICE_ATTR(vq, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       VQ_ARGS);

static IIO_DEVICE_ATTR(fw_kp, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FW_KP_ARGS);

static IIO_DEVICE_ATTR(fw_ki, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FW_KI_ARGS);

static IIO_DEVICE_ATTR(fixed_angle_cpr, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       FIXED_ANGLE_ARGS);

static IIO_DEVICE_ATTR(sample_interval_us, 0644,
		       foc_ctrl_show,
		       foc_ctrl_store,
		       SAMPLE_TIME);

static struct attribute *foc_attributes[] = {
	&iio_dev_attr_flux_acc.dev_attr.attr,
	&iio_dev_attr_flux_err.dev_attr.attr,
	&iio_dev_attr_torque_acc.dev_attr.attr,
	&iio_dev_attr_torque_err.dev_attr.attr,
	&iio_dev_attr_PPR.dev_attr.attr,
	&iio_dev_attr_control_mode.dev_attr.attr,
	&iio_dev_attr_fixed_period_ctrl.dev_attr.attr,
	&iio_dev_attr_flux_sp.dev_attr.attr,
	&iio_dev_attr_flux_kp.dev_attr.attr,
	&iio_dev_attr_flux_ki.dev_attr.attr,
	&iio_dev_attr_flux_kd.dev_attr.attr,
	&iio_dev_attr_torque_sp.dev_attr.attr,
	&iio_dev_attr_torque_kp.dev_attr.attr,
	&iio_dev_attr_torque_ki.dev_attr.attr,
	&iio_dev_attr_torque_kd.dev_attr.attr,
	&iio_dev_attr_speed_sp.dev_attr.attr,
	&iio_dev_attr_speed_ki.dev_attr.attr,
	&iio_dev_attr_speed_kp.dev_attr.attr,
	&iio_dev_attr_speed_kd.dev_attr.attr,
	&iio_dev_attr_angle_sh.dev_attr.attr,
	&iio_dev_attr_vd.dev_attr.attr,
	&iio_dev_attr_vq.dev_attr.attr,
	&iio_dev_attr_fw_kp.dev_attr.attr,
	&iio_dev_attr_fw_ki.dev_attr.attr,
	&iio_dev_attr_fixed_angle_cpr.dev_attr.attr,
	&iio_dev_attr_ap_ctrl.dev_attr.attr,
	&iio_dev_attr_sample_interval_us.dev_attr.attr,
	NULL,
};

static const struct attribute_group foc_attribute_group = {
	.attrs = foc_attributes,
};

static const struct iio_info foc_info = {
	.read_raw = &foc_read_attribute,
	.attrs = &foc_attribute_group,
	.debugfs_reg_access = foc_debugfs_reg_access,
};

static int foc_fill_buffer(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);
	/* data buffer for channel data and timestamp */
	unsigned int data[9 + sizeof(s64) / sizeof(int)] = {0};
	unsigned int val, chan = 0, i = 0;
	s64 time;

	time = iio_get_time_ns(indio_dev);
	/*
	 * Read data for active channels from device
	 */
	for_each_set_bit(chan, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		foc_read_reg(st, indio_dev->channels[chan].address, &val);
		data[i] = val;
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data, time);

	return 0;
}

static int foc_capture_thread(void *data)
{
	struct iio_dev *indio_dev = data;
	struct private_data *st = iio_priv(indio_dev);
	u32 interval = st->interval;

	do {
		/* Read the data from sensor and push it to buffers */
		int ret = foc_fill_buffer(indio_dev);

		if (ret < 0)
			return ret;
		usleep_range(interval, interval + 20);
	} while (!kthread_should_stop());

	return 0;
}

static int foc_buffer_enable(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);
	struct task_struct *task;

	task = kthread_run(foc_capture_thread, (void *)indio_dev,
			      "%s", indio_dev->name);

	if (IS_ERR(task))
		return PTR_ERR(task);

	get_task_struct(task);
	wake_up_process(task);
	st->task = task;

	return 0;
}

static int foc_buffer_disable(struct iio_dev *indio_dev)
{
	struct private_data *st = iio_priv(indio_dev);

	if (st->task) {
		kthread_stop(st->task);
		put_task_struct(st->task);
		st->task = NULL;
	}
	return 0;
}

static const struct iio_buffer_setup_ops foc_setup_ops = {
	.postenable = &foc_buffer_enable,
	.predisable = &foc_buffer_disable,
};

static const struct of_device_id foc_ids[] = {
	{ .compatible = "xlnx,hls-foc-periodic-1.0", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, foc_ids);

static int foc_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct private_data *data;
	struct resource *res;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(foc_ids, pdev->dev.of_node);

	if (!id)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));

	if (!indio_dev) {
		dev_err(&pdev->dev, "iio allocation failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &foc_info;
	indio_dev->name = KBUILD_MODNAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = foc_channels;
	indio_dev->num_channels = ARRAY_SIZE(foc_channels);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	data->interval = 200;

	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	foc_write_reg(data, AP_CTRL, DEFAULT_AP_CTRL);
	foc_write_reg(data, PPR_ARGS, DEFAULT_PPR_ARGS);
	foc_write_reg(data, CONTROL_MODE_ARGS, DEFAULT_CONTROL_MODE_ARGS);
	foc_write_reg(data, CONTROL_FIXPERIOD_ARGS, DEFAULT_CONTROL_FIXPERIOD_ARGS);
	foc_write_reg(data, FLUX_SP_ARGS, DEFAULT_FLUX_SP_ARGS);
	foc_write_reg(data, FLUX_KP_ARGS, DEFAULT_FLUX_KP_ARGS);
	foc_write_reg(data, FLUX_KI_ARGS, DEFAULT_FLUX_KI_ARGS);
	foc_write_reg(data, FLUX_KD_ARGS, DEFAULT_FLUX_KD_ARGS);
	foc_write_reg(data, TORQUE_SP_ARGS, DEFAULT_TORQUE_SP_ARGS);
	foc_write_reg(data, TORQUE_KP_ARGS, DEFAULT_TORQUE_KP_ARGS);
	foc_write_reg(data, TORQUE_KI_ARGS, DEFAULT_TORQUE_KI_ARGS);
	foc_write_reg(data, TORQUE_KD_ARGS, DEFAULT_TORQUE_KD_ARGS);
	foc_write_reg(data, SPEED_SP_ARGS, DEFAULT_SPEED_SP_ARGS);
	foc_write_reg(data, SPEED_KP_ARGS, DEFAULT_SPEED_KP_ARGS);
	foc_write_reg(data, SPEED_KI_ARGS, DEFAULT_SPEED_KI_ARGS);
	foc_write_reg(data, SPEED_KD_ARGS, DEFAULT_SPEED_KD_ARGS);
	foc_write_reg(data, ANGLE_SH_ARGS, DEFAULT_ANGLE_SH_ARGS);
	foc_write_reg(data, VD_ARGS, DEFAULT_VD_ARGS);
	foc_write_reg(data, VQ_ARGS, DEFAULT_VQ_ARGS);
	foc_write_reg(data, FW_KP_ARGS, DEFAULT_FW_KP_ARGS);
	foc_write_reg(data, FW_KI_ARGS, DEFAULT_FW_KI_ARGS);
	foc_write_reg(data, FIXED_ANGLE_ARGS, DEFAULT_FIXED_ANGLE);
	ret = devm_iio_kfifo_buffer_setup(&indio_dev->dev, indio_dev,
					  &foc_setup_ops);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Successfully registered device FOC");
	platform_set_drvdata(pdev, indio_dev);

	return 0;
}

static int foc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	return 0;
}

static struct platform_driver foc_drv = {
	.probe = foc_probe,
	.remove = foc_remove,
	.driver = {
		.name = "hls-foc-periodic",
		.of_match_table = of_match_ptr(foc_ids),
	},
};
module_platform_driver(foc_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vivekananda Dayananda <vivekananda.dayananda@amd.com>");
