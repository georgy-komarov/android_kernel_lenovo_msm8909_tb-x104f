/*****************************************************************************
 *
 * Copyright (c) 2015 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/hqsysfs.h>
#include "bstclass.h"

#define ACC_NAME  "ACC"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define MCUBE_FUNC_DEBUG
#define MCUBE_LOG_DEBUG
#define MCUBE_ERROR_DEBUG

#define MC3XXX_DEV_NAME       "mc3xxx"
#define MC3XXX_DEV_VERSION    "2.1.0"
#define MC3XXX_INPUT_NAME     "accelerometer"
#define MC3XXX_I2C_ADDR       0x4c


#ifdef MCUBE_FUNC_DEBUG
    #define MC_FUNC_PRINT(x...)        printk(x)
#else
    #define MC_FUNC_PRINT(x...)
#endif

#ifdef MCUBE_LOG_DEBUG
    #define MC_LOG_PRINT(x...)        printk(x)
#else
    #define MC_LOG_PRINT(x...)
#endif

#ifdef MCUBE_ERROR_DEBUG
    #define MC_ERR_PRINT(x...)        printk(x)
#else
    #define MC_ERR_PRINT(x...)
#endif

// register address define
#define MC3XXX_XOUT_REG		0x00
#define MC3XXX_YOUT_REG		0x01
#define MC3XXX_ZOUT_REG			0x02
#define MC3XXX_TILT_REG			0x03
#define MC3XXX_OPSTAT_REG		0x04
#define MC3XXX_SC_REG			0x05
#define MC3XXX_INTEN_REG		0x06
#define MC3XXX_MODE_REG		0x07
#define MC3XXX_SAMPR_REG		0x08
#define MC3XXX_TAPEN_REG		0x09
#define MC3XXX_TAPP_REG			0x0A
#define MC3XXX_DROP_REG		0x0B
#define MC3XXX_SHDB_REG		0x0C
#define MC3XXX_XOUT_EX_L_REG	0x0D
#define MC3XXX_XOUT_EX_H_REG	0x0E
#define MC3XXX_YOUT_EX_L_REG	0x0F
#define MC3XXX_YOUT_EX_H_REG	0x10
#define MC3XXX_ZOUT_EX_L_REG	0x11
#define MC3XXX_ZOUT_EX_H_REG	0x12

#define MC3XXX_CHIPID_REG		0x18

#define MC3XXX_OUTCFG_REG		0x20
#define MC3XXX_XOFFL_REG		0x21
#define MC3XXX_XOFFH_REG		0x22
#define MC3XXX_YOFFL_REG		0x23
#define MC3XXX_YOFFH_REG		0x24
#define MC3XXX_ZOFFL_REG		0x25
#define MC3XXX_ZOFFH_REG		0x26
#define MC3XXX_XGAIN_REG		0x27
#define MC3XXX_YGAIN_REG		0x28
#define MC3XXX_ZGAIN_REG		0x29

#define MC3XXX_SHAKE_TH_REG	0x2B
#define MC3XXX_UD_Z_TH_REG		0x2C
#define MC3XXX_UD_X_TH_REG		0x2D
#define MC3XXX_RL_Z_TH_REG		0x2E
#define MC3XXX_RL_Y_TH_REG		0x2F
#define MC3XXX_FB_Z_TH_REG		0x30
#define MC3XXX_DROP_TH_REG		0x31
#define MC3XXX_TAP_TH_REG		0x32

#define MC3XXX_PCODE_REG		0x3B

// Mode
#define MC3XXX_MODE_AUTO			0
#define MC3XXX_MODE_WAKE			1
#define MC3XXX_MODE_SNIFF			2
#define MC3XXX_MODE_STANDBY		3

// Range
#define MC3XXX_RANGE_2G			0
#define MC3XXX_RANGE_4G			1
#define MC3XXX_RANGE_8G			2
#define MC3XXX_RANGE_12G			2
#define MC3XXX_RANGE_16G			3

// Resolution
#define MC3XXX_RES_6BIT				0
#define MC3XXX_RES_7BIT				1
#define MC3XXX_RES_8BIT				2
#define MC3XXX_RES_10BIT			3
#define MC3XXX_RES_12BIT			4
#define MC3XXX_RES_14BIT			5

// Bandwidth
#define MC3XXX_BW_512HZ			0
#define MC3XXX_BW_256HZ			1
#define MC3XXX_BW_128HZ			2
#define MC3XXX_BW_64HZ				3
#define MC3XXX_BW_32HZ				4
#define MC3XXX_BW_16HZ				5
#define MC3XXX_BW_8HZ				6

// Product code
#define MC3XXX_PCODE_3210			0x90
#define MC3XXX_PCODE_3230			0x19
#define MC3XXX_PCODE_3250			0x88
#define MC3XXX_PCODE_3410			0xA8
#define MC3XXX_PCODE_3410N			0xB8
#define MC3XXX_PCODE_3430			0x29
#define MC3XXX_PCODE_3430N			0x39
#define MC3XXX_PCODE_3510			0x40
#define MC3XXX_PCODE_3530			0x30
#define MC3XXX_PCODE_3216			0x10
#define MC3XXX_PCODE_3236			0x60
#define MC3XXX_PCODE_3416			0x20


#define MC3XXX_PCODE_RESERVE_1		0x20
#define MC3XXX_PCODE_RESERVE_2		0x11
#define MC3XXX_PCODE_RESERVE_3		0x21
#define MC3XXX_PCODE_RESERVE_4		0x61
#define MC3XXX_PCODE_RESERVE_5		0xA0
#define MC3XXX_PCODE_RESERVE_6		0xE0
#define MC3XXX_PCODE_RESERVE_7		0x91
#define MC3XXX_PCODE_RESERVE_8		0xA1
#define MC3XXX_PCODE_RESERVE_9		0xE1

#define MC3XXX_PCODE_RESERVE_10	0x99

#define MC3XXX_AXIS_X			0
#define MC3XXX_AXIS_Y			1
#define MC3XXX_AXIS_Z			2
#define MC3XXX_AXES_NUM		3
#define CALICOUNT       			3
#define ABS(a) ( (a)>0 ? (a):(-(a)))

// 1g constant value
#define GRAVITY_1G_VALUE			16384

// Initial value
#define MC3XXX_RANGE_SET			MC3XXX_RANGE_8G  /* +/-8g */
#define MC3XXX_RESO_SET				MC3XXX_RES_14BIT /* 14bit */
#define MC3XXX_BW_SET				MC3XXX_BW_128HZ /* 128HZ  */
#define MC3XXX_MAX_DELAY			200
#define ABSMIN_1_5G					(-3 * GRAVITY_1G_VALUE / 2)
#define ABSMAX_1_5G				    	(3 * GRAVITY_1G_VALUE / 2)
#define ABSMIN_2G					(-2 * GRAVITY_1G_VALUE)
#define ABSMAX_2G				    	(2 * GRAVITY_1G_VALUE)
#define ABSMIN_8G					(-8 * GRAVITY_1G_VALUE)
#define ABSMAX_8G					(8 * GRAVITY_1G_VALUE)

// MC3XXX power supply VDD 1.7V-3.6V VIO 1.7-3.6V
#define MC3XXX_VDD_MIN_UV			2000000
#define MC3XXX_VDD_MAX_UV			3400000
#define MC3XXX_VIO_MIN_UV			1800000
#define MC3XXX_VIO_MAX_UV			3400000

// Polling delay in msecs
#define POLL_INTERVAL_MIN_MS		10
#define POLL_INTERVAL_MAX_MS		2000
#define POLL_DEFAULT_INTERVAL_MS 	200

// Interrupt delay in msecs
#define MC3XXX_INT_MAX_DELAY		64

#define MC3XXX_MAX_RETRY_I2C_XFER	(100)
#define I2C_RETRY_DELAY()           	usleep_range(1000, 2000)


// End type
enum mc3xxx_sensor_type {
	MC3XXX_LOW_END = 0,
	MC3XXX_HIGH_END
};

enum mc3xxx_placement {
	MC3XXX_TOP_LEFT_DOWN = 0,		// 0: top, left-down
	MC3XXX_TOP_RIGHT_DOWN,		// 1: top, reight-down
	MC3XXX_TOP_RIGHT_UP,			// 2: top, right-up
	MC3XXX_TOP_LEFT_UP,			// 3: top, left-up
	MC3XXX_BOTTOM_LEFT_DOWN,		// 4: bottom, left-down
	MC3XXX_BOTTOM_RIGHT_DOWN,	// 5: bottom, right-down
	MC3XXX_BOTTOM_RIGHT_UP,		// 6: bottom, right-up
	MC3XXX_BOTTOM_LEFT_UP		// 7: bottom, left-up
};

struct mc3xxx_hwmsen_convert {
	signed char sign[3];
	unsigned char map[3];
};

struct mc3xxx_acc {
	signed int x;
	signed int y;
	signed int z;
};

struct mc3xxx_platform_data {
	int poll_interval;
	int gpio_int;
	unsigned int int_flag;
	unsigned char place;
	bool int_en;
};

struct mc3xxx_suspend_state {
	bool powerEn;
};

struct mc3xxx_pcode_data {
	unsigned char pcode;
	unsigned char chipid;
	unsigned char mpol;
	bool mcfm12;
	bool mcfm3x;
};

struct mc3xxx_data {
	struct i2c_client *mc3xxx_client;
	struct sensors_classdev cdev;
	atomic_t delay;
	atomic_t enable;
	struct input_dev *input;

	struct bst_dev *bst_acc;

	struct mc3xxx_acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct workqueue_struct *data_wq;
	struct delayed_work delay_work;
	struct work_struct irq_work;
	struct regulator *vdd;
	struct regulator *vio;

	struct hrtimer	poll_timer;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct mc3xxx_platform_data *pdata;
	struct mc3xxx_suspend_state suspend_state;

	unsigned int int_flag;
	int IRQ;
	struct mc3xxx_pcode_data pcode_data;
	unsigned short gain;
	unsigned char endtype;
	//unsigned char bandwidth;
	//unsigned char range;
	//unsigned char resolution;
	//unsigned char mode;
	bool power_enabled;
};


static void mc3xxx_set_enable(struct device *dev, int enable);
static int offset_x = 0, offset_y = 0, offset_z = 0;


static const struct mc3xxx_hwmsen_convert mc3xxx_cvt[] = {
	{{   1,    1,    1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 0: top   , left-down
	{{ -1,    1,    1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 1: top   , right-down
	{{ -1,  -1,    1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 2: top   , right-up
	{{   1,  -1,    1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 3: top   , left-up
	{{ -1,    1,  -1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 4: bottom, left-down
	{{   1,    1,  -1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 5: bottom, right-down
	{{   1,  -1,  -1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 6: bottom, right-up
	{{ -1,  -1,  -1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 7: bottom, left-up
};

static struct sensors_classdev sensors_cdev = {
		.name = "mc3xxx-accel",
		.vendor = "mcube",
		.version = 1,
		.handle = SENSORS_ACCELERATION_HANDLE,
		.type = SENSOR_TYPE_ACCELEROMETER,
		.max_range = "156.8",	/* 16g */
		.resolution = "0.009570",	/* 0.1mg */
		.sensor_power = "0.1",	/* typical value */
		.min_delay = POLL_INTERVAL_MIN_MS * 1000, /* in microseconds */
		.max_delay = POLL_INTERVAL_MAX_MS,
		.max_latency = POLL_INTERVAL_MAX_MS,
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = POLL_DEFAULT_INTERVAL_MS, /* in millisecond */
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
		.sensors_self_test = NULL,
};


#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3xxx_early_suspend(struct early_suspend *h);
static void mc3xxx_late_resume(struct early_suspend *h);
#endif

static int mc3xxx_open_init(struct i2c_client *client, struct mc3xxx_data *data);
static int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode);
static int mc3xxx_get_mode(struct i2c_client *client, unsigned char *mode);
static int mc3xxx_power_ctl(struct mc3xxx_data *data, bool on);

static int mc3xxx_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -EIO;
	*data = dummy & 0x000000ff;

	return 0;
}

static int mc3xxx_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -EIO;
	udelay(2);
	return 0;
}

static int mc3xxx_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -EIO;
	return 0;
}

#if 0
static int mc3xxx_i2c_burst_read(struct i2c_client *client, unsigned char reg_addr,
		unsigned char *data, unsigned short len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < MC3XXX_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			I2C_RETRY_DELAY();
	}

	if (MC3XXX_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif
/*
static int mc3xxx_validate_pcode(unsigned char *pbPCode, unsigned char *pbHwID)
{
    if (   (0x01 == *pbHwID)
        || (0x03 == *pbHwID)
        || ((0x04 <= *pbHwID) && (*pbHwID <= 0x0F)))
    {
        if ((MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3230 == *pbPCode) || (MC3XXX_PCODE_3250 == *pbPCode))
            return 0;
    }
    else if (   (0x02 == *pbHwID)
             || (0x21 == *pbHwID)
             || ((0x10 <= *pbHwID) && (*pbHwID <= 0x1F)))
    {
        if (   (MC3XXX_PCODE_3210 == *pbPCode) || (MC3XXX_PCODE_3230  == *pbPCode)
            || (MC3XXX_PCODE_3250 == *pbPCode)
            || (MC3XXX_PCODE_3410 == *pbPCode) || (MC3XXX_PCODE_3410N == *pbPCode)
            || (MC3XXX_PCODE_3430 == *pbPCode) || (MC3XXX_PCODE_3430N == *pbPCode))
        {
            return 0;
        }
    }
    else if ((0xC0 <= *pbHwID) && (*pbHwID <= 0xCF))
    {
        *pbPCode = (*pbPCode & 0x71);

        if ((MC3XXX_PCODE_3510 == *pbPCode) || (MC3XXX_PCODE_3530 == *pbPCode))
            return 0;
    }
    else if ((0x20 == *pbHwID) || ((0x22 <= *pbHwID) && (*pbHwID <= 0x2F)))
    {
        *pbPCode = (*pbPCode & 0xF1);

        if (   (MC3XXX_PCODE_3210      == *pbPCode) || (MC3XXX_PCODE_3216      == *pbPCode) || (MC3XXX_PCODE_3236      == *pbPCode)
            || (MC3XXX_PCODE_RESERVE_1 == *pbPCode) || (MC3XXX_PCODE_RESERVE_2 == *pbPCode) || (MC3XXX_PCODE_RESERVE_3 == *pbPCode)
            || (MC3XXX_PCODE_RESERVE_4 == *pbPCode) || (MC3XXX_PCODE_RESERVE_5 == *pbPCode) || (MC3XXX_PCODE_RESERVE_6 == *pbPCode)
            || (MC3XXX_PCODE_RESERVE_7 == *pbPCode) || (MC3XXX_PCODE_RESERVE_8 == *pbPCode) || (MC3XXX_PCODE_RESERVE_9 == *pbPCode))
        {
            return 0;
        }
    }

    return -1;
}
*/
static int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode)
{
	unsigned char data;
	int ret = -1;
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_LOG_PRINT("%s: %d\n", __func__, mode);

	mutex_lock(&mc3xxx->mode_mutex);

	if (MC3XXX_MODE_STANDBY == mode) {
		data = 0x43;
		ret = mc3xxx_smbus_write_byte(client, MC3XXX_MODE_REG, &data);
	}
	else if (MC3XXX_MODE_WAKE == mode) {
		data = 0xe9; //0x41;
		ret = mc3xxx_smbus_write_byte(client, MC3XXX_MODE_REG, &data);
	}
	mutex_unlock(&mc3xxx->mode_mutex);

	return ret;
}

static int mc3xxx_get_mode(struct i2c_client *client, unsigned char *mode)
{
	unsigned char data;
	int ret = -1;
	//struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	ret = mc3xxx_smbus_read_byte(client, MC3XXX_MODE_REG, &data);
	*mode = data & 0x03;

	return ret;
}

static int mc3xxx_check_product_code(struct i2c_client *client,	struct mc3xxx_data *mc3xxx)
{
	const unsigned char auto_probe_addr[] = {0x4C, 0x6C};
	const unsigned char auto_probe_count =  (sizeof(auto_probe_addr) / sizeof(auto_probe_addr[0]));

	int i;
	int ret = 0;
	unsigned char pcode = 0;
	unsigned char chipid = 0;
	unsigned char mpol = 0;

	for (i = 0; i < auto_probe_count; i ++) {
		client->addr = auto_probe_addr[i];

		ret = mc3xxx_smbus_read_byte(client, MC3XXX_PCODE_REG, &pcode);
		if (ret) {
			MC_ERR_PRINT("%s: read pcode error: %x\n", __func__, ret);
			continue;
		}
		pcode = (pcode & 0x31);//for mensa

		ret = mc3xxx_smbus_read_byte(client, MC3XXX_CHIPID_REG, &chipid);
		if (ret) {
			MC_ERR_PRINT("%s: read chipid error: %x\n", __func__, ret);
			continue;
		}

		ret = mc3xxx_smbus_read_byte(client, 0x2a, &mpol);
		if (ret) {
			MC_ERR_PRINT("%s: read mpol error: %x\n", __func__, ret);
			continue;
		}

		//if (0 == mc3xxx_validate_pcode(&pcode, &chipid)) {
		if (0xA0 == chipid) {//for mensa
			mc3xxx->pcode_data.pcode = pcode;
			mc3xxx->pcode_data.chipid = chipid;
			mc3xxx->pcode_data.mpol = mpol;
			mc3xxx->pcode_data.mcfm12 = (0xc0 <= chipid) && (chipid <= 0xcf);
			mc3xxx->pcode_data.mcfm3x = (0x20 == chipid) || ((0x22 <= chipid) && (chipid <= 0x2f));
			
			MC_LOG_PRINT("%s: 0x%x, 0x%x\n", __func__, pcode, chipid);
			return 0;
		}
	}

	return -1;
}

static int mc3xxx_set_endtype(struct i2c_client *client, struct mc3xxx_data *mc3xxx)
{
	int err = 0;
  //MC_FUNC_PRINT("mc3xxx->pcode_data.pcode :0x%x\n", mc3xxx->pcode_data.pcode);
	switch (mc3xxx->pcode_data.pcode) {
	case MC3XXX_PCODE_3230:
	case MC3XXX_PCODE_3430:
	case MC3XXX_PCODE_3430N:
	case MC3XXX_PCODE_3530:
	case MC3XXX_PCODE_3236:
		mc3xxx->endtype = MC3XXX_LOW_END;
		break;

	case MC3XXX_PCODE_3210:
	case MC3XXX_PCODE_3250:
	case MC3XXX_PCODE_3410:
	case MC3XXX_PCODE_3410N:
	case MC3XXX_PCODE_3510:
	case MC3XXX_PCODE_3216:
	case MC3XXX_PCODE_3416:
		mc3xxx->endtype = MC3XXX_HIGH_END;
		break;

	default:
		err = -1;
		break;
	}

	return err;
}

static int mc3xxx_init_configure(struct i2c_client *client, struct mc3xxx_data *mc3xxx)
{
	int ret = 0;
	unsigned char tmp = 0;

	//MC_FUNC_PRINT("%s called\n", __func__);

	// set sample rate
	if (mc3xxx->pcode_data.mcfm12 || mc3xxx->pcode_data.mcfm3x) {
		switch (mc3xxx->pcode_data.mpol & 0xc0) {
		case 0:
			tmp = 0;
			break;
		case 0x40:
			tmp = 0x08;
			break;
		case 0x80:
			tmp = 0x09;
			break;
		case 0xc0:
			tmp = 0x0a;
			break;
		}
	}
	tmp = 0x05;//for memsa
	ret += mc3xxx_smbus_write_byte(client, MC3XXX_SAMPR_REG, &tmp);

	// configure range
	if (mc3xxx->pcode_data.mcfm12 || mc3xxx->pcode_data.mcfm3x) {
		if (MC3XXX_LOW_END == mc3xxx->endtype)
			tmp = 0x02;
		else
			tmp = 0x25;
	}
	else if (MC3XXX_LOW_END == mc3xxx->endtype)
		tmp = 0x32;
	else
		tmp = 0x3f;
	
	tmp = 0x29;//for mensa
	ret += mc3xxx_smbus_write_byte(client, MC3XXX_OUTCFG_REG, &tmp);

	// set gain value
	if (MC3XXX_LOW_END == mc3xxx->endtype) {
		if (mc3xxx->pcode_data.mcfm12 || mc3xxx->pcode_data.mcfm3x)
			mc3xxx->gain = 64;
		else
			mc3xxx->gain = 86;
	}
	else
		mc3xxx->gain = 1024;
	
	mc3xxx->gain = 4096;//for mensa
	return ret;
}

static void mc3xxx_remap_sensor_data(struct mc3xxx_data *mc3xxx, struct mc3xxx_acc *acc)
{
	int tmp[3];
	const struct mc3xxx_hwmsen_convert *pCvt = &mc3xxx_cvt[mc3xxx->pdata->place];

	if (MC3XXX_PCODE_3250 == mc3xxx->pcode_data.pcode) {
		tmp[0] = acc->y;
		tmp[1] = - acc->x;
		tmp[2] = acc->z;
	}
	else {
		if (mc3xxx->pcode_data.mpol & 0x01)
			tmp[0] = -acc->x;
		else
			tmp[0] = acc->x;
		if (mc3xxx->pcode_data.mpol & 0x02)
			tmp[1] = -acc->y;
		else
			tmp[1] = acc->y;
		tmp[2] = acc->z;
	}

	tmp[0] = tmp[0] * GRAVITY_1G_VALUE / mc3xxx->gain;
	tmp[1] = tmp[1] * GRAVITY_1G_VALUE / mc3xxx->gain;
	tmp[2] = tmp[2] * GRAVITY_1G_VALUE / mc3xxx->gain;

	acc->x = pCvt->sign[MC3XXX_AXIS_X] * tmp[pCvt->map[MC3XXX_AXIS_X]];
	acc->y = pCvt->sign[MC3XXX_AXIS_Y] * tmp[pCvt->map[MC3XXX_AXIS_Y]];
	acc->z = pCvt->sign[MC3XXX_AXIS_Z] * tmp[pCvt->map[MC3XXX_AXIS_Z]];
}

static int mc3xxx_read_accel_xyz(struct i2c_client *client, struct mc3xxx_acc *acc)
{
	int comres = 0;
	unsigned char data[6];
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	if (MC3XXX_HIGH_END == mc3xxx->endtype) {
		comres = mc3xxx_smbus_read_byte_block(client, MC3XXX_XOUT_EX_L_REG, data, 6);

		acc->x = (signed short)((data[1]<<8) |data[0]);
		acc->y = (signed short)((data[3]<<8) |data[2]);
		acc->z = (signed short)((data[5]<<8) |data[4]);
	}
	else {
		comres = mc3xxx_smbus_read_byte_block(client, MC3XXX_XOUT_REG, data, 3);

		acc->x = (signed char)data[0];
		acc->y = (signed char)data[1];
		acc->z = (signed char)data[2];
	}

	//MC_LOG_PRINT("%s: %d, %d, %d\n", __func__, acc->x, acc->y, acc->z);

	mc3xxx_remap_sensor_data(mc3xxx, acc);

	return comres;
}


static int mc3xxx_read_accel_xyz_calibrate(struct i2c_client *client, struct mc3xxx_acc *acc)
{
	int comres = 0;
	unsigned char data[6];
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	if (MC3XXX_HIGH_END == mc3xxx->endtype) {
		comres = mc3xxx_smbus_read_byte_block(client, MC3XXX_XOUT_EX_L_REG, data, 6);

		acc->x = (signed short)((data[1]<<8) |data[0]);
		acc->y = (signed short)((data[3]<<8) |data[2]);
		acc->z = (signed short)((data[5]<<8) |data[4]);
	}
	else {
		comres = mc3xxx_smbus_read_byte_block(client, MC3XXX_XOUT_REG, data, 3);

		acc->x = (signed char)data[0];
		acc->y = (signed char)data[1];
		acc->z = (signed char)data[2];
	}

	MC_LOG_PRINT("%s: %d, %d, %d\n", __func__, acc->x, acc->y, acc->z);

	mc3xxx_remap_sensor_data(mc3xxx, acc);	
	
	
	return comres;
}

static void mc3xxx_report_axis_data(struct mc3xxx_data *mc3xxx, struct mc3xxx_acc *value)
{
	ktime_t ts;
	int err;

	//MC_FUNC_PRINT("%s called\n", __func__);

	ts = ktime_get_boottime();
	err = mc3xxx_read_accel_xyz(mc3xxx->mc3xxx_client, value);
	if (err < 0) {
		dev_err(&mc3xxx->mc3xxx_client->dev,
			"read accel data failed! err = %d\n", err);
		return;
	}
	input_report_abs(mc3xxx->input, ABS_X, (value->x - offset_x));
	input_report_abs(mc3xxx->input, ABS_Y, (value->y - offset_y));
	input_report_abs(mc3xxx->input, ABS_Z, (value->z - offset_z));
	input_event(mc3xxx->input, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(mc3xxx->input, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);
	input_sync(mc3xxx->input);
}

static void mc3xxx_work_func(struct work_struct *work)
{
	struct mc3xxx_data *mc3xxx = container_of((struct delayed_work *)work,
			struct mc3xxx_data, delay_work);
	struct mc3xxx_acc value;

	//MC_FUNC_PRINT("%s called\n", __func__);

	mc3xxx_report_axis_data(mc3xxx, &value);
	mutex_lock(&mc3xxx->value_mutex);
	mc3xxx->value = value;
	mutex_unlock(&mc3xxx->value_mutex);
	//queue_delayed_work(mc3xxx->data_wq, &mc3xxx->delay_work, 
		//msecs_to_jiffies(atomic_read(&mc3xxx->delay)));
}


static int byteToInt4(const char buf[],int offset)
{
	return ((buf[offset]&0xff) << 24) |
		   ((buf[offset + 1] & 0xff) << 16) |
		   ((buf[offset + 2] & 0xff) << 8) |
		   ((buf[offset + 3] & 0xff));
}

#if 1
static ssize_t mc3xxx_calibration_xyz_store(struct device *dev,
		struct device_attribute *attr,const char *buf, size_t count)
{
	MC_FUNC_PRINT("%s called\n", __func__);
		
	offset_x = byteToInt4(buf,0);
	offset_y = byteToInt4(buf,4);
	offset_z = byteToInt4(buf,8);
	//printk("linson x = %d,y = %d,z = %d\n", offset_x, offset_y, offset_z);
	
	return count;
}
#endif
static int mc3xxx_calibration_xyz_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i,cali_fail_cnt=0;
	struct mc3xxx_acc acc_value={0};
	struct mc3xxx_acc acc_value_sum={0};
	struct mc3xxx_acc offset={0};

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *data = i2c_get_clientdata(client);
	
	int pre_enable = atomic_read(&data->enable);
	//printk("cali_pre_enable= %d ", pre_enable);
	
	if(pre_enable == 0)
		mc3xxx_set_enable(dev, 1);
	
	MC_FUNC_PRINT("%s called\n", __func__);

	for(i=0;i<CALICOUNT;i++)
	{
retry:
		mc3xxx_read_accel_xyz_calibrate(data->mc3xxx_client, &acc_value);
		if( ABS(acc_value.x) > (GRAVITY_1G_VALUE/5) || ABS(acc_value.y)>(GRAVITY_1G_VALUE/5)
			|| ABS(ABS(acc_value.z) - GRAVITY_1G_VALUE) > (GRAVITY_1G_VALUE/4)){
			printk("data offset is bigger than the thershold,  x = %d,y = %d,z = %d \n ", acc_value.x, acc_value.y, acc_value.z);
			cali_fail_cnt++;
			if(cali_fail_cnt > 3)
			{
				if(pre_enable == 0)
				mc3xxx_set_enable(dev, 0);
				return 0;
			}
			else
			{
				mdelay(30);
				goto retry;	
			}
		}
		else{
			//printk("jungle_cali_single_data x = %d,y = %d,z = %d\n", acc_value.x, acc_value.y, acc_value.z);
			acc_value_sum.x += acc_value.x;
			acc_value_sum.y += acc_value.y;
			acc_value_sum.z += acc_value.z;
		}
		mdelay(30);
	}
	offset.x=acc_value_sum.x/CALICOUNT-0;
	offset.y=acc_value_sum.y/CALICOUNT-0;
	offset.z=ABS(acc_value_sum.z/CALICOUNT)-GRAVITY_1G_VALUE;
	
	printk("cali_final_offset x = %d,y = %d,z = %d \n", offset.x, offset.y, offset.z);
    offset_x=offset.x;
    offset_y=offset.y;
    offset_z=offset.z;
    
    if(pre_enable == 0)
		mc3xxx_set_enable(dev, 0);
	
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d", offset.x, offset.y, offset.z);	
}


static ssize_t mc3xxx_place_store(struct device *dev,
		struct device_attribute *attr,const char *buf,size_t count)
{

	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	mc3xxx->pdata->place = data;

	return count;
}

static ssize_t mc3xxx_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	sscanf(buf, "%3d %3d", &address, &value);
	if (mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, (unsigned char)address,
				(unsigned char *)&value) < 0)
		return -EINVAL;
	return count;
}

static ssize_t mc3xxx_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x40];
	int i;

	MC_FUNC_PRINT("%s called\n", __func__);

	for (i = 0; i < 0x40; i++) {
		mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, i, reg+i);

		count += snprintf(&buf[count], PAGE_SIZE,
			"0x%x: %x\n", i, reg[i]);
	}
	return count;
}

static ssize_t mc3xxx_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	if (mc3xxx_get_mode(mc3xxx->mc3xxx_client, &data) < 0)
		return snprintf(buf, PAGE_SIZE, "Read error\n");

	return snprintf(buf, PAGE_SIZE, "0x%x\n", data);
}

static ssize_t mc3xxx_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (mc3xxx_set_mode(mc3xxx->mc3xxx_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mc3xxx_value_cache_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct mc3xxx_data *mc3xxx = input_get_drvdata(input);
	struct mc3xxx_acc acc_value;

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&mc3xxx->value_mutex);
	acc_value = mc3xxx->value;
	mutex_unlock(&mc3xxx->value_mutex);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t mc3xxx_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct mc3xxx_data *mc3xxx = input_get_drvdata(input);
	struct mc3xxx_acc acc_value;

	MC_FUNC_PRINT("%s called\n", __func__);

	mc3xxx_read_accel_xyz(mc3xxx->mc3xxx_client, &acc_value);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", acc_value.x - offset_x, acc_value.y - offset_y,
			acc_value.z - offset_z );
}

static ssize_t mc3xxx_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mc3xxx->delay));

}

static ssize_t mc3xxx_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	return snprintf(buf, PAGE_SIZE, "0x%x, 0x%x\n", mc3xxx->pcode_data.pcode, mc3xxx->pcode_data.chipid);
}


static ssize_t mc3xxx_place_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int place;

	MC_FUNC_PRINT("%s called\n", __func__);

	place = mc3xxx->pdata->place;

	return snprintf(buf, PAGE_SIZE, "%d\n", place);
}


static ssize_t mc3xxx_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data < POLL_INTERVAL_MIN_MS)
		data = POLL_INTERVAL_MIN_MS;
	if (data > POLL_INTERVAL_MAX_MS)
		data = POLL_INTERVAL_MAX_MS;
	atomic_set(&mc3xxx->delay, (unsigned int) data);

	return count;
}


static ssize_t mc3xxx_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mc3xxx->enable));

}

static void mc3xxx_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&mc3xxx->enable);
	unsigned char tmp = 0;

	MC_LOG_PRINT("%s: %d, %d\n", __func__, enable, pre_enable);

	mutex_lock(&mc3xxx->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			if (mc3xxx_power_ctl(mc3xxx, true)) {
				dev_err(dev, "power failed\n");
				goto mutex_exit;
			}
			if (mc3xxx_open_init(mc3xxx->mc3xxx_client, mc3xxx) < 0) {
				dev_err(dev, "set init failed\n");
				goto mutex_exit;
			}

			if (mc3xxx->pdata->int_en) {
				tmp = 0x80; // Enable ACQ_INT for Merak
				mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, MC3XXX_INTEN_REG, &tmp);
				
				enable_irq(mc3xxx->IRQ);
			}
			else {
				#if 0 
				queue_delayed_work(mc3xxx->data_wq,
					&mc3xxx->delay_work,
					msecs_to_jiffies
					(atomic_read(&mc3xxx->delay)));
				#else
				hrtimer_start(&mc3xxx->poll_timer,
					ns_to_ktime(atomic_read(&mc3xxx->delay) * 1000000UL),
					HRTIMER_MODE_REL);
				#endif
			}

			mc3xxx_set_mode(mc3xxx->mc3xxx_client,
					MC3XXX_MODE_WAKE);
					
			atomic_set(&mc3xxx->enable, 1);
		}
	} else {
		if (pre_enable == 1) {
			mc3xxx_set_mode(mc3xxx->mc3xxx_client,
					MC3XXX_MODE_STANDBY);

			if (mc3xxx->pdata->int_en) {
				disable_irq(mc3xxx->IRQ);
				
				tmp = 0; // Disable ACQ_INT for Merak
				mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, MC3XXX_INTEN_REG, &tmp);
			}
			else {
				#if 0
				cancel_delayed_work_sync(&mc3xxx->delay_work);
				#else
				hrtimer_cancel(&mc3xxx->poll_timer);
				cancel_work_sync(&mc3xxx->delay_work.work);
				#endif

				atomic_set(&mc3xxx->enable, 0);
				if (mc3xxx_power_ctl(mc3xxx, false)) {
					dev_err(dev, "power failed\n");
					goto mutex_exit;
				}
			}
		}
	}
	
mutex_exit:
	mutex_unlock(&mc3xxx->enable_mutex);
	dev_dbg(&client->dev,
		"set enable: en=%d, en_state=%d, use_int=%d\n",
		enable, atomic_read(&mc3xxx->enable),
		mc3xxx->pdata->int_en);
}

static ssize_t mc3xxx_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	MC_FUNC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		mc3xxx_set_enable(dev, data);

	return count;
}

static int mc3xxx_cdev_enable(struct sensors_classdev *sensors_cdev,
				unsigned int enable)
{
	struct mc3xxx_data *data = container_of(sensors_cdev,
					struct mc3xxx_data, cdev);

	mc3xxx_set_enable(&data->mc3xxx_client->dev, enable);
	return 0;
}

static int mc3xxx_is_power_enabled(struct mc3xxx_data *data)
{
	return atomic_read(&data->enable);
}

static int mc3xxx_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct mc3xxx_data *data = container_of(sensors_cdev,
					struct mc3xxx_data, cdev);

	if (delay_ms < POLL_INTERVAL_MIN_MS)
		delay_ms = POLL_INTERVAL_MIN_MS;
	if (delay_ms > POLL_INTERVAL_MAX_MS)
		delay_ms = POLL_INTERVAL_MAX_MS;
	atomic_set(&data->delay, (unsigned int) delay_ms);
	return 0;
}
static DEVICE_ATTR(op_mode, S_IRUSR|S_IRGRP|S_IWUSR,
		mc3xxx_mode_show, mc3xxx_mode_store);
static DEVICE_ATTR(value, S_IRUSR|S_IRGRP,
		mc3xxx_value_show, NULL);
static DEVICE_ATTR(value_cache, S_IRUSR|S_IRGRP,
		mc3xxx_value_cache_show, NULL);
static DEVICE_ATTR(delay, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		mc3xxx_delay_show, mc3xxx_delay_store);
static DEVICE_ATTR(enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		mc3xxx_enable_show, mc3xxx_enable_store);
static DEVICE_ATTR(reg, S_IRUSR|S_IRGRP|S_IWUSR,
		mc3xxx_register_show, mc3xxx_register_store);
static DEVICE_ATTR(chip_id, S_IRUSR|S_IRGRP,
		mc3xxx_chip_id_show, NULL);
static DEVICE_ATTR(place, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		mc3xxx_place_show, mc3xxx_place_store);
static DEVICE_ATTR(calibration, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
	    mc3xxx_calibration_xyz_show, mc3xxx_calibration_xyz_store);


static struct attribute *mc3xxx_attributes[] = {
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_value_cache.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_reg.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_place.attr,
  &dev_attr_calibration.attr,

	NULL
};

static struct attribute_group mc3xxx_attribute_group = {
	.attrs = mc3xxx_attributes
};

static void mc3xxx_irq_work_func(struct work_struct *work)
{
	struct mc3xxx_data *mc3xxx = container_of((struct work_struct *)work,
			struct mc3xxx_data, irq_work);
	struct mc3xxx_acc value;

	unsigned char status = 0;

	mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, MC3XXX_TILT_REG, &status);
	
	mc3xxx_report_axis_data(mc3xxx, &value);
	
	mutex_lock(&mc3xxx->value_mutex);
	mc3xxx->value = value;
	mutex_unlock(&mc3xxx->value_mutex);
}

static irqreturn_t mc3xxx_irq_handler(int irq, void *handle)
{
	struct mc3xxx_data *data = handle;

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->mc3xxx_client == NULL)
		return IRQ_HANDLED;

	queue_work(data->data_wq, &data->irq_work);

	return IRQ_HANDLED;
}


static int mc3xxx_power_ctl(struct mc3xxx_data *data, bool on)
{
	int ret = 0;
	int err = 0;

	//MC_FUNC_PRINT("%s called\n", __func__);

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else {
		dev_info(&data->mc3xxx_client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int mc3xxx_power_init(struct mc3xxx_data *data)
{
	int ret;

	MC_FUNC_PRINT("%s called\n", __func__);

	data->vdd = regulator_get(&data->mc3xxx_client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->mc3xxx_client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				MC3XXX_VDD_MIN_UV,
				MC3XXX_VDD_MAX_UV);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->mc3xxx_client->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		dev_err(&data->mc3xxx_client->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
				MC3XXX_VIO_MIN_UV,
				MC3XXX_VIO_MAX_UV);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
			"Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, MC3XXX_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int mc3xxx_power_deinit(struct mc3xxx_data *data)
{
	int ret;

	MC_FUNC_PRINT("%s called\n", __func__);

	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->mc3xxx_client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, MC3XXX_VDD_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vio) > 0)
		regulator_set_voltage(data->vio, 0, MC3XXX_VIO_MAX_UV);

	regulator_put(data->vio);

	return 0;
}

#ifdef CONFIG_OF
static int mc3xxx_parse_dt(struct device *dev,
			struct mc3xxx_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	MC_FUNC_PRINT("%s called\n", __func__);

	rc = of_property_read_u32(np, "mcube,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		pdata->poll_interval = temp_val;
	}

	rc = of_property_read_u32(np, "mcube,place", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read sensor place paramater\n");
		return rc;
	}
	if (temp_val > 7 || temp_val < 0) {
		dev_err(dev, "Invalid place parameter, use default value 0\n");
		pdata->place = 0;
	} else {
		pdata->place = temp_val;
	}

	pdata->int_en = of_property_read_u32(np, "mcube,use-interrupt", &temp_val);

	pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
				"mcube,gpio-int", 0, &pdata->int_flag);

	return 0;
}
#else
static int mc3xxx_parse_dt(struct device *dev,
			struct mc3xxx_platform_data *pdata)
{
	return -EINVAL;
}
#endif

static int mc3xxx_open_init(struct i2c_client *client, struct mc3xxx_data *data)
{
	int err;

	//MC_FUNC_PRINT("%s called\n", __func__);

	err = mc3xxx_set_mode(client, MC3XXX_MODE_STANDBY);
	if (err < 0) {
		dev_err(&client->dev, "set mode error\n");
		return err;
	}

	err = mc3xxx_set_endtype(client, data);
	if (err < 0) {
		dev_err(&client->dev, "set endtype error\n");
		return err;
	}
	
	err = mc3xxx_init_configure(client, data);
	if (err < 0) {
		dev_err(&client->dev, "init configure error\n");
		return err;
	}

	return 0;
}

static int mc3xxx_get_interrupt_gpio(const struct mc3xxx_data *data,
			const unsigned int gpio)
{
	struct i2c_client *client = data->mc3xxx_client;
	int err;

	if (!gpio_is_valid(gpio)) {
		dev_err(&client->dev,
			"gpio(%d) is invalid,\n", gpio);
		return -EINVAL;
	}

	err = gpio_request(gpio, "mc3xxx_gpio_int");
	if (err) {
		dev_err(&client->dev,
			"Unable to request gpio %d, err=%d\n",
			gpio, err);
		return err;
	}

	err = gpio_direction_input(gpio);
	if (err) {
		dev_err(&client->dev,
			"Unable to set gpio direction %d, err=%d\n",
			gpio, err);
		gpio_free(gpio);
		return err;
	}

	client->irq = gpio_to_irq(gpio);
	dev_dbg(&client->dev, "Interrupt gpio=%d, irq=%d\n", gpio, client->irq);

	return 0;
}



#if 0
static int mc3xxx_pinctrl_init(struct mc3xxx_data *data)
{
	return 0;
}
#endif

static enum hrtimer_restart mc3xxx_timer_func(struct hrtimer *timer)
{
	struct mc3xxx_data *data = container_of(timer, struct mc3xxx_data, poll_timer);;

	//MC_FUNC_PRINT("%s called\n", __func__);

	queue_work(data->data_wq, &data->delay_work.work);
	hrtimer_forward_now(&data->poll_timer,
			ns_to_ktime(atomic_read(&data->delay) * 1000000UL));
	

	return HRTIMER_RESTART;
}

static void mc3xxx_pinctrl_state(struct mc3xxx_data *data,
			bool active)
{
}

static int mc3xxx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct mc3xxx_data *data;
	struct input_dev *dev;
	struct bst_dev  *dev_acc;
	struct mc3xxx_platform_data *pdata;

//	struct input_dev *dev_interrupt;

	MC_FUNC_PRINT("%s called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		MC_ERR_PRINT("%s: i2c_check_functionality error\n", __func__);
		dev_err(&client->dev, "i2c_check_functionality error\n");
		err = -EPERM;
		goto exit;
	}
	data = kzalloc(sizeof(struct mc3xxx_data), GFP_KERNEL);
	if (!data) {
		MC_ERR_PRINT("%s: alloc mc3xxx_data error\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			MC_ERR_PRINT("%s: failed to allcated memory\n", __func__);
			dev_err(&client->dev, "Failed to allcated memory\n");
			err = -ENOMEM;
			goto kfree_exit;
		}
		err = mc3xxx_parse_dt(&client->dev, pdata);
		if (err) {
			MC_ERR_PRINT("%s: failed to parse device tree\n", __func__);
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto pdata_free_exit;
		}
	} else {
		pdata = client->dev.platform_data;
		MC_ERR_PRINT("%s: use platform data\n", __func__);
		dev_err(&client->dev, "Use platform data\n");
	}

	if (!pdata) {
		MC_ERR_PRINT("%s: cannot get device platform data\n", __func__);
		dev_err(&client->dev, "Cannot get device platform data\n");
		err = -EINVAL;
		goto kfree_exit;
	}
	data->pdata = pdata;
	i2c_set_clientdata(client, data);
	data->mc3xxx_client = client;

	err = mc3xxx_power_init(data);
	if (err) {
		MC_ERR_PRINT("%s: failed to get sensor regulators\n", __func__);
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto free_i2c_clientdata_exit;
	}
	err = mc3xxx_power_ctl(data, true);
	if (err) {
		MC_ERR_PRINT("%s: failed to enable sensor power\n", __func__);
		dev_err(&client->dev, "Failed to enable sensor power\n");
		err = -EINVAL;
		goto deinit_power_exit;
	}

	/* read and check product code */
	if (mc3xxx_check_product_code(client, data) < 0) {
		MC_ERR_PRINT("%s: can't detect mCube's g-sensor\n", __func__);
		err = -EINVAL;
		goto disable_power_exit;
	}

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
	//data->bandwidth = MC3XXX_BW_SET;
	//data->range = MC3XXX_RANGE_SET;
	//data->sensitivity = mc3xxx_range_map[0];
	err = mc3xxx_open_init(client, data);
	if (err < 0) {
		MC_ERR_PRINT("%s: failed to initialize\n", __func__);
		err = -EINVAL;
		goto disable_power_exit;
	}

	if (pdata->int_en) {
		data->int_flag = pdata->int_flag;
		err = mc3xxx_get_interrupt_gpio(data, pdata->gpio_int);
		if (err) {
			dev_err(&client->dev,
				"Failed to get interrupt gpio, err=%d\n",
				err);
			err = -EINVAL;
			goto disable_power_exit;
		}

		data->IRQ = client->irq;
		if (!data->int_flag)
			data->int_flag = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

		dev_dbg(&client->dev, "IRQ=%d, int_flag=0x%x\n",
			data->IRQ, data->int_flag);

		err = request_irq(data->IRQ, mc3xxx_irq_handler,
			data->int_flag, "mc3xxx", data);
		if (err) {
			dev_err(&client->dev,  "Could not request irq\n");
			goto free_interrupt_gpio;
		}
		disable_irq(data->IRQ);
		INIT_WORK(&data->irq_work, mc3xxx_irq_work_func);
	} else {
		//INIT_DELAYED_WORK(&data->delay_work, mc3xxx_work_func);
		hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC,
					HRTIMER_MODE_REL);
		data->poll_timer.function = mc3xxx_timer_func;
		INIT_WORK(&data->delay_work.work, mc3xxx_work_func);
	}

	data->data_wq = alloc_workqueue("mc3xxx_data_work",
				WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	//create_freezable_workqueue("mc3xxx_data_work");
	if (!data->data_wq) {
		MC_ERR_PRINT("%s: cannot get create workqueue\n", __func__);
		dev_err(&client->dev, "Cannot get create workqueue!\n");
		goto free_irq_exit;
	}

	atomic_set(&data->delay, POLL_DEFAULT_INTERVAL_MS);
	atomic_set(&data->enable, 0);

	dev = devm_input_allocate_device(&client->dev);
	if (!dev) {
		MC_ERR_PRINT("%s: cannot allocate input device\n", __func__);
		dev_err(&client->dev, "Cannot allocate input device\n");
		err = -ENOMEM;
		goto destroy_workqueue_exit;
	}

	/* only value events reported */
	dev->name = MC3XXX_INPUT_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_MISC);

	if (MC3XXX_HIGH_END == data->endtype) {
		input_set_abs_params(dev, ABS_X, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_Y, ABSMIN_8G, ABSMAX_8G, 0, 0);
		input_set_abs_params(dev, ABS_Z, ABSMIN_8G, ABSMAX_8G, 0, 0);
	}
	else {
		input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
		input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
		input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	}

	input_set_drvdata(dev, data);
	err = input_register_device(dev);
	if (err < 0) {
		MC_ERR_PRINT("%s: cannot register input device\n", __func__);
		dev_err(&client->dev, "Cannot register input device\n");
		goto destroy_workqueue_exit;
	}

	data->input = dev;	

	err = sysfs_create_group(&data->input->dev.kobj,
			&mc3xxx_attribute_group);
	if (err < 0) {
		MC_ERR_PRINT("%s: cannot create sysfs for mc3xxx\n", __func__);
		dev_err(&client->dev, "Cannot create sysfs for mc3xxx\n");
		goto destroy_workqueue_exit;
	}
	
	//add for new node
	dev_acc = bst_allocate_device();
	if (!dev_acc) {
		dev_err(&client->dev,
			"Cannot allocate bst device\n");
		err = -ENOMEM;
		goto remove_mc3xxx_sysfs_exit;
	}
	dev_acc->name = ACC_NAME;

	bst_set_drvdata(dev_acc, data);

	err = bst_register_device(dev_acc);
	if (err < 0) {
		dev_err(&client->dev,
			"Cannot register bst device\n");
		goto bst_free_acc_exit;
	}

	data->bst_acc = dev_acc;
	err = sysfs_create_group(&data->bst_acc->dev.kobj,
			&mc3xxx_attribute_group);

	if (err < 0) {
		dev_err(&client->dev,
			"Cannot create sysfs for bst_acc.\n");
		goto bst_free_exit;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mc3xxx_early_suspend;
	data->early_suspend.resume = mc3xxx_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	data->cdev = sensors_cdev;
	data->cdev.min_delay = POLL_INTERVAL_MIN_MS * 1000;
	data->cdev.delay_msec = pdata->poll_interval;
	data->cdev.sensors_enable = mc3xxx_cdev_enable;
	data->cdev.sensors_poll_delay = mc3xxx_cdev_poll_delay;
	data->cdev.sensors_calibrate = NULL; //mc3xxx_self_calibration_xyz;
	data->cdev.sensors_write_cal_params = NULL; //mc3xxx_write_cal_params;
	if (MC3XXX_HIGH_END == data->endtype)
		data->cdev.resolution = "0.00957031";
	else
		data->cdev.resolution = "0.153125";
	if (pdata->int_en)
		data->cdev.max_delay = MC3XXX_INT_MAX_DELAY;
	err = sensors_classdev_register(&data->input->dev, &data->cdev);
	if (err) {
		dev_err(&client->dev, "create class device file failed!\n");
		err = -EINVAL;
		goto remove_sysfs_exit;
	}
  
  	hq_regiser_hw_info(HWID_GSENSOR,"G-sensor:mcube MC3416-P");

	dev_notice(&client->dev, "mc3xxx driver probe successfully\n");
	MC_LOG_PRINT("%s: mc3xxx(0x%x) probe OK\n", __func__, data->pcode_data.pcode);
	mc3xxx_pinctrl_state(data, false);
	mc3xxx_power_ctl(data, false);
	return 0;

remove_sysfs_exit:
	sysfs_remove_group(&data->input->dev.kobj,
			&mc3xxx_attribute_group);
destroy_workqueue_exit:
	destroy_workqueue(data->data_wq);
free_irq_exit:
free_interrupt_gpio:
	if (pdata->int_en) {
		gpio_free(pdata->gpio_int);
	}
	

bst_free_exit:
	bst_unregister_device(dev_acc);

bst_free_acc_exit:
	bst_free_device(dev_acc);

remove_mc3xxx_sysfs_exit:
	sysfs_remove_group(&data->input->dev.kobj,
			&mc3xxx_attribute_group);
//set_pinctrl_sleep:
//	if (pdata->int_en) {
//		mc3xxx_pinctrl_state(data, false);
//	}
disable_power_exit:
	mc3xxx_power_ctl(data, false);
deinit_power_exit:
	mc3xxx_power_deinit(data);
free_i2c_clientdata_exit:
	i2c_set_clientdata(client, NULL);
pdata_free_exit:
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	data->pdata = NULL;
kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3xxx_early_suspend(struct early_suspend *h)
{
	struct mc3xxx_data *data =
		container_of(h, struct mc3xxx_data, early_suspend);

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_STANDBY);
		if (!data->pdata->int_en)
			cancel_delayed_work_sync(&data->delay_work);
	}
	mutex_unlock(&data->enable_mutex);
}

static void mc3xxx_late_resume(struct early_suspend *h)
{
	struct mc3xxx_data *data =
		container_of(h, struct mc3xxx_data, early_suspend);

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_WAKE);
		if (!data->pdata->int_en)
			queue_delayed_work(data->data_wq,
				&data->delay_work,
				msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
}
#endif

static int mc3xxx_remove(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	sensors_classdev_unregister(&data->cdev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	if (data->input)
		sysfs_remove_group(&data->input->dev.kobj,
				&mc3xxx_attribute_group);
	if (data->bst_acc) {
		bst_unregister_device(data->bst_acc);
		bst_free_device(data->bst_acc);
	}

	destroy_workqueue(data->data_wq);
	mc3xxx_set_enable(&client->dev, 0);
	mc3xxx_power_deinit(data);
	i2c_set_clientdata(client, NULL);
	if (data->pdata && (client->dev.of_node))
		devm_kfree(&client->dev, data->pdata);
	data->pdata = NULL;

	kfree(data);

	return 0;
}

void mc3xxx_shutdown(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_STANDBY);
	mutex_unlock(&data->enable_mutex);
}

#ifdef CONFIG_PM
static int mc3xxx_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);
	
	data->suspend_state.powerEn = mc3xxx_is_power_enabled(data);
	mc3xxx_set_enable(&client->dev, 0);
	return 0;
}

static int mc3xxx_resume(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_FUNC_PRINT("%s called\n", __func__);
	
	if (data->suspend_state.powerEn)
		mc3xxx_set_enable(&client->dev, 1);

	return 0;
}

#else

#define mc3xxx_suspend      NULL
#define mc3xxx_resume       NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id mc3xxx_id[] = {
	{ MC3XXX_DEV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mc3xxx_id);

static const struct of_device_id mc3xxx_of_match[] = {
	{ .compatible = "mcube,mc3xxx", },
	{ },
};

static struct i2c_driver mc3xxx_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = MC3XXX_DEV_NAME,
		.of_match_table = mc3xxx_of_match,
	},
	.suspend    = mc3xxx_suspend,
	.resume     = mc3xxx_resume,
	.id_table   = mc3xxx_id,
	.probe      = mc3xxx_probe,
	.remove     = mc3xxx_remove,
	.shutdown   = mc3xxx_shutdown,
};

static int __init mc3xxx_init(void)
{
	MC_FUNC_PRINT("%s called\n", __func__);
	return i2c_add_driver(&mc3xxx_driver);
}

static void __exit mc3xxx_exit(void)
{
	MC_FUNC_PRINT("%s called\n", __func__);
	i2c_del_driver(&mc3xxx_driver);
}

module_init(mc3xxx_init);
module_exit(mc3xxx_exit);

MODULE_AUTHOR("mCube-inc");
MODULE_DESCRIPTION("MC3XXX ACCELEROMETER SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

