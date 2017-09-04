/* drivers/input/touchscreen/melfas_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
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
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/melfas_ts.h>
#include <linux/firmware.h>

#include "mcs8000_download.h"

#if defined(CONFIG_MAX8986_MUIC)
#include <linux/mfd/max8986/max8986.h>
#endif

/** for Cooper VE *****************************************/
#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE

//#define __TOUCH_DEBUG__  //TODO :  ø 쿡  ø.
#define USE_THREADED_IRQ 1 //TODO : QUEUE  ƴ THREAD  . ̷ ϴ, IRQ   ߵ.
#define DELAY_BEFORE_VDD

#define SET_DOWNLOAD_BY_GPIO 0 //TODO : TSP ʱȭ ƾ ʿ  ֽ FW  Ʈ ϴ ƾ ϸ ȵ.
#define LATEST_FW_VER_HW00   0x15 //TODO : ̺κ 0x0  ϸ, SET_DOWNLOAD_BY_GPIO 1 ̾  .
#define LATEST_FW_VER_HW01   0x15 //TODO : ̺κ 0x0  ϸ, SET_DOWNLOAD_BY_GPIO 1 ̾  .
//#define FORCED_DOWNLOAD_OF_BLANKMEMORY	// TSP blank memory( No firmware ) ½ ڵ ߿ ٿε
//#define ALWAYS_DOWNLOAD

#define __TOUCH_KEY__
//#define __TOUCH_KEYLED__
#define I2C_RETRY_CNT	3
static int tsp_testmode = 0;
static int index =0;

#if defined(CONFIG_MAX8986_MUIC)
#define __TOUCH_TA_CHECK__		// for AT&T Charger
#endif

#endif	// CONFIG_TOUCHSCREEN_MMS128_COOPERVE
/***********************************************************/

/** for Tass VE *****************************************/
#ifdef CONFIG_TOUCHSCREEN_MMS128_TASSVE

//#define __TOUCH_DEBUG__  //TODO :  ø 쿡  ø.
#define USE_THREADED_IRQ 1 //TODO : QUEUE  ƴ THREAD  . ̷ ϴ, IRQ   ߵ.
#define DELAY_BEFORE_VDD

#define SET_DOWNLOAD_BY_GPIO 0 //TODO : TSP ʱȭ ƾ  ֽ FW  Ʈ ϴ ƾ ϸ ȵ.
#define LATEST_FW_VER   0x03 //TODO : ̺κ 0x0  ϸ, SET_DOWNLOAD_BY_GPIO 1 ̾  .
//#define FORCED_DOWNLOAD_OF_BLANKMEMORY	// TSP blank memory( No firmware ) ½ ڵ ߿ ٿε
//#define ALWAYS_DOWNLOAD

//#define __TOUCH_KEY__
//#define __TOUCH_KEYLED__

#if defined(CONFIG_MAX8986_MUIC)
#define __TOUCH_TA_CHECK__		// for AT&T Charger
#endif

#endif	// CONFIG_TOUCHSCREEN_MMS128_TASSVE
/***********************************************************/

#define TOUCH_ON  1
#define TOUCH_OFF 0

#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE
#define SEC_TSP 
#define TSP_FACTORY_TEST

#ifdef SEC_TSP
struct device *sec_touchscreen;
EXPORT_SYMBOL(sec_touchscreen);
#define TS_READ_EXCITING_CH_ADDR	0x2E
#define TS_READ_SENSING_CH_ADDR	    0x2F
#define TS_WRITE_REGS_LEN		    16
#define RMI_ADDR_TESTMODE           0xA0
#define UNIVERSAL_CMD_RESULT_SIZE   0xAE
#define UNIVERSAL_CMD_RESULT        0xAF
#ifdef TSP_FACTORY_TEST
struct device *qt602240_noise_test;
EXPORT_SYMBOL(qt602240_noise_test);
#endif

//struct class *touch_class;
//EXPORT_SYMBOL(touch_class);
//struct device *firmware_dev;
//EXPORT_SYMBOL(firmware_dev);
#endif
#endif


#if defined (__TOUCH_KEYLED__)
static struct regulator *touchkeyled_regulator=NULL;

static int g_keyled_cnt=0; 		// for auto-off
static bool g_check_action=false;
static bool g_check_keyled=false;	// for check keyled on/off status
#endif
static bool init_lowleveldata=true;
static int g_exciting_ch, g_sensing_ch;

#define MAX_RX_	10
#define MAX_TX_	15

static const uint16_t SCR_ABS_UPPER_SPEC[MAX_RX_][MAX_TX_] =
{
        {4065, 3925, 3845, 3778, 3732, 3695, 3667, 3650, 3649, 3635, 3620, 3623, 3625, 3627, 3599},
        {4058, 3914, 3833, 3774, 3722, 3673, 3634, 3603, 3587, 3563, 3547, 3543, 3525, 3522, 3537},
        {4059, 3927, 3850, 3789, 3740, 3705, 3677, 3660, 3648, 3628, 3625, 3628, 3628, 3624, 3593},
        {4063, 3928, 3852, 3789, 3743, 3704, 3679, 3659, 3645, 3628, 3624, 3627, 3625, 3622, 3584},
        {4058, 3927, 3850, 3789, 3743, 3704, 3679, 3660, 3644, 3627, 3622, 3623, 3624, 3609, 3582},
        {4060, 3928, 3852, 3793, 3749, 3712, 3687, 3664, 3648, 3627, 3627, 3627, 3624, 3609, 3585},
        {4064, 3929, 3854, 3797, 3755, 3718, 3689, 3668, 3649, 3632, 3629, 3628, 3619, 3609, 3588},
        {4073, 3930, 3854, 3799, 3758, 3722, 3690, 3669, 3652, 3633, 3630, 3628, 3609, 3608, 3590},
        {4094, 3938, 3873, 3810, 3753, 3705, 3667, 3640, 3608, 3585, 3577, 3550, 3549, 3549, 3557},
        {4092, 3932, 3860, 3809, 3767, 3727, 3695, 3673, 3652, 3639, 3637, 3620, 3617, 3617, 3595},
};

static const uint16_t SCR_ABS_LOWER_SPEC[MAX_RX_][MAX_TX_] =
{
        {2439, 2355, 2307, 2266, 2238, 2217, 2199, 2190, 2189, 2181, 2172, 2173, 2175, 2175, 2159},
        {2434, 2348, 2299, 2264, 2232, 2203, 2180, 2161, 2151, 2137, 2127, 2125, 2115, 2112, 2121},
        {2435, 2355, 2310, 2273, 2244, 2223, 2205, 2196, 2188, 2176, 2175, 2176, 2176, 2174, 2155},
        {2437, 2356, 2310, 2273, 2245, 2222, 2207, 2195, 2187, 2176, 2174, 2175, 2175, 2172, 2150},
        {2434, 2355, 2310, 2273, 2245, 2222, 2207, 2196, 2186, 2175, 2172, 2173, 2174, 2165, 2148},
        {2436, 2356, 2310, 2275, 2249, 2226, 2211, 2198, 2188, 2175, 2175, 2175, 2174, 2165, 2151},
        {2438, 2357, 2312, 2277, 2253, 2230, 2213, 2200, 2189, 2178, 2177, 2176, 2171, 2165, 2152},
        {2443, 2358, 2312, 2279, 2254, 2232, 2214, 2201, 2190, 2179, 2178, 2176, 2165, 2164, 2154},
        {2456, 2362, 2323, 2286, 2251, 2223, 2199, 2184, 2164, 2151, 2145, 2130, 2129, 2129, 2133},
        {2454, 2358, 2316, 2285, 2259, 2235, 2217, 2203, 2190, 2183, 2181, 2172, 2169, 2169, 2157},
};

/** structures **/
struct muti_touch_info
{
    int state;
    int strength;
    int width;
    int posX;
    int posY;
};

struct melfas_ts_data
{
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct  work;
    struct hrtimer timer;
    struct work_struct work_timer;
    uint32_t flags;
    //int (*power)(int on);
    struct early_suspend early_suspend;
    int VenderID;
    int hw_rev;
    int fw_ver;
};

/** variables **/
static struct muti_touch_info g_Mtouch_info[TS_MAX_TOUCH];
static struct melfas_ts_data *ts;
//struct melfas_ts_data *ts_global_melfas;
#if defined (CONFIG_TOUCHSCREEN_TMA340) || defined (CONFIG_TOUCHSCREEN_TMA340_COOPERVE) || defined (CONFIG_TOUCHSCREEN_F760)
static int8_t MMS128_Connected = 0;
#endif
static struct regulator *touch_regulator = NULL;
static int firmware_ret_val = -1;
static DEFINE_SPINLOCK(melfas_spin_lock);
#if USE_THREADED_IRQ

#else
static struct workqueue_struct *melfas_wq;
#endif

#ifdef FORCED_DOWNLOAD_OF_BLANKMEMORY
static bool bBlankMemory = false;
#endif

#if defined (__TOUCH_TA_CHECK__)		// for AT&T Charger
static u8 pre_charger_type = 0;
extern u8 g_charger_type;
//static u8 pre_charger_adc = 0;
extern u8 g_charger_adc;
#endif

static struct workqueue_struct *check_ic_wq;


/** functions **/
extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);

struct class *touch_class_melfas;
EXPORT_SYMBOL(touch_class_melfas);
struct device *firmware_dev_melfas;
EXPORT_SYMBOL(firmware_dev_melfas);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE
//static ssize_t raw_enable_tst200(struct device *dev, struct device_attribute *attr, char *buf);
//static ssize_t raw_disable_tst200(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t rawdata_pass_fail_tst200(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tkey_rawcounter_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tkey_rawcounter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t	check_init_lowleveldata(void);
#endif

#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE
static DEVICE_ATTR(firmware	, S_IRUGO | S_IWUSR | S_IWGRP, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret	, S_IRUGO | S_IWUSR | S_IWGRP, firmware_ret_show, firmware_ret_store);
//static DEVICE_ATTR(raw_enable, 0444, raw_enable_tst200, NULL) ;
//static DEVICE_ATTR(raw_disable, 0444, raw_disable_tst200, NULL) ;
static DEVICE_ATTR(raw_value, 0444, rawdata_pass_fail_tst200, NULL) ;
static DEVICE_ATTR(tkey_rawcounter, S_IRUGO | S_IWUSR | S_IWGRP, tkey_rawcounter_show, tkey_rawcounter_store);
#else
static DEVICE_ATTR(firmware	, 0444, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret	, 0444, firmware_ret_show, firmware_ret_store);
#endif
static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int melfas_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

#if defined (CONFIG_TOUCHSCREEN_TMA340) || defined (CONFIG_TOUCHSCREEN_TMA340_COOPERVE) || defined (CONFIG_TOUCHSCREEN_F760)
extern int Is_Synaptics_Connected(void);

int Is_MMS128_Connected(void)
{
    return (int) MMS128_Connected;
}
#endif

static int tsp_reset(void);

#if defined (__TOUCH_KEYLED__)
void touch_keyled_ctrl_regulator_mms128(int on_off)
{
	g_keyled_cnt =0;
	if (on_off == TOUCH_ON)
	{
		regulator_set_voltage(touchkeyled_regulator,3300000,3300000);
		regulator_enable(touchkeyled_regulator);
		g_check_keyled = true;
		
	}
	else
	{
		regulator_disable(touchkeyled_regulator);
		g_check_keyled = false;
	}
}
EXPORT_SYMBOL(touch_keyled_ctrl_regulator_mms128);
#endif

//TODO : touch_ctrl_regulator() Լ ٸ Ͽ ȵ, export Ǿ . //synaptics_i2c_rmi_tma340_cooperve.c synaptics_i2c_rmi_tma340_tassveve.c
void touch_ctrl_regulator_mms128(int on_off)
{
	if (on_off == TOUCH_ON)
	{
		regulator_set_voltage(touch_regulator, 2900000, 2900000);
		regulator_enable(touch_regulator);
	}
	else
	{
		regulator_disable(touch_regulator);
	}
#if defined (__TOUCH_KEYLED__) 	
	touch_keyled_ctrl_regulator_mms128(on_off);
#endif
}
EXPORT_SYMBOL(touch_ctrl_regulator_mms128);

int tsp_i2c_read_melfas(u8 reg, unsigned char *rbuf, int buf_size) //same with tsp_i2c_read()
{
	int i, ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;
  int retry = 3;

	for (i = 0; i < retry; i++)
	{
		rmsg.addr = ts->client->addr;
		rmsg.flags = 0;//I2C_M_WR;
		rmsg.len = 1;
		rmsg.buf = &start_reg;
		start_reg = reg;
		
		ret = i2c_transfer(ts->client->adapter, &rmsg, 1);

		if(ret >= 0) 
		{
			rmsg.flags = I2C_M_RD;
			rmsg.len = buf_size;
			rmsg.buf = rbuf;
			ret = i2c_transfer(ts->client->adapter, &rmsg, 1 );

			if (ret >= 0)
				break; // i2c success
		}

		if( i == (retry - 1) )
		{
			printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
		}
	}

	return ret;
}


static int melfas_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{
	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("[TSP][MMS128][%s] set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("[TSP][MMS128][%s] fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	return 0;
}

static int melfas_i2c_write(struct i2c_client* p_client, u8* data, int len)
{
	struct i2c_msg msg;

	msg.addr = p_client->addr;
	msg.flags = 0; /* I2C_M_WR */
	msg.len = len;
	msg.buf = data ;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("[TSP][MMS128][%s] set data pointer fail!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int melfas_init_panel(struct melfas_ts_data *ts)
{
	int ret;
	u8 buf = 0x10;
	ret = melfas_i2c_write(ts->client, &buf, 1);
	ret = melfas_i2c_write(ts->client, &buf, 1);

	if (ret < 0)
	{
		printk(KERN_DEBUG "[TSP][MMS128][%s] melfas_i2c_write() failed\n [%d]", __func__, ret);
		return 0;
	}

	return 1;
}

#if defined (__TOUCH_TA_CHECK__)		// for AT&T Charger

/*
typedef enum  {
	PMU_MUIC_CHGTYP_NONE,
	PMU_MUIC_CHGTYP_USB,
	PMU_MUIC_CHGTYP_DOWNSTREAM_PORT,
	PMU_MUIC_CHGTYP_DEDICATED_CHGR,
	PMU_MUIC_CHGTYP_SPL_500MA,
	PMU_MUIC_CHGTYP_SPL_1A,
	PMU_MUIC_CHGTYP_RESERVED,
	PMU_MUIC_CHGTYP_DEAD_BATT_CHG,

	PMU_MUIC_CHGTYP_INIT
}pmu_muic_chgtyp;
*/

/*
  0x01 : normal charger
  0x02 : AT&T charger
  0x00 : not connected,   ʿ
  0xff : need to check TA
*/

static bool b_Firmware_store = false;
u8 inform_ATcharger_connection(void)
{
	if(g_charger_adc == PMU_MUIC_ADC_OUTPUT_200K)
	{
        printk(KERN_DEBUG "[TSP][MMS128][%s] AT&T Charger \n [0x%x]", __func__, g_charger_adc);
		return true;
	}
	else
	{
        printk(KERN_DEBUG "[TSP][MMS128][%s] No AT&T Charger \n [0x%x]", __func__, g_charger_adc);
		return false;
	}
}

int inform_charger_connection(u8 state)
{
	int ret;
	u8 buf[2], temp_type;
	bool isATcharger = inform_ATcharger_connection();

	buf[0] = 0xAB;
	if( state == PMU_MUIC_CHGTYP_NONE )
		buf[1] = 0x00;
	else if ( (isATcharger == true) || (state == PMU_MUIC_CHGTYP_SPL_1A) )
		buf[1] = 0x02;
	else
		buf[1] = 0x01;

	temp_type = pre_charger_type;
	pre_charger_type = state;

	printk("[TSP][MMS128][%s] set : %d, %d !\n", __func__, state, buf[1]);	

	ret = melfas_i2c_write(ts->client, buf, 2);
	if(ret != 0)
	{
		printk(KERN_DEBUG "[TSP][MMS128][%s] melfas_i2c_write() failed\n [%d]", __func__, ret);
		pre_charger_type = temp_type;
	}

	return ret;

}

#endif

/* additional TS work */
static void check_ic_work_func_melfas(struct work_struct *work)
{
// for empty routine!!
#if !defined (__TOUCH_TA_CHECK__) && !defined (__TOUCH_KEYLED__)
	volatile int a;
	a=0;
#endif

#if defined (__TOUCH_TA_CHECK__)		// for AT&T Charger
	if(pre_charger_type != g_charger_type)
	{
		if( !b_Firmware_store )
			inform_charger_connection(g_charger_type);
	}
#endif

#if defined (__TOUCH_KEYLED__)		// auto-off LED 
	if(g_check_keyled)
	{
		if (g_check_action)
		{
			g_keyled_cnt=0;
			g_check_action=false;
		}
		else if(  g_keyled_cnt == 25 )
		{
			touch_keyled_ctrl_regulator_mms128(TOUCH_OFF);
			printk(KERN_DEBUG "[TSP] led off\n",__func__);			
		}
		else
			g_keyled_cnt++;
	}
#endif
	return;
}



static enum hrtimer_restart melfas_watchdog_timer_func(struct hrtimer *timer)
{
	queue_work(check_ic_wq, &ts->work_timer);

	hrtimer_start(&ts->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}



#if USE_THREADED_IRQ
static irqreturn_t melfas_ts_work_func(int irq, void *dev_id)
#else
static void melfas_ts_work_func(struct work_struct *work)
#endif
{
#if USE_THREADED_IRQ
	struct melfas_ts_data *ts = dev_id;
#else
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
#endif
	int ret = 0, i, j;
	uint8_t buf[66];
	int read_num = 0, touchType = 0, touchState = 0, fingerID = 0, keyID = 0;
	unsigned long flags;

#ifdef __TOUCH_KEYLED__
	bool b_keyledOn = false;
#endif

#ifdef __TOUCH_DEBUG__
	printk(KERN_DEBUG "[TSP][MMS128][%s] \n",__func__);
#endif
	if (ts == NULL)
		printk("[TSP][MMS128][%s] : TS NULL\n",__func__);

	/**
	Simple send transaction:
	S Addr Wr [A]  Data [A] Data [A] ... [A] Data [A] P
	Simple recv transaction:
	S Addr Rd [A]  [Data] A [Data] A ... A [Data] NA P
	*/
	for (i = 0; i < 10; i++)
	{
		ret = melfas_i2c_read(ts->client, 0x0F, buf, 1);
		if (ret >= 0)
			break; // i2c success
	}
	spin_lock_irqsave(&melfas_spin_lock, flags);
	if (ret < 0)
	{
		printk("[TSP][MMS128][%s] i2c failed : %d\n", __func__, ret);
		enable_irq(ts->client->irq);
		spin_unlock_irqrestore(&melfas_spin_lock, flags);
		touch_ctrl_regulator_mms128(TOUCH_OFF);
		touch_ctrl_regulator_mms128(TOUCH_ON);
		melfas_init_panel(ts);
#if USE_THREADED_IRQ
		return IRQ_HANDLED;
#else
		return ;
#endif
	}
	else
	{
		read_num = buf[0];
	}

	if (read_num > 0)
	{
		for (i = 0; i < 10; i++)
		{
			ret = melfas_i2c_read(ts->client, 0x10, buf, read_num);
			if (ret >= 0)
				break; // i2c success
		}

		if (ret < 0)
		{
			printk("[TSP][MMS128][%s] i2c failed : %d\n", __func__, ret);
			enable_irq(ts->client->irq);
			spin_unlock_irqrestore(&melfas_spin_lock, flags);
			touch_ctrl_regulator_mms128(TOUCH_OFF);
			touch_ctrl_regulator_mms128(TOUCH_ON);
			melfas_init_panel(ts);
#if USE_THREADED_IRQ
			return IRQ_HANDLED;
#else
			return ;
#endif
		}
		else
		{
			bool touched_src = false;

			if (buf[0] == 0x0f)
			{
				printk("[TSP][MMS128][%s] ESD defense!!  : %d\n", __func__, fingerID);
				enable_irq(ts->client->irq);
				spin_unlock_irqrestore(&melfas_spin_lock, flags);
				touch_ctrl_regulator_mms128(TOUCH_OFF);
				touch_ctrl_regulator_mms128(TOUCH_ON);
				melfas_init_panel(ts);
#if USE_THREADED_IRQ
				return IRQ_HANDLED;
#else
				return ;
#endif
			}
			
			for (i = 0; i < read_num; i = i + 6)
			{
				touchType = (buf[i] >> 5) & 0x03;
				touchState = (buf[i] & 0x80);

				if (touchType == 1)	//Screen
				{
					touched_src = true;
					fingerID = (buf[i] & 0x0F) - 1;

					if ((fingerID > TS_MAX_TOUCH - 1) || (fingerID < 0))
					{
						printk("[TSP][MMS128][%s] fingerID : %d\n", __func__, fingerID);
						enable_irq(ts->client->irq);
						spin_unlock_irqrestore(&melfas_spin_lock, flags);
						touch_ctrl_regulator_mms128(TOUCH_OFF);
						touch_ctrl_regulator_mms128(TOUCH_ON);
						melfas_init_panel(ts);
#if USE_THREADED_IRQ
						return IRQ_HANDLED;
#else
						return ;
#endif
					}

					g_Mtouch_info[fingerID].posX = (uint16_t)(buf[i + 1] & 0x0F) << 8 | buf[i + 2];
					g_Mtouch_info[fingerID].posY = (uint16_t)(buf[i + 1] & 0xF0) << 4 | buf[i + 3];
					g_Mtouch_info[fingerID].width = buf[i + 4];

					if (touchState)
						g_Mtouch_info[fingerID].strength = buf[i + 5];
					else
						g_Mtouch_info[fingerID].strength = 0;
				}
 #ifdef __TOUCH_KEY__
				else if (touchType == 2)	//Key
				{
					keyID = (buf[i] & 0x0F);

					if (keyID == 0x1)
						input_report_key(ts->input_dev, KEY_MENU, touchState ? PRESS_KEY : RELEASE_KEY);
					if (keyID == 0x2)
						input_report_key(ts->input_dev, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);

#ifdef __TOUCH_KEYLED__
					if( !g_check_keyled)
					{
						b_keyledOn = true;
					}
#endif

#ifdef __TOUCH_DEBUG__
					printk(KERN_DEBUG "[TSP][MMS128][%s] keyID: %d, State: %d\n", __func__, keyID, touchState);
#endif
				}
 #endif	//  __TOUCH_KEY__
			}
			if (touched_src)
			{
				for (j = 0; j < TS_MAX_TOUCH; j ++)
				{
					if (g_Mtouch_info[j].strength == -1)
						continue;

					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, j);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[j].posX);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[j].posY);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[j].strength);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[j].width);
					input_mt_sync(ts->input_dev);
#ifdef __TOUCH_DEBUG__
//					printk(KERN_DEBUG "[TSP][MMS128][%s] fingerID: %d, State: %d, x: %d, y: %d, z: %d, w: %d\n",
					printk(KERN_DEBUG "fingerID: %d, State: %d, x: %d, y: %d, z: %d, w: %d\n",
						j, (g_Mtouch_info[j].strength > 0), g_Mtouch_info[j].posX, g_Mtouch_info[j].posY, g_Mtouch_info[j].strength, g_Mtouch_info[j].width);
#endif
					if (g_Mtouch_info[j].strength == 0)
						g_Mtouch_info[j].strength = -1;
				}
			}				
			input_sync(ts->input_dev);

#ifdef __TOUCH_KEYLED__
			g_check_action = true;
#endif
		}
	}
#ifdef __TOUCH_DEBUG__	
	else
	{
		printk("[TSP][MMS128][%s] : read_num=%d\n",__func__, read_num);
	}
#endif

#if USE_THREADED_IRQ

#else
	enable_irq(ts->client->irq);
#endif

	spin_unlock_irqrestore(&melfas_spin_lock, flags);

#ifdef __TOUCH_KEYLED__
	if( b_keyledOn )
	{
		touch_keyled_ctrl_regulator_mms128(TOUCH_ON);
		msleep(70);
		printk(KERN_DEBUG "[TSP] led on\n",__func__);			
	} 
#endif

#if USE_THREADED_IRQ
	return IRQ_HANDLED;
#else

#endif
}


static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	//struct melfas_ts_data *ts = dev_id;

#if USE_THREADED_IRQ

#else
	disable_irq_nosync(ts->client->irq);
#endif

#if USE_THREADED_IRQ
	return IRQ_WAKE_THREAD;
#else
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
#endif
}


bool melfas_ts_upgrade_check(void)
{
/*********************************************/
#ifdef ALWAYS_DOWNLOAD
	return true;
#endif
/*********************************************/
#ifdef FORCED_DOWNLOAD_OF_BLANKMEMORY
	if(bBlankMemory) 
		return true;
#endif
/*********************************************/
#if SET_DOWNLOAD_BY_GPIO
#if defined(CONFIG_TOUCHSCREEN_MMS128_COOPERVE)
	if( ts->hw_ver==00)
	{
		if(ts->fw_ver < LATEST_FW_VER_HW00)
			return true;
	}
	else if(ts->hw_ver==01)
	{
		if(ts->fw_ver < LATEST_FW_VER_HW01)
			return true;
	}
	else
	{
		printk(KERN_DEBUG "[TSP][%s] wrong hw_ver\n", __func__);
	}
#elif defined(CONFIG_TOUCHSCREEN_MMS128_TASSVE)
	if( ts->fw_ver < LATEST_FW_VER)
		return true;
#else
#error "[TSP][Error] Melfas_ts_mms128_tasscooper.c : Feature missing!!"
#endif
#endif	// SET_DOWNLOAD_BY_GPIO
/*********************************************/

	return false;
}

void melfas_upgrade(INT32 hw_ver)
{
	int ret;
	unsigned char buf[2];

	printk("[TSP][MMS128][F/W D/L] Entry mcsdl_download_binary_data\n");
	printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", ts->hw_rev);
	printk("[TOUCH] Current F/W version: 0x%02x.\n", ts->fw_ver);	

	disable_irq(ts->client->irq);
	local_irq_disable();

	ret = mcsdl_download_binary_data(ts->hw_rev);

	local_irq_enable();
	enable_irq(ts->client->irq);

	printk("[TSP] melfas_upgrade()--  ret=%d\n", ret);

	if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf, 2))
	{
		ts->hw_rev = buf[0];
		ts->fw_ver = buf[1];
		printk("[TSP][MMS128][%s] HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
	}
	else
	{
		ts->hw_rev = 0;
		ts->fw_ver = 0;
		printk("[TSP][MMS128][%s] Can't find HW Ver, FW ver!\n", __func__);
	}

	if (ret > 0)
	{
		if ((ts->hw_rev < 0) || (ts->fw_ver < 0))
			printk(KERN_DEBUG "[TSP][MMS128][%s] i2c_transfer failed\n",__func__);
		else
			printk("[TSP][MMS128][%s] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", __func__, ts->hw_rev, ts->fw_ver);
	}
	else
	{
		printk("[TSP][MMS128][%s] Firmware update failed.. RESET!\n",__func__);
		mcsdl_vdd_off();
		mdelay(500);
		mcsdl_vdd_on();
		mdelay(200);
	}

	if(ret == MCSDL_RET_SUCCESS)
		firmware_ret_val = 1;
	else
		firmware_ret_val = 0;	

}

int melfas_ts_check(struct melfas_ts_data *ts)
{
	int ret, i;
	uint8_t buf_tmp[3]={0,0,0};
	int retry = 3;

	ret = tsp_i2c_read_melfas(0x1B, buf_tmp, sizeof(buf_tmp));		

	// i2c read retry
	if(ret <= 0)
	{
		for(i=0; i<retry;i++)
		{
			ret=tsp_i2c_read_melfas( 0x1B, buf_tmp, sizeof(buf_tmp));

			if(ret > 0)
				break;
		}
	}

	if (ret <= 0) 
	{
		printk("[TSP][MMS128][%s] %s\n", __func__,"Failed melfas i2c");
#if defined (CONFIG_TOUCHSCREEN_TMA340) || defined (CONFIG_TOUCHSCREEN_TMA340_COOPERVE) || defined (CONFIG_TOUCHSCREEN_F760)
		MMS128_Connected = 0;
#endif
	}
	else 
	{
		printk("[TSP][MMS128][%s] %s\n", __func__,"Passed melfas i2c");
#if defined (CONFIG_TOUCHSCREEN_TMA340) || defined (CONFIG_TOUCHSCREEN_TMA340_COOPERVE) || defined (CONFIG_TOUCHSCREEN_F760)
		MMS128_Connected = 1;
#endif
	}

	ts->VenderID = buf_tmp[0];
	ts->hw_rev = buf_tmp[1];
	ts->fw_ver = buf_tmp[2];	

	printk("[TSP][MMS128][%s][SlaveAddress : 0x%x][ret : %d] [ID : 0x%x] [HW : 0x%x] [SW : 0x%x]\n", __func__,ts->client->addr, ret, buf_tmp[0],buf_tmp[1],buf_tmp[2]);

	if( (ret > 0) && (ts->VenderID == 0xa0 ) )
	{
		ret = 1;
		printk("[TSP][MMS128][%s] %s\n", __func__,"Passed melfas_ts_check");
	}
#ifdef FORCED_DOWNLOAD_OF_BLANKMEMORY
	else if ( (ret > 0) && (ts->VenderID == 0x0)&& (ts->hw_rev == 0x0) && (ts->fw_ver == 0x0) )
	{
		ret = 1;
		bBlankMemory = true;
		printk("[TSP][MMS128][%s] %s\n", __func__,"Blank memory !!");
		printk("[TSP][MMS128][%s] %s\n", __func__,"Passed melfas_ts_check");
	}
#endif
	else
	{
		ts->hw_rev = 0;
		ts->fw_ver = 0;
		ret = 0;
		printk("[TSP][MMS128][%s] %s\n", __func__,"Failed melfas_ts_check");
	}

	return ret;
}

#ifdef SEC_TSP

/* Touch Reference ************************************************************/
static ssize_t raw_store_mms128(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	printk("[TSP] %s\n", __func__);

	if(strncasecmp(buf, "start", 5) == 0)
	{
		tsp_testmode = 1;
		printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);
	}
	else if(strncasecmp(buf, "stop", 4) == 0)
	{

		printk("[TSP] %s stop. line : %d, \n", __func__,__LINE__);
        
    		tsp_reset();
		msleep(300);    
		tsp_testmode = 0;
	}
      else
      {
      		printk("[TSP] %s error-unknown command. line : %d, \n", __func__,__LINE__);
      }
    return size ;
}

static ssize_t raw_show_mms128(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 setLowLevelData[2];
	u8 read_data_buf[50] = {0,};    

	u16 read_data_buf_rawdata[15][10] = {{0,},};
	u16 read_data_buf_sensitivitydata[17][10] = {{0,},};
	
	int read_data_len, sensing_ch;
	int ret, i,j;
	int written_bytes = 0 ; 

	tsp_testmode = 1;
	printk("%s : start \n", __func__);    

	disable_irq(ts->client->irq);

	check_init_lowleveldata();    

	read_data_len = g_exciting_ch * 2;
	sensing_ch	 = g_sensing_ch;
	
//////////////////////// Read garbage Data of 0xB2 Data buffer ///////////////////
	// Read Raw Data (1)
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x01;  // Raw Data
    
	printk("Read Raw Data (1) ++ \n");

	for(i = 0; i < sensing_ch; i++ ) 
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read rawdata_pass_fail_tst200 Data %dth\n", i);

		udelay(5);

		for(j = 0 ;j < read_data_len / 2; j++)
		{
			read_data_buf_rawdata[i][j] = (read_data_buf[j*2] <<8) + read_data_buf[j*2+1];
			printk(" %5d", read_data_buf_rawdata[i][j]);
		}
		printk("\n");
	}

	printk("Read Raw Data (1) --\n\n");
	msleep(50);
//////////////////////////////////////////////////////////////////////////////////
    // Read Raw Data (2)
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x01;  // Raw Data
    
	printk("Read Raw Data (2) ++ \n");

	for(i = 0; i < sensing_ch ; i++ )
	{
        
        ////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read rawdata_pass_fail_tst200 Data %dth\n", i);

		udelay(5);

		for(j = 0 ; j < read_data_len /2 ; j++)
		{
			read_data_buf_rawdata[i][j] = (read_data_buf[j*2] <<8) + read_data_buf[j*2+1];
			printk(" %5d", read_data_buf_rawdata[i][j]);
		}
	       printk("\n");
	} 

	printk("Read Raw Data (2) -- \n\n");
    	msleep(50);
//////////////////////// Read garbage Data of 0xB2 Data buffer ///////////////////

	printk("Read Intensity Data (1) ++ \n");

	read_data_len = g_exciting_ch + 2; // +2 menu, back key.
	sensing_ch	 = g_sensing_ch;
	
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x04;  // Intensity Data
	
	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		///////////////prppp///////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);

		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			printk("[%02d],",read_data_buf[j]);
		}
		printk("\n");
	}	
	
	printk("Read Intensity Data (1) -- \n\n");	
	msleep(50);	
//////////////////////////////////////////////////////////////////////////////////

	printk("Read Intensity Data (2) ++ \n");

	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x04;  // Intensity Data

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		///////////////prppp///////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);

		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			read_data_buf_sensitivitydata[i][j] = read_data_buf[j];
			printk("[%02d],",read_data_buf_sensitivitydata[i][j]);
		}
		printk("\n");
	}	
	
	printk("Read Intensity Data (2) -- \n");

//////////////////////////////////////////////////////////////////////////////////

	for (i = 0; i < g_sensing_ch ; i++)
	{
		for(j = 0 ; j < g_exciting_ch ; j++)
		{
			written_bytes += sprintf(buf+written_bytes, "%d %d\n", read_data_buf_rawdata[i][j], read_data_buf_sensitivitydata[i][j]) ;
		}
	}
//////////////////////////////////////////////////////////////////////////////////
	enable_irq(ts->client->irq);
	tsp_testmode = 0;
	tsp_reset();
	msleep(200);
    
	printk("%s : written_bytes=[%d] end \n", __func__, written_bytes);    
    
	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1");
}

static ssize_t reference_show_mms128(struct device *dev, struct device_attribute *attr, char *buf1)
{
	u8 setLowLevelData[2];
	u8 read_data_buf[50] = {0,};    
	u16 read_data_buf1[50] = {0,};
	int read_data_len,sensing_ch;
	int ret, i,j;

	tsp_testmode = 1;
    
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	disable_irq(ts->client->irq);

	ret = check_init_lowleveldata();

	read_data_len = g_exciting_ch * 2;
	sensing_ch	 = g_sensing_ch;


	/* Read Reference Data */
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x02;  // Reference Data

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);
        
		for(j = 0 ;j < read_data_len / 2; j++)
		{
			read_data_buf1[j] = (read_data_buf[j*2] <<8) + read_data_buf[j*2+1];
			printk(" %4d", read_data_buf1[j]);
		}

		printk("\n");
	}
    
	enable_irq(ts->client->irq);

	tsp_testmode = 0;

	tsp_reset();	
	
	return 0;
}

static ssize_t diff_show_mms128(struct device *dev, struct device_attribute *attr, char *buf1)
{
	u8 setLowLevelData[2] = {0x09, 0x04,};
	u8 read_data_buf[50] = {0,};    
	int read_data_len,sensing_ch;
	int ret, i,j;
	int menuKey, backKey;

	menuKey = 0;
	backKey = 0;
	disable_irq(ts->client->irq);

	ret = check_init_lowleveldata();

	////////////////////////
	// Writing Low Level Data(1)
	////////////////////////
//	ret = melfas_i2c_write(ts->client, setLowLevelData, 2);
	
	read_data_len = g_exciting_ch + 2; // +2 for key value
	sensing_ch	 = g_sensing_ch;

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);
		
		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			printk("[%02d],",read_data_buf[j]);
		}
		printk("\n");

	}	

	enable_irq(ts->client->irq);
	tsp_reset();
	msleep(100);
	
	printk("exciting_ch=[%d],sensing_ch=[%d],menuKey=[%02d],backKey=[%02d]\n", read_data_len, sensing_ch, menuKey, backKey) ;
	mdelay(1);   

	return 0;
}

static ssize_t set_tsp_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret, i;
	uint8_t buf_tmp[3]={0,0,0};
	int retry = 3;

	ret = tsp_i2c_read_melfas(0x1B, buf_tmp, sizeof(buf_tmp));		

	// i2c read retry
	if(ret <= 0)
	{
		for(i=0; i<retry;i++)
		{
			ret=tsp_i2c_read_melfas( 0x1B, buf_tmp, sizeof(buf_tmp));

			if(ret > 0)
				break;
		}
	}

	if(ret >0)
		printk("%s :VendorID:0x%02x, HW Ver:0x%02x, FW Ver:0x%02x\n", __func__, buf_tmp[0],buf_tmp[1],buf_tmp[2]);
	else
		return -1;

	ts->VenderID = buf_tmp[0];
	ts->hw_rev = buf_tmp[1];
	ts->fw_ver = buf_tmp[2];	

	return sprintf(buf, "%#02x\n", ts->fw_ver);
}

static ssize_t set_tsp_test_mode_enable0(struct device *dev, struct device_attribute *attr, char *buf)
{
	tsp_testmode = 1; 
	printk("set_tsp_test_mode_enable0 \n");
	return 0;
}

static ssize_t set_tsp_test_mode_disable0(struct device *dev, struct device_attribute *attr, char *buf)
{
	tsp_reset();
	
	msleep(300);    
	printk("set_tsp_test_mode_disable0 \n");
    
	tsp_testmode = 0;
    
	return 0;
}

static DEVICE_ATTR(reference,  S_IRUGO | S_IWUSR | S_IWGRP, reference_show_mms128, NULL) ;
static DEVICE_ATTR(raw,  S_IRUGO | S_IWUSR | S_IWGRP, raw_show_mms128, raw_store_mms128) ;
static DEVICE_ATTR(diff,  S_IRUGO | S_IWUSR | S_IWGRP, diff_show_mms128, NULL) ;
static DEVICE_ATTR(versname,  S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_read_show, NULL) ;
static DEVICE_ATTR(enable0,  S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_test_mode_enable0, NULL) ;
static DEVICE_ATTR(disable0,  S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_test_mode_disable0, NULL) ;

static ssize_t set_tsp_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	static const int NEW_FIRMWARE_VERSION = 0x11; // 17 ?
	
	// Read TSP version of Phone
	return sprintf(buf, "%d\n", NEW_FIRMWARE_VERSION);// kernel source version
}

static ssize_t set_tsp_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct mcs8000_ts_driver *ts = dev_get_drvdata(dev);
	u8 threshold=32;

	return sprintf(buf, "%d\n", threshold);
}

static ssize_t tsp_touchtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char temp[15];

//	sprintf(temp, "MELFAS,MMS136\n");
	sprintf(temp, "MMS128\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_show, NULL);/* PHONE*/	/* firmware version resturn in phone driver version */
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_read_show, NULL);/*PART*/	/* firmware version resturn in TSP panel version */
static DEVICE_ATTR(tsp_threshold, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_threshold_mode_show, NULL);
static DEVICE_ATTR(mxt_touchtype, S_IRUGO | S_IWUSR | S_IWGRP,	tsp_touchtype_show, NULL);

static struct attribute *sec_touch_attributes[] = {
	&dev_attr_tsp_firm_version_phone.attr,
	&dev_attr_tsp_firm_version_panel.attr,
	&dev_attr_tsp_threshold.attr,
	&dev_attr_mxt_touchtype.attr,
	NULL,
};

static struct attribute_group sec_touch_attr_group = {
	.attrs = sec_touch_attributes,
};

#endif //SEC_TSP


#ifdef TSP_FACTORY_TEST
static bool debug_print = true;
static u16 inspection_data[180] = { 0, };
static u16 lntensity_data[180] = { 0, };
static u16 CmDelta_data[176] = { 0, };

static int atoi(char *str)
{
	int result = 0;
	int count = 0;
	if( str == NULL ) 
		return -1;
	while( str[count] != NULL && str[count] >= '0' && str[count] <= '9' )
	{		
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
}

static void check_intensity_data(struct melfas_ts_data *ts)
{
   	u8 setLowLevelData[2];
	u8 read_data_buf[50] = {0,};    
	u16 read_data_buf1[50] = {0,};
	int read_data_len,sensing_ch;
	int ret, i,j;

	tsp_testmode = 1;
    
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	disable_irq(ts->client->irq);

	ret = check_init_lowleveldata();

	read_data_len = g_exciting_ch * 2;
	sensing_ch	 = g_sensing_ch;


	/* Read Reference Data */
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x02;  // Reference Data

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);
        
		for(j = 0 ;j <g_exciting_ch; j++)
			inspection_data[(i * g_exciting_ch) + j] = (read_data_buf[j*2] << 8) + read_data_buf[j*2+1];

	}
    
#if 1
	printk("[TSP] inspection_data\n");
	
	for (i = 0; i < g_exciting_ch * g_sensing_ch; i++) {
		if (0 == i % g_exciting_ch)
			printk("\n");
		printk("0x%4d, ", inspection_data[i]);
	}
	printk("\n\n");
#endif
	msleep(50);

/////////////////////////////////////////////////////////////////////////////////////////////////
	/* Read Dummy Data */
	printk("Read Dummy Data : START \n");

	read_data_len = g_exciting_ch + 2; // +2 menu, back key.
	sensing_ch	 = g_sensing_ch;

	/* Read Intensity Data */	
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x04;  // Intensity Data
	
	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		///////////////prppp///////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);
#if 1
		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			printk("[%03d],",read_data_buf[j]);
		}
		printk("\n");
#endif		
	}	
	
	printk("Read Dummy Data : END \n\n");
	msleep(50);
/////////////////////////////////////////////////////////////////////////////////////////////////

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		///////////////prppp///////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);

		for(j=0;j<g_exciting_ch;j++)
			lntensity_data[(i * g_exciting_ch) + j] = read_data_buf[j];

#if 1
		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			printk("[%02d],",read_data_buf[j]);
		}
		printk("\n");
#endif	

	}	

#if 0
	printk("[TSP] lntensity_data\n");
	
	for (i = 0; i < g_exciting_ch*sensing_ch; i++) {
		if (0 == i % g_exciting_ch)
			printk("\n");
//		printk("0x%02x, ", lntensity_data[i]);
		printk("%02d, ", lntensity_data[i]);
	}
	printk("\n");
#endif 

	enable_irq(ts->client->irq);
	tsp_testmode = 0;
	tsp_reset();
	msleep(10);
	
	printk("%s : end \n", __func__);

}

static ssize_t set_tsp_module_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;	

	melfas_ts_late_resume(NULL);
	msleep(10);
	ret = 0;
	
	if (ret  == 0)
		*buf = '1';
	else
		*buf = '0';

	return 0;
}

static ssize_t set_tsp_module_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;

	melfas_ts_early_suspend(NULL);
	msleep(10);
	ret = 0;
	
	if (ret  == 0)
		*buf = '1';
	else
		*buf = '0';

	return 0;
}

static ssize_t set_intensity4_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[155];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_refer4_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[155];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_intensity3_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[25];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_refer3_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[25];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_intensity2_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[97];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_refer2_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[97];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_intensity1_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[144];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_refer1_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[144];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_intensity0_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[14];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_refer0_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	//struct mcs8000_ts_driver *ts = dev_get_drvdata(dev);
    
	check_intensity_data(ts);

	refrence = inspection_data[14];
	return sprintf(buf, "%u\n", refrence);
}

ssize_t disp_all_deltadata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("disp_all_deltadata_show : value %d\n", CmDelta_data[index]);
	return sprintf(buf, "%u\n",  CmDelta_data[index]);
}

ssize_t disp_all_deltadata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index = atoi(buf);
	printk(KERN_ERR "Delta data %d", index);
  	return size;
}

static void check_delta_data(struct melfas_ts_data *ts)
{
	u8 read_buf[2] = {0,};
	u8 setLowLevelData[4];
	u8 read_data_buf[50] = {0,};    
	u16 read_data_buf1[50] = {0,};
	int read_data_len,sensing_ch, exciting_ch;
	int ret, i,j;

	tsp_testmode = 1;
    
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	disable_irq(ts->client->irq);

	ret = check_init_lowleveldata();

	exciting_ch	 = g_exciting_ch;
	sensing_ch	 = g_sensing_ch;

	for(i = 0; i < sensing_ch; i++ )
	{
		for (j = 0; j < exciting_ch; j++)
		{
			setLowLevelData[0] = RMI_ADDR_TESTMODE; 
			setLowLevelData[1] = 0x42; //cmd id
			setLowLevelData[2] = j; //TX CH.(exciting ch.)
			setLowLevelData[3] = i; //RX CH.(sensing ch.)
            
			ret = melfas_i2c_write(ts->client, setLowLevelData, 4);
            
			//////////////////////
			// Checking INT
			///////////////prppp///////
			while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
			{
				udelay(50);
			}

			udelay(300);

			ret = melfas_i2c_read(ts->client, UNIVERSAL_CMD_RESULT_SIZE, read_buf, 1);

			if(ret < 0)
			{
				printk("can't read UNIVERSAL_CMD_RESULT_SIZE \n", i);
				return ;
			}
			
			read_data_len = read_buf[0];
//			printk("read_data_len = 0x%02x \n ", read_data_len);
			
//			mdelay(100);
//			mdelay(10);

			ret = melfas_i2c_read(ts->client, UNIVERSAL_CMD_RESULT, read_data_buf, read_data_len);
			
	            	if(ret < 0)
			{
				printk("can't read UNIVERSAL_CMD_RESULT Data \n", i);
				return ;
			}
					
//			printk("[0] = 0x%02x, [1] = 0x%02x \n", read_data_buf[0], read_data_buf[1]);

			CmDelta_data[(i * exciting_ch) + j] = (read_data_buf[1] << 8) + read_data_buf[0];

		}

	}	

#if 0
	printk("[TSP] CmDelta_data\n");
	
	for (i = 0; i < g_exciting_ch*g_exciting_ch; i++) {
		if (0 == i % g_exciting_ch)
			printk("\n");
		printk("0x%04x, ", CmDelta_data[i]);
	}
	printk("\n");
#endif 

	enable_irq(ts->client->irq);
	tsp_testmode = 0;
	tsp_reset();
	msleep(10);
	printk("%s : end \n", __func__);      
}


static int check_debug_data(struct melfas_ts_data *ts)
{
	u8 setLowLevelData[2];
	u8 read_data_buf[50] = {0,};    
	u16 read_data_buf1[50] = {0,};
	int read_data_len,sensing_ch;
	int ret, i,j, status;

	tsp_testmode = 1;
    
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	disable_irq(ts->client->irq);

	ret = check_init_lowleveldata();

	read_data_len = g_exciting_ch * 2;
	sensing_ch	 = g_sensing_ch;


	/* Read Reference Data */
	setLowLevelData[0] = 0x09;  // Low level data output mode 
	setLowLevelData[1] = 0x02;  // Reference Data

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);
         
		for(j = 0 ;j <g_exciting_ch; j++)
		{
			inspection_data[(i * g_exciting_ch) + j] = (read_data_buf[j*2] << 8) + read_data_buf[j*2+1];
            
			if ((inspection_data[(i * g_exciting_ch) + j] >= SCR_ABS_LOWER_SPEC[i][j])
				&& (inspection_data[(i * g_exciting_ch) + j] <= SCR_ABS_UPPER_SPEC[i][j]))
				status = 0;
			else 
				status = 1;
		}
        }

#if 1
	printk("[TSP] inspection_data\n");
	
	for (i = 0; i < g_exciting_ch * g_sensing_ch; i++) {
		if (0 == i % g_exciting_ch)
			printk("\n");
//		printk("0x%4x, ", inspection_data[i]);
		printk("%4d, ", inspection_data[i]);
	}
	printk("\n");
#endif

	enable_irq(ts->client->irq);
	tsp_testmode = 0;
	tsp_reset();
	msleep(100);
	
	printk("%s : end \n", __func__);

	return status;
}

static ssize_t set_all_delta_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;

	check_delta_data(ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);

	return sprintf(buf, "%u\n", status);
}

static ssize_t set_all_refer_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;

	status = check_debug_data(ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);

	return sprintf(buf, "%u\n", status);
}

ssize_t disp_all_refdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n",  inspection_data[index]);
}

ssize_t disp_all_refdata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index = atoi(buf);

	printk(KERN_ERR "%s : value %d\n", __func__, index);

  	return size;
}

ssize_t disp_all_intendata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	index = atoi(buf);
	printk(KERN_ERR "Intensity data %d", index);
  	return size;
}

ssize_t disp_all_intendata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("disp_all_intendata_show : value %d, index=%d\n", lntensity_data[index], index);
    return sprintf(buf, "%u\n",  lntensity_data[index]);
}

static ssize_t set_all_intensity_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;

	check_intensity_data(ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);

	return sprintf(buf, "%u\n", status);
}

static DEVICE_ATTR(set_module_on, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_module_on_show, NULL);
static DEVICE_ATTR(set_module_off, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_module_off_show, NULL);
static DEVICE_ATTR(set_all_refer, S_IRUGO | S_IWUSR | S_IWGRP, set_all_refer_mode_show, NULL);
#if 0  // Change authority for CTS, but if you want to make LCD TEST MODE and then enable.
static DEVICE_ATTR(disp_all_refdata, S_IRWXU|S_IRWXG|S_IRWXO, disp_all_refdata_show, disp_all_refdata_store);
#else
static DEVICE_ATTR(disp_all_refdata, S_IRUGO|S_IWUSR|S_IWGRP, disp_all_refdata_show, disp_all_refdata_store);
#endif
static DEVICE_ATTR(set_all_delta, S_IRUGO | S_IWUSR | S_IWGRP, set_all_delta_mode_show, NULL);
static DEVICE_ATTR(disp_all_deltadata, S_IRUGO | S_IWUSR | S_IWGRP, disp_all_deltadata_show, disp_all_deltadata_store);
static DEVICE_ATTR(set_all_intensity, S_IRUGO | S_IWUSR | S_IWGRP, set_all_intensity_mode_show, NULL);
static DEVICE_ATTR(disp_all_intendata, S_IRUGO | S_IWUSR | S_IWGRP, disp_all_intendata_show, disp_all_intendata_store);
static DEVICE_ATTR(set_refer0, S_IRUGO | S_IWUSR | S_IWGRP, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO | S_IWUSR | S_IWGRP, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO | S_IWUSR | S_IWGRP, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO | S_IWUSR | S_IWGRP, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO | S_IWUSR | S_IWGRP, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity4_mode_show, NULL);
static DEVICE_ATTR(set_threshould, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_threshold_mode_show, NULL);	/* touch threshold return */

static struct attribute *sec_touch_facotry_attributes[] = {
	&dev_attr_set_module_on.attr,
	&dev_attr_set_module_off.attr,
	&dev_attr_set_all_refer.attr,
	&dev_attr_disp_all_refdata.attr,
	&dev_attr_set_all_delta.attr,
	&dev_attr_disp_all_deltadata.attr,
	&dev_attr_set_all_intensity.attr,
	&dev_attr_disp_all_intendata.attr,
	&dev_attr_set_refer0.attr,
	&dev_attr_set_delta0.attr,
	&dev_attr_set_refer1.attr,
	&dev_attr_set_delta1.attr,
	&dev_attr_set_refer2.attr,
	&dev_attr_set_delta2.attr,
	&dev_attr_set_refer3.attr,
	&dev_attr_set_delta3.attr,
	&dev_attr_set_refer4.attr,
	&dev_attr_set_delta4.attr,
	&dev_attr_set_threshould.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};
#endif //TSP_FACTORY_TEST

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, i;

#if defined (CONFIG_TOUCHSCREEN_TMA340) || defined (CONFIG_TOUCHSCREEN_TMA340_COOPERVE) || defined (CONFIG_TOUCHSCREEN_F760)
	printk("[TSP][MMS128][%s] %s\n", __func__, "Called");
	if (Is_Synaptics_Connected() == 1)
	{
		printk("[TSP][MMS128][%s] %s\n", __func__, "Synaptics already detected !!");
		return -ENXIO;
	}
#endif

	printk(KERN_DEBUG"+-----------------------------------------+\n");
	printk(KERN_DEBUG "|  Melfas Touch Driver Probe!            |\n");
	printk(KERN_DEBUG"+-----------------------------------------+\n");


	/* Pin Initialize ****************************************************/
	//disable BB internal pulls for touch int, scl, sda pin
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, 0);
	bcm_gpio_pull_up_down_enable(GPIO_TSP_SCL, 0);
	bcm_gpio_pull_up_down_enable(GPIO_TSP_SDA, 0);
	
	//gpio_direction_output( GPIO_TOUCH_INT , 0 );
	gpio_direction_output( GPIO_TSP_SCL , 0 ); 
	gpio_direction_output( GPIO_TSP_SDA , 0 ); 

	gpio_request(GPIO_TOUCH_INT, "ts_irq");
	gpio_direction_input(GPIO_TOUCH_INT);
	set_irq_type(GPIO_TO_IRQ(GPIO_TOUCH_INT), IRQF_TRIGGER_FALLING);	
	
	bcm_gpio_pull_up(GPIO_TOUCH_INT, false);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);

	//touch_ctrl_regulator_mms128(TOUCH_OFF);

	/****************************************************************/	

	msleep(100);

	gpio_direction_output( GPIO_TSP_SCL , 1 ); 
	gpio_direction_output( GPIO_TSP_SDA , 1 ); 
	//gpio_direction_output( TSP_INT , 1 ); 

	gpio_direction_input(GPIO_TOUCH_INT);
	bcm_gpio_pull_up(GPIO_TOUCH_INT, true);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);

#ifdef DELAY_BEFORE_VDD
	msleep(10);
#endif
	touch_ctrl_regulator_mms128(TOUCH_ON);
	msleep(70);
	/****************************************************************/
	ts = kzalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL)
	{
		printk(KERN_DEBUG "[TSP][MMS128][%s] failed to create a state of melfas-ts\n",__func__);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

#if USE_THREADED_IRQ

#else
	INIT_WORK(&ts->work, melfas_ts_work_func);
#endif

	INIT_WORK(&ts->work_timer, check_ic_work_func_melfas);


	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* Melfas TSP check routine */
	ret = melfas_ts_check(ts);
	if (ret <= 0)
	{
		i2c_release_client(client);	

		gpio_direction_output( GPIO_TOUCH_INT , 0 );
		gpio_direction_output( GPIO_TSP_SCL , 0 ); 
		gpio_direction_output( GPIO_TSP_SDA , 0 ); 

		bcm_gpio_pull_up(GPIO_TOUCH_INT, false);
		bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);
		
		touch_ctrl_regulator_mms128(TOUCH_OFF); //Melfas  TOUCH_OFF ϸ, Cypress  OFF ȴ.
		kfree(ts);

		return -ENXIO;
	}
	/* ~Melfas TSP check routine */

	/* sys fs */
	touch_class_melfas = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class_melfas))
		pr_err("[TSP][MMS128] Failed to create class(touch)!\n");
	firmware_dev_melfas = device_create(touch_class_melfas, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev_melfas))
		pr_err("[TSP][MMS128] Failed to create device(firmware)!\n");
	if (device_create_file(firmware_dev_melfas, &dev_attr_firmware) < 0)
		pr_err("[TSP][MMS128] Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_firmware_ret) < 0)
		pr_err("[TSP][MMS128] Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
	
#ifdef 	CONFIG_TOUCHSCREEN_MMS128_COOPERVE
/*
	if (device_create_file(firmware_dev_melfas, &dev_attr_raw_enable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_enable.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_raw_disable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_disable.attr.name);
*/	
	if (device_create_file(firmware_dev_melfas, &dev_attr_raw_value) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_value.attr.name);	
	if (device_create_file(firmware_dev_melfas, &dev_attr_tkey_rawcounter) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_rawcounter.attr.name);
	/* ~sys fs */
#endif

#ifdef SEC_TSP
	if (device_create_file(firmware_dev_melfas, &dev_attr_versname) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_versname.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_reference) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_reference.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_raw) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_diff) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_diff.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_enable0) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_enable0.attr.name);
	if (device_create_file(firmware_dev_melfas, &dev_attr_disable0) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_disable0.attr.name);
	
	sec_touchscreen = device_create(touch_class_melfas, NULL, 0, NULL, "sec_touchscreen");
	if (IS_ERR(sec_touchscreen))
		pr_err("[TSP] Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&sec_touchscreen->kobj, &sec_touch_attr_group);
	if (ret)
		pr_err("[TSP] Failed to create sysfs group\n");
    
#ifdef TSP_FACTORY_TEST
	qt602240_noise_test = device_create(touch_class_melfas, NULL, 0, NULL, "qt602240_noise_test");

	if (IS_ERR(qt602240_noise_test))
		pr_err("[TSP] Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&qt602240_noise_test->kobj, &sec_touch_factory_attr_group);
	if (ret)
		pr_err("[TSP] Failed to create sysfs group\n");
#endif
#endif // SEC_TSP

	if ( melfas_ts_upgrade_check() )
	{
		melfas_upgrade(ts->hw_rev);
		msleep(1);
		touch_ctrl_regulator_mms128(TOUCH_OFF);
		msleep(1000);
		touch_ctrl_regulator_mms128(TOUCH_ON);
		msleep(70);
		//local_irq_disable();
		//mcsdl_download_binary_data();
		//printk("[TSP] enable_irq : %d\n", __LINE__);
		//local_irq_enable();
	}

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev)
	{
		printk(KERN_DEBUG "[TSP][MMS128][%s] Failed to allocate input device\n",__func__);
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "sec_touchscreen" ;

	/** Event handler  ************************************************/
#if defined(CONFIG_TOUCHSCREEN_MMS128_COOPERVE) //CooperVE
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, TS_MAX_TOUCH - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);

	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	ts->input_dev->keybit[BIT_WORD(KEY_POWER)] |= BIT_MASK(KEY_POWER);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);	
#elif defined(CONFIG_TOUCHSCREEN_MMS128_TASSVE) //TassVE
    ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
    ts->input_dev->keybit[BIT_WORD(KEY_POWER)] |= BIT_MASK(KEY_POWER);

    set_bit(BTN_TOUCH, ts->input_dev->keybit);
    set_bit(EV_ABS,  ts->input_dev->evbit);
    ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 240 /*TS_MAX_X_COORD*/, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 320 /*TS_MAX_Y_COORD*/, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, TS_MAX_TOUCH - 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);

    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);
#else
#error "[TSP][Error] Melfas_ts_mms128_tasscooper.c : Feature missing!!"
#endif
	/*******************************************************************/

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		printk(KERN_DEBUG "[TSP][MMS128][%s] Unable to register %s input device\n", __func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk("[TSP][MMS128][%s] irq=%d\n", __func__, client->irq);

#if 0
	gpio_request(GPIO_TOUCH_INT, "ts_irq");
	gpio_direction_input(GPIO_TOUCH_INT);
	bcm_gpio_pull_up(GPIO_TOUCH_INT, true);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);
	set_irq_type(GPIO_TO_IRQ(GPIO_TOUCH_INT), IRQF_TRIGGER_FALLING);
#endif

	if (ts->client->irq)
	{
		printk(KERN_DEBUG "[TSP][MMS128][%s] trying to request irq: %s-%d\n", __func__, ts->client->name, ts->client->irq);
#if USE_THREADED_IRQ
		ret = request_threaded_irq(ts->client->irq, melfas_ts_irq_handler, melfas_ts_work_func, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts->client->name, ts);
#else
		ret = request_irq(ts->client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);
#endif
		if (ret > 0)
		{
			printk(KERN_DEBUG "[TSP][MMS128][%s] Can't allocate irq %d, ret %d\n", __func__, ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	//schedule_work(&ts->work);
	//queue_work(melfas_wq, &ts->work);

	for (i = 0; i < TS_MAX_TOUCH ; i++)
		g_Mtouch_info[i].strength = -1;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = melfas_watchdog_timer_func;
	hrtimer_start(&ts->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "[TSP][MMS128][%s] Start touchscreen. name: %s, irq: %d\n", __func__, ts->client->name, ts->client->irq);

	return 0;

#if 1
err_request_irq:
	printk(KERN_DEBUG "[TSP][MMS128][%s] err_request_irq failed\n",__func__);
	free_irq(client->irq, ts);
#endif
err_input_register_device_failed:
	printk(KERN_DEBUG "[TSP][MMS128][%s] err_input_register_device failed\n",__func__);
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_DEBUG "[TSP][MMS128][%s] err_input_dev_alloc failed\n",__func__);
err_alloc_data_failed:
	printk(KERN_DEBUG "[TSP][MMS128][%s] err_alloc_data failed_\n",__func__);
#if 0
err_detect_failed:
	printk(KERN_DEBUG "melfas-ts: err_detect failed\n");
	kfree(ts);
err_check_functionality_failed:
	printk(KERN_DEBUG "melfas-ts: err_check_functionality failed_\n");
#endif
	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP][MMS128] %s+", __func__);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	printk("[TSP][MMS128] %s-", __func__);
	
	return 0;
}

static void release_all_fingers(struct melfas_ts_data *ts)
{
	int i;

	printk("[TSP][MMS128][%s] \n", __FUNCTION__);

	for (i = 0; i < TS_MAX_TOUCH; i++)
	{
		if (g_Mtouch_info[i].strength == -1)
			continue;

		g_Mtouch_info[i].strength = 0;
		g_Mtouch_info[i].width = 0;
		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;

		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
		input_mt_sync(ts->input_dev);

		if (g_Mtouch_info[i].strength == 0)
			g_Mtouch_info[i].strength = -1;
	}
	input_sync(ts->input_dev);
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	release_all_fingers(ts);

	disable_irq(client->irq);

	printk("[TSP][MMS128][%s] irq=%d\n", __func__, client->irq);

	ret = cancel_work_sync(&ts->work);

	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	msleep(70);

	melfas_init_panel(ts);
	enable_irq(client->irq); // scl wave

	printk("[TSP][MMS128][%s] irq=%d\n", __func__, client->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	int ret;

	printk("[TSP][MMS128][%s] ++ \n", __FUNCTION__);
	//release_all_fingers(ts);

	disable_irq(ts->client->irq);

	ret=cancel_work_sync(&ts->work_timer);

	ret=cancel_work_sync(&ts->work);
	if(ret)	 /* if work was pending disable-count is now 2 */
	{
		printk("[TSP]ck\n");	// temporary comment
		enable_irq(ts->client->irq);
	}

	hrtimer_cancel(&ts->timer);

	gpio_direction_output( GPIO_TOUCH_INT , 0 );
	gpio_direction_output( GPIO_TSP_SCL , 0 ); 
	gpio_direction_output( GPIO_TSP_SDA , 0 ); 

	bcm_gpio_pull_up(GPIO_TOUCH_INT, false);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);

	touch_ctrl_regulator_mms128(TOUCH_OFF);

	release_all_fingers(ts);	// position is important!!

	printk("[TSP][MMS128][%s] -- \n", __FUNCTION__);	
	//ts = container_of(h, struct melfas_ts_data, early_suspend);
	//melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	printk("[TSP][MMS128][%s] ++ \n", __FUNCTION__);

	gpio_direction_output( GPIO_TSP_SCL , 1 ); 
	gpio_direction_output( GPIO_TSP_SDA , 1 ); 
	//gpio_direction_output( TSP_INT , 1 ); 

	gpio_direction_input(GPIO_TOUCH_INT);
	bcm_gpio_pull_up(GPIO_TOUCH_INT, true);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);

#ifdef DELAY_BEFORE_VDD
	msleep(5);
#endif

	touch_ctrl_regulator_mms128(TOUCH_ON);
	msleep(50);

	pre_charger_type = 0xff;
	inform_charger_connection(g_charger_type);	

	enable_irq(ts->client->irq);
	//ts = container_of(h, struct melfas_ts_data, early_suspend);
	//melfas_ts_resume(ts->client);

	hrtimer_start(&ts->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);

	printk("[TSP][MMS128][%s] -- \n", __FUNCTION__);	
}
#endif

static const struct i2c_device_id melfas_ts_id[] =
{
	{ MELFAS_TS_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver =
{
	.probe	 = melfas_ts_probe,
	.remove = __devexit_p(melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= melfas_ts_suspend,
	.resume	= melfas_ts_resume,
#endif
	.id_table	= melfas_ts_id,
	.driver = 	{
		.name	= MELFAS_TS_NAME,
	},
};

static int __devinit melfas_ts_init(void)
{
	printk("[TSP] %s\n", __func__ );
#if defined (CONFIG_TOUCHSCREEN_TMA340) || defined (CONFIG_TOUCHSCREEN_TMA340_COOPERVE) || defined (CONFIG_TOUCHSCREEN_F760)

	if (Is_Synaptics_Connected() == 1)
	{
		printk("[TSP][MMS128][%s] %s\n", __func__, "Synaptics already detected !!");
		return -ENXIO;
	}
#endif

#if USE_THREADED_IRQ

#else	
	melfas_wq = create_workqueue("melfas_wq");
	if (!melfas_wq)
		return -ENOMEM;
#endif

	check_ic_wq = create_workqueue("check_ic_wq");
	if(!check_ic_wq)
		return -ENOMEM;

	touch_regulator = regulator_get(NULL, "touch_vcc");

#if defined (__TOUCH_KEYLED__)
	touchkeyled_regulator = regulator_get(NULL,"touch_keyled");
#endif

	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	printk("[TSP] %s\n", __func__ );

	if (touch_regulator)
	{
		regulator_put(touch_regulator);
		touch_regulator = NULL;
	}


#if defined (__TOUCH_KEYLED__)
	if (touchkeyled_regulator) 
	{
		regulator_put(touchkeyled_regulator);
		touchkeyled_regulator = NULL;
	}
#endif

	i2c_del_driver(&melfas_ts_driver);

#if USE_THREADED_IRQ

#else
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
#endif
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	u8 buf1[2] = {0,};
	int hw_rev, fw_ver, phone_ver;
#if defined(CONFIG_TOUCHSCREEN_MMS128_COOPERVE)
	const int temp_hw_rev =0x01;
#endif

	if (0 == melfas_i2c_read(ts->client, MCSTS_MODULE_VER_REG, buf1, 2))
	{
		hw_rev = buf1[0];
		fw_ver = buf1[1];

#if defined(CONFIG_TOUCHSCREEN_MMS128_COOPERVE)
		if(hw_rev==0x00)
			phone_ver = LATEST_FW_VER_HW00;
		else if(hw_rev==0x01)
			phone_ver = LATEST_FW_VER_HW01;
		else
			phone_ver = 0x00;

		hw_rev = temp_hw_rev;	// 2011.11.02 for viewing to screen
#elif defined(CONFIG_TOUCHSCREEN_MMS128_TASSVE)
		phone_ver = LATEST_FW_VER;
#else
#error "[TSP][Error] Melfas_ts_mms128_tasscooper.c : Feature missing!!"
#endif

		/*
			TSP phone Firmware version : phone_ver (xx)
			TSP IC	  Firmware version   : fw_ver (xx)
			HW 				  version	  : hw_rev  (xxx)
		*/

		sprintf(buf, "%03X%02X%02X\n", hw_rev, fw_ver, phone_ver); // 10003xx

		printk("[TSP][MMS128][%s]  phone_ver=%d, fw_ver=%d, hw_rev=%d\n",buf, phone_ver,fw_ver, hw_rev );

	}
	else
	{	 
		printk("[TSP][MMS128][%s] Can't find HW Ver, FW ver!\n", __func__);
	}

	return sprintf(buf, "%s", buf); 
}

static ssize_t firmware_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	char *after;
	int ret;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	firmware_ret_val = -1;

	printk("[TSP] firmware_store  value : %ld\n",value);
	if ( value == 1 )
	{
		printk("[TSP] Firmware update start!!\n" );

		//firm_update( );
		//	  melfas_upgrade(ts->hw_rev);
#if defined (__TOUCH_TA_CHECK__)		// for AT&T Charger
		b_Firmware_store = true;
#endif

		disable_irq(ts->client->irq);
		local_irq_disable();
	
		cancel_work_sync(&ts->work_timer);
		hrtimer_cancel(&ts->timer);

		ret = mms100_ISC_download_binary_data(ts->hw_rev);

		local_irq_enable();
		enable_irq(ts->client->irq);

		hrtimer_start(&ts->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);

#if defined (__TOUCH_TA_CHECK__)		// for AT&T Charger
		b_Firmware_store = false;
#endif

		if(ret == MCSDL_RET_SUCCESS)
			firmware_ret_val = 1;
		else
			firmware_ret_val = 0;	

		printk("[TSP] Firmware update end!!\n" );		

		return size;
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk("[TSP][MMS128][%s] !\n", __func__);

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	printk("[TSP][MMS128][%s] operate nothing!\n", __func__);

	return size;
}
/*
#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE
static ssize_t raw_enable_tst200(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, ret;
	uint8_t buf1[2] = {0,};
#if 0
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x70;//value
		ret = i2c_master_send(ts->client, buf1, 2);	//enter Inspection Mode
		if (ret >= 0)
			break; // i2c success
	}
	
	tsp_testmode = 1;
	printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);

	mdelay(300); 
#endif
	return 1;
}
#endif


#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE
static ssize_t raw_disable_tst200(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, ret;
	uint8_t buf1[2] = {0,};

#if 0	
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts->client, buf1, 2);	//exit Inspection Mode

		if (ret >= 0)
			break; // i2c success
	}
	
	tsp_testmode = 0;
	printk("[TSP] %s stop. line : %d, \n", __func__,__LINE__);
#endif
	return 1;
}
#endif
*/

#ifdef CONFIG_TOUCHSCREEN_MMS128_COOPERVE
static ssize_t rawdata_pass_fail_tst200(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 setLowLevelData[2] = {0x09, 0x01,};
	u8 read_data_buf[50] = {0,};    
	u16 read_data_buf1[50] = {0,};    
	int read_data_len,sensing_ch;
	int ret, i,j;

	tsp_testmode = 1;

	disable_irq(ts->client->irq);

	check_init_lowleveldata();

	read_data_len = g_exciting_ch * 2;
	sensing_ch	 = g_sensing_ch;

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read rawdata_pass_fail_tst200 Data %dth\n", i);

		udelay(5);

		for(j = 0 ;j < read_data_len / 2; j++)
		{
			read_data_buf1[j] = (read_data_buf[j*2] <<8) + read_data_buf[j*2+1];
//			printk(" %d", read_data_buf1[j]);

			if((SCR_ABS_UPPER_SPEC[i][j] < read_data_buf1[j]) 
				|| (SCR_ABS_LOWER_SPEC[i][j] > read_data_buf1[j]))
			{
				printk("\n SCR_ABS_UPPER_SPEC[i][j] = %d", SCR_ABS_UPPER_SPEC[i][j]);
				printk("\n SCR_ABS_LOWER_SPEC[i][j] = %d", SCR_ABS_LOWER_SPEC[i][j]);
				printk("\n i=%d, j=%d,read_data_buf1[j]=%d", i,j,read_data_buf1[j]);
				enable_irq(ts->client->irq);
				udelay(10);
				tsp_reset();
				return sprintf(buf, "0"); // fail
			}
		}

		printk("\n");
		
#if 0
		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			printk("[%03d],",read_data_buf[j]);
		}
		printk("\n");

#endif
		msleep(1);
	
	}	

	enable_irq(ts->client->irq);

	tsp_testmode = 0;

	tsp_reset();
				
    return sprintf(buf, "1"); // success
 }

static ssize_t	check_init_lowleveldata()
{
	u8 read_buf[1] = {0,};
	int ret=1;
	
	if(init_lowleveldata)
	{
		//////////////////////
		// read Exciting CH.
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0x2e, read_buf, 1);
		if(ret < 0) 
		{
			printk("[TSP] Exciting CH. melfas_i2c_read fail! %s : %d, \n", __func__,__LINE__);
			return 0;
		}
		g_exciting_ch = read_buf[0]; // 15

		//////////////////////
		// read Sensing CH.
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0x2f, read_buf, 1);
		if(ret < 0) 
		{
			printk("[TSP] Sensing CH. melfas_i2c_read fail! %s : %d, \n", __func__,__LINE__);
			return 0;
		}
		g_sensing_ch = read_buf[0]; // 10

		init_lowleveldata = false;
	}

	return ret;
}

static ssize_t tkey_rawcounter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 setLowLevelData[2] = {0x09, 0x04,};
	u8 read_data_buf[50] = {0,};    
	int read_data_len,sensing_ch;
	int ret, i,j;
	int menuKey, backKey;

	menuKey = 0;
	backKey = 0;
	disable_irq(ts->client->irq);

	ret = check_init_lowleveldata();

	////////////////////////
	// Writing Low Level Data(1)
	////////////////////////
//	ret = melfas_i2c_write(ts->client, setLowLevelData, 2);
	
	read_data_len = g_exciting_ch + 2; // +2 for key value
	sensing_ch	 = g_sensing_ch;

	for(i = 0; i < sensing_ch; i++ )
	{
		////////////////////////
		// Writing Low Level Data(2)
		////////////////////////
		ret = melfas_i2c_write(ts->client, setLowLevelData, 2);

		//////////////////////
		// Checking INT
		//////////////////////
		while(gpio_get_value(GPIO_TOUCH_INT)) // wait for Low
		{
			udelay(50);
		}

		udelay(300);

		//////////////////////
		// Read Data Buf
		//////////////////////
		ret = melfas_i2c_read(ts->client, 0xb2, read_data_buf, read_data_len);

		if(ret < 0)
	            printk("can't read Intensity Data %dth\n", i);

		udelay(5);
		
		if(i==0)
			menuKey = read_data_buf[read_data_len-1];
		else if(i==1)
			backKey = read_data_buf[read_data_len-1];
		else
				;
#if 0
		printk("[%d]:", i);	
		for(j=0;j<read_data_len;j++)
		{
			printk("[%02d],",read_data_buf[j]);
		}
		printk("\n");
#endif		
	}	

	enable_irq(ts->client->irq);
	
	printk("exciting_ch=[%d],sensing_ch=[%d],menuKey=[%02d],backKey=[%02d]\n", read_data_len, sensing_ch, menuKey, backKey) ;
	mdelay(1);
		
	return sprintf(buf, "%d %d", menuKey, backKey) ;
}

static ssize_t tkey_rawcounter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	if(value == 0)
	{
		hrtimer_cancel(&ts->timer);
		tsp_reset();
//		prev_wdog_val = -1;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	//	hrtimer_start(&ts->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);
	}

	return size;
}

static int tsp_reset(void)
{

	if (ts->client->irq)
	{
		disable_irq(ts->client->irq);
	}

	gpio_direction_output( GPIO_TOUCH_INT , 0 );
	gpio_direction_output( GPIO_TSP_SCL , 0 ); 
	gpio_direction_output( GPIO_TSP_SDA , 0 ); 

	bcm_gpio_pull_up(GPIO_TOUCH_INT, false);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);
	touch_ctrl_regulator_mms128(TOUCH_OFF);

	msleep(200);

	gpio_direction_output( GPIO_TSP_SCL , 1 ); 
	gpio_direction_output( GPIO_TSP_SDA , 1 ); 
	//gpio_direction_output( TSP_INT , 1 ); 

	gpio_direction_input(GPIO_TOUCH_INT);
	bcm_gpio_pull_up(GPIO_TOUCH_INT, true);
	bcm_gpio_pull_up_down_enable(GPIO_TOUCH_INT, true);

#ifdef DELAY_BEFORE_VDD
    msleep(10);
#endif
	
	touch_ctrl_regulator_mms128(TOUCH_ON);

	msleep(70);

	enable_irq(ts->client->irq);
	
	return 1;
}
#endif // CONFIG_TOUCHSCREEN_MMS128_COOPERVE




#if 0 //TSP_TEST_MODE
static ssize_t tsp_test_inspection_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk(KERN_DEBUG "Reference START %s\n", __func__) ;

    return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d \n",
                   tsp_test_inspection[inspec_y_channel_num][0], tsp_test_inspection[inspec_y_channel_num][1], tsp_test_inspection[inspec_y_channel_num][2], tsp_test_inspection[inspec_y_channel_num][3],
                   tsp_test_inspection[inspec_y_channel_num][4], tsp_test_inspection[inspec_y_channel_num][5], tsp_test_inspection[inspec_y_channel_num][6], tsp_test_inspection[inspec_y_channel_num][7],
                   tsp_test_inspection[inspec_y_channel_num][8], tsp_test_inspection[inspec_y_channel_num][9]);
}

static ssize_t tsp_test_inspection_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
    unsigned int position;
    int j, i, ret;
    uint16_t ref_value;
    uint8_t buf1[2], buff[20];

    sscanf(buf, "%d\n", &position);

    if (position == 100)
    {
        inspec_test_cnt = 0;
        printk("reset_reference_value\n");
    }

    if (!inspec_test_cnt)
    {

        /* disable TSP_IRQ */
        disable_irq(ts->client->irq);
        for (i = 0;i < 14;i++)
        {
            for (j = 0;j < 10;j++)
            {
                buf1[0] = 0xA0 ;		/* register address */
                buf1[1] = 0x42 ;
                buf1[2] = i;
                buf1[3] = j;

                if (melfas_i2c_write(ts->client, buf1, 4) != 0)
                {
                    printk(KERN_DEBUG "Failed to enter testmode\n") ;
                }

                while (1)
                {
                    if (MCSDL_GPIO_RESETB_IS_HIGH() == 0) //TSP INT Low
                        break;
                }

                if (melfas_i2c_read(ts->client, 0xAE, buff, 1) != 0)
                {
                    printk(KERN_DEBUG "Failed to read(referece data)\n") ;
                }

                if (melfas_i2c_read(ts->client, 0xAF, buff, 2) != 0)
                {
                    printk(KERN_DEBUG "Failed to read(referece data)\n") ;
                }

                printk("ref value0=%x\n", buff[0]);
                printk("ref value1=%x\n", buff[1]);

                ref_value = (uint16_t)(buff[1] << 8) | buff[0] ;
                tsp_test_inspection[i][j] = ref_value;
                printk("ins value[%d]=%d\n", i, ref_value);
                inspec_test_cnt = 1;
            }
        }
        mcsdl_vdd_off();
        mdelay(50);
        mcsdl_vdd_on();
        mdelay(250);
        printk("[TOUCH] reset.\n");
        /* enable TSP_IRQ */
        enable_irq(ts->client->irq);
    }

    if (position < 0 || position > 14)
    {
        printk(KERN_DEBUG "Invalid values\n");
        return -EINVAL;
    }

    inspec_y_channel_num = (uint8_t)position;

    return size;
}

static DEVICE_ATTR(tsp_inspection, 0664, tsp_test_inspection_show, tsp_test_inspection_store);
#endif

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
