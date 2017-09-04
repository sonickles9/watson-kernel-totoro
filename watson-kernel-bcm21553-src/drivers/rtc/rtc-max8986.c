/*****************************************************************************
*  Copyright 2009 - 2010 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/max8986/max8986.h>
#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
#include <linux/mfd/max8986/max8986-private.h>
#include <linux/reboot.h>
#endif

#define MAX8986_YEAR_BASE		100	/*2000 - 1900 */
#define SEC_YEAR_BASE 			11  /* 2011 */

#define RTC_ALARM1			1
#define RTC_ALARM2			2
#define RTC_ALARM_BIT_MASK		0x7F
#define RTC_ALARM_MONTH_BIT_MASK	0x1F
#define RTC_ALARM_DOM_BIT_MASK		0x3F
#define RTC_ALARM_HOUR_BIT_MASK		0x3F
#define RTC_ALARM_BIT			0x80
#define RTC_HOUR_PM_BIT_MASK  (1 << 6)

#define RTC_NO_OF_TIME_REGS		7
#define RTC_NO_OF_ALARM_REGS		7
#define MAX8986_RTC_UPDATE_REGISTERS	0x01

/* MAX8986 time registers offset with seconds register as base */
enum {
	MAX8986_SEC_REG_OFFSET = 0,
	MAX8986_MIN_REG_OFFSET,
	MAX8986_HR_REG_OFFSET,
	MAX8986_DOW_REG_OFFSET,
	MAX8986_MONTH_REG_OFFSET,
	MAX8986_YEAR_REG_OFFSET,
	MAX8986_DOM_REG_OFFSET
};

struct max8986_rtc {
	struct max8986 *max8986;
	struct rtc_device *rtc;
	int alarm_enabled;
};

 //Debug code for RTC init problem @ power on/off test 
static u8 g_max8986_rtc_time_erased = 0;
static struct rtc_time g_current_rtc_time;
static struct rtc_time g_reconverted_rtc_time;
void max8986_rtc_reset_debug_print( void );
u8 is_max8986_rtc_reset( void );

static int max8986_validate_year(struct rtc_time *tm)
{
	int max_rtc_year;
	max_rtc_year = tm->tm_year - MAX8986_YEAR_BASE;
	if (max_rtc_year < 0 || max_rtc_year > 99) {
		pr_err("%s: invalid year: %d\n", __func__, tm->tm_year);
		return -EINVAL;
	}
	return 0;
}

/*
 * Read current time and date in RTC
 */
static int max8986_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;
	int ret = 0;
	u8 regval;
	u8 time_regs[RTC_NO_OF_TIME_REGS];

	pr_debug("%s\n", __func__);

	memset(time_regs, 0, RTC_NO_OF_TIME_REGS);
	ret = max8986->read_mul_dev(max8986, MAX8986_RTC_REG_SECOND,
				    RTC_NO_OF_TIME_REGS, &time_regs[0]);
	if (ret < 0) {
		pr_err("%s: read time failed\n", __func__);
		return ret;
	}

	tm->tm_year = time_regs[MAX8986_YEAR_REG_OFFSET] + MAX8986_YEAR_BASE;
	/* 'mon' in tm structure is stored in 0-11 form, RTC requires it in
	 * 1-12 form.
	 */
	tm->tm_mon = time_regs[MAX8986_MONTH_REG_OFFSET] - 1;
	tm->tm_mday = time_regs[MAX8986_DOM_REG_OFFSET];
	tm->tm_wday = 0;
	regval = time_regs[MAX8986_DOW_REG_OFFSET];
	while (regval != 0x01) {
		regval >>= 1;
		tm->tm_wday++;
	}
	tm->tm_hour = time_regs[MAX8986_HR_REG_OFFSET]
	    & RTC_ALARM_HOUR_BIT_MASK;
	tm->tm_min = time_regs[MAX8986_MIN_REG_OFFSET];
	tm->tm_sec = time_regs[MAX8986_SEC_REG_OFFSET];

	pr_debug("%s: year = %d\n", __func__, tm->tm_year);
	pr_debug("%s: mon  = %d\n", __func__, tm->tm_mon);
	pr_debug("%s: mday = %d\n", __func__, tm->tm_mday);
	pr_debug("%s: wday = %d\n", __func__, tm->tm_wday);
	pr_debug("%s: hour = %d\n", __func__, tm->tm_hour);
	pr_debug("%s: min  = %d\n", __func__, tm->tm_min);
	pr_debug("%s: sec  = %d\n", __func__, tm->tm_sec);

	return 0;
}

/*
 * Set current time and date in RTC
 */
static int max8986_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;
	int ret = 0;
	u8 update_reg;
	u8 time_regs[RTC_NO_OF_TIME_REGS];

	pr_debug("%s\n", __func__);

	if (max8986_validate_year(tm) == -EINVAL)
		return -EINVAL;

	memset(time_regs, 0, RTC_NO_OF_TIME_REGS);

	time_regs[MAX8986_YEAR_REG_OFFSET] = tm->tm_year - MAX8986_YEAR_BASE;
	time_regs[MAX8986_MONTH_REG_OFFSET] = tm->tm_mon + 1;
	time_regs[MAX8986_DOM_REG_OFFSET] = tm->tm_mday;
	time_regs[MAX8986_DOW_REG_OFFSET] = 1 << tm->tm_wday;
	time_regs[MAX8986_HR_REG_OFFSET] = tm->tm_hour;
	time_regs[MAX8986_MIN_REG_OFFSET] = tm->tm_min;
	time_regs[MAX8986_SEC_REG_OFFSET] = tm->tm_sec;

	pr_debug("%s: year = %d\n", __func__, tm->tm_year);
	pr_debug("%s: mon  = %d\n", __func__, tm->tm_mon);
	pr_debug("%s: mday = %d\n", __func__, tm->tm_mday);
	pr_debug("%s: wday = %d\n", __func__, tm->tm_wday);
	pr_debug("%s: hour = %d\n", __func__, tm->tm_hour);
	pr_debug("%s: min  = %d\n", __func__, tm->tm_min);
	pr_debug("%s: sec  = %d\n", __func__, tm->tm_sec);

	ret |= max8986->write_mul_dev(max8986, MAX8986_RTC_REG_SECOND,
				      RTC_NO_OF_TIME_REGS, &time_regs[0]);
	ret |= max8986->read_dev(max8986, MAX8986_RTC_REG_UPDATE1, &update_reg);
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_UPDATE1,
				  (update_reg | MAX8986_RTC_UPDATE_REGISTERS));

	if (ret < 0) {
		pr_err("%s set time failed %d\n", __func__, ret);
		return ret;
	}

	/* the UDF bit in UPDATE1 register seems to be set always. so, not
	 * able to 'poll' on it as the manual (rev2) says. hence this delay
	 * here
	 */
	msleep(50);

	return ret;
}

static int max8986_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	int ret = 0;
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;

	pr_debug("%s: enabled: %d\n", __func__, enabled);

	if (enabled)
		ret = max8986_enable_irq(max8986, MAX8986_IRQID_INT1_RTCA);
	else
		ret = max8986_disable_irq(max8986, MAX8986_IRQID_INT1_RTCA);

	if (ret < 0)
		pr_err("%s: irq enable/disable failed: %d\n", __func__, ret);

	return ret;
}

static int max8986_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;
	int ret = 0;
	u8 alarm_regs[RTC_NO_OF_ALARM_REGS];
	u8 regval;

	pr_debug("%s\n", __func__);

	memset(alarm_regs, 0, RTC_NO_OF_ALARM_REGS);

	ret = max8986->read_mul_dev(max8986, MAX8986_RTC_REG_SEC_ALARM1,
				    RTC_NO_OF_ALARM_REGS, &alarm_regs[0]);
	if (ret < 0) {
		pr_err("%s: read alarm regs failed\n", __func__);
		return ret;
	}

	alm->time.tm_year = (alarm_regs[MAX8986_YEAR_REG_OFFSET]
			     & RTC_ALARM_BIT_MASK) + MAX8986_YEAR_BASE;
	alm->time.tm_mon = (alarm_regs[MAX8986_MONTH_REG_OFFSET]
			    & RTC_ALARM_MONTH_BIT_MASK) - 1;
	alm->time.tm_mday = (alarm_regs[MAX8986_DOM_REG_OFFSET]
			     & RTC_ALARM_DOM_BIT_MASK);
	alm->time.tm_wday = 0;
	regval = (alarm_regs[MAX8986_DOW_REG_OFFSET] & RTC_ALARM_BIT_MASK);
	while (regval != 0x01) {
		regval >>= 1;
		alm->time.tm_wday++;
	}
	alm->time.tm_hour = alarm_regs[MAX8986_HR_REG_OFFSET]
	    & RTC_ALARM_HOUR_BIT_MASK;
	alm->time.tm_min = alarm_regs[MAX8986_MIN_REG_OFFSET]
	    & RTC_ALARM_BIT_MASK;
	alm->time.tm_sec = alarm_regs[MAX8986_SEC_REG_OFFSET]
	    & RTC_ALARM_BIT_MASK;

	if (max8986_rtc->alarm_enabled)
		alm->enabled = 1;
	else
		alm->enabled = 0;

	pr_debug("%s: alm->year    = %d\n", __func__, alm->time.tm_year);
	pr_debug("%s: alm->mon     = %d\n", __func__, alm->time.tm_mon);
	pr_debug("%s: alm->mday    = %d\n", __func__, alm->time.tm_mday);
	pr_debug("%s: alm->wday    = %d\n", __func__, alm->time.tm_wday);
	pr_debug("%s: alm->hour    = %d\n", __func__, alm->time.tm_hour);
	pr_debug("%s: alm->min     = %d\n", __func__, alm->time.tm_min);
	pr_debug("%s: alm->sec     = %d\n", __func__, alm->time.tm_sec);
	pr_debug("%s: alm->enabled = %d\n", __func__, alm->enabled);
	pr_debug("%s: alm->pending = %d\n", __func__, alm->pending);

	return 0;
}

#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
static int max8986_rtc_read_alarm_boot(struct device *dev, struct rtc_wkalrm *alm)
{
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;
	int ret = 0;
	u8 alarm_regs[RTC_NO_OF_ALARM_REGS];
	u8 regval;

	pr_debug("%s\n", __func__);

	memset(alarm_regs, 0, RTC_NO_OF_ALARM_REGS);

	ret = max8986->read_mul_dev(max8986, MAX8986_RTC_REG_SEC_ALARM2,
				    RTC_NO_OF_ALARM_REGS, &alarm_regs[0]);
	if (ret < 0) {
		pr_err("%s: read alarm regs failed\n", __func__);
		return ret;
	}

	printk("[BSYSTAR] %s : tm(%04d.%03d.%03d %03d:%03d:%03d [%03d])\n", __func__,
		alarm_regs[MAX8986_YEAR_REG_OFFSET], alarm_regs[MAX8986_MONTH_REG_OFFSET], alarm_regs[MAX8986_DOM_REG_OFFSET],
					alarm_regs[MAX8986_HR_REG_OFFSET], alarm_regs[MAX8986_MIN_REG_OFFSET], alarm_regs[MAX8986_SEC_REG_OFFSET], alarm_regs[MAX8986_DOW_REG_OFFSET]);

	alm->time.tm_year = (alarm_regs[MAX8986_YEAR_REG_OFFSET]
			     & RTC_ALARM_BIT_MASK) + MAX8986_YEAR_BASE;
	alm->time.tm_mon = (alarm_regs[MAX8986_MONTH_REG_OFFSET]
			    & RTC_ALARM_MONTH_BIT_MASK) - 1;
	alm->time.tm_mday = (alarm_regs[MAX8986_DOM_REG_OFFSET]
			     & RTC_ALARM_DOM_BIT_MASK);
	alm->time.tm_wday = 0;
	regval = (alarm_regs[MAX8986_DOW_REG_OFFSET] & RTC_ALARM_BIT_MASK);
	while (regval != 0x01) {
		regval >>= 1;
		alm->time.tm_wday++;
	}
	alm->time.tm_hour = alarm_regs[MAX8986_HR_REG_OFFSET]
	    & RTC_ALARM_HOUR_BIT_MASK;
	alm->time.tm_min = alarm_regs[MAX8986_MIN_REG_OFFSET]
	    & RTC_ALARM_BIT_MASK;
	alm->time.tm_sec = alarm_regs[MAX8986_SEC_REG_OFFSET]
	    & RTC_ALARM_BIT_MASK;

	if (max8986_rtc->alarm_enabled)
		alm->enabled = 1;
	else
		alm->enabled = 0;

	printk("[BSYSTAR] %s : tm(%d %04d.%02d.%02d %02d:%02d:%02d)\n", __func__,alm->enabled,
		alm->time.tm_year+1900, alm->time.tm_mon+1, alm->time.tm_mday, alm->time.tm_hour, alm->time.tm_min, alm->time.tm_sec);

	pr_debug("%s: alm->year    = %d\n", __func__, alm->time.tm_year);
	pr_debug("%s: alm->mon     = %d\n", __func__, alm->time.tm_mon);
	pr_debug("%s: alm->mday    = %d\n", __func__, alm->time.tm_mday);
	pr_debug("%s: alm->wday    = %d\n", __func__, alm->time.tm_wday);
	pr_debug("%s: alm->hour    = %d\n", __func__, alm->time.tm_hour);
	pr_debug("%s: alm->min     = %d\n", __func__, alm->time.tm_min);
	pr_debug("%s: alm->sec     = %d\n", __func__, alm->time.tm_sec);
	pr_debug("%s: alm->enabled = %d\n", __func__, alm->enabled);
	pr_debug("%s: alm->pending = %d\n", __func__, alm->pending);

	return 0;
}
#endif

static int max8986_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;
	int ret = 0;
	u8 update_reg;
	u8 alarm_regs[RTC_NO_OF_ALARM_REGS];

	pr_debug("%s: alm->year     = %d\n", __func__, alm->time.tm_year);
	pr_debug("%s: alm->mon      = %d\n", __func__, alm->time.tm_mon);
	pr_debug("%s: alm->mday     = %d\n", __func__, alm->time.tm_mday);
	pr_debug("%s: alm->wday     = %d\n", __func__, alm->time.tm_wday);
	pr_debug("%s: alm->hour     = %d\n", __func__, alm->time.tm_hour);
	pr_debug("%s: alm->min      = %d\n", __func__, alm->time.tm_min);
	pr_debug("%s: alm->sec      = %d\n", __func__, alm->time.tm_sec);
	pr_debug("%s: alm->enabled  = %d\n", __func__, alm->enabled);
	pr_debug("%s: alm->pending  = %d\n", __func__, alm->pending);

	if (alm->enabled == 0) {
		max8986_rtc_alarm_irq_enable(dev, 0);
		return 0;
	}

	if (max8986_validate_year(&alm->time) == -EINVAL)
		return -EINVAL;

	memset(alarm_regs, 0, RTC_NO_OF_ALARM_REGS);

	/* if the top layer sets the wday field to -1(don't care)
	 * we do not enable the alarm bit for WDAY field and also set
	 * its value to 1 (to have a valid value in the register). If
	 * the top layer sets this field to -1 we cannot update the
	 * wday field correctly without first reading it from the PMU.
	 *
	 * the rtc framework (rtc-dev.c) takes care to set valid values
	 * in mday, year and mon fields. hence these fields are not
	 * checked here for -1.
	 */

#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
	if (alm->time.tm_wday == -1) {
		alm->time.tm_wday = 1;

		alarm_regs[MAX8986_DOW_REG_OFFSET] =
		    ((1 << alm->time.tm_wday) & RTC_ALARM_BIT_MASK);
	}
#else
	if (alm->time.tm_wday == -1) {
		alm->time.tm_wday = 1;

		alarm_regs[MAX8986_DOW_REG_OFFSET] =
		    ((1 << alm->time.tm_wday) & RTC_ALARM_BIT_MASK);
	} else {
		alarm_regs[MAX8986_DOW_REG_OFFSET] = RTC_ALARM_BIT |
		    ((1 << alm->time.tm_wday) & RTC_ALARM_BIT_MASK);
	}
#endif

	/*store the alarm values in respective regs */
	alarm_regs[MAX8986_YEAR_REG_OFFSET] = RTC_ALARM_BIT |
	    ((alm->time.tm_year - MAX8986_YEAR_BASE) & RTC_ALARM_BIT_MASK);

	alarm_regs[MAX8986_MONTH_REG_OFFSET] = RTC_ALARM_BIT |
	    ((alm->time.tm_mon + 1) & RTC_ALARM_MONTH_BIT_MASK);

	alarm_regs[MAX8986_DOM_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_mday & RTC_ALARM_DOM_BIT_MASK);

	alarm_regs[MAX8986_HR_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_hour & RTC_ALARM_HOUR_BIT_MASK);

	/*Bug with RTC alarm ?? 
	Alarm does not fire if AM/PM bit is not set even 
	if RTC is configured for 24 hour mode */
	if(alm->time.tm_hour >= 12)
		alarm_regs[MAX8986_HR_REG_OFFSET] |= RTC_HOUR_PM_BIT_MASK;
		

	alarm_regs[MAX8986_MIN_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_min & RTC_ALARM_BIT_MASK);

	alarm_regs[MAX8986_SEC_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_sec & RTC_ALARM_BIT_MASK);

	ret = max8986->write_mul_dev(max8986, MAX8986_RTC_REG_SEC_ALARM1,
				     RTC_NO_OF_ALARM_REGS, &alarm_regs[0]);
	ret |= max8986->read_dev(max8986, MAX8986_RTC_REG_UPDATE1, &update_reg);
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_UPDATE1,
				  (update_reg | MAX8986_RTC_UPDATE_REGISTERS));
	if (ret != 0) {
		pr_err("%s: setting alarm failed: ret %d\n", __func__, ret);
		return ret;
	}

	max8986_rtc_alarm_irq_enable(dev, 1);

	/* the UDF bit in UPDATE1 register seems to be set always. so, not
	 * able to 'poll' on it as the manual (rev2) says. hence this delay
	 * here
	 */
	msleep(50);

	return ret;
}

#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
static int max8986_rtc_set_alarm_boot(struct device *dev, struct rtc_wkalrm *alm)
{
	struct max8986_rtc *max8986_rtc = dev_get_drvdata(dev);
	struct max8986 *max8986 = max8986_rtc->max8986;
	int ret = 0;
	u8 update_reg;
	u8 alarm_regs[RTC_NO_OF_ALARM_REGS];

	pr_debug("%s: alm->year     = %d\n", __func__, alm->time.tm_year);
	pr_debug("%s: alm->mon      = %d\n", __func__, alm->time.tm_mon);
	pr_debug("%s: alm->mday     = %d\n", __func__, alm->time.tm_mday);
	pr_debug("%s: alm->wday     = %d\n", __func__, alm->time.tm_wday);
	pr_debug("%s: alm->hour     = %d\n", __func__, alm->time.tm_hour);
	pr_debug("%s: alm->min      = %d\n", __func__, alm->time.tm_min);
	pr_debug("%s: alm->sec      = %d\n", __func__, alm->time.tm_sec);
	pr_debug("%s: alm->enabled  = %d\n", __func__, alm->enabled);
	pr_debug("%s: alm->pending  = %d\n", __func__, alm->pending);

	if (alm->enabled == 0) {
		printk("[BSYSTAR] power off alarm disable!!\n");
		alarm_regs[MAX8986_DOW_REG_OFFSET] = 0x01;
		/*store the alarm values in respective regs */
		alarm_regs[MAX8986_YEAR_REG_OFFSET] = 0x00;
		alarm_regs[MAX8986_MONTH_REG_OFFSET] = 0x00;
		alarm_regs[MAX8986_DOM_REG_OFFSET] = 0x01;
		alarm_regs[MAX8986_HR_REG_OFFSET] = 0x00;
		alarm_regs[MAX8986_MIN_REG_OFFSET] = 0x00;
		alarm_regs[MAX8986_SEC_REG_OFFSET] = 0x00;

		ret = max8986->write_mul_dev(max8986, MAX8986_RTC_REG_SEC_ALARM2,
					     RTC_NO_OF_ALARM_REGS, &alarm_regs[0]);
		ret |= max8986->read_dev(max8986, MAX8986_RTC_REG_UPDATE1, &update_reg);
		ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_UPDATE1,
					  (update_reg | MAX8986_RTC_UPDATE_REGISTERS));
		if (ret != 0) {
			pr_err("%s: setting alarm failed: ret %d\n", __func__, ret);
			return ret;
		}
		max8986_rtc_alarm_irq_enable(dev, 0);
		msleep(50);

		return 0;
	}

	memset(alarm_regs, 0, RTC_NO_OF_ALARM_REGS);

	/* if the top layer sets the wday field to -1(don't care)
	 * we do not enable the alarm bit for WDAY field and also set
	 * its value to 1 (to have a valid value in the register). If
	 * the top layer sets this field to -1 we cannot update the
	 * wday field correctly without first reading it from the PMU.
	 *
	 * the rtc framework (rtc-dev.c) takes care to set valid values
	 * in mday, year and mon fields. hence these fields are not
	 * checked here for -1.
	 */
	alm->time.tm_wday = 0x0;
	alarm_regs[MAX8986_DOW_REG_OFFSET] = 0x01;

	/*store the alarm values in respective regs */
	alarm_regs[MAX8986_YEAR_REG_OFFSET] = RTC_ALARM_BIT |
	    ((alm->time.tm_year - MAX8986_YEAR_BASE) & RTC_ALARM_BIT_MASK);

	alarm_regs[MAX8986_MONTH_REG_OFFSET] = RTC_ALARM_BIT |
	    ((alm->time.tm_mon + 1) & RTC_ALARM_MONTH_BIT_MASK);

	alarm_regs[MAX8986_DOM_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_mday & RTC_ALARM_DOM_BIT_MASK);

	alarm_regs[MAX8986_HR_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_hour & RTC_ALARM_HOUR_BIT_MASK);

	/*Bug with RTC alarm ?? 
	Alarm does not fire if AM/PM bit is not set even 
	if RTC is configured for 24 hour mode */
	if(alm->time.tm_hour >= 12)
		alarm_regs[MAX8986_HR_REG_OFFSET] |= RTC_HOUR_PM_BIT_MASK;

	alarm_regs[MAX8986_MIN_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_min & RTC_ALARM_BIT_MASK);

	alarm_regs[MAX8986_SEC_REG_OFFSET] = RTC_ALARM_BIT |
	    (alm->time.tm_sec & RTC_ALARM_BIT_MASK);

 	printk("[BSYSTAR] %s : tm(%04d.%03d.%03d %03d:%03d:%03d [%3d])\n", __func__, alarm_regs[MAX8986_YEAR_REG_OFFSET],
		alarm_regs[MAX8986_MONTH_REG_OFFSET], alarm_regs[MAX8986_DOM_REG_OFFSET], alarm_regs[MAX8986_HR_REG_OFFSET],
		alarm_regs[MAX8986_MIN_REG_OFFSET], alarm_regs[MAX8986_SEC_REG_OFFSET], alarm_regs[MAX8986_DOW_REG_OFFSET]);

 	ret = max8986->write_mul_dev(max8986, MAX8986_RTC_REG_SEC_ALARM2,
				     RTC_NO_OF_ALARM_REGS, &alarm_regs[0]);
	ret |= max8986->read_dev(max8986, MAX8986_RTC_REG_UPDATE1, &update_reg);
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_UPDATE1,
				  (update_reg | MAX8986_RTC_UPDATE_REGISTERS));
	if (ret != 0) {
		pr_err("%s: setting alarm failed: ret %d\n", __func__, ret);
		return ret;
	}

	max8986_rtc_alarm_irq_enable(dev, 1);

	/* the UDF bit in UPDATE1 register seems to be set always. so, not
	 * able to 'poll' on it as the manual (rev2) says. hence this delay
	 * here
	 */
	msleep(50);	

	return ret;
}
#endif

static void max8986_rtc_isr(int irq, void *data)
{
	unsigned long events = 0;
	struct max8986_rtc *max8986_rtc = data;

	pr_debug("%s\n", __func__);

	events |= RTC_IRQF | RTC_AF;
	rtc_update_irq(max8986_rtc->rtc, 1, events);
}

static const struct rtc_class_ops max8986_rtc_ops = {
	.read_time = max8986_rtc_read_time,
	.set_time = max8986_rtc_set_time,
	.read_alarm = max8986_rtc_read_alarm,
	.set_alarm = max8986_rtc_set_alarm,
#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
	.set_alarm_boot = max8986_rtc_set_alarm_boot,
	.read_alarm_boot = max8986_rtc_read_alarm_boot,
#endif
	.alarm_irq_enable = max8986_rtc_alarm_irq_enable,
	.update_irq_enable = max8986_rtc_alarm_irq_enable,
};

#ifdef CONFIG_PM
static int max8986_rtc_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int max8986_rtc_resume(struct device *dev)
{
	pr_debug("%s\n", __func__);

#if defined(CONFIG_RTC_ANDROID_ALARM_WORKAROUND)
	/* This option selects temporary fix for alarm handling in 'Android'
	 * environment. This option enables code to disable alarm in the
	 * 'resume' handler of RTC driver. In the normal mode,
	 * android handles all alarms in software without using the RTC chip.
	 * Android sets the alarm in the rtc only in the suspend path (by
	 * calling .set_alarm with struct rtc_wkalrm->enabled set to 1).
	 * In the resume path, android tries to disable alarm by calling
	 * .set_alarm with struct rtc_wkalrm->enabled' field set to 0.
	 * But unfortunately, it memsets the rtc_wkalrm struct to 0, which
	 * causes the rtc lib to flag error and control does not reach this
	 * driver. Hence this workaround.
	 */
	max8986_rtc_alarm_irq_enable(dev, 0);
#endif

	return 0;
}
#else
#define max8986_rtc_suspend NULL
#define max8986_rtc_resume NULL
#endif

/* The RTC base year is set to 2000 and hence the RTC default time
 * is 1st Jan 2000, 00:00:00. But the default wday field in RTC is
 * set to sunday, whereas 1st Jan, 2000 is a Saturday. So if this
 * condition is detected, then reprogram the wday field in RTC.
 */
static void max8986_rtc_time_fixup(struct device *dev)
{
	struct rtc_time current_rtc_time;
	struct rtc_time reconverted_rtc_time;
	unsigned long current_time;

	/* The wday field in the time read from RTC may be incorrect */
	max8986_rtc_read_time(dev, &current_rtc_time);
	rtc_tm_to_time(&current_rtc_time, &current_time);
	/* convert the number of seconds since epoch time back to
	 * rtc_time. During this conversion wday will be correctly
	 * computed.
	 */
	rtc_time_to_tm(current_time, &reconverted_rtc_time);

	/* If the wday field read from rtc differs from the computed time
	 * in Linux, then fixup wday field and set the RTC time.
	 */
#if 1	//Condition for default year instead of day of week
	/* Reset values of RTC are year 100, mon 0, mday 1, and Day of week is 0 (Sunday) */
	if ((current_rtc_time.tm_wday != reconverted_rtc_time.tm_wday) &&
			(current_rtc_time.tm_year == 100)  &&
			(current_rtc_time.tm_mon == 0)     &&
			(current_rtc_time.tm_mday == 1))
	{
		pr_info("%s: Fixing up RTC time\n", __func__);

		current_rtc_time.tm_year += SEC_YEAR_BASE;      //set base year to 2011 in case of RTC reset.
		rtc_tm_to_time(&current_rtc_time, &current_time);
		rtc_time_to_tm(current_time, &reconverted_rtc_time);

		max8986_rtc_set_time(dev, &reconverted_rtc_time);
		//RTC infomation was reset
		g_max8986_rtc_time_erased = 1;
		/* Store current time data to debug */
		g_current_rtc_time.tm_sec = current_rtc_time.tm_sec;
		g_current_rtc_time.tm_min = current_rtc_time.tm_min;		
		g_current_rtc_time.tm_hour = current_rtc_time.tm_hour;			
		g_current_rtc_time.tm_mday = current_rtc_time.tm_mday;
		g_current_rtc_time.tm_mon	= current_rtc_time.tm_mon;		
		g_current_rtc_time.tm_year = current_rtc_time.tm_year;			
		g_current_rtc_time.tm_wday = current_rtc_time.tm_wday;
		g_current_rtc_time.tm_yday = current_rtc_time.tm_yday;		
		g_current_rtc_time.tm_isdst = current_rtc_time.tm_isdst;		
		/* Store reconverted time data to debug */		
		g_reconverted_rtc_time.tm_sec = reconverted_rtc_time.tm_sec;
		g_reconverted_rtc_time.tm_min = reconverted_rtc_time.tm_min;		
		g_reconverted_rtc_time.tm_hour = reconverted_rtc_time.tm_hour;			
		g_reconverted_rtc_time.tm_mday = reconverted_rtc_time.tm_mday;
		g_reconverted_rtc_time.tm_mon = reconverted_rtc_time.tm_mon;		
		g_reconverted_rtc_time.tm_year = reconverted_rtc_time.tm_year;			
		g_reconverted_rtc_time.tm_wday = reconverted_rtc_time.tm_wday;
		g_reconverted_rtc_time.tm_yday = reconverted_rtc_time.tm_yday;		
		g_reconverted_rtc_time.tm_isdst = reconverted_rtc_time.tm_isdst;
		/* Print out debug infomation for RTC reset */		
		max8986_rtc_reset_debug_print( );
	}
#else
	if (current_rtc_time.tm_wday != reconverted_rtc_time.tm_wday) {
		pr_info("%s: Fixing up RTC time\n", __func__);

		current_rtc_time.tm_year += SEC_YEAR_BASE;      //set base year to 2011 in case of RTC reset.
		rtc_tm_to_time(&current_rtc_time, &current_time);
		rtc_time_to_tm(current_time, &reconverted_rtc_time);

		max8986_rtc_set_time(dev, &reconverted_rtc_time);
	}
#endif
}

#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
struct rtc_time current_alarm_time;
struct max8986 *info_autopower_rtc;
static struct delayed_work pollling_work_alarmboot;
u32 sec_bootmode;
EXPORT_SYMBOL(sec_bootmode);

static __init int setup_boot_mode(char *opt)
{
	sec_bootmode = (u32) memparse(opt, &opt);
	return 0;
}
__setup("BOOT_MODE=", setup_boot_mode);

static void check_alarm_boot_workqueue(void)
{
	int ret;
	u8 data[7];
	unsigned long time_sec, alarm_sec;
	struct rtc_time current_rtc_time;

	ret = info_autopower_rtc->read_mul_dev(info_autopower_rtc, MAX8986_RTC_REG_SECOND,
				    7, &data[0]);
	if (ret < 0) {
		pr_err("%s: read time failed\n", __func__);
		return ret;
	}

	current_rtc_time.tm_year = data[5] + 100;
	current_rtc_time.tm_mon = data[4] - 1;
	current_rtc_time.tm_mday = data[6];
	current_rtc_time.tm_wday = 0;
	ret = data[3];
	while (ret != 0x01) {
		ret >>= 1;
		current_rtc_time.tm_wday++;
	}
	current_rtc_time.tm_hour = data[2] & 0x3F;
	current_rtc_time.tm_min = data[1];
	current_rtc_time.tm_sec = data[0];

	rtc_tm_to_time(&current_rtc_time, &time_sec);
	rtc_tm_to_time(&current_alarm_time, &alarm_sec);

	printk("[BSYSTAR] check_alarm_boot_kernel : alarm_sec - time_sec =%ld\n",
			alarm_sec - time_sec);

	if((time_sec > alarm_sec - 25) && (time_sec < alarm_sec + 5))
		machine_restart("alarmboot");

	schedule_delayed_work(&pollling_work_alarmboot, 3000);
}
#endif

 //Debug code for RTC init problem @ power on/off test 
void max8986_rtc_reset_debug_print( void )
{
	  pr_info("%s: RTC reset was occurred!!!", __func__);
	  
		pr_info("%s: current_rtc_time -> tm_sec : %d, tm_min : %d,tm_hour : %d,tm_mday : %d,tm_mon : %d,tm_year : %d,tm_wday : %d,tm_yday : %d,tm_isdst : %d, \n", __func__,
			g_current_rtc_time.tm_sec,
			g_current_rtc_time.tm_min,
			g_current_rtc_time.tm_hour,
			g_current_rtc_time.tm_mday,
			g_current_rtc_time.tm_mon,
			g_current_rtc_time.tm_year,
			g_current_rtc_time.tm_wday,
			g_current_rtc_time.tm_yday,
			g_current_rtc_time.tm_isdst);
   
		pr_info("%s: reconverted_rtc_time -> tm_sec : %d, tm_min : %d,tm_hour : %d,tm_mday : %d,tm_mon : %d,tm_year : %d,tm_wday : %d,tm_yday : %d,tm_isdst : %d, \n", __func__,
			g_reconverted_rtc_time.tm_sec,
			g_reconverted_rtc_time.tm_min,
			g_reconverted_rtc_time.tm_hour,
			g_reconverted_rtc_time.tm_mday,
			g_reconverted_rtc_time.tm_mon,
			g_reconverted_rtc_time.tm_year,
			g_reconverted_rtc_time.tm_wday,
			g_reconverted_rtc_time.tm_yday,
			g_reconverted_rtc_time.tm_isdst);
}
 //Debug code for RTC init problem @ power on/off test 
u8 is_max8986_rtc_reset( void )
{
	return g_max8986_rtc_time_erased;		
}
static int max8986_rtc_probe(struct platform_device *pdev)
{
	struct max8986 *max8986 = dev_get_drvdata(pdev->dev.parent);
	struct max8986_rtc *max8986_rtc;
#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
	unsigned char alarm_data[7];
	int alarm_en = 1;
#endif
	int ret = 0;

	pr_info("%s\n", __func__);

	max8986_rtc = kzalloc(sizeof(struct max8986_rtc), GFP_KERNEL);
	if (unlikely(max8986_rtc == NULL)) {
		pr_err("%s: failed. No memory\n", __func__);
		ret = -ENOMEM;
		goto err_alloc;
	}

	platform_set_drvdata(pdev, max8986_rtc);
	max8986_rtc->max8986 = max8986;

	device_init_wakeup(&pdev->dev, 1);

	ret = max8986_request_irq(max8986, MAX8986_IRQID_INT1_RTCA,
				  false, max8986_rtc_isr, max8986_rtc);
	if (unlikely(ret < 0)) {
		pr_err("%s: IRQ register failed\n", __func__);
		goto err_irq_req;
	}

	max8986_rtc->rtc = rtc_device_register(pdev->name, &pdev->dev,
					       &max8986_rtc_ops, THIS_MODULE);
	if (IS_ERR(max8986_rtc->rtc)) {
		pr_err("%s: rtc_device_register failed\n", __func__);
		ret = PTR_ERR(max8986_rtc->rtc);
		goto err_rtc_register;
	}

	/* Initialize MAX8986 RTC subsystem */
	/* Enable write to control register */
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_CTRL_MASK, 0x03);
	/* Configure for Binary, 24hrs mode  */
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_CONTROL, 0x02);

	/*Flush write buffer */

	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_UPDATE1,
			  (MAX8986_RTC_UPDATE_REGISTERS));

	mdelay(50);
	/* Update completed. So, disable write to control register */
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_CTRL_MASK, 0x00);
	ret |= max8986->write_dev(max8986, MAX8986_RTC_REG_UPDATE1,
				   MAX8986_RTC_UPDATE_REGISTERS);


	if (unlikely(ret != 0)) {
		goto err_max_rtc_init;
		pr_err("%s: failed: %d\n", __func__, ret);
	}

#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
	{
		extern struct rtc_device *alarm_boot_rtc_dev;
		struct rtc_wkalrm alm;
		printk("[BSYSTAR] RTC_PROBE : First Alarm RTC values!!\n");
		rtc_read_alarm_boot(alarm_boot_rtc_dev, &alm);
	}
#endif

	/* Fix the initial RTC time */
	max8986_rtc_time_fixup(&pdev->dev);
	pr_info("%s: passed\n", __func__);

#if defined(CONFIG_RTC_CHN_ALARM_BOOT)
	ret = max8986->read_mul_dev(max8986, MAX8986_RTC_REG_SEC_ALARM2,
				    7, &alarm_data[0]);
	if (ret < 0) {
		pr_err("%s: read alarm regs failed\n", __func__);
		return ret;
	}
	current_alarm_time.tm_year = (alarm_data[5] & 0x7F) + 100;
	current_alarm_time.tm_mon = (alarm_data[4] & 0x1F) - 1;
	current_alarm_time.tm_mday = (alarm_data[6] & 0x3F);
	current_alarm_time.tm_wday = 0;
	ret = (alarm_data[3] & 0x7F);
	while (ret != 0x01) {
		ret >>= 1;
		current_alarm_time.tm_wday++;
	}
	current_alarm_time.tm_hour = alarm_data[2] & 0x3F;
	current_alarm_time.tm_min = alarm_data[1] & 0x7F;
	current_alarm_time.tm_sec = alarm_data[0] & 0x7F;

	for(ret = 0 ; ret < 3 ; ret++) {
		if(!(alarm_data[ret] & 0x80))
			alarm_en = 0;
	}

	printk("[BSYSTAR] alarm_en = %d, sec_bootmode = %d\n", alarm_en, sec_bootmode); 
	if(alarm_en && sec_bootmode) {
		info_autopower_rtc = max8986;
		INIT_DELAYED_WORK_DEFERRABLE(&pollling_work_alarmboot,
			check_alarm_boot_workqueue);
		schedule_delayed_work(&pollling_work_alarmboot, 3000);
	}
#endif

	return 0;

err_max_rtc_init:
	rtc_device_unregister(max8986_rtc->rtc);

err_rtc_register:
	max8986_free_irq(max8986_rtc->max8986, MAX8986_IRQID_INT1_RTCA);

err_irq_req:
	kfree(max8986_rtc);

err_alloc:
	return ret;
}

static int __devexit max8986_rtc_remove(struct platform_device *pdev)
{
	struct max8986_rtc *max8986_rtc = platform_get_drvdata(pdev);

	max8986_free_irq(max8986_rtc->max8986, MAX8986_IRQID_INT1_RTCA);
	rtc_device_unregister(max8986_rtc->rtc);
	kfree(max8986_rtc);

	return 0;
}

static struct dev_pm_ops max8986_rtc_pm_ops = {
	.suspend = max8986_rtc_suspend,
	.resume = max8986_rtc_resume,
	.thaw = max8986_rtc_resume,
	.restore = max8986_rtc_resume,
	.poweroff = max8986_rtc_suspend,
};

static struct platform_driver max8986_rtc_driver = {
	.probe = max8986_rtc_probe,
	.remove = __devexit_p(max8986_rtc_remove),
	.driver = {
		   .name = "max8986-rtc",
		   .pm = &max8986_rtc_pm_ops,
		   },
};

static int __init max8986_rtc_init(void)
{
	return platform_driver_register(&max8986_rtc_driver);
}

module_init(max8986_rtc_init);

static void __exit max8986_rtc_exit(void)
{
	platform_driver_unregister(&max8986_rtc_driver);
}

module_exit(max8986_rtc_exit);

MODULE_DESCRIPTION("RTC driver for the Maxim MAX8986 PMU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:max8986-rtc");
