/* leds-bcm21553.c - BCM21553 LEDs driver.
 *
 * Copyright (C) 2011 Jeeon Park <jeeon.park@samsung.com>
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <linux/gpio.h>

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>

static struct regulator *touchkeyled_regulator=NULL;


static bool  bled_status = false;

static void bcm21553_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	printk("[KeyLED] %s: value=%d\n", __func__, value);

	if(value)
	{
		if( !bled_status )
		{
			regulator_set_voltage(touchkeyled_regulator,3300000,3300000);
			regulator_enable(touchkeyled_regulator);
			bled_status = true;
		}
	}
	else
	{
		if( bled_status )
		{	
			regulator_disable(touchkeyled_regulator);
			bled_status = false;			
		}
	}

	return;
}


static struct led_classdev bcm21553_kp_bl_led = {
	.name			= "button-backlight",
	.brightness_set		= bcm21553_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int bcm21553_led_probe(struct platform_device *pdev)
{
	int rc, ret = 0;

	printk("[KeyLED] %s\n", __func__ );

	touchkeyled_regulator = regulator_get(NULL,"touch_keyled");

	rc = led_classdev_register(&pdev->dev, &bcm21553_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	
	//bcm21553_keypad_bl_led_set(&bcm21553_kp_bl_led, LED_OFF);	// need to check!
	return rc;
}

static int __devexit bcm21553_led_remove(struct platform_device *pdev)
{
	printk("[KeyLED] %s\n", __func__ );
	led_classdev_unregister(&bcm21553_kp_bl_led);

	return 0;
}


static int bcm21553_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	printk("[KeyLED] %s\n", __func__ );
	led_classdev_suspend(&bcm21553_kp_bl_led);

	return 0;
}

static int bcm21553_led_resume(struct platform_device *dev)
{
	printk("[KeyLED] %s\n", __func__ );
	led_classdev_resume(&bcm21553_kp_bl_led);

	return 0;
}



static struct platform_driver bcm21553_led_driver = {
	.probe		= bcm21553_led_probe,
	.remove		= __devexit_p(bcm21553_led_remove),
	.suspend	= bcm21553_led_suspend,
	.resume		= bcm21553_led_resume,
	.driver		= {
		.name	= "bcm21553-leds",
		.owner	= THIS_MODULE,
	},
};


static int __init bcm21553_led_init(void)
{
	printk("[KeyLED] %s\n", __func__ );

	return platform_driver_register(&bcm21553_led_driver);
}
module_init(bcm21553_led_init);

static void __exit bcm21553_led_exit(void)
{
	printk("[KeyLED] %s\n", __func__ );
	platform_driver_unregister(&bcm21553_led_driver);
}
module_exit(bcm21553_led_exit);

MODULE_DESCRIPTION("BCM21553 LEDs driver");
MODULE_ALIAS("platform:bcm21553-leds");


