/*******************************************************************************
*                                                                              *
*   File Name:    taos.c                                                      *
*   Description:   Linux device driver for Taos ambient light and         *
*   proximity sensors.                                     *
*   Author:         John Koshi                                             *
*   History:   09/16/2009 - Initial creation                          *
*           10/09/2009 - Triton version         *
*           12/21/2009 - Probe/remove mode                *
*           02/07/2010 - Add proximity          *
*                                                                                       *
********************************************************************************
*    Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/
// includes
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/i2c/taos_common.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>



#include "tmd2772.h"

#define LOG_TAG "SENSOR_ALS_PROX"
#define DEBUG_ON //DEBUG SWITCH

// #define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

// ZTEMT ADD by zhubing 2012-2-20 V8000/X501
// added the work mode marco
//#define WORK_UES_POLL_MODE
// ZTEMT ADD by zhubing 2012-2-20 V8000/X501 END

//#define IRQ_TRIGER_LEVEL_LOW

struct taos_data;

// forward declarations
static int tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp);
static int tmd2772_remove(struct i2c_client *client);
static int taos_get_lux(void);
static int taos_lux_filter(int raw_lux);
static int taos_device_name(unsigned char *bufp, char **device_name);
static int taos_prox_poll(struct taos_prox_info *prxp);
static void taos_prox_poll_timer_func(unsigned long param);
static void taos_prox_poll_timer_start(void);
//iVIZM
static int taos_prox_threshold_set(struct taos_data *taos_datap);
static int taos_als_get_data(void);
static int taos_interrupts_clear(void);
static int taos_resume(struct i2c_client *client);
static int taos_suspend(struct i2c_client *client,pm_message_t mesg);
//CLLI@


static int taos_sensors_als_poll_on(void);
static int taos_sensors_als_poll_off(void);
static void taos_als_poll_work_func(struct work_struct *work);
static int taos_als_gain_set(unsigned als_gain);
static void taos_update_sat_als(void);
static int taos_prox_on(void);
static int taos_prox_off(void);
static int taos_prox_calibrate(void);
static void taos_prox_offset_cal_work_func(struct work_struct *work);
static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable);
static int taos_write_cal_file(char *file_path,unsigned int value);
static int taos_read_cal_value(char *file_path);
static enum hrtimer_restart  taos_prox_unwakelock_work_func(struct hrtimer *timer);


static dev_t const tmd2772_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static dev_t const tmd2772_light_dev_t     = MKDEV(MISC_MAJOR, 102);

static struct class         *proximity_class;
static struct class         *light_class;


//iVIZM
static int mcount = 0; //iVIZM
static bool pro_ft = false; //by clli2
static bool flag_prox_debug = false;
static bool flag_als_debug  = false;
static bool flag_just_open_light = false;
static unsigned int als_poll_delay = 1000;
static unsigned int prox_debug_delay_time = 0;
static int als_poll_time_mul  = 1;
static unsigned char reg_addr = 0;
static bool wakeup_from_sleep = false;


static const struct i2c_device_id tmd2772_idtable_id[] = {
	{ "ams,ams-sensor", 0 },
	{ },
};

static struct of_device_id of_tmd2772_idtable[] = {
	{ .compatible = "ams,ams-sensor",},
	{}
};

MODULE_DEVICE_TABLE(i2c, tmd2772_idtable);

struct i2c_driver tmd2772_driver = {
	.driver = {
		.name = "ams-sensor-tmd2772",
	.of_match_table = of_tmd2772_idtable,
		//.pm = NULL,
	},
	.id_table = tmd2772_idtable_id,
	.probe = tmd2772_probe,
	.remove = __devexit_p(tmd2772_remove),
#ifdef CONFIG_PM_SLEEP //by clli2
    .resume = taos_resume,
    .suspend = taos_suspend,
#endif

};


// per-device data
struct taos_data {
	struct i2c_client *client;
	struct cdev cdev;
	unsigned int addr;
	//struct input_dev *input_dev;//iVIZM
	//struct work_struct work;//iVIZM
	struct delayed_work work;//iVIZM
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct taos_wake_lock proximity_wakelock;//iVIZM
	struct mutex lock;
	struct device *class_dev;
	struct delayed_work als_poll_work;
	struct delayed_work prox_offset_cal_work;
	struct delayed_work prox_flush_work;
	struct hrtimer  prox_unwakelock_timer;
	struct input_dev *p_idev;
	struct input_dev *a_idev;

	struct device *proximity_dev;
	struct device *light_dev;
	struct device *gesture_dev;

	char taos_id;
	char taos_name[TAOS_ID_NAME_SIZE];

	char *prox_name;
	char *als_name;
	bool prox_calibrate_flag;
	bool prox_calibrate_result;
	bool prox_offset_cal_result;

	bool phone_is_sleep;

	int  light_percent;

	bool prox_offset_cal_verify;
	bool prox_calibrate_verify;

	int  prox_calibrate_times;
	int  prox_thres_hi_max;
	int  prox_thres_hi_min;
	int  prox_thres_lo_max;
	int  prox_thres_lo_min;
	int  prox_data_max;
	int  prox_manual_calibrate_threshold;
	int  irq_pin_num;
	int  prox_led_plus_cnt;
	int  prox_offset_cal_ability;
	int  prox_offset_cal_per_bit;
	int  prox_uncover_data;

	char *chip_name;
	struct regulator	*vdd;
	struct regulator	*vio;


	bool prox_on;
	bool als_on;
	bool irq_enabled;
	bool irq_work_status;
	bool init;
	int als_poll_time_mul;

	struct semaphore update_lock;
	char valid;
} *taos_datap;

// device configuration by clli2
struct taos_cfg *taos_cfgp;

// fixme:
static const u32 calibrate_target_param = 300000;
static const u16 als_time_param = 41;
static const u16 gain_trim_param = 512;          //NULL
static const u8 filter_history_param = 3;        //NULL
static const u8 filter_count_param = 1;          //NULL

static u16 scale_factor_param_prox = 6;
static u16 scale_factor_param_als = 6;
/* gain_param  00--1X, 01--8X, 10--16X, 11--120X
 */
static u8 gain_param = 0;                  //same as prox-gain_param 1:0 8X
static u16 prox_calibrate_hi_param = 500;
static u16 prox_calibrate_lo_param = 330;
static const u16 prox_threshold_hi_param = PROX_DEFAULT_THRESHOLD_HIGH;
static const u16 prox_threshold_lo_param = PROX_DEFAULT_THRESHOLD_LOW;
static const u16 als_threshold_hi_param  = 3000;
static const u16 als_threshold_lo_param  = 10;
static u8  prox_int_time_param     = 0xF0;//0xCD; // time of the ALS ADC TIME, TIME = (255 - prox_int_time_param) * 2.72ms
static u8  prox_adc_time_param     = 0xFF; // time of the PRO ADC TIME, TIME = (255 - prox_int_time_param) * 2.72ms
static u8  prox_wait_time_param    = 0xFF; // time of the    Wait TIME, TIME = (255 - prox_int_time_param) * 2.72ms
/*7~4->pls,3~0->als*/
static const  u8 prox_intr_filter_param  = 0x33; // Int filter, Bit7--Bit4:PROX  Bit3--Bit0:ALS
static const u8  prox_config_param       = 0x00; // wait long time disable
/*pulse/62.5Khz  less  32 recommand*/
static u8  prox_pulse_cnt_param    = PROX_LED_PULSE_CNT; //PROX LED pluse count to send for each measure 0x00--0xff:0--255
/* 7:6 11->100ma        00->12.5ma
   5:4 01->ch0          10->ch1    11->both
   1:0(als gain ctrol)  1X 8X 16X 128X        */
// Proximity uses the CH1 diode ??
static u8  prox_gain_param = 0x20;   //50ma     8X
static u8  prox_config_offset_param  = 0x0;
// prox info
struct taos_prox_info prox_cal_info[20];
struct taos_prox_info prox_cur_info;
struct taos_prox_info * const prox_cur_infop = &prox_cur_info;
static struct timer_list prox_poll_timer;
static u16 sat_als = 0;
static u16 sat_prox = 0;



// device reg init values
u8 taos_triton_reg_init[16] = {0x00,0xFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00};

// lux time scale
struct time_scale_factor  {
    u16 numerator;
    u16 denominator;
    u16 saturation;
};
struct time_scale_factor TritonTime = {1, 0, 0};
struct time_scale_factor *lux_timep = &TritonTime;

// gain table
u8 taos_triton_gain_table[] = {1, 8, 16, 120};

// lux data
struct lux_data {
    u16 ratio;
    u16 clear;
    u16 ir;
};
struct lux_data TritonFN_lux_data[] = {
    { 9830,  8320,  15360 },
    { 12452, 10554, 22797 },
    { 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
    { 0,     0,     0     }
};
struct lux_data *lux_tablep = TritonFN_lux_data;
static int lux_history[TAOS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};//iVIZM

static int taos_get_data(void);

static void taos_enable_irq(void)
{
	// check:
	if (taos_datap->irq_enabled) {
		pr_info("double enable-irq, return here\n");
		return;
	} else {
		taos_datap->irq_enabled  = true;
	}
	enable_irq(taos_datap->client->irq);
}

static void taos_disable_irq(bool flag_sync)
{
	if (!taos_datap->irq_enabled) {
		pr_info("double disable irq, return here\n");
		return;
	} else {
		taos_datap->irq_enabled  = false;
	}

	if (flag_sync) {
		disable_irq(taos_datap->client->irq);
	} else {
		disable_irq_nosync(taos_datap->client->irq);
	}
}

static ssize_t attr_set_prox_led_pulse_cnt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int ret;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	prox_pulse_cnt_param = val;

	// nonsense: if (NULL!=taos_cfgp) {	/* how could this be? */
	taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0) {
		dev_err(dev, "failed to write the prox_pulse_cnt reg\n");
	}

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_prox_led_pulse_cnt(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "prox_led_pluse_cnt is %d\n", taos_cfgp->prox_pulse_cnt);
}

// fixme: max_value!
static inline int set_register(struct device *dev, const char *buf, size_t size, u8* out, int register_name)
{
	unsigned long val;	/* not int? */
	int value;

	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}
	value = 255 - val;
	*out = value;

	if (i2c_smbus_write_byte_data(taos_datap->client, register_name,
				      value) < 0) {
		dev_err(dev, "failed to write the wait_time reg\n");
	}

	dev_err(dev, "exit\n");
	return size;
}

// is there a mutex synchronization?
static ssize_t attr_set_als_adc_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return set_register(dev, buf, size, &taos_cfgp->prox_int_time, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME));
}

static ssize_t attr_get_als_adc_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "als_adc_time is 2.72 * %d ms\n", 255 - taos_cfgp->prox_int_time);
}

static ssize_t attr_set_prox_adc_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return set_register(dev, buf, size, &taos_cfgp->prox_adc_time, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME));
}

static ssize_t attr_get_prox_adc_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "prox_adc_time is 2.72 * %d ms\n", 255 - taos_cfgp->prox_adc_time);
}

static ssize_t attr_set_wait_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return set_register(dev, buf, size, /* taos_datap,*/ &taos_cfgp->prox_wait_time, (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME));
}

static ssize_t attr_get_wait_time(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "wait_time is 2.72 * %d ms\n", 255 - taos_cfgp->prox_wait_time);
}

static ssize_t attr_set_prox_led_strength_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int ret;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val>4 || val<=0) {
		dev_err(dev, "input error, please input a number 1~~4");
	} else {
		val = 4 - val;
		prox_gain_param = (prox_gain_param & 0x3F) | (val<<6);

		taos_cfgp->prox_gain = prox_gain_param;
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0) { /* mmc: 0f ?? is that TAOS_TRITON_GAIN? */
			dev_err(dev, "failed to write the prox_led_strength reg\n");
			}
	}

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_prox_led_strength_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *p_led_strength[4] = {"100", "50", "25", "12.5"};
	return sprintf(buf, "prox_led_strength is %s mA\n", p_led_strength[(taos_cfgp->prox_gain) >> 6]);
}


static ssize_t attr_set_als_gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	int ret;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val>4 || val<=0) {
		dev_err(dev, "input error, please input a number 1~~4");
	} else {
		val = val-1;
		prox_gain_param = (prox_gain_param & 0xFC) | val;
		gain_param      = prox_gain_param & 0x03;

		taos_cfgp->gain      = gain_param;
		taos_cfgp->prox_gain = prox_gain_param;
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0) {
			dev_err(dev, "failed to write the prox_led_strength reg\n");
		}
	}


	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_als_gain(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 als_gain[4] = {1, 8, 16, 120};
	return sprintf(buf, "als gain is x%d\n", als_gain[taos_cfgp->prox_gain & 0x03]);
}

static ssize_t attr_set_prox_debug_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	prox_debug_delay_time = val;

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_prox_debug_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "prox_debug_delay_time is %d\n", prox_debug_delay_time);
}


inline static int set_boolean(struct device *dev,
			      const char *buf, size_t size, bool *variable)
{
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}
	if (val) {
		*variable = true;
	} else {
		*variable = false;
	}

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_prox_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	return set_boolean(dev, buf, size, &flag_prox_debug);
}

static ssize_t attr_prox_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "flag_prox_debug is %s\n", flag_prox_debug? "true" : "false");
}

static ssize_t attr_prox_calibrate_start_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "flag_prox_calibrate_start is %s\n", flag_prox_debug ? "true" : "false");
}


static ssize_t attr_prox_offset_cal_start_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "flag_prox_offset_cal_start is %s\n", flag_prox_debug? "true" : "false");
}


static ssize_t attr_prox_phone_is_sleep_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size) {
	struct taos_data *chip = dev_get_drvdata(dev);
	unsigned int recv;
	int ret = kstrtouint(buf, 10, &recv);
	if (ret) {
		dev_err(dev, "input error\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	if (recv==chip->phone_is_sleep) {
		dev_info(dev, "double %s phone_is_sleep\n",recv? "enable" : "false");
	} else {
		chip->phone_is_sleep = recv;
		dev_info(dev, "success %s phone_is_sleep\n",recv? "enable" : "false");
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t attr_prox_phone_is_sleep_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct taos_data *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "prox: phone sleep is %s\n\n", chip->phone_is_sleep? "true" : "false");
}

static ssize_t attr_prox_prox_wakelock_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size) {
	struct taos_data *chip = dev_get_drvdata(dev);
	unsigned int recv;
	int ret = kstrtouint(buf, 10, &recv);
	if (ret) {
		dev_err(dev, "input error\n");
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	if (recv) {
		taos_wakelock_ops(&(chip->proximity_wakelock),true);
	} else {
		//cancel_delayed_work_sync(&chip->prox_unwakelock_work);
		hrtimer_cancel(&taos_datap->prox_unwakelock_timer);
		taos_wakelock_ops(&(chip->proximity_wakelock),false);
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t attr_prox_prox_wakelock_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct taos_data *chip = dev_get_drvdata(dev);
	dev_info(dev, "proximity_wakelock is %s\n",chip->proximity_wakelock.locked ? "true" : "false");
	return snprintf(buf, PAGE_SIZE, "proximity_wakelock is %s\n",chip->proximity_wakelock.locked ? "true" : "false");
}


//als
static ssize_t attr_set_als_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	return set_boolean(dev, buf, size, &flag_als_debug);
}

static ssize_t attr_get_als_debug(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "flag_prox_debug is %s\n", flag_als_debug? "true" : "false");
}

static ssize_t attr_set_irq(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val) {
		taos_enable_irq();
	} else {
		taos_disable_irq(true);
	}

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_irq(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "flag_irq is %s\n", taos_datap->irq_enabled? "true" : "false");
}


static ssize_t attr_set_prox_calibrate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	int val,ret;

	ret=kstrtouint(buf, 10, &val);
	dev_err(dev, "enter %s\n", __func__);
	if (ret<0) {
		return -EINVAL;
	}

	if (val>1) {
		taos_datap->prox_calibrate_times= val;
		taos_prox_calibrate();
	} else {
		dev_err(dev, "your input error, please input a number that bigger than 1\n");
	}

	return size;
}

static ssize_t attr_prox_thres_high_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val>0) {
		prox_calibrate_hi_param = val;
	} else {
		dev_err(dev, "you input error, please input a number that bigger than 0\n");
	}

	dev_err(dev, "exit\n");
	return size;
}


static ssize_t attr_prox_thres_high_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "prox_calibrate_hi_param is %d\n",prox_calibrate_hi_param);
}

static ssize_t attr_prox_thres_low_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val>0) {
		prox_calibrate_lo_param = val;
	} else {
		dev_err(dev, "you input error, please input a number that bigger than 0\n");
	}

	dev_err(dev, "exit\n");
	return size;
}


static ssize_t attr_prox_thres_low_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "prox_calibrate_lo_param is %d\n",prox_calibrate_lo_param);
}

static ssize_t attr_prox_thres_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	// misleading: not prox_calibrate_lo_param, but prox_threshold_lo !
	return sprintf(buf, "prox_calibrate_lo_param is %d\n prox_calibrate_hi_param is %d\n",taos_cfgp->prox_threshold_lo,taos_cfgp->prox_threshold_hi);
}

static ssize_t attr_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	static long value;
	int rc;

	rc = kstrtol(buf, 10, &value);
	if (rc)
		return -EINVAL;
	mutex_lock(&taos_datap->lock);

	if (value==1) {
		if( (rc=taos_read_cal_value(CAL_THRESHOLD))<0) {
			mutex_unlock(&taos_datap->lock);
			return -EINVAL;
		} else {
			taos_datap->prox_calibrate_flag = false;
			taos_datap->prox_manual_calibrate_threshold =rc;
			taos_cfgp->prox_threshold_hi = rc;

			taos_cfgp->prox_threshold_hi = (taos_cfgp->prox_threshold_hi < taos_datap->prox_thres_hi_max) ? taos_cfgp->prox_threshold_hi : taos_datap->prox_thres_hi_max;
			taos_cfgp->prox_threshold_hi = (taos_cfgp->prox_threshold_hi > taos_datap->prox_thres_hi_min) ? taos_cfgp->prox_threshold_hi : taos_datap->prox_thres_hi_min;

			taos_cfgp->prox_threshold_lo = taos_cfgp->prox_threshold_hi - PROX_THRESHOLD_DISTANCE;

			taos_cfgp->prox_threshold_lo = (taos_cfgp->prox_threshold_lo < taos_datap->prox_thres_lo_max) ? taos_cfgp->prox_threshold_lo : taos_datap->prox_thres_lo_max;
			taos_cfgp->prox_threshold_lo = (taos_cfgp->prox_threshold_lo > taos_datap->prox_thres_lo_min) ? taos_cfgp->prox_threshold_lo : taos_datap->prox_thres_lo_min;
			input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
			input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
			input_sync(taos_datap->p_idev);
			dev_err(dev, "prox_th_high  = %d\n", taos_cfgp->prox_threshold_hi);
			dev_err(dev, "prox_th_low   = %d\n", taos_cfgp->prox_threshold_lo);
		}
	} else {
		mutex_unlock(&taos_datap->lock);
		return -EINVAL;
	}

	mutex_unlock(&taos_datap->lock);

	return size;
}
static ssize_t attr_set_als_scale_factor_param_prox(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val>0) {
		scale_factor_param_prox = val;
		taos_cfgp->scale_factor_prox = scale_factor_param_prox;
	} else {
		dev_err(dev, "you input error, please input a number that bigger than 0\n");
	}

	dev_err(dev, "exit\n");
	return size;
}


static ssize_t attr_get_als_scale_factor_param_prox(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "als_scale_factor_param_prox is %d\n",taos_cfgp->scale_factor_prox);
}

static ssize_t attr_set_als_scale_factor_param_als(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (val>0) {
		scale_factor_param_als = val;
		taos_cfgp->scale_factor_als = scale_factor_param_als;
	} else {
		dev_err(dev, "you input error, please input a number that bigger than 0\n");
	}


	dev_err(dev, "exit\n");
	return size;
}


static ssize_t attr_get_als_scale_factor_param_als(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "als_scale_factor_param_als is %d\n",taos_cfgp->scale_factor_als);
}


static ssize_t attr_get_prox_threshold_high(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_cfgp->prox_threshold_hi);
}

static ssize_t attr_set_prox_threshold_high(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	taos_cfgp->prox_threshold_hi = val;

	dev_err(dev, "exit\n");
	return size;
}



static ssize_t attr_get_prox_threshold_low(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_cfgp->prox_threshold_lo);
}

static ssize_t attr_set_prox_threshold_low(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	taos_cfgp->prox_threshold_lo = val;

	dev_err(dev, "exit\n");
	return size;
}


static ssize_t attr_set_prox_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	int ret;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	prox_config_offset_param = val;

	taos_cfgp->prox_config_offset = prox_config_offset_param;
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0) {
		dev_err(dev, "failed to write the prox_config_offset  reg\n");
	}

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_prox_offset(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "prox_config_offset_param is %d\n", taos_cfgp->prox_config_offset);
}

static ssize_t attr_prox_calibrate_result_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_datap->prox_calibrate_result);
}
static ssize_t attr_prox_offset_cal_result_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_datap->prox_offset_cal_result);
}

static ssize_t attr_prox_thres_hi_max(struct device *dev,
		struct device_attribute *attr, char *buf) {
	dev_info(dev,  "prox_thres_hi_max is %d\n",taos_datap->prox_thres_hi_max);
	return sprintf(buf, "%d", PROX_THRESHOLD_HIGH_MAX);
}


static ssize_t attr_prox_thres_hi_min(struct device *dev,
		struct device_attribute *attr, char *buf) {
	dev_info(dev, "prox_thres_hi_min is %d\n",taos_datap->prox_thres_hi_min);
	return sprintf(buf, "%d", taos_datap->prox_thres_hi_min);
}

static ssize_t attr_prox_data_safa_range_max_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	dev_info(dev,  "PROX_DATA_SAFE_RANGE_MAX is %d\n",PROX_DATA_SAFE_RANGE_MAX);
	return sprintf(buf, "%d", PROX_DATA_SAFE_RANGE_MAX);
}


static ssize_t attr_prox_data_safa_range_min_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	dev_info(dev, "PROX_DATA_SAFE_RANGE_MIN is %d\n",PROX_DATA_SAFE_RANGE_MIN);

	return sprintf(buf, "%d", PROX_DATA_SAFE_RANGE_MIN);
}

static ssize_t attr_chip_name_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%s", taos_datap->chip_name);
}


static ssize_t attr_prox_data_max(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_datap->prox_data_max);
}

static ssize_t attr_prox_manual_calibrate_threshold(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_datap->prox_manual_calibrate_threshold);
}

static ssize_t attr_set_reg_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	reg_addr = val;

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_get_reg_addr(struct device *dev,
		struct device_attribute *attr, char *buf) {

	dev_err(dev, "enter %s\n", __func__);
	dev_err(dev, "reg_addr = 0x%02X\n",reg_addr);
	return strlen(buf);
	dev_err(dev, "exit\n");

}


static ssize_t attr_set_reg_data(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val;
	int ret;
	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	if (100==reg_addr) {
		dev_err(dev, "reg addr error!\n");
	} else {
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|reg_addr), val))) < 0) {
			dev_err(dev, "failed write reg\n");
		}
	}

	dev_err(dev, "exit\n");
	return size;
}


static ssize_t attr_get_reg_data(struct device *dev,
		struct device_attribute *attr, char *buf) {
	unsigned char i;
	if (100 == reg_addr) {
		for (i=0x00; i<=0x0F; i++) {
			i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | i));
			dev_err(dev, "reg[0x%02X] = 0x%02X",i,i2c_smbus_read_byte(taos_datap->client));
		}
		for (i=0x11; i<=0x19; i++) {
			i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | i));
			dev_err(dev, "reg[0x%02X] = 0x%02X",i,i2c_smbus_read_byte(taos_datap->client));
		}

		i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | 0x1F));
		dev_err(dev, "reg[0x1F] = 0x%02X",i2c_smbus_read_byte(taos_datap->client));
	} else {
		i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | reg_addr));
		dev_err(dev, "reg[0x%02X] = 0x%02X",reg_addr,i2c_smbus_read_byte(taos_datap->client));
	}

	return strlen(buf);
}

static ssize_t attr_get_prox_value(struct device *dev,
		struct device_attribute *attr, char *buf) {
	dev_err(dev, "get_prox_value\n");
	schedule_delayed_work(&taos_datap->prox_flush_work, msecs_to_jiffies(200));
	return sprintf(buf, "%d\n", prox_cur_infop->prox_data % 100000);
}

static ssize_t attr_get_als_value(struct device *dev,
		struct device_attribute *attr, char *buf) {
	dev_err(dev, "get_prox_value\n");
	//taos_als_get_data();
	schedule_delayed_work(&taos_datap->als_poll_work, msecs_to_jiffies(200));
	return strlen(buf);
}

///***************************************************************************************///
//light
static ssize_t attr_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct taos_data *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_on);
}

static ssize_t attr_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	struct taos_data *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	mutex_lock(&chip->lock);

	if (value) {
		taos_sensors_als_poll_on();
	} else {
		taos_sensors_als_poll_off();
	}

	mutex_unlock(&chip->lock);

	return size;
}

static ssize_t attr_als_poll_time_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "als_poll_time = %d\n", als_poll_delay);
}

static ssize_t attr_als_poll_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	unsigned long time;
	int rc;

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return -EINVAL;
	als_poll_delay = time;
	return size;
}

//prox
static ssize_t attr_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct taos_data *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_on);
}

static ssize_t attr_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	struct taos_data *chip = dev_get_drvdata(dev);
	bool value;
	if (strtobool(buf, &value))
		return -EINVAL;
	mutex_lock(&chip->lock);
	dev_info(dev, "enter %s\n", __func__);

	if (value) {
		taos_prox_on();
	} else {
		taos_prox_off();
	}
	dev_info(dev, "exit\n");
	mutex_unlock(&chip->lock);

	return size;
}

static ssize_t attr_prox_init_show(struct device *dev,
	struct device_attribute *attr, char *buf) {
	struct taos_data *chip = dev_get_drvdata(dev);
	return sprintf(buf, "chip->init%d\n", chip->init);

}

static ssize_t attr_prox_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	int value =1;
	int ret =0,err=1;
	ret = kstrtouint(buf, 10, &value);
	if (ret)
		return -EINVAL;
	mutex_lock(&taos_datap->lock);
	dev_info(dev, "enter %s\n", __func__);

	if (value ==1) {
		if((ret=taos_read_cal_value(PATH_PROX_OFFSET))>0) {
			taos_cfgp->prox_config_offset = ret;
		}

		if((ret=taos_read_cal_value(PATH_PROX_UNCOVER_DATA))>0) {
			taos_datap->prox_uncover_data = ret;
			taos_datap->prox_thres_hi_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_SAFE_DISTANCE;
			taos_datap->prox_thres_hi_max = taos_datap->prox_thres_hi_min + PROX_THRESHOLD_DISTANCE / 2;
			taos_datap->prox_thres_hi_max = (taos_datap->prox_thres_hi_max > PROX_THRESHOLD_HIGH_MAX) ? PROX_THRESHOLD_HIGH_MAX : taos_datap->prox_thres_hi_max;
			taos_datap->prox_thres_lo_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_DISTANCE;
			taos_datap->prox_thres_lo_max = taos_datap->prox_uncover_data + PROX_THRESHOLD_DISTANCE * 2;

			dev_err(dev, "prox_uncover_data = %d\n", taos_datap->prox_uncover_data);
			dev_err(dev, "prox_thres_hi range is [%d--%d]\n", taos_datap->prox_thres_hi_min, taos_datap->prox_thres_hi_max);
			dev_err(dev, "prox_thres_lo range is [%d--%d]\n", taos_datap->prox_thres_lo_min, taos_datap->prox_thres_lo_max);
		}

		if((ret=taos_read_cal_value(CAL_THRESHOLD))<0) {
			dev_err(dev, "tmg399x_prox_init<0\n");
			err=taos_write_cal_file(CAL_THRESHOLD,0);
			if(err<0) {
				dev_err(dev, "ERROR=%s\n",CAL_THRESHOLD);
				mutex_unlock(&taos_datap->lock);
				return -EINVAL;
			}
			taos_datap->prox_calibrate_flag = true;
		} else {
			if (ret==0) {
				taos_datap->prox_calibrate_flag = true;
				dev_err(dev, "taos_prox_calibrate==1\n");
			} else {
				taos_datap->prox_calibrate_flag = false;
				taos_datap->prox_manual_calibrate_threshold = ret;

				taos_cfgp->prox_threshold_hi = ret;
				taos_cfgp->prox_threshold_hi = (taos_cfgp->prox_threshold_hi < taos_datap->prox_thres_hi_max) ? taos_cfgp->prox_threshold_hi : taos_datap->prox_thres_hi_max;
				taos_cfgp->prox_threshold_hi = (taos_cfgp->prox_threshold_hi > taos_datap->prox_thres_hi_min) ? taos_cfgp->prox_threshold_hi : taos_datap->prox_thres_hi_min;
				taos_cfgp->prox_threshold_lo = taos_cfgp->prox_threshold_hi - PROX_THRESHOLD_DISTANCE;
				taos_cfgp->prox_threshold_lo = (taos_cfgp->prox_threshold_lo < taos_datap->prox_thres_lo_max) ? taos_cfgp->prox_threshold_lo : taos_datap->prox_thres_lo_max;
				taos_cfgp->prox_threshold_lo = (taos_cfgp->prox_threshold_lo > taos_datap->prox_thres_lo_min) ? taos_cfgp->prox_threshold_lo : taos_datap->prox_thres_lo_min;
				input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
				input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
				input_sync(taos_datap->p_idev);
				dev_err(dev, "taos_prox_init %d\n", ret);
			}
		}
	} else {
		dev_err(dev, "ERROR=tmg399x_prox_init_store\n");
		mutex_unlock(&taos_datap->lock);
		return -EINVAL;
	}

	dev_info(dev, "prox_threshold_hi = %d, prox_threshold_lo = %d\n", taos_cfgp->prox_threshold_hi, taos_cfgp->prox_threshold_lo);

	dev_info(dev, "exit\n");
	mutex_unlock(&taos_datap->lock);

	return size;
}

static ssize_t attr_prox_offset_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	int value =1;
	int ret =0;
	ret = kstrtouint(buf, 10, &value);
	if (ret) {
		return -EINVAL;
	}
	mutex_lock(&taos_datap->lock);
	dev_info(dev, "enter %s\n", __func__);

	if (value ==1) {
		schedule_delayed_work(&taos_datap->prox_offset_cal_work, msecs_to_jiffies(0));
		mutex_unlock(&taos_datap->lock);
	} else {
		dev_err(dev, "input error\n");
		mutex_unlock(&taos_datap->lock);
		return -EINVAL;
	}
	dev_info(dev, "exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_verify_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_datap->prox_offset_cal_verify);
}

static ssize_t attr_prox_offset_cal_verify_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val = 0;

	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	taos_datap->prox_offset_cal_verify = val;

	dev_err(dev, "exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_verify_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d", taos_datap->prox_calibrate_verify);
}

static ssize_t attr_prox_calibrate_verify_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val = 0;

	dev_err(dev, "enter %s\n", __func__);
	if (strict_strtoul(buf, 10, &val)) {
		return -EINVAL;
	}

	taos_datap->prox_calibrate_verify = val;

	dev_err(dev, "exit\n");
	return size;
}


static struct device_attribute attrs_light[] = {
    __ATTR(enable,                         0640,   attr_als_enable_show,                       attr_als_enable_store),
    __ATTR(light_gain,                     0644,   attr_get_als_gain,                          attr_set_als_gain),
    __ATTR(light_debug,                    0644,   attr_get_als_debug,                         attr_set_als_debug),
    __ATTR(light_adc_time,                 0644,   attr_get_als_adc_time,                      attr_set_als_adc_time),
    __ATTR(light_scale_factor_param,       0644,   attr_get_als_scale_factor_param_als,        attr_set_als_scale_factor_param_als),
    __ATTR(prox_scale_factor_param,        0644,   attr_get_als_scale_factor_param_prox,       attr_set_als_scale_factor_param_prox),
    __ATTR(delay,                          0640,   attr_als_poll_time_show,                    attr_als_poll_time_store),
    __ATTR(light_value,                    0444,   attr_get_als_value,                         NULL),
};

/*
static struct device_attribute attributes[] = {
#ifdef CONFIG_FEATURE_ZTEMT_SENSORS_LOG_ON
	__ATTR(taos_als_adc_time,                   0644,   attr_get_als_adc_time,                      attr_set_als_adc_time),

	__ATTR(taos_prox_calibrate,                 0644,   NULL,                                       attr_set_prox_calibrate),
	__ATTR(taos_prox_calibrate_hi_param,        0644,   attr_get_prox_calibrate_hi_param,           attr_set_prox_calibrate_hi_param),
	__ATTR(taos_prox_calibrate_lo_param,        0644,   attr_get_prox_calibrate_lo_param,           attr_set_prox_calibrate_lo_param),

	__ATTR(th_hi,                               0644,   attr_get_threshold_hi,                      attr_set_threshold_hi),
	__ATTR(th_low,                              0644,   attr_get_threshold_lo,                      attr_set_threshold_lo),
};
*/




static struct device_attribute attrs_prox[] = {
    __ATTR(chip_name,                      0640,   attr_chip_name_show,                        NULL),
    __ATTR(enable,                         0640,   attr_prox_enable_show,                      attr_prox_enable_store),
    __ATTR(prox_init,                      0640,   attr_prox_init_show,                        attr_prox_init_store),
    __ATTR(prox_led_pulse_cnt,             0644,   attr_get_prox_led_pulse_cnt,                attr_set_prox_led_pulse_cnt),
    __ATTR(prox_adc_time,                  0644,   attr_get_prox_adc_time,                     attr_set_prox_adc_time),
    __ATTR(prox_led_strength_level,        0644,   attr_get_prox_led_strength_level,           attr_set_prox_led_strength_level),
    __ATTR(prox_debug_delay,               0644,   attr_get_prox_debug_delay,                  attr_set_prox_debug_delay),
    __ATTR(prox_calibrate,                 0644,   NULL,                                       attr_set_prox_calibrate),
    __ATTR(prox_threshold_high,            0644,   attr_get_prox_threshold_high,               attr_set_prox_threshold_high),
    __ATTR(prox_threshold_low,             0644,   attr_get_prox_threshold_low,                attr_set_prox_threshold_low),
    __ATTR(prox_offset,                    0644,   attr_get_prox_offset,                       attr_set_prox_offset),
    __ATTR(prox_value,                     0644,   attr_get_prox_value,                        NULL),
    __ATTR(prox_calibrate_result,          0640,   attr_prox_calibrate_result_show,            NULL),
    __ATTR(prox_thres_param_high,          0640,   attr_prox_thres_high_show,                  attr_prox_thres_high_store),
    __ATTR(prox_thres_param_low,           0640,   attr_prox_thres_low_show,                   attr_prox_thres_low_store),
    __ATTR(prox_thres,                     0640,   attr_prox_thres_show,                       attr_prox_thres_store),
    __ATTR(prox_debug,                     0640,   attr_prox_debug_show,                       attr_prox_debug_store),
    __ATTR(prox_calibrate_start,           0640,   attr_prox_calibrate_start_show,             attr_prox_debug_store), /* ?? */
    __ATTR(prox_thres_max,                 0644,   attr_prox_thres_hi_max,                     NULL),
    __ATTR(prox_thres_min,                 0644,   attr_prox_thres_hi_min,                     NULL),
    __ATTR(prox_data_max,                  0640,   attr_prox_data_max,                         NULL),
    __ATTR(prox_manual_calibrate_threshold,0644,   attr_prox_manual_calibrate_threshold,       NULL),
    __ATTR(prox_phone_is_sleep,            0640,   attr_prox_phone_is_sleep_show,              attr_prox_phone_is_sleep_store),
    __ATTR(prox_wakelock,                  0640,   attr_prox_prox_wakelock_show,               attr_prox_prox_wakelock_store),
    __ATTR(reg_addr,                       0644,   attr_get_reg_addr,                          attr_set_reg_addr),
    __ATTR(reg_data,                       0644,   attr_get_reg_data,                          attr_set_reg_data),
    __ATTR(irq_status,                     0644,   attr_get_irq,                               attr_set_irq),
    __ATTR(wait_time,                      0644,   attr_get_wait_time,                         attr_set_wait_time),
    __ATTR(prox_offset_cal_start,          0640,   attr_prox_offset_cal_start_show,     attr_prox_debug_store), /* ?? */
    __ATTR(prox_offset_cal,                0640,   attr_get_prox_offset,           attr_prox_offset_cal_store),
    __ATTR(prox_offset_cal_result,         0640,   attr_prox_offset_cal_result_show,            NULL),
    __ATTR(prox_data_safe_range_max,       0644,   attr_prox_data_safa_range_max_show,  NULL),
    __ATTR(prox_data_safe_range_min,       0644,   attr_prox_data_safa_range_min_show,  NULL),
    __ATTR(prox_offset_cal_verify,         0644,   attr_prox_offset_cal_verify_show,     attr_prox_offset_cal_verify_store),
    __ATTR(prox_calibrate_verify,          0644,   attr_prox_calibrate_verify_show,      attr_prox_calibrate_verify_store),
};


static int create_sysfs_interfaces(struct device *dev, struct device_attribute *attrs, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, attrs + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int create_sysfs_interfaces_prox(struct device *dev)
{
	return create_sysfs_interfaces(dev, attrs_prox, ARRAY_SIZE(attrs_prox));
}

static int create_sysfs_interfaces_light(struct device *dev)
{
	return create_sysfs_interfaces(dev, attrs_light, ARRAY_SIZE(attrs_light));
}


static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable)
{
	if (enable == wakelock->locked) {
		pr_info("double %s %s, do nothing\n",enable? "lock" : "unlock", wakelock->name);
		return;
	}

	if (enable) {
		wake_lock(&wakelock->lock);
	} else {
		wake_unlock(&wakelock->lock);
	}

	wakelock->locked = enable;

	pr_info("%s %s \n",enable? "lock" : "unlock",wakelock->name);
}

static int taos_write_cal_file(char *file_path,unsigned int value)
{
	struct file *file_p;
	char write_buf[10];
	mm_segment_t old_fs;
	int vfs_write_retval=0;
	if (NULL==file_path) {
		pr_err("file_path is NULL\n");

	}
	memset(write_buf, 0, sizeof(write_buf));
	sprintf(write_buf, "%d\n", value);
	file_p = filp_open(file_path, O_CREAT|O_RDWR , 0665);
	if (IS_ERR(file_p)) {
		pr_err("[open file <%s>failed]\n",file_path);
		goto error;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write_retval = vfs_write(file_p, (char*)write_buf, sizeof(write_buf), &file_p->f_pos);
	if (vfs_write_retval < 0) {
		pr_err("[write file <%s>failed]\n",file_path);
		goto error;
	}

	set_fs(old_fs);
	filp_close(file_p, NULL);


	return 1;

error:
	return -1;
}


static int taos_read_cal_value(char *file_path)
{
	struct file *file_p;
	int vfs_read_retval = 0;
	mm_segment_t old_fs;
	char read_buf[32];
	unsigned short read_value;

	if (NULL==file_path) {
		pr_err("file_path is NULL\n");
		goto error;
	}

	memset(read_buf, 0, 32);

	file_p = filp_open(file_path, O_RDONLY , 0);
	if (IS_ERR(file_p)) {
		pr_err("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read_retval = vfs_read(file_p, (char*)read_buf, 16, &file_p->f_pos);
	if (vfs_read_retval < 0) {
		pr_err("[read file <%s>failed]\n",file_path);
		goto error;
	}

	set_fs(old_fs);
	filp_close(file_p, NULL);

	if (kstrtou16(read_buf, 10, &read_value) < 0) {
		pr_err("[kstrtou16 %s failed]\n",read_buf);
		goto error;
	}

	pr_err("[the content of %s is %s]\n", file_path, read_buf);

	return read_value;

error:
	return -1;
}

static void taos_irq_work_func(struct work_struct * work) //iVIZM
{
	int retry_times = 0;
	int ret;
	mutex_lock(&taos_datap->lock);
	pr_info("enter %s\n", __func__);
	if (wakeup_from_sleep) {
		pr_info("wakeup_from_sleep = true\n");
		mdelay(50);
		wakeup_from_sleep = false;
	}

	for (retry_times=0; retry_times<=50; retry_times++) {
		ret = taos_get_data();
		if (ret >= 0) {
			break;
		}
		mdelay(20);
	}
	taos_interrupts_clear();

	hrtimer_cancel(&taos_datap->prox_unwakelock_timer);
	taos_datap->irq_work_status = false;
// pr_info("########  taos_irq_work_func enter   hrtimer_start #########\n");
	hrtimer_start(&taos_datap->prox_unwakelock_timer, ktime_set(3, 0), HRTIMER_MODE_REL);

	taos_enable_irq();
	pr_info("retry_times = %d\n", retry_times);
	mutex_unlock(&taos_datap->lock);
}

static void taos_flush_work_func(struct work_struct * work) //iVIZM
{
	taos_prox_threshold_set(taos_datap);
}
static irqreturn_t taos_irq_handler(int irq, void *dev_id) //iVIZM
{
	pr_info("enter %s\n", __func__);
	taos_datap->irq_work_status = true;
	taos_disable_irq(false);
	taos_wakelock_ops(&(taos_datap->proximity_wakelock), true);
	if (0==queue_work(taos_datap->irq_work_queue, &taos_datap->irq_work)) {
		pr_info("schedule_work failed!\n");
	}
	pr_info("exit\n");
	return IRQ_HANDLED;
}

static int taos_get_data(void)//iVIZM
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS));

	if (ret < 0) {
		pr_err("read TAOS_TRITON_STATUS failed\n");
		return ret;
	} else {
		ret = taos_prox_threshold_set(taos_datap);
	}
	return ret;
}


static int taos_interrupts_clear(void)//iVIZM
{
	int ret = 0;
	if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07)))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte(2) failed in taos_work_func()\n");
		return (ret);
	}
	return ret;
}

static int taos_als_get_data(void)//iVIZM
{
	int ret = 0;
	u8 reg_val;
	int lux_val = 0;
	if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
		return (ret);
	}
	reg_val = i2c_smbus_read_byte(taos_datap->client);
	if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
		return -ENODATA;
	if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
		return (ret);
	}
	reg_val = i2c_smbus_read_byte(taos_datap->client);
	if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
		return -ENODATA;

	if ((lux_val = taos_get_lux()) < 0) {
		pr_err("TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n", lux_val);
	}

	if (lux_val<TAOS_ALS_GAIN_DIVIDE && gain_param!=TAOS_ALS_GAIN_8X) {
		taos_als_gain_set(TAOS_ALS_GAIN_8X);
	} else {
		if (lux_val>TAOS_ALS_GAIN_DIVIDE && gain_param!=TAOS_ALS_GAIN_1X) {
			taos_als_gain_set(TAOS_ALS_GAIN_1X);
		}
	}

	if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME)))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte failed in ioctl als_data\n");
		return (ret);
	}

	reg_val = i2c_smbus_read_byte(taos_datap->client);

	if (flag_als_debug) {
		pr_info("reg_val = %d lux_val = %d\n", reg_val, lux_val);
	}

	if (reg_val != prox_int_time_param) {
		lux_val = (lux_val * (101 - (0XFF - reg_val))) / 20;
	}

	lux_val = taos_lux_filter(lux_val);

	if (flag_als_debug) {
		pr_info("lux_val = %d",lux_val);
	}

	lux_val = lux_val * taos_datap->light_percent / 100;
	lux_val = lux_val > 10000 ? 10000 : lux_val;

	input_report_rel(taos_datap->a_idev, REL_X, lux_val + 1);
	input_sync(taos_datap->a_idev);

	return ret;
}

static int taos_prox_threshold_set(struct taos_data *taos_datap)
{
	static char pro_buf[4]; //iVIZM
	int i,ret = 0;
	u8 chdata[6];
	u16 proxdata = 0;
	u16 cleardata = 0;

	for (i = 0; i < 6; i++) {
		chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW| (TAOS_TRITON_ALS_CHAN0LO + i))));
	}
	cleardata = chdata[0] + chdata[1]*256;
	proxdata = chdata[4] + chdata[5]*256;

	if (pro_ft || flag_prox_debug) {
		pro_buf[0] = 0xff;
		pro_buf[1] = 0xff;
		pro_buf[2] = 0xff;
		pro_buf[3] = 0xff;

		for( mcount=0; mcount<4; mcount++) {
			// mmc:
			if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x08) + mcount, pro_buf[mcount]))) < 0) {
				pr_err("TAOS: i2c_smbus_write_byte_data failed in taos prox threshold set\n");
				return (ret);
			}
		}

		if (pro_ft) {
			pr_info("init the prox threshold");
		}

		if (flag_prox_debug) {
			mdelay(prox_debug_delay_time);
			pr_info( "proxdata = %d",proxdata);
			input_report_rel(taos_datap->p_idev, REL_MISC, proxdata>0? proxdata:1);

		}
		pro_ft = false;
	} else {
		if (proxdata < taos_cfgp->prox_threshold_lo)
		{   //FAR
			pro_buf[0] = 0x0;
			pro_buf[1] = 0x0;
			pro_buf[2] = taos_cfgp->prox_threshold_hi & 0x0ff;
			pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;
			pr_info("Far!!! proxdata = %d, hi = %d, low = %d\n",proxdata,taos_cfgp->prox_threshold_hi,taos_cfgp->prox_threshold_lo);
			input_report_rel(taos_datap->p_idev, REL_X, proxdata>0? proxdata:1);
		} else {
			if (proxdata > taos_cfgp->prox_threshold_hi)
			{   //NEAR
				if (cleardata > ((sat_als * 80) / 100)) {
					pr_err("TAOS: %u <= %u*0.8 int data\n", proxdata, sat_als);
					msleep(100);
					return -ENODATA;
				}
				pro_buf[0] = taos_cfgp->prox_threshold_lo & 0x0ff;
				pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
				pro_buf[2] = 0xff;
				pro_buf[3] = 0xff;
				pr_info("Near!!! proxdata = %d, hi = %d, low = %d\n",proxdata,taos_cfgp->prox_threshold_hi,taos_cfgp->prox_threshold_lo);
				input_report_rel(taos_datap->p_idev, REL_X, proxdata);
			} else {
				if( (taos_cfgp->prox_threshold_hi-proxdata) > (proxdata-taos_cfgp->prox_threshold_lo)) {
					//FAR
					pro_buf[0] = 0x0;
					pro_buf[1] = 0x0;
					pro_buf[2] = taos_cfgp->prox_threshold_hi & 0x0ff;
					pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;
					pr_info("Far!!! proxdata = %d, hi = %d, low = %d\n",proxdata,taos_cfgp->prox_threshold_hi,taos_cfgp->prox_threshold_lo);
					input_report_rel(taos_datap->p_idev, REL_X, (taos_cfgp->prox_threshold_lo-50)>0? (taos_cfgp->prox_threshold_lo-50):1);
				} else {
					//NEAR
					if (cleardata > ((sat_als*80)/100)) {
						pr_err("TAOS: %u <= %u*0.8 int data\n", proxdata, sat_als);
						msleep(100);
						return -ENODATA;
					}
					pro_buf[0] = taos_cfgp->prox_threshold_lo & 0x0ff;
					pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
					pro_buf[2] = 0xff;
					pro_buf[3] = 0xff;
					pr_info("Near!!! proxdata = %d, hi = %d, low = %d\n", proxdata, taos_cfgp->prox_threshold_hi, taos_cfgp->prox_threshold_lo);
					input_report_rel(taos_datap->p_idev, REL_X, taos_cfgp->prox_threshold_hi+50);
				}
			}
		}
	}

	input_sync(taos_datap->p_idev);

	for( mcount=0; mcount<4; mcount++) {
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x08) + mcount, pro_buf[mcount]))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte_data failed in taos prox threshold set\n");
			return (ret);
		}
	}

	return ret;
}

// driver init
static int __init taos_init(void)
{
#ifdef CONFIG_ZTEMT_SENSORS_ALS_PS_AUTO_DETECT
	return 0;
#else
	return i2c_add_driver(&tmd2772_driver);
#endif
}

// driver exit
static void __exit taos_exit(void)
{
	i2c_del_driver(&tmd2772_driver);
}

static int tmd2772_parse_dt(struct taos_data *chip)
{
	int rc = 0;
	u32 tmp;
	struct device_node *np = chip->client->dev.of_node;

	chip->irq_pin_num = of_get_named_gpio(np, "ams,irq-gpio", 0);
	pr_info("irq_pin_num is %d\n",chip->irq_pin_num);

	rc = of_property_read_u32(np, "ams,prox-offset-cal-ability-tmd2772", &tmp);
	chip->prox_offset_cal_ability = (!rc ? tmp : 8);
	pr_info("prox_offset_cal_ability is %d\n", chip->prox_offset_cal_ability);

	rc = of_property_read_u32(np, "ams,prox-led-plus-cnt-tmd2772", &tmp);
	chip->prox_led_plus_cnt = (!rc ? tmp : 8);
	pr_info("prox_led_plus_cnt is %d\n", chip->prox_led_plus_cnt);

	rc = of_property_read_u32(np, "ams,light-percent", &tmp);
	chip->light_percent = (!rc ? tmp : 100);
	pr_info("light_percent is %d\n", chip->light_percent);

	return 0;
}

static void tmd2772_data_init(struct taos_data *taos_datap)
{
	taos_datap->als_on  = false;
	taos_datap->prox_on = false;
	taos_datap->init = false;
	taos_datap->als_poll_time_mul = 1;
	taos_datap->prox_name = "proximity";
	taos_datap->als_name  = "light";
	taos_datap->chip_name = "tmd2772";
	taos_datap->prox_calibrate_result = false;
	taos_datap->prox_offset_cal_result = false;
	taos_datap->prox_offset_cal_verify = true;
	taos_datap->prox_calibrate_verify = true;
	taos_datap->prox_thres_hi_max = PROX_THRESHOLD_HIGH_MAX;
	taos_datap->prox_thres_hi_min = PROX_THRESHOLD_HIGH_MIN;
	taos_datap->prox_data_max     = PROX_DATA_MAX;
	taos_datap->prox_offset_cal_per_bit = taos_datap->prox_offset_cal_ability * taos_datap->prox_led_plus_cnt;
	taos_datap->prox_uncover_data = 0;
	taos_datap->prox_calibrate_times = 10;
	taos_datap->prox_calibrate_flag = true;//true :auto_calibrate,false :manual_calibrate
	taos_datap->prox_manual_calibrate_threshold = 0;
	taos_datap->proximity_wakelock.name = "proximity-wakelock";
	taos_datap->proximity_wakelock.locked = false;
	taos_datap->phone_is_sleep = false;
	taos_datap->irq_work_status = false;
	taos_datap->irq_enabled = true;
}
/*POWER SUPPLY VOLTAGE RANGE */

#define TMD2772_VDD_MIN_UV 2000000
#define TMD2772_VDD_MAX_UV 3300000
#define TMD2772_VIO_MIN_UV 1750000
#define TMD2772_VIO_MAX_UV 1950000

static int tmd2772_power_init(struct taos_data *chip, bool on)
{
	int rc = 0;
	pr_info("on= %d\n", on);

	if (!on) {
		if (regulator_count_voltages(chip->vdd) > 0)
			regulator_set_voltage(chip->vdd, 0,
					      TMD2772_VDD_MAX_UV);
		regulator_put(chip->vdd);

		if (regulator_count_voltages(chip->vio) > 0)
			regulator_set_voltage(chip->vio, 0,
					      TMD2772_VIO_MAX_UV);
		regulator_put(chip->vio);
	} else {
		chip->vdd = regulator_get(&chip->client->dev,"vdd");
		if(IS_ERR(chip->vdd)) {
			rc = PTR_ERR(chip->vdd);
			dev_err(&chip->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (regulator_count_voltages(chip->vdd) > 0) {
			rc = regulator_set_voltage(chip->vdd,
						   TMD2772_VDD_MIN_UV, TMD2772_VDD_MAX_UV);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto err_vdd_set;
			}
		}

		chip->vio = regulator_get(&chip->client->dev, "vio");
		if (IS_ERR(chip->vio)) {
			rc = PTR_ERR(chip->vio);
			dev_err(&chip->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(chip->vio) > 0) {
			rc = regulator_set_voltage(chip->vio,
						   TMD2772_VIO_MIN_UV, TMD2772_VIO_MAX_UV);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set;
			}
		}

		rc = regulator_enable(chip->vdd);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(chip->vio);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
	}

	pr_err("success\n");

	return 0;
err_vio_enable:
	regulator_disable(chip->vdd);
err_vdd_enable:
	if (regulator_count_voltages(chip->vio) > 0)
		regulator_set_voltage(chip->vio, 0, TMD2772_VIO_MAX_UV);
err_vio_set:
	regulator_put(chip->vio);
err_vio_get:
	if (regulator_count_voltages(chip->vdd) > 0)
		regulator_set_voltage(chip->vdd, 0, TMD2772_VDD_MAX_UV);
err_vdd_set:
	regulator_put(chip->vdd);
	return rc;
}


static int tmd2772_chip_detect(struct taos_data *chip)
{
	int i                = 0;
	int ret              = 0;
	int chip_id          = 0;
	int detect_max_times = 10;

	for (i=0; i<detect_max_times; i++) {
		chip_id = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + TAOS_TRITON_CHIPID)));

		if (chip_id == 0x39) {
			break;
		} else {
			pr_err("retry %d time, chip id is [0x%x] was read does not match [TMD27723-0x39]\n", i + 1, chip_id);
		}
	}

	if (i>=detect_max_times) {
		pr_err("chip detect failed\n");
		ret =  -ENODEV;
	}

	return ret;
}

// client probe
static int __devinit tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp)
{
	int ret = 0;
	int i = 0;
	unsigned char buf[TAOS_MAX_DEVICE_REGS];
	char *device_name;
	pr_info("Probe Start\n");

	if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c smbus byte data functions unsupported\n");
		return -EOPNOTSUPP;
	}

	taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (!taos_datap) {
		pr_err("kmalloc for struct taos_data failed\n");
		return -ENOMEM;
	}

	taos_datap->client = clientp;

	i2c_set_clientdata(clientp, taos_datap);

	ret = tmd2772_power_init(taos_datap, 1);
	if (ret < 0) {
		goto power_init_failed;
	}

	msleep(10);

	ret = tmd2772_chip_detect(taos_datap);
	if (ret) {
		goto read_chip_id_failed;
	}

	INIT_WORK(&(taos_datap->irq_work),taos_irq_work_func);

	sema_init(&taos_datap->update_lock,1);
	mutex_init(&(taos_datap->lock));
	wake_lock_init(&taos_datap->proximity_wakelock.lock, WAKE_LOCK_SUSPEND, "proximity-wakelock");

	tmd2772_parse_dt(taos_datap);

	tmd2772_data_init(taos_datap);

	for (i = 0; i < TAOS_MAX_DEVICE_REGS; i++) {
		if ((ret = (i2c_smbus_write_byte(clientp, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + i))))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte() to control reg failed in taos_probe()\n");
			return(ret);
		}
		buf[i] = i2c_smbus_read_byte(clientp);
	}

	if ((ret = taos_device_name(buf, &device_name)) == 0) {
		pr_err("TAOS: chip id that was read found mismatched by taos_device_name(), in taos_probe()\n");
		return -ENODEV;
	}
	if (strcmp(device_name, TAOS_DEVICE_ID)) {
		pr_err("TAOS: chip id that was read does not match expected id in taos_probe()\n");
		return -ENODEV;
	} else {
		pr_err("TAOS: chip id of %s that was read matches expected id in taos_probe()\n", device_name);
		pr_err( "TAOS: chip id of %s that was read matches expected id in taos_probe()\n", device_name);
	}
	if ((ret = (i2c_smbus_write_byte(clientp, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte() to control reg failed in taos_probe()\n");
		return(ret);
	}
	strlcpy(clientp->name, TAOS_DEVICE_ID, I2C_NAME_SIZE);
	strlcpy(taos_datap->taos_name, TAOS_DEVICE_ID, TAOS_ID_NAME_SIZE);
	taos_datap->valid = 0;
	if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL))) {
		pr_err("TAOS: kmalloc for struct taos_cfg failed in taos_probe()\n");
		return -ENOMEM;
	}
	taos_cfgp->calibrate_target = calibrate_target_param;
	taos_cfgp->als_time = als_time_param;
	taos_cfgp->scale_factor_als = scale_factor_param_als;
	taos_cfgp->scale_factor_prox = scale_factor_param_prox;
	taos_cfgp->gain_trim = gain_trim_param;
	taos_cfgp->filter_history = filter_history_param;
	taos_cfgp->filter_count = filter_count_param;
	taos_cfgp->gain = gain_param;
	taos_cfgp->als_threshold_hi = als_threshold_hi_param;//iVIZM
	taos_cfgp->als_threshold_lo = als_threshold_lo_param;//iVIZM
	taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
	taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
	taos_cfgp->prox_int_time = prox_int_time_param;
	taos_cfgp->prox_adc_time = prox_adc_time_param;
	taos_cfgp->prox_wait_time = prox_wait_time_param;
	taos_cfgp->prox_intr_filter = prox_intr_filter_param;
	taos_cfgp->prox_config = prox_config_param;
	taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
	taos_cfgp->prox_gain = prox_gain_param;
	taos_cfgp->prox_config_offset=prox_config_offset_param;
	sat_als = (256 - taos_cfgp->prox_int_time) << 10;
	sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;

	/*dmobile ::power down for init ,Rambo liu*/
	pr_err("TAOS:Rambo::light sensor will pwr down \n");
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CNTRL), 0x00))) < 0) {
		pr_err("TAOS:Rambo, i2c_smbus_write_byte_data failed in power down\n");
		return (ret);
	}


	taos_datap->irq_work_queue = create_singlethread_workqueue("taos_work_queue");
	if (!taos_datap->irq_work_queue) {
		ret = -ENOMEM;
		pr_info("---------%s: %d: cannot create work taos_work_queue, err = %d",__func__,__LINE__,ret);
		return ret;
	}

	ret = gpio_request(taos_datap->irq_pin_num, "ALS_PS_INT");
	if (ret) {
		pr_info("gpio %d is busy and then to free it\n",taos_datap->irq_pin_num);

		gpio_free(taos_datap->irq_pin_num);
		ret = gpio_request(taos_datap->irq_pin_num, "ALS_PS_INT");
		if (ret) {
			pr_info("gpio %d is busy and then to free it\n",taos_datap->irq_pin_num);
			return ret;
		}
	} else {
		pr_info("get gpio %d success\n",taos_datap->irq_pin_num);
	}


	ret = gpio_tlmm_config(GPIO_CFG(taos_datap->irq_pin_num, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	taos_datap->client->irq = gpio_to_irq(taos_datap->irq_pin_num);

	ret = request_threaded_irq(taos_datap->client->irq, NULL, &taos_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "taos_irq", taos_datap);
	if (ret != 0) {
		gpio_free(taos_datap->irq_pin_num);
		return(ret);
	}

	taos_disable_irq(true);
	INIT_DELAYED_WORK(&taos_datap->als_poll_work, taos_als_poll_work_func);
	INIT_DELAYED_WORK(&taos_datap->prox_offset_cal_work, taos_prox_offset_cal_work_func);
	INIT_DELAYED_WORK(&taos_datap->prox_flush_work, taos_flush_work_func);

	//  INIT_DELAYED_WORK(&taos_datap->prox_unwakelock_work, taos_prox_unwakelock_work_func);
	hrtimer_init(&taos_datap->prox_unwakelock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	(taos_datap->prox_unwakelock_timer).function = taos_prox_unwakelock_work_func ;
	// hrtimer_start(&taos_datap->prox_unwakelock_timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	proximity_class = class_create(THIS_MODULE, "proximity");
	light_class     = class_create(THIS_MODULE, "light");

	taos_datap->proximity_dev = device_create(proximity_class, NULL, tmd2772_proximity_dev_t, &tmd2772_driver ,"proximity");
	if (IS_ERR(taos_datap->proximity_dev)) {
		ret = PTR_ERR(taos_datap->proximity_dev);
		pr_err("device_create proximity failed\n");
		goto create_proximity_dev_failed;
	}

	taos_datap->light_dev= device_create(light_class, NULL, tmd2772_light_dev_t, &tmd2772_driver ,"light");
	if (IS_ERR(taos_datap->light_dev)) {
		ret = PTR_ERR(taos_datap->light_dev);
		pr_err("device_create light failed\n");
		goto create_light_dev_failed;
	}

	//prox input
	taos_datap->p_idev = input_allocate_device();
	if (!taos_datap->p_idev) {
		pr_err("no memory for input_dev '%s'\n",taos_datap->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	taos_datap->p_idev->name = taos_datap->prox_name;
	taos_datap->p_idev->id.bustype = BUS_I2C;
	dev_set_drvdata(&taos_datap->p_idev->dev, taos_datap);
	ret = input_register_device(taos_datap->p_idev);
	if (ret) {
		input_free_device(taos_datap->p_idev);
		pr_err("cant register input '%s'\n",taos_datap->prox_name);
		goto input_p_register_failed;
	}

	set_bit(EV_REL, taos_datap->p_idev->evbit);
	set_bit(REL_X,  taos_datap->p_idev->relbit);
	set_bit(REL_Y,  taos_datap->p_idev->relbit);
	set_bit(REL_Z,  taos_datap->p_idev->relbit);
	set_bit(REL_MISC,  taos_datap->p_idev->relbit);

	//light input
	taos_datap->a_idev = input_allocate_device();
	if (!taos_datap->a_idev) {
		pr_err("no memory for input_dev '%s'\n",taos_datap->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	taos_datap->a_idev->name = taos_datap->als_name;
	taos_datap->a_idev->id.bustype = BUS_I2C;

	/*
	  set_bit(EV_ABS, chip->a_idev->evbit);
	  set_bit(ABS_MISC, chip->a_idev->absbit);
	  input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	*/

	set_bit(EV_REL, taos_datap->a_idev->evbit);
	set_bit(REL_X,  taos_datap->a_idev->relbit);
	set_bit(REL_Y,  taos_datap->a_idev->relbit);


	//chip->a_idev->open = tmg399x_als_idev_open;
	//chip->a_idev->close = tmg399x_als_idev_close;
	dev_set_drvdata(&taos_datap->a_idev->dev, taos_datap);
	ret = input_register_device(taos_datap->a_idev);
	if (ret) {
		input_free_device(taos_datap->a_idev);
		pr_err("cant register input '%s'\n",taos_datap->prox_name);
		goto input_a_register_failed;
	}

	dev_set_drvdata(taos_datap->proximity_dev, taos_datap);
	dev_set_drvdata(taos_datap->light_dev, taos_datap);


	create_sysfs_interfaces_prox(taos_datap->proximity_dev);
	create_sysfs_interfaces_light(taos_datap->light_dev);

	pr_info("Probe OK\n");
	return 0;


input_a_register_failed:
	input_free_device(taos_datap->a_idev);
input_a_alloc_failed:

input_p_register_failed:
	input_free_device(taos_datap->p_idev);
input_p_alloc_failed:

create_light_dev_failed:
	taos_datap->light_dev = NULL;
	class_destroy(light_class);

create_proximity_dev_failed:
	taos_datap->proximity_dev = NULL;
	class_destroy(proximity_class);
read_chip_id_failed:
	tmd2772_power_init(taos_datap, 0);
power_init_failed:
	kfree(taos_datap);

	pr_info("Probe Failed\n");
	return (ret);
}

#ifdef CONFIG_PM_SLEEP
//don't move these pm blew to ioctl
//resume
static int taos_resume(struct i2c_client *client)
{
	int ret = 0;
	pr_info("enter %s\n", __func__);
	if(1 == taos_datap->prox_on) {
		pr_info("----------%s: %d: disable irq wakeup\n",__func__,__LINE__);
		ret = disable_irq_wake(taos_datap->client->irq);
	}
	if(ret < 0)
		pr_err("TAOS: disable_irq_wake failed\n");
	pr_info("eixt\n");
	return ret ;
}

//suspend
static int taos_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	pr_info("enter %s\n", __func__);
	if(1 == taos_datap->prox_on) {
		pr_info("----------%s: %d: enable irq wakeup\n",__func__,__LINE__);
		ret = enable_irq_wake(taos_datap->client->irq);
	}
	if(ret < 0) {
		pr_err("TAOS: enable_irq_wake failed\n");
	}

	wakeup_from_sleep = true;

	pr_info("eixt\n");
	return ret ;
}

#endif
// client remove
static int __devexit tmd2772_remove(struct i2c_client *client)
{
	int ret = 0;

	return (ret);
}


// read/calculate lux value
static int taos_get_lux(void)
{
	int raw_clear = 0, raw_ir = 0, raw_lux = 0;
	u32 lux = 0;
	u32 ratio = 0;
	u8 dev_gain = 0;
	u16 Tint = 0;
	struct lux_data *p;
	int ret = 0;
	u8 chdata[4];
	int tmp = 0, i = 0,tmp_gain=1;
	for (i = 0; i < 4; i++) {
		if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte() to chan0/1/lo/hi reg failed in taos_get_lux()\n");
			return (ret);
		}
		chdata[i] = i2c_smbus_read_byte(taos_datap->client);
	}

	tmp = (taos_cfgp->als_time + 25)/50;            //if atime =100  tmp = (atime+25)/50=2.5   time = 2.7*(256-atime)=  412.5
	TritonTime.numerator = 1;
	TritonTime.denominator = tmp;

	tmp = 300 * taos_cfgp->als_time;               //tmp = 300*atime  400
	if(tmp > 65535)
		tmp = 65535;
	TritonTime.saturation = tmp;
	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir    = chdata[3];
	raw_ir    <<= 8;
	raw_ir    |= chdata[2];

	raw_clear *= ((taos_cfgp->scale_factor_als )*tmp_gain);
	raw_ir *= (taos_cfgp->scale_factor_prox );

	if(raw_ir > raw_clear) {
		raw_lux = raw_ir;
		raw_ir = raw_clear;
		raw_clear = raw_lux;
	}
	dev_gain = taos_triton_gain_table[taos_cfgp->gain & 0x3];
	if(raw_clear >= lux_timep->saturation)
		return(TAOS_MAX_LUX);
	if(raw_ir >= lux_timep->saturation)
		return(TAOS_MAX_LUX);
	if(raw_clear == 0)
		return(0);
	if(dev_gain == 0 || dev_gain > 127) {
		pr_err("TAOS: dev_gain = 0 or > 127 in taos_get_lux()\n");
		return -1;
	}
	if(lux_timep->denominator == 0) {
		pr_err("TAOS: lux_timep->denominator = 0 in taos_get_lux()\n");
		return -1;
	}
	ratio = (raw_ir<<15)/raw_clear;
	for (p = lux_tablep; p->ratio && p->ratio < ratio; p++);
#ifdef WORK_UES_POLL_MODE
	if(!p->ratio) {//iVIZM
		if(lux_history[0] < 0)
			return 0;
		else
			return lux_history[0];
	}
#endif
	Tint = taos_cfgp->als_time;
	raw_clear = ((raw_clear*400 + (dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
	raw_ir = ((raw_ir*400 +(dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
	lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
	lux = (lux + 32000)/64000;
	if(lux > TAOS_MAX_LUX) {
		lux = TAOS_MAX_LUX;
	}
	return(lux);
}

static int taos_lux_filter(int lux)
{
	static u8 middle[] = {1,0,2,0,0,2,0,1};
	int index;

	lux_history[2] = lux_history[1];
	lux_history[1] = lux_history[0];
	lux_history[0] = lux;

	if(lux_history[2] < 0) { //iVIZM
		if(lux_history[1] > 0)
			return lux_history[1];
		else
			return lux_history[0];
	}
	index = 0;
	if(lux_history[0] > lux_history[1] )
		index += 4;
	if(lux_history[1] > lux_history[2] )
		index += 2;
	if(lux_history[0] > lux_history[2] )
		index++;
	return(lux_history[middle[index]]);
}

// verify device
static int taos_device_name(unsigned char *bufp, char **device_name)
{
	/*
	  int i=0 ,j;
	  for (i = 0; i < TAOS_MAX_DEVICE_REGS; i++) {
	  j=bufp[i];
	  printk("(bufp[i=%x]=%x,\n",i,j);
	  }
	*/
	*device_name="tritonFN";
	return(1);
}

// proximity poll
static int taos_prox_poll(struct taos_prox_info *prxp)
{
	int i = 0, ret = 0; //wait_count = 0;
	u8 chdata[6];

	for (i = 0; i < 6; i++) {
		chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_AUTO | (TAOS_TRITON_ALS_CHAN0LO + i))));
	}
	prxp->prox_clear = chdata[1];
	prxp->prox_clear <<= 8;
	prxp->prox_clear |= chdata[0];
	if (prxp->prox_clear > ((sat_als * 80) / 100)) {
		pr_err("TAOS: %u <= %u*0.8 poll data\n", prxp->prox_clear, sat_als);
		return -ENODATA;
	}
	prxp->prox_data = chdata[5];
	prxp->prox_data <<= 8;
	prxp->prox_data |= chdata[4];

	return (ret);
}

// prox poll timer function
static void taos_prox_poll_timer_func(unsigned long param)
{
	int ret = 0;

	if ((ret = taos_prox_poll(prox_cur_infop)) < 0) {
		pr_err("TAOS: call to prox_poll failed in taos_prox_poll_timer_func()\n");
		return;
	}
	taos_prox_poll_timer_start();
	return;
}

// start prox poll timer
static void taos_prox_poll_timer_start(void)
{
    init_timer(&prox_poll_timer);
    prox_poll_timer.expires = jiffies + (HZ/10);
    prox_poll_timer.function = taos_prox_poll_timer_func;
    add_timer(&prox_poll_timer);
    return;
}

static void taos_update_sat_als(void)
{
	u8 reg_val = 0;
	int ret = 0;

	if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME)))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte failed in ioctl als_calibrate\n");
		return;
	}

	reg_val = i2c_smbus_read_byte(taos_datap->client);

	sat_als = (256 - reg_val) << 10;
}

static int taos_als_gain_set(unsigned als_gain)
{
	int ret;
	prox_gain_param = (prox_gain_param & 0xFC) | als_gain;
	gain_param      = prox_gain_param & 0x03;

	if (NULL!=taos_cfgp) {
		taos_cfgp->gain      = gain_param;
		taos_cfgp->prox_gain = prox_gain_param;
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0) {
			pr_err("failed to write the prox_led_strength reg\n");
			return -EINVAL;
		}
	} else {
		pr_err("taos_cfgp is NULL\n");
		return -EINVAL;
	}

	return ret;
}

static void taos_als_poll_work_func(struct work_struct *work)
{
	taos_als_get_data();
	if (true == taos_datap->als_on) {
		schedule_delayed_work(&taos_datap->als_poll_work, msecs_to_jiffies(als_poll_time_mul * als_poll_delay));
	}
}

static int taos_prox_offset_cal_prepare(void)
{
	int ret =1;
	if (NULL!=taos_cfgp) {
		taos_cfgp->prox_config_offset = 0;
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0) {
			pr_err("failed to write the prox_config_offset  reg\n");
			return ret;
		}
	} else {
		pr_err("taos_cfgp is NULL\n");
		return -EINVAL;
	}
	return ret;
}

static int taos_prox_offset_calculate(int data, int target)
{
	int offset;

	if (data > PROX_DATA_TARGET) {
		offset = (data - PROX_DATA_TARGET) * 10 / taos_datap->prox_offset_cal_per_bit;
	} else {
		offset = (PROX_DATA_TARGET - data) * 16 / taos_datap->prox_offset_cal_per_bit + 128;
	}

	pr_err("TAOS:------------ taos_cfgp->prox_offset = %d\n",offset );

	return offset;
}

static int taos_prox_uncover_data_get(void)
{
	u8 i = 0, j = 0;
	int prox_sum = 0, ret = 0;
	static struct taos_prox_info prox_info_temp;

	mdelay(20);
	for (i = 0, j = 0; i < PROX_OFFSET_CAL_BUFFER_SIZE / 5; i++) {
		if ((ret = taos_prox_poll(&prox_info_temp)) < 0) {
			pr_err("failed to tmd2772_prox_read_data\n");
		} else {
			j++;
			prox_sum += prox_info_temp.prox_data;
			pr_err("prox_data is %d\n",prox_info_temp.prox_data);
		}
		mdelay(20);
	}

	if(j == 0) {
		ret = -1;
		goto error;
	}

	taos_datap->prox_uncover_data = prox_sum / j;
	taos_datap->prox_thres_hi_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_SAFE_DISTANCE;
	taos_datap->prox_thres_hi_max = taos_datap->prox_thres_hi_min + PROX_THRESHOLD_DISTANCE / 2;
	taos_datap->prox_thres_hi_max = (taos_datap->prox_thres_hi_max > PROX_THRESHOLD_HIGH_MAX) ? PROX_THRESHOLD_HIGH_MAX : taos_datap->prox_thres_hi_max;
	taos_datap->prox_thres_lo_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_DISTANCE;
	taos_datap->prox_thres_lo_max = taos_datap->prox_uncover_data + PROX_THRESHOLD_DISTANCE * 2;

	pr_err("prox_uncover_data = %d\n", taos_datap->prox_uncover_data);
	pr_err("prox_thres_hi range is [%d--%d]\n", taos_datap->prox_thres_hi_min, taos_datap->prox_thres_hi_max);
	pr_err("prox_thres_lo range is [%d--%d]\n", taos_datap->prox_thres_lo_min, taos_datap->prox_thres_lo_max);
	taos_write_cal_file(PATH_PROX_UNCOVER_DATA, taos_datap->prox_uncover_data);

	return 0;

error:
	return ret;
}

static int configure_proximity(void)// taos_datap, taos_cfgp   todo: maybe the const char* call_name; to append in err message:
{
	int ret;
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), taos_cfgp->prox_int_time))) < 0) {
		pr_err("failed write prox_int_time reg\n");
		return ret;
	}
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME), taos_cfgp->prox_adc_time))) < 0) {
		pr_err("failed write prox_adc_time reg\n");
		return ret;
	}
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME), taos_cfgp->prox_wait_time))) < 0) {
		pr_err("failed write prox_wait_time reg\n");
		return ret;
	}
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG), taos_cfgp->prox_config))) < 0) {
		pr_err("failed write prox_config reg\n");
		return ret;
	}
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0) {
		pr_err("failed write prox_pulse_cnt reg\n");
		return ret;
	}
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0) {
		pr_err("i2c_smbus_write_byte_data failed in ioctl prox_on\n");
		return (ret);
	}
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0) {
		pr_err("failed write prox_gain reg\n");
		return (ret);
	}

	return ret;
}

static int taos_prox_offset_cal_process(void)
{
	int ret;
	int prox_sum = 0, prox_mean = 0;
	int i = 0, j = 0;
	u8 reg_val = 0;
	u8 reg_cntrl = 0;

	struct taos_prox_info *prox_cal_info = NULL;
	prox_cal_info = kzalloc(sizeof(struct taos_prox_info) * (PROX_OFFSET_CAL_BUFFER_SIZE), GFP_KERNEL);
	if (NULL == prox_cal_info) {
		pr_err("malloc prox_cal_info failed\n");
		ret = -1;
		goto prox_offset_cal_buffer_error;
	}

	if ((ret = configure_proximity())) {
		goto prox_calibrate_offset_error;
	}

	reg_cntrl = reg_val | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		pr_err("failed write cntrl reg\n");
		goto prox_calibrate_offset_error;
	}

	mdelay(30);

	for (i = 0; i < (PROX_OFFSET_CAL_BUFFER_SIZE); i++) {
		if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) {
			j++;
			pr_err("TAOS: call to prox_poll failed in ioctl prox_calibrate\n");
		}
		prox_sum += prox_cal_info[i].prox_data;

		pr_err("prox get time %d data is %d",i,prox_cal_info[i].prox_data);
		mdelay(30);
	}

	prox_mean = prox_sum/(PROX_OFFSET_CAL_BUFFER_SIZE);
	pr_err("TAOS:------------ taos_cfgp->prox_mean = %d\n",prox_mean );
	if(j != 0)
		goto prox_calibrate_offset_error;

	prox_config_offset_param = taos_prox_offset_calculate(prox_mean, PROX_DATA_TARGET);

	taos_cfgp->prox_config_offset = prox_config_offset_param;

	if((ret=taos_write_cal_file(PATH_PROX_OFFSET,taos_cfgp->prox_config_offset)) < 0)
		goto prox_calibrate_offset_error;


	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0) {
		pr_err(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
		return (ret);
	}

	//read prox data register after update offset register
	ret = taos_prox_uncover_data_get();
	if (ret < 0) {
		pr_err("failed to tmd2772_prox_uncover_data_get\n");
		goto prox_calibrate_offset_error;
	}

	for (i = 0; i < sizeof(taos_triton_reg_init); i++) {
		if(i !=11) {
			if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)),
							      taos_triton_reg_init[i]))) < 0) {
				pr_err("failed write triton_init reg\n");
				goto prox_calibrate_offset_error;
			}
		}
	}




	kfree(prox_cal_info);
	return 1;
prox_calibrate_offset_error:
	pr_err("ERROR\n");
	kfree(prox_cal_info);
prox_offset_cal_buffer_error:

	return -1;
}

static void taos_prox_offset_cal_finish(void)
{

	if (true == (taos_datap->prox_on)) {
		taos_prox_on();
	} else {
		taos_prox_off();
	}
}
static int taos_prox_offset_cal(void)
{
	int ret = 0;
	taos_datap->prox_offset_cal_result = false;

	if ((ret=taos_prox_offset_cal_prepare())<0)
		goto error;

	mdelay(50);

	if ((ret= taos_prox_offset_cal_process())>=0) {
		taos_datap->prox_offset_cal_result = true;
	}

	taos_prox_offset_cal_finish();

	return ret;
error:
	return ret;
}


static void taos_prox_offset_cal_work_func(struct work_struct *work)
{
	taos_prox_offset_cal();
}

static enum hrtimer_restart  taos_prox_unwakelock_work_func(struct hrtimer *timer)
{
	pr_info("######## taos_prox_unwakelock_timer_func #########\n");
	if(false == taos_datap->irq_work_status )
		taos_wakelock_ops(&(taos_datap->proximity_wakelock),false);
	return HRTIMER_NORESTART;

}

static int taos_sensors_als_poll_on(void)
{
	int  ret = 0, i = 0;
	u8   reg_val = 0, reg_cntrl = 0;

	pr_info("######## TAOS IOCTL ALS ON #########\n");

	for (i = 0; i < TAOS_FILTER_DEPTH; i++) {
		lux_history[i] = -ENODATA;
	}

	if (taos_datap->prox_on) {
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), TAOS_ALS_ADC_TIME_WHEN_PROX_ON))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
			return (ret);
		}
	} else {
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), taos_cfgp->prox_int_time))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
			return (ret);
		}
	}

	reg_val = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN));

	//pr_info("reg[0x0F] = 0x%02X\n",reg_val);

	reg_val = reg_val & 0xFC;
	reg_val = reg_val | (taos_cfgp->gain & 0x03);//*16
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
		return (ret);
	}

	reg_cntrl = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL));
	pr_info("reg[0x00] = 0x%02X\n",reg_cntrl);

	reg_cntrl |= (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON);
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
		return (ret);
	}

	schedule_delayed_work(&taos_datap->als_poll_work, msecs_to_jiffies(200));

	flag_just_open_light = true;

	taos_datap->als_on = true;

	taos_update_sat_als();

	return ret;
}

static int taos_sensors_als_poll_off(void)
{
	int  ret = 0, i = 0;
	u8  reg_val = 0;

	pr_info("######## TAOS IOCTL ALS OFF #########\n");

	for (i = 0; i < TAOS_FILTER_DEPTH; i++) {
		lux_history[i] = -ENODATA;
	}

	reg_val = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL));

	//pr_info("reg[0x00] = 0x%02X\n",reg_val);

	if ((reg_val & TAOS_TRITON_CNTL_PROX_DET_ENBL) == 0x00 && (0 == taos_datap->prox_on)) {
		pr_info("TAOS_TRITON_CNTL_PROX_DET_ENBL = 0\n");
		reg_val = 0x00;
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_val))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
			return (ret);
		}

	}

	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), 0XFF))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
		return (ret);
	}

	taos_datap->als_on = false;

	cancel_delayed_work_sync(&taos_datap->als_poll_work);

	taos_update_sat_als();

	return (ret);
}

static int taos_prox_on(void)
{
	int prox_sum = 0, prox_mean = 0, prox_max = 0;
	int  ret = 0;
	u8 reg_cntrl = 0, i = 0 ,j = 0;

	taos_datap->prox_on = 1;
	als_poll_time_mul = 2;

	pr_info("######## TAOS IOCTL PROX ON  ######## \n");

	if (true==taos_datap->als_on) {
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), TAOS_ALS_ADC_TIME_WHEN_PROX_ON))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
			return (ret);
		}
	} else {
		if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), 0XFF))) < 0) {
			pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
			return (ret);
		}
	}
	taos_update_sat_als();

	// fixme: bug!  TAOS_TRITON_ALS_TIME should not be set!   (TAOS_TRITON_PRX_OFFSET  was set as LAST one);
	//
	if ((ret = configure_proximity())) {
		return ret;
	}
	// and this was set in the middle!
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT), taos_cfgp->prox_intr_filter))) < 0) {
		printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
		return (ret);
	}

	reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON    | TAOS_TRITON_CNTL_PROX_INT_ENBL |
		TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_WAIT_TMR_ENBL  ;
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
		return (ret);
	}
	pro_ft = true;
	if (taos_datap->prox_calibrate_flag) {
		prox_sum = 0;
		prox_max = 0;

		mdelay(20);
		for (i = 0, j = 0; i < 5; i++) {
			if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) {
				j++;
				pr_err("TAOS: call to prox_poll failed in ioctl prox_calibrate\n");
			}
			prox_sum += prox_cal_info[i].prox_data;
			if (prox_cal_info[i].prox_data > prox_max)
				prox_max = prox_cal_info[i].prox_data;
			mdelay(20);
		}

		prox_mean = prox_sum/5;
		if (j==0) {
			taos_cfgp->prox_threshold_hi = prox_mean + PROX_THRESHOLD_SAFE_DISTANCE;
			taos_cfgp->prox_threshold_hi = (taos_cfgp->prox_threshold_hi > PROX_THRESHOLD_HIGH_MIN) ? taos_cfgp->prox_threshold_hi : PROX_THRESHOLD_HIGH_MIN;
			taos_cfgp->prox_threshold_hi = (taos_cfgp->prox_threshold_hi > taos_datap->prox_thres_hi_min) ? taos_cfgp->prox_threshold_hi : taos_datap->prox_thres_hi_min;
			taos_cfgp->prox_threshold_lo = taos_cfgp->prox_threshold_hi - PROX_THRESHOLD_DISTANCE;

			if(prox_mean >800 || taos_cfgp->prox_threshold_hi > 1000 || taos_cfgp->prox_threshold_lo > 900) {
				taos_cfgp->prox_threshold_hi= 800;
				taos_cfgp->prox_threshold_lo = taos_cfgp->prox_threshold_hi - PROX_THRESHOLD_DISTANCE;
			}

			pr_info("prox_threshold_hi = %d\n",taos_cfgp->prox_threshold_hi );
			pr_info("prox_threshold_lo = %d\n",taos_cfgp->prox_threshold_lo );

			input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
			input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
			input_sync(taos_datap->p_idev);
		}
	}
	taos_prox_threshold_set(taos_datap);
	taos_enable_irq();
	return (ret);
}


static int taos_prox_off(void)
{
	int ret = 0;
	pr_info("########  TAOS IOCTL PROX OFF  ########\n");

	if (true == (taos_datap->proximity_wakelock).locked) {
		hrtimer_cancel(&taos_datap->prox_unwakelock_timer);
		taos_wakelock_ops(&(taos_datap->proximity_wakelock), false);
	}

	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
		pr_err("TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
		return (ret);
	}

	taos_datap->prox_on = 0;
	als_poll_time_mul = 1;

	if (true == taos_datap->als_on) {
		taos_sensors_als_poll_on();
	}

	// cancel_work_sync(&taos_datap->irq_work);
	if (true == taos_datap->irq_enabled) {
		taos_disable_irq(true);
	}

	return (ret);
}

static int taos_prox_calibrate(void)
{
	int ret;
	int prox_sum = 0, prox_mean = 0, prox_max = 0,prox_min = 1024;
	u8 reg_cntrl = 0;
	u8 reg_val = 0;
	int i = 0, j = 0;

	struct taos_prox_info *prox_cal_info = NULL;
	prox_cal_info = kzalloc(sizeof(struct taos_prox_info) * (taos_datap->prox_calibrate_times), GFP_KERNEL);
	if (NULL == prox_cal_info) {
		pr_err("malloc prox_cal_info failed\n");
		ret = -1;
		goto prox_calibrate_error1;
	}
	// configure with given taos_cfgp:
	if ((ret = configure_proximity())) {
		goto prox_calibrate_error;
	}

	reg_cntrl = reg_val | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
		pr_err("failed write cntrl reg\n");
		goto prox_calibrate_error;
	}

	prox_sum = 0;
	prox_max = 0;
	prox_min =1024;
	mdelay(30);
	for (i = 0; i < (taos_datap->prox_calibrate_times); i++) {
		if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) {
			j++;
			pr_err("TAOS: call to prox_poll failed in ioctl prox_calibrate\n");
		}
		prox_sum += prox_cal_info[i].prox_data;
		if (prox_cal_info[i].prox_data > prox_max)
			prox_max = prox_cal_info[i].prox_data;
		if (prox_cal_info[i].prox_data < prox_min)
			prox_min = prox_cal_info[i].prox_data;
		pr_err("prox get time %d data is %d",i,prox_cal_info[i].prox_data);
		mdelay(30);
	}

	prox_mean = prox_sum/(taos_datap->prox_calibrate_times);
	if(j == 0) {
		taos_cfgp->prox_threshold_hi = ((((prox_max - prox_mean) * prox_calibrate_hi_param) + 50) / 100) + prox_mean + 120;
		taos_cfgp->prox_threshold_lo = ((((prox_max - prox_mean) * prox_calibrate_lo_param) + 50) / 100) + prox_mean + 40;
	}

	if(prox_mean > 700 || taos_cfgp->prox_threshold_hi > 1000 || taos_cfgp->prox_threshold_lo > 900) {
		taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
		taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
		prox_config_offset_param=0x0;
		taos_cfgp->prox_config_offset = prox_config_offset_param;
	}
	pr_err("TAOS:------------ taos_cfgp->prox_threshold_hi = %d\n",taos_cfgp->prox_threshold_hi );
	pr_err("TAOS:------------ taos_cfgp->prox_threshold_lo = %d\n",taos_cfgp->prox_threshold_lo );
	pr_err("TAOS:------------ taos_cfgp->prox_mean = %d\n",prox_mean );
	pr_err("TAOS:------------ taos_cfgp->prox_max = %d\n",prox_max );
	pr_err("TAOS:------------ taos_cfgp->prox_min = %d\n",prox_min );

	for (i = 0; i < sizeof(taos_triton_reg_init); i++) {
		if(i !=11) {
			if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), taos_triton_reg_init[i]))) < 0) {
				pr_err("failed write triton_init reg\n");
				goto prox_calibrate_error;

			}
		}
	}

	input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
	input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
	input_sync(taos_datap->p_idev);
	kfree(prox_cal_info);
	return 1;
prox_calibrate_error:
	pr_err("exit\n");
	kfree(prox_cal_info);
prox_calibrate_error1:

	return -1;
}


MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(taos_init);
module_exit(taos_exit);
