/* arch/arm/mach-rockchip/rk28_battery.c
 *
 * Copyright (C) 2009 Rockchip Corporation.
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
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
/* Debug */
#define BAT_DBG 0
#if BAT_DBG
#define DBG(x...)	printk(KERN_INFO x)
ktime_t ktime_now,ktime_pre,ktime_start;

#else
#define DBG(x...)
#endif

extern void rk29_pm_power_off(void);

/*use adc sample battery capacity*/
#define AC_INSERT_VALUE       100
#define BATT_1V2_MODIFY	100
#define BATT_1V2_VALUE		1420*BATT_1V2_MODIFY/100 
#define BATT_2V50_VALUE	 	  3085//3126//3056//2515

#define BATT_FULL_VALUE	       4200
#define BATT_STEP_FULL_VALUE    4000
#define BATT_EMPTY_VALUE	       3500
#define PERCENT				100
#define BATT_LEVEL_FULL		100
#define BATT_LEVEL_EMPTY	0
#define BATT_PRESENT_TRUE	 1
#define BATT_PRESENT_FALSE  0
#define BATT_NOMAL_VOL_VALUE	4000
#define BATT_VOLTAGE_MAX	4200
#define BATT_VOLTAGE_MIN	3350//3300//3500
#define AD_SAMPLE_TIMES	6
#define AC_OFFSET     500
#define PER_MINUTE	600//(60*1000*1000*1000/(TS_POLL_DELAY))
#define CHARGE_FULL_GATE 4130//4100

#define AD_NO_BATT_VALE       200
#define AD_NO_DC_VALE         200

#define TS_POLL_DELAY		(100*1000*1000)
#define SEC_NUM				2  ///8
#define PER_SEC_NUM		20  ///10

static int bat_vol_cnt = 0;  
static int bat_vol_up_cnt = 0; 
static int bat_vol_no_power_cnt = 0;  
static int bat_status =  POWER_SUPPLY_STATUS_UNKNOWN;
static int bat_health = POWER_SUPPLY_HEALTH_GOOD;
static int bat_capacity = BATT_LEVEL_EMPTY;
static int bat_present = BATT_PRESENT_TRUE;
static int bat_voltage =  BATT_NOMAL_VOL_VALUE;
static int ad_sample_current_time = 0;
unsigned int sample_times = 0;			/*count times (report batter status)*/
static int charger_change_cnt = 0;
//static int g_charge_full_cnt = 0;
/*
 * 注意[xjh]:
 * 界面要显示电池容量的百分比，
 * 而不是当前电池电压在整个锂电池工作电压范围的百分比
 * 电池容量和电池电压不成正比
 * 
 * 锂电池的放电特性:
 * 充满电后，一开始放电，前面锂电池的电压下降比较剧烈(
但不代表电池容量也急剧下降)，
 * 然后进入一个稳定期，在稳定期电压下降很不明显
 * 当电量接近放完时，又有一个剧烈的电压下降，如果继续放电，则会毁损电池。
 * 
 * 锂电池的充电特性:
 * 刚开始以恒定电流进行充电，电压上升比较快。
 * 当达到限制电压后，以涓流方式进行恒压充电。
 */

//电池放电数组
static int batt_step_table[56]={
       /*3510,3515,3520,3530,3545,3560,3575,3590,3600,3610,
	3620,3630,3640,3650,3660,3670,3680,3690,3700,3710,
	3720,3735,3750,3765,3780,3790,3805,3820,3835,3850,
	3865,3880,3895,3910,3925,3940,3955,3970,3985,4000,
	4015,4030,4045,4060,4075,4100,4130,4150,4200,4200,	
	4200,4200,4200,4200,4200,4200*/
	//提高关机时的电压
	3380,3400,3410,3430,3460,3480,3520,3580,3600,3610,
	3620,3630,3640,3650,3660,3670,3680,3690,3700,3710,
	3720,3735,3750,3765,3780,3790,3805,3820,3835,3850,
	3865,3880,3895,3910,3925,3940,3955,3970,3985,4000,
	4015,4030,4045,4060,4075,4100,4130,4150,4200,4200,	
	4200,4200,4200,4200,4200,4200
};

//电池充电数组
static int batt_no_current_step_table[56]={
       /*3490,3510,3765,3780,3795,3810,3820,3830,3840,3850,
	3860,3870,3880,3890,3900,3910,3920,3925,3930,3940,
	3950,3960,3970,3980,3990,4000,4010,4020,4030,4040,
	4050,4060,4070,4080,4090,4100,4105,4110,4115,4120,
	4125,4130,4145,4150,4155,4165,4180,4190,4200,4200,
	4200,4200,4200,4200,4200,4200*/
	3430,3450,3480,3510,3560,3580,3650,3700,3780,3830,
	3860,3870,3880,3890,3900,3910,3920,3925,3930,3940,
	3950,3960,3970,3980,3990,4000,4010,4020,4030,4040,
	4050,4060,4070,4080,4090,4100,4105,4110,4115,4120,
	4125,4130,4145,4150,4155,4165,4180,4190,4200,4200,
	4200,4200,4200,4200,4200,4200
};

static int batt_disp_table[56]={
	0, 1, 3, 5, 7, 9, 11,13,15,17,
	19,21,23,25,27,29,31,33,35,37,
	39,41,43,45,47,50,53,56,59,62,
	65,68,71,74,77,80,83,86,89,92,
	93,94,96,99,99,99,99,99,100,100,
	100,100,100,100,100,100	
};

static int batt_disp_table_no_current[56]={
	0, 1, 3, 5, 7, 9, 11,13,15,17,
	19,21,23,25,27,29,31,33,35,37,
	39,41,43,45,47,50,53,56,59,62,
	65,68,71,74,77,80,83,86,89,92,
	93,94,96,99,99,99,99,99,100,100,
	100,100,100,100,100,100
};

static u16 g_adcbat = 0;	//adccharge;
static int   ac_power_off = 0;   // 0 is not power down,1 is power down enter sleep level2
static int   full_flag = 0;
static int   full_time_cnt = 0;
static int   adj_cnt = 0;

extern     int    get_msc_connect_flag( void );

struct rk2918_battery_data {
	spinlock_t lock;
	int  adc_val;
	int  dc_det_pin;
       int  dc_det_active_level;
	int  charge_ok_pin;
	int  charge_ok_active_level;
	struct power_supply 	battery;
	//struct power_supply	usb;
	struct power_supply	ac;
	struct hrtimer  timer;	
	struct adc_client *client; 
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

struct rk2918_battery_data  *gbatterydata;

static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);
static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val);
static int get_ac_charge_status(void);


static enum power_supply_property rockchip_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property rockchip_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};


static struct power_supply rockchip_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = rockchip_battery_properties,
		.num_properties = ARRAY_SIZE(rockchip_battery_properties),
		.get_property = rockchip_battery_get_property,
	},
	/*{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_usb_get_property,
	},*/
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_ac_get_property,
	},
};
/*
static void record_battery_log(char *buf)
{
	struct file *fp;
	char buf1[500]= "battery test";
	mm_segment_t fs;
	
	fp = filp_open("/flash/battery.txt",O_RDWR|O_APPEND |O_CREAT  ,0644);
	if(IS_ERR(fp)){
		printk("create file error!\n");
		return;
	}
	fs = get_fs();
	set_fs(KERNEL_DS);
	//vfs_read(fp,buf1,sizeof(buf1),&pos);
	vfs_write(fp,buf,strlen(buf),&fp->f_pos);
	set_fs(fs);
	filp_close(fp,NULL);
	//printk("%s--->%d\n",__FUNCTION__,__LINE__);
	return;
}
*/
static void rk2918_battery_callback(struct adc_client *client, void *param, int result)
{
       gbatterydata->adc_val = result;
	return;
}

static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	
	//todo 
	charger =  CHARGER_USB;
       DBG("--------%s-->%s-->%d\n",__FILE__,__FUNCTION__,__LINE__);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		else
	         val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;

}


static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	charger_type_t charger;
	
	//todo 
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE: 
		val->intval = get_ac_charge_status();
		break;
	default:
		return -EINVAL;
	}
	
	return 0;

}

static  int get_ac_charge_status(void)
 {
       gpio_pull_updown(gbatterydata->dc_det_pin, GPIOPullUp);	
       if((gpio_get_value (gbatterydata->dc_det_pin)==gbatterydata->dc_det_active_level)){
		return 1;
 	}
	else {
		full_flag = 0;
	 	return 0;
	}
 }

static int get_battery_charge_status( void )
{
    return 0;
}

static int rockchip_get_battery_status(void)
{

	int  current_vol,i;

	if((get_battery_charge_status()||get_ac_charge_status()) 
		/*&& (bat_capacity !=  BATT_LEVEL_FULL) */)    /*the battery charge*/

		bat_status =POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status =POWER_SUPPLY_STATUS_NOT_CHARGING ;	/*no charge*/

	if(full_flag)
		bat_status =POWER_SUPPLY_STATUS_NOT_CHARGING ;//避免充电的符号闪动	

	ad_sample_current_time++;
       g_adcbat = (g_adcbat + gbatterydata->adc_val)/2;
	   
	adc_async_read(gbatterydata->client);

	if(ad_sample_current_time < AD_SAMPLE_TIMES) 
           return 1;
	
	ad_sample_current_time = 0;		
	if(g_adcbat < AD_NO_BATT_VALE)	/*haven't battery*/ 
	{
		bat_present = BATT_PRESENT_FALSE;	
		goto nobattery;
	}
	bat_present = BATT_PRESENT_TRUE;	/*have battery*/
	
	/*get present voltage*/
	#if 0
	if(bat_voltage > 3700)
	{
		current_vol = (g_adcbat * 2UL * BATT_3V07_VALUE)/1024;		/*current voltage*/
		current_vol += 280;	
	}
	else if(bat_voltage > 3600)
	{
		current_vol = (g_adcbat * 2UL * (BATT_3V07_VALUE))/1024;		/*current voltage*/
		current_vol += 240;	
	}
	else if(bat_voltage > 3500)
	{
		current_vol = (g_adcbat * 2UL * (BATT_3V07_VALUE))/1024;		/*current voltage*/
		current_vol += 220;	
	}
	else //3400~3500
	{
		current_vol = (g_adcbat * 2UL * (BATT_3V05_VALUE ))/1024;		/*current voltage*/
		current_vol += 280;	
	}
       #else
       current_vol = (g_adcbat * 2UL * (BATT_2V50_VALUE ))/1024;
	#endif
	bat_voltage = current_vol;
	
	/*get battery health status*/
	if(batt_step_table[0]>=current_vol)
	{
		if((get_battery_charge_status()||get_ac_charge_status()) )
		{
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =  1;
			bat_vol_no_power_cnt = 0;	
			
		}
		else
		{
			printk("POWER_SUPPLY_HEALTH_GOOD bat_vol_no_power_cnt%d\n",bat_vol_no_power_cnt);
			bat_vol_no_power_cnt++;
			if(bat_vol_no_power_cnt< 80){
			    bat_capacity = 1;
			    return 1;
			}
			bat_vol_no_power_cnt = 0;
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =0;   ///9;
			printk("battery is too poor>>power down!!!");
		}
		return 1;
	}
	else if(CHARGE_FULL_GATE <=current_vol)
	{
			if((gpio_get_value (gbatterydata->charge_ok_pin)==gbatterydata->charge_ok_active_level)) /* current voltage full */							/*xxm*/
			{
			
				bat_health = POWER_SUPPLY_HEALTH_GOOD;
				bat_vol_no_power_cnt = 0;
				bat_capacity =  BATT_LEVEL_FULL;
				//printk("battery charge ok\n");
				full_flag = 1;
				full_time_cnt = 0;
			}else
				{
					bat_health = POWER_SUPPLY_HEALTH_GOOD;
					bat_vol_no_power_cnt = 0;
					bat_capacity =  99;
				}
		return 1;
	}
	bat_vol_no_power_cnt = 0;
#if 1
	if(get_ac_charge_status()){
		for(i=0; i<55; i++){		
		    if((batt_no_current_step_table[i]<=current_vol)&&(batt_no_current_step_table[i+1]>current_vol))break;		
	    }
		bat_capacity = batt_disp_table_no_current[i];
	}else{
	    for(i=0; i<55; i++){		
		    if((batt_step_table[i]<=current_vol)&&(batt_step_table[i+1]>current_vol))break;		
	    }
		bat_capacity = batt_disp_table[i];
	}
#endif
	bat_health = POWER_SUPPLY_HEALTH_GOOD;
       //printk("yyz__________________bat_voltage=%d,g_adcbat=%d,bat_capacity=%d\n",bat_voltage,g_adcbat,bat_capacity);
	return 1;
nobattery:
	if(/* (!get_msc_connect_flag()) ||*/ get_ac_charge_status() )	/*the battery charge*/
		bat_status =POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status =POWER_SUPPLY_STATUS_NOT_CHARGING ;	/*no charge*/
	bat_health = POWER_SUPPLY_HEALTH_GOOD;
	return 0;

}

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	//DBG("--------%s-->%s-->property_psp%d\n",__FILE__,__FUNCTION__,psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* get power supply */
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* Todo return battery level */	
		val->intval = bat_capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val ->intval = bat_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = BATT_VOLTAGE_MAX;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = BATT_VOLTAGE_MIN;
		break;
	default:		
		return -EINVAL;
	}
	
	return 0;
}


 static enum hrtimer_restart rk2918_battery_dostimer(struct hrtimer *handle)
{

	struct rk2918_battery_data *data = container_of(handle, struct rk2918_battery_data, timer);
	int old_bat_status = bat_status;
	int old_bat_capacity = bat_capacity;
	unsigned long flags;
	#if BAT_DBG
	char bat_info[500];
	#endif
	spin_lock_irqsave(&data->lock, flags);
	rockchip_get_battery_status();		/*have battery*/
	full_time_cnt ++;
	if(full_time_cnt >= 2 * PER_MINUTE) 
	{
		full_flag = 0;
		full_time_cnt = 0;
	}
	/*if have usb supply power*/		
	if((bat_present == BATT_PRESENT_TRUE)&&(old_bat_status != bat_status))
	{
		
		charger_change_cnt = 80;
		printk("charge status changed::charger_change_cnt %d old_bat_status %d bat_status %d\n",
			charger_change_cnt,old_bat_capacity,bat_capacity);
		if(bat_capacity-old_bat_capacity<15)
		bat_capacity = old_bat_capacity;
		DBG("\n----usbchange----->%s:  old_bat_status==%i  ->bat_status==%i\n",data->battery.name,old_bat_status,bat_status);
		DBG("---->battery adcbat = %d\n",g_adcbat);
		DBG("---->battery present = %d\n",bat_present);
		DBG("---->battery status  = %d\n",bat_status);
		DBG("---->pb0 status = %d pb1 status = %d  usbchargestatus== %d\n");
	//	GPIOGetPinLevel(GPIOPortB_Pin0),GPIOGetPinLevel(GPIOPortB_Pin1),!get_msc_connect_flag());
		DBG("---->battery current voltage = %d\n",bat_voltage);
		DBG("---->battery capacity = %d\n",bat_capacity);
		power_supply_changed(&data->battery);
		goto next;
	}

	
	if(charger_change_cnt) { //避免拔插适配器电量显示浮动大
		//printk("charge status changed2::charger_change_cnt %d old_bat_status %d bat_status %d\n",
		//charger_change_cnt,old_bat_capacity,bat_capacity);
		charger_change_cnt--;
		bat_capacity = old_bat_capacity;
		goto update;
	}
	
	//避免任何情况下很大的跳动，比如拨到HOLD键时电池采样电压会高0.2v，这会造成很大的浮动
	if(((old_bat_capacity-bat_capacity)>10) || ((bat_capacity-old_bat_capacity)>10))
	{
		adj_cnt++;
		if(adj_cnt< 80)
			bat_capacity = old_bat_capacity;
		else{
			adj_cnt = 0;
			old_bat_capacity = bat_capacity;
		}
		goto update;	
	}
		
	/*fine set battery capacity*/
	if((get_ac_charge_status())&&(bat_capacity < old_bat_capacity)){	

		if((old_bat_capacity-bat_capacity)<10){
		    bat_capacity = old_bat_capacity;
		    bat_vol_up_cnt = 0;
		}else{
		    bat_vol_up_cnt++;
			if(bat_vol_up_cnt > 80 /*20*/)
			    bat_vol_up_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
	    }
	}		
	if((!get_ac_charge_status())&&(bat_capacity > old_bat_capacity)){		
	if((bat_capacity-old_bat_capacity)<10){
		bat_capacity = old_bat_capacity;
		bat_vol_cnt = 0;
	}else{
		bat_vol_cnt++;
		if(bat_vol_cnt > 80)
		    bat_vol_cnt = 0;
		else	
		    bat_capacity = old_bat_capacity;
	}	
	}
update:	
	sample_times ++;						/*count times (report batter status)*/
	//printk("yyz___________sample_times=%d\n",sample_times);
	if((bat_present == BATT_PRESENT_TRUE)&&(sample_times > SEC_NUM * PER_SEC_NUM) )
	{
		sample_times = 0;
		#if BAT_DBG
		ktime_now = ktime_get();
		DBG("\ntime at %Lu nanosec(interal: %Lu)\n" ,ktime_to_ns( ktime_now),ktime_to_ns( ktime_sub( ktime_now,ktime_pre )) );		
		ktime_pre = ktime_now;
		#endif
		DBG("****>battery adcbat = %d \n",g_adcbat);
		DBG("---->battery present = %d\n",bat_present);
		DBG("---->battery status  = %d\n",bat_status);
		DBG("---->pb0 status = %d pb1 status = %d  usbchargestatus== %d\n");
		//GPIOGetPinLevel(GPIOPortB_Pin0),GPIOGetPinLevel(GPIOPortB_Pin1),!get_msc_connect_flag());
		DBG("---->battery current voltage = %d\n",bat_voltage);
		DBG("---->battery capacity = %d\n",bat_capacity);
		//printk("---->battery capacity = %d\n",bat_capacity);
		power_supply_changed(&data->battery);
	}
next:
	spin_unlock_irqrestore(&data->lock, flags);
	handle->_expires= ktime_add(handle->_expires, ktime_set(0,TS_POLL_DELAY));
	return HRTIMER_RESTART;

 }

static irqreturn_t battery_low_isr(int irq, void *dev_id)
{
      printk("yyz____low power wakeup");
      return IRQ_HANDLED;
}

 
static int rk2918_battery_probe(struct platform_device *pdev)
{
	int  rc,i;
	int ret;
	struct adc_client *client;
	struct rk2918_battery_data  *data;
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;
	
	printk("rk2918_battery_probe\n");

       if (pdata && pdata->io_init) {
		ret = pdata->io_init();
		if (ret) {
                 pdata->io_deinit();
		}	
		printk("rk2918_battery_probe  error\n");
              return 0;
	}

	//dc det
	ret = gpio_request(pdata->dc_det_pin, NULL);
	if (ret) {
		printk("failed to request dc_det gpio\n");
		goto err_free_gpio_detpin;
	}
	
	gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
	ret = gpio_direction_input(pdata->dc_det_pin);
	if (ret) {
		printk("failed to set gpio dc_det input\n");
		goto err_free_gpio_detpin;
	}
    
    //charge_ok
	ret = gpio_request(pdata->charge_ok_pin, NULL);
	if (ret) {
		printk("failed to request charge_ok gpio\n");
		goto err_free_gpio_chargepin;
	}
	
	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
	ret = gpio_direction_input(pdata->charge_ok_pin);
	if (ret) {
		printk("failed to set gpio charge_ok input\n");
		goto err_free_gpio_chargepin;
	}
	
	ret = gpio_request(pdata->batt_low_pin, NULL);
	if (ret) {
		printk("failed to request batt_low_pin gpio\n");
		goto err_free_gpio_battlowpin;
	}
	
	gpio_pull_updown(pdata->batt_low_pin, GPIOPullUp);//important
	ret = gpio_direction_input(pdata->batt_low_pin);
	if (ret) {
		printk("failed to set gpio batt_low_pin input\n");
		goto err_free_gpio_battlowpin;
	}

	ret = request_irq(gpio_to_irq(pdata->batt_low_pin), battery_low_isr,
					    IRQF_TRIGGER_FALLING,
					     "battery_low",
					      pdata);
        if (ret) {
             gpio_free(pdata->batt_low_pin);
             goto err_free_gpio_battlowpin;
        }
	
	device_init_wakeup(&pdev->dev, 1);//enable low pin to wake up the machine
	
	/* init power supplier framework */
	data=kzalloc(sizeof(*data), GFP_KERNEL);
	if(!data){
            printk("battery struct  malloc  failed\n");
	     goto err_malloc_failed;
	}
	spin_lock_init(&data->lock);
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = rk2918_battery_dostimer;
	data ->battery = rockchip_power_supplies[0];
	//data ->usb = rockchip_power_supplies[1];
	data ->ac  = rockchip_power_supplies[1];
	data->dc_det_pin=pdata->dc_det_pin;
	data->dc_det_active_level=pdata->dc_det_level;
	data->charge_ok_pin=pdata->charge_ok_pin;
	data->charge_ok_active_level=pdata->charge_ok_level;
	
	//DBG("test %s-->%s-->%s\n",data->battery.name,data->usb.name,data->ac.name);

	client = adc_register(0, rk2918_battery_callback, NULL);
       if(!client){
	   printk("adc register failed");
	   goto err_malloc_failed;
       }

       data->client=client;
	data->adc_val=800;
	g_adcbat=data->adc_val;
       gbatterydata=data;
	platform_set_drvdata(pdev, data);
	
	rc = power_supply_register(&pdev->dev, &data ->battery);
	if (rc)
	{
		printk(KERN_ERR "Failed to register battery power supply (%d)\n", rc);
		goto err_battery_fail;
	}

	rc = power_supply_register(&pdev->dev, &data ->ac);
	if (rc)
	{
		printk(KERN_ERR "Failed to register ac power supply (%d)\n", rc);
		goto err_ac_fail;
	}
	DBG("--------cur time:0x%Lx\n",__FILE__,__FUNCTION__,ktime_get() );

   	bat_vol_no_power_cnt = 81;
	g_adcbat =  gbatterydata->adc_val;
	
	/*get originally battery stauts*/
	for(i=0;i<AD_SAMPLE_TIMES;i++)
	{
		rockchip_get_battery_status();	
		mdelay(100);
	}
	/*low battery low need power down*/
	DBG("---->battery adcbat = %d \n",g_adcbat);
	DBG("---->battery present = %d\n",bat_present);
	DBG("---->battery status  = %d\n",bat_status);
	DBG("---->battery current voltage = %d\n",bat_voltage);
	DBG("---->battery capacity = %d\n",bat_capacity);

	#if 1
	if((bat_capacity == 0)){        //低电不开机
		 rk29_pm_power_off();
		 return 0;
	}
	#endif
	hrtimer_start(&data->timer,ktime_set(10,TS_POLL_DELAY),HRTIMER_MODE_REL);
	return 0;
err_battery_fail:
	power_supply_unregister(&data->battery);
err_ac_fail:
	power_supply_unregister(&data->ac);
err_malloc_failed:
err_free_gpio_battlowpin:
	gpio_free(pdata->batt_low_pin);
 err_free_gpio_chargepin:
 	gpio_free(pdata->charge_ok_pin);
 err_free_gpio_detpin:
 	gpio_free(pdata->dc_det_pin);
	return rc;
		
}

static int rk2918_battery_remove(struct platform_device *pdev)
{
	struct rk2918_battery_data *data = platform_get_drvdata(pdev);
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;

	device_init_wakeup(&pdev->dev, 0);
	power_supply_unregister(&data->battery);
       //power_supply_unregister(&data->usb);
	power_supply_unregister(&data->ac);
	gpio_free(pdata->charge_ok_pin);
 	gpio_free(pdata->dc_det_pin);
	gpio_free(pdata->batt_low_pin);
	kfree(data);
	gbatterydata = NULL;
	return 0;
}


#ifdef CONFIG_PM
static int battery_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;

	if (device_may_wakeup(&pdev->dev)) {		
               int irq = gpio_to_irq(pdata->batt_low_pin);
               enable_irq_wake(irq);		
	}
	
	return 0;
}

static int battery_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;

	if (device_may_wakeup(&pdev->dev)) {
             int irq = gpio_to_irq(pdata->batt_low_pin);
             disable_irq_wake(irq);		
	}
	
	return 0;
}

static const struct dev_pm_ops battery_pm_ops = {
	.suspend	= battery_suspend,
	.resume	= battery_resume,
};
#endif


static struct platform_driver rockchip_battery_driver = {
	.probe	= rk2918_battery_probe,
	.remove	= rk2918_battery_remove,
	.driver	= {
		.name	= "rk2918-battery",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
             .pm= &battery_pm_ops,
#endif
	},
};

static int __init rk2918_battery_init(void)
{
	printk("%s::========================================\n",__func__);
	return platform_driver_register(&rockchip_battery_driver);
}


static int __exit rk2918_battery_exit(void)
{
	printk("%s::========================================\n",__func__);
       return platform_driver_register(&rockchip_battery_driver);
}

late_initcall(rk2918_battery_init);
module_exit(rk2918_battery_exit);
MODULE_DESCRIPTION("Rockchip Battery Driver");
MODULE_LICENSE("GPL");
