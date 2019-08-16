/* drivers/input/touchscreen/gslX68X.h
 *
 * 2010 - 2013 SLIEAD Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the SLIEAD's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
 #include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include "gsl_ts_driver.h"
//#include "gsl_ts_8074.h"
//#include "gsl_ts_i2c.h"
//#include "gsl_ts_test.h"
//#include "gsl_ts_common.h"
#define GSL_REPORT_POINT_SLOT
#define IRQ_WITH_WORKQUEUE
#ifdef CONFIG_GET_HARDWARE_INFO
    #include <asm/hardware_info.h>
#endif

#ifdef GSL_REPORT_POINT_SLOT
    #include <linux/input/mt.h>
#endif
#define GSL_ALG_ID
/* Timer Function */
#ifdef GSL_TIMER
#define GSL_TIMER_CHECK_CIRCLE 200
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static char int_1st[4] ={0x0};
static char int_2nd[4]={0x0};
#endif

/* Gesture Resume */
#ifdef GSL_GESTURE
typedef enum{
    GE_DISABLE = 0,
    GE_ENABLE = 1,
    GE_WAKEUP = 2,
    GE_NOWORK =3,
}GE_T;
static GE_T gsl_gesture_status = GE_DISABLE;
static volatile unsigned int gsl_gesture_flag = 1;
static char gsl_gesture_c = 0;
#endif
 struct gsl_ts_data *gsl_data = NULL;
/* Process for Android Debug Bridge */
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
//static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif
#define GSL_I2C_RETRY 3

/*define global variable*/
//u8 gsl_cfg_index 0;
struct gsl_ts_data *private_ts = NULL;
#define GSL_TEST_TP
#ifdef GSL_TEST_TP
extern void gsl_write_test_config(unsigned int cmd,int value);
extern unsigned int gsl_read_test_config(unsigned int cmd);
extern int gsl_obtain_array_data_ogv(short *ogv,int i_max,int j_max);
extern int gsl_obtain_array_data_dac(unsigned int *dac,int i_max,int j_max);
extern int gsl_tp_module_test(void);
#define GSL_PARENT_PROC_NAME "android_touch"
#define GSL_OPENHSORT_PROC_NAME "self_test"
#define GSL_RAWDATA_PROC_NAME "ctp_rawdata"
#endif

/*
int gsl_config_data_id_DJ[] =
{
    #include "gsl_ts_config_data.h"
};

struct fw_data GSLX68X_FW_DJ[] = {
    #include "gsl_ts_fw.h"
};
*/
/*
struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};
*/


#ifdef GSL_HAVE_TOUCH_KEY
struct key_data gsl_key_data[GSL_KEY_NUM] = {
    {KEY_MENU,50,100,806,846},
    {KEY_HOMEPAGE,200,250,806,846},
    {KEY_BACK,370,420,806,846},
};
#endif

#ifdef CONFIG_GET_HARDWARE_INFO
static int gsl_register_hardware_info(struct gsl_ts_data *ts_data);
char gsl_ctp_hardware_info[MAX_HARDWARE_INFO_LEN] = {0};
#endif
static void gsl_sw_init(struct i2c_client *client);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void gsl_early_suspend(struct early_suspend *handler);
static void gsl_early_resume(struct early_suspend *handler);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf,
         __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:900:40:40"
     ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:900:40:40"
     ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:900:40:40"
     "\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.GSL_TP",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void gsl_ts_virtual_keys_init(void)
{
    int ret =0;
    struct kobject *properties_kobj;

    printk("%s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
            &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif


int gsl_ts_read(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
    int err = 0;
    u8 temp = reg;

    mutex_lock(&gsl_data->gsl_i2c_lock);

    if(temp < 0x80) {
        temp = (temp + 8) & 0x5c;
        i2c_master_send(client, &temp, 1);  //write page addr that want to read
        err = i2c_master_recv(client, &buf[0], 4);//recv data form chip

        temp = reg;
        i2c_master_send(client, &temp, 1);
        err = i2c_master_recv(client, &buf[0], 4);
    }

    i2c_master_send(client,&reg,1);
    err = i2c_master_recv(client,&buf[0],num);

    mutex_unlock(&gsl_data->gsl_i2c_lock);

    return (err == num)?1:-1;
}

int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
  /*  int err = 0;
    u8 temp = reg;

    mutex_lock(&ts_data->gsl_i2c_lock);
    if(temp < 0x80)
    {
        temp = (temp+8)&0x5c;
        err = i2c_master_send(client,&temp,1);
        if(err < 0) {
            goto err_i2c_transfer;
        }
        err = i2c_master_recv(client,&buf[0],4);
        if(err < 0) {
            goto err_i2c_transfer;
        }
        temp = reg;
        err = i2c_master_send(client,&temp,1);
        if(err < 0) {
            goto err_i2c_transfer;
        }
        err = i2c_master_recv(client,&buf[0],4);
        if(err < 0) {
            goto err_i2c_transfer;
        }
    }
    err = i2c_master_send(client,&reg,1);
    if(err < 0) {
        goto err_i2c_transfer;
    }
    err = i2c_master_recv(client,&buf[0],num);
    if(err != num) {
        err = -1;
        goto err_i2c_transfer;
    }
    mutex_unlock(&ts_data->gsl_i2c_lock);
    return 1;
err_i2c_transfer:
    mutex_unlock(&ts_data->gsl_i2c_lock);
    return err;*/
	 int err = 0;
    u8 temp = reg;

    mutex_lock(&gsl_data->gsl_i2c_lock);

    if(temp < 0x80) {
        temp = (temp + 8) & 0x5c;
        i2c_master_send(client, &temp, 1);  //write page addr that want to read
        err = i2c_master_recv(client, &buf[0], 4);//recv data form chip

        temp = reg;
        i2c_master_send(client, &temp, 1);
        err = i2c_master_recv(client, &buf[0], 4);
    }

    i2c_master_send(client,&reg,1);
    err = i2c_master_recv(client,&buf[0],num);

    mutex_unlock(&gsl_data->gsl_i2c_lock);

    return (err == num)?1:-1;
}

int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
    struct i2c_msg xfer_msg[1];
    int err = 0;
    u8 tmp_buf[num+1];
    tmp_buf[0] = reg;
    memcpy(tmp_buf + 1, buf, num);
    xfer_msg[0].addr = client->addr;
    xfer_msg[0].len = num + 1;
    xfer_msg[0].flags = client->flags & I2C_M_TEN;
    xfer_msg[0].buf = tmp_buf;
    //xfer_msg[0].timing = 400;

    mutex_lock(&gsl_data->gsl_i2c_lock);
    err= i2c_transfer(client->adapter, xfer_msg, 1);
    mutex_unlock(&gsl_data->gsl_i2c_lock);

    return err;
}

#ifdef GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,
                unsigned int addr,unsigned int len)
{
    u8 buf[4] ={0x0};
    int i =0;
printk("tp-gsl-gesture ");
   printk("gsl_read_oneframe_data:::addr=%x,len=%x", addr, len);
    for (i = 0; i < len/2; i++){
        buf[0] = ((addr + i * 8) / 0x80) & 0xff;
        buf[1] = (((addr + i * 8) / 0x80) >>8) & 0xff;
        buf[2] = (((addr + i * 8) / 0x80) >>16) & 0xff;
        buf[3] = (((addr + i * 8) / 0x80) >>24) & 0xff;
        gsl_write_interface(gsl_data->client,0xf0,buf,4);
        gsl_read_interface(gsl_data->client,(addr+i*8)%0x80,(char *)&data[i*2],8);
    }
    if(len%2){
        buf[0] = (( addr + len * 4 - 4) / 0x80) & 0xff;
        buf[1] = (((addr + len * 4 - 4) / 0x80) >> 8) & 0xff;
        buf[2] = (((addr + len * 4 - 4) / 0x80) >> 16) & 0xff;
        buf[3] = (((addr + len * 4 - 4) / 0x80) >> 24) & 0xff;
        gsl_write_interface(gsl_data->client, 0xf0, buf,4);
        gsl_read_interface(gsl_data->client, (addr + len * 4 - 4) % 0x80,(char *)&data[len-1],4);
    }
    #ifdef GSL_DEBUG
    //for(i = 0; i < len; i++) {
       printk("gsl_read_oneframe_data =%x", data[i]);
    //}
    #endif

    return len;
}
#endif

static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
    u8 buf[4] = {0};
    u8 addr = 0;
    u32 source_line = 0;
    u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

   printk("=============gsl_load_fw start==============\n");

    for (source_line = 0; source_line < source_len; source_line++) {
        /* init page trans, set the page val */
        addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
        memcpy(buf, &GSL_DOWNLOAD_DATA[source_line].val, 4);
        gsl_write_interface(client, addr, buf, 4);
    }
   printk("=============gsl_load_fw end  ==============\n");
}

static void gsl_io_control(struct i2c_client *client)
{
#if GSL9XX_VDDIO_1800
    u8 buf[4] = {0};
    int i = 0;
    for(i = 0; i < 5; i++){
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0xfe;
        buf[3] = 0x1;
        gsl_write_interface(client, 0xf0, buf, 4);
        buf[0] = 0x5;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0x80;
        gsl_write_interface(client, 0x78, buf, 4);
        msleep(5);
    }
    msleep(50);
#endif
}

static void gsl_start_core(struct i2c_client *client)
{
    u8 buf[4] = {0};
    buf[0] = 0;
    gsl_write_interface(client, 0xe0, buf, 4);
#ifdef GSL_ALG_ID
    {
        gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
    }
#endif
}

static void gsl_reset_core(struct i2c_client *client)
{
    u8 buf[4] = {0x00};

#ifdef HW_RST_CHIP
    gpio_set_value(gsl_data->pdata->reset_gpio, 0);
    msleep(20);
    gpio_set_value(gsl_data->pdata->reset_gpio, 1);
#else
    buf[0] = 0x88;
    gsl_write_interface(client,0xe0,buf,4);
    msleep(5);
#endif

    buf[0] = 0x04;
    gsl_write_interface(client,0xe4,buf,4);
    msleep(5);

    buf[0] = 0;
    gsl_write_interface(client,0xbc,buf,4);
    msleep(5);

    gsl_io_control(client);
}

static void gsl_clear_reg(struct i2c_client *client)
{
    u8 buf[4] = {0};
    //clear reg
    buf[0] = 0x88;
    gsl_write_interface(client, 0xe0, buf, 4);
    msleep(20);
    buf[0] = 0x3;
    gsl_write_interface(client, 0x80, buf, 4);
    msleep(5);
    buf[0] = 0x4;
    gsl_write_interface(client, 0xe4, buf, 4);
    msleep(5);
    buf[0] = 0x0;
    gsl_write_interface(client, 0xe0, buf, 4);
    msleep(20);
    //clear reg

}
#ifdef GSL_TEST_TP
UINT ReadReg(UINT addr)
{
    u8 tmp_buf[4] = {0};
    u32 data =0;
    gsl_read_interface(gsl_data->client, addr, tmp_buf, 4);
    //gsl_ts_read(gsl_data->client, 0, buf, 128);
    data = (u32)tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
    return data;
}
EXPORT_SYMBOL(ReadReg);
void ReadPage(unsigned int addr, unsigned int *buf)
{
	u8 tmp_buf[4]={0};
	tmp_buf[3]=(u8)(addr>>24);
	tmp_buf[2]=(u8)(addr>>16);
	tmp_buf[1]=(u8)(addr>>8);
	tmp_buf[0]=(u8)(addr);
	gsl_write_interface(gsl_data->client,0xf0,tmp_buf,4);
	gsl_read_interface(gsl_data->client,0,(u8 *)buf,128);
}
EXPORT_SYMBOL(ReadPage);
UINT ReadMem(unsigned int addr)
{
	u8 tmp_buf[4]={0};	
    u32 data =0;
	tmp_buf[3]=(u8)((addr/0x80)>>24);
	tmp_buf[2]=(u8)((addr/0x80)>>16);
	tmp_buf[1]=(u8)((addr/0x80)>>8);
	tmp_buf[0]=(u8)((addr/0x80));
	gsl_write_interface(gsl_data->client,0xf0,tmp_buf,4);
	gsl_read_interface(gsl_data->client,addr%0x80,tmp_buf,4);
	data = tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
    return data;
}
EXPORT_SYMBOL(ReadMem);
#endif

#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
    if(ch >= '0' && ch <= '9')
        return (ch -'0');
    else
        return (ch-'a'+ 10);
}

static int gsl_config_read_proc(struct seq_file *m,void *v)
{
    char temp_data[5] = {0};
    unsigned int tmp = 0;

    if('v'== gsl_read[0] && 's'== gsl_read[1]){
#ifdef GSL_ALG_ID
        tmp=gsl_version_id();
#else
        tmp=0x20121215;
#endif
        seq_printf(m,"version:%x\n",tmp);
    }
    else if('r' == gsl_read[0]&&'e'== gsl_read[1]){
        if('i' == gsl_read[3]){
#ifdef GSL_ALG_ID
            tmp=(gsl_data_proc[5] << 8) | gsl_data_proc[4];
            seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
            if((tmp >= 0) && (tmp < gsl_cfg_table[gsl_cfg_index].data_size))
                seq_printf(m,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]);
#endif
        }else {
           printk("data proc:1 = %02x, 2 = %02x, 3= %02x , 4 = %02x,***",
               gsl_data_proc[0], gsl_data_proc[1], gsl_data_proc[2], gsl_data_proc[3]);
            /*
            tmp = (gsl_data_proc[7] << 24) + (gsl_data_proc[6] << 16) + (gsl_data_proc[5] << 8) + gsl_data_proc[4];
            if(tmp >= 1 && gsl_data_proc[0] == 0x7c){
                tmp -= 1;
                gsl_data_proc[7] = (tmp>>24) & 0xff;
                gsl_data_proc[6] = (tmp>>16) & 0xff;
                gsl_data_proc[5] = (tmp>> 8) & 0xff;
                gsl_data_proc[4] = (tmp>> 0) & 0xff;
            }
            */
            gsl_write_interface(gsl_data->client, 0Xf0, &gsl_data_proc[4], 4);

            if(gsl_data_proc[0] < 0x80)
                gsl_read_interface(gsl_data->client, gsl_data_proc[0], temp_data,4);

            gsl_read_interface(gsl_data->client, gsl_data_proc[0], temp_data, 4);

            printk("proc tmp data:1 = %02x, 2 = %02x, 3= %02x , 4 = %02x,***",temp_data[0],temp_data[1],temp_data[2],temp_data[3]);
            //seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
            //seq_printf(m,"%02x",temp_data[3]);
            //seq_printf(m,"%02x",temp_data[2]);
            //seq_printf(m,"%02x",temp_data[1]);
            //seq_printf(m,"%02x};\n",temp_data[0]);
        }
    }
    return (0);
}
static int gsl_config_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
    u8 buf[8] = {0};
    char temp_buf[CONFIG_LEN] ={0x0};
    char *path_buf;
    int tmp = 0;
    int tmp1 = 0;
   printk("[tp-gsl][%s] \n",__func__);


    if(count > 512){
     printk("size not match [%d:%d]\n", CONFIG_LEN, count);
        return -EFAULT;
    }
    path_buf=kzalloc(count,GFP_KERNEL);
    if(!path_buf){
        printk("alloc path_buf memory error \n");
        return -1;
    }
    if(copy_from_user(path_buf, buffer, count)){
      printk("copy from user fail\n");
        goto exit_write_proc_out;
    }
    memcpy(temp_buf, path_buf, (count < CONFIG_LEN ? count : CONFIG_LEN));
  printk("[%s]", temp_buf);


    buf[3] = char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);
    buf[2] = char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
    buf[1] = char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
    buf[0] = char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);

   printk("buf[0] = %d, buf[1] = %d ,buf[2] = %d, buf[3] = %d,buf[4] = %d, buf[5] = %d ,buf[6] = %d, buf[7] = %d",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);

    buf[7] = char_to_int(temp_buf[5]) << 4 | char_to_int(temp_buf[6]);
    buf[6] = char_to_int(temp_buf[7]) << 4 | char_to_int(temp_buf[8]);
    buf[5] = char_to_int(temp_buf[9]) << 4 | char_to_int(temp_buf[10]);
    buf[4] = char_to_int(temp_buf[11]) << 4 | char_to_int(temp_buf[12]);
    if('v'== temp_buf[0]&& 's'==temp_buf[1]){
        memcpy(gsl_read,temp_buf,4);
       printk("gsl version");
    }
    else if(('s'== temp_buf[0] )&& ('t'== temp_buf[1])){//start //st
    #ifdef GSL_TIMER
        cancel_delayed_work_sync(&gsl_timer_check_work);
    #endif
        gsl_proc_flag = 1;
        gsl_reset_core(gsl_data->client);
    }
    else if('e'==temp_buf[0]&&'n'==temp_buf[1]){//end //en
        msleep(20);
        gsl_reset_core(gsl_data->client);
        gsl_start_core(gsl_data->client);
        gsl_proc_flag = 0;
    }
    else if('r'==temp_buf[0]&&'e'==temp_buf[1]){//read buf //
        memcpy(gsl_read,temp_buf,4);
        memcpy(gsl_data_proc,buf,8);
    }
    else if('w'==temp_buf[0]&&'r'==temp_buf[1]){//write buf
        gsl_write_interface(gsl_data->client,buf[4],buf,4);
    }
#ifdef GSL_ALG_ID
    else if('i'==temp_buf[0]&&'d'==temp_buf[1]){//write id config //
        tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
        tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
        if(tmp1>=0 && tmp1<gsl_cfg_table[gsl_cfg_index].data_size){
            gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
        }
    }
#endif
exit_write_proc_out:
    kfree(path_buf);
    return count;
}
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
    return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
    .open = gsl_server_list_open,
    .read = seq_read,
    .release = single_release,
    .write = gsl_config_write_proc,
    .owner = THIS_MODULE,
};
#endif

/* Modified by lihuiqin to delete the invalid logs ZQ1989-155 begin */
#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{
    struct gsl_ts_data *ts = gsl_data;
    struct i2c_client *gsl_client = ts->client;
    static int i2c_lock_flag = 0;
    char read_buf[4]  = {0};
    char init_chip_flag = 0;
    int i=0;
    int flag =0;

    printk("----------------gsl_monitor_worker-----------------");
    if(i2c_lock_flag != 0)
        return;
    else
        i2c_lock_flag = 1;

    /*check 0xb4 register,check interrupt if ok*/
    gsl_read_interface(gsl_client, 0xb4, read_buf, 4);
    memcpy(int_2nd, int_1st,4);
    memcpy(int_1st, read_buf,4);

    if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&
        int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0]){
        printk("int 1st: %x %x %x %x === int_2nd: %xs %x %x %x ======",
            //int_1st[3], int_1st[2], int_1st[1], int_1st[0],
            //int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
        init_chip_flag = 1;
        goto queue_monitor_work;
    }

    /*check 0xb0 register,check firmware if ok*/
    for(i = 0; i < 5; i++){
        gsl_read_interface(gsl_client, 0xb0, read_buf, 4);

        printk("0xb0 before judgment = {0x%02x%02x%02x%02x}",
                        //read_buf[3],read_buf[2],read_buf[1],read_buf[0]);

        if(read_buf[3] != 0x5a || read_buf[2] != 0x5a ||
            read_buf[1] != 0x5a || read_buf[0] != 0x5a){

            printk("0xb0 after judgment = {0x%02x%02x%02x%02x}",
                //read_buf[3],read_buf[2],read_buf[1],read_buf[0]);

            flag = 1;
        }else{
            flag = 0;
            break;
        }

    }
    if(flag == 1){
        init_chip_flag = 1;
        goto queue_monitor_work;
    }

    /*check 0xbc register,check dac if normal*/

    for (i = 0; i < 5; i++) {
        gsl_read_interface(gsl_client, 0xbc, read_buf, 4);

        printk("0xbc before judgment = {0x%02x%02x%02x%02x}",
            //read_buf[3],read_buf[2],read_buf[1],read_buf[0]);

        if(read_buf[3] != 0 || read_buf[2] != 0 ||
            read_buf[1] != 0 || read_buf[0] != 0){

            printk("0xbc after judgment = {0x%02x%02x%02x%02x}",
                //read_buf[3],read_buf[2],read_buf[1],read_buf[0]);

            flag = 1;
        } else {
            flag = 0;
            break;
        }
    }
    if(flag == 1){
        gsl_reset_core(gsl_client);
        gsl_start_core(gsl_client);
        init_chip_flag = 0;
    }
queue_monitor_work:
    if(init_chip_flag){
        gsl_sw_init(gsl_client);
        memset(int_1st,0xff,sizeof(int_1st));
    }

    if(!gsl_data->suspended){
        queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 200);
    }
    i2c_lock_flag = 0;

}
#endif
/* Modified by lihuiqin to delete the invalid logs ZQ1989-155 end */

static int gsl_compatible_id(struct i2c_client *client)
{
    u8 buf[4] ={0x0};
    int i=0;
    int err =0;

    for(i = 0; i < 5; i++) {
        err = gsl_read_interface(client, 0xfc, buf, 4);
      printk("reg:0xfc = {0x%02x%02x%02x%02x}", buf[3], buf[2],
            buf[1],buf[0]);
        if(!(err < 0)) {
            err = 1;
            break;
        }
    }
    return err;
}

#ifdef GSL_COMPATIBLE_GPIO
static int gsl_read_TotalAdr(struct i2c_client *client,u32 addr,u32 *data)
{
    u8 buf[4] ={0x0};
    int err =0;

    buf[3]=(u8)((addr/0x80)>>24);
    buf[2]=(u8)((addr/0x80)>>16);
    buf[1]=(u8)((addr/0x80)>>8);
    buf[0]=(u8)((addr/0x80));
    gsl_write_interface(client,0xf0,buf,4);
    err = gsl_read_interface(client,addr%0x80,buf,4);
    if(err > 0){
        *data = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
    }
    return err;
}
static int gsl_write_TotalAdr(struct i2c_client *client,u32 addr,u32 *data)
{
    int err =0;
    u8 buf[4]={0x0};
    u32 value = *data;

    buf[3] = (u8)((addr/0x80)>>24);
    buf[2] = (u8)((addr/0x80)>>16);
    buf[1] = (u8)((addr/0x80)>>8);
    buf[0] = (u8)((addr/0x80));
    gsl_write_interface(client,0xf0,buf,4);

    buf[3] = (u8)((value)>>24);
    buf[2] = (u8)((value)>>16);
    buf[1] = (u8)((value)>>8);
    buf[0] = (u8)((value));
    err = gsl_write_interface(client,addr%0x80,buf,4);

    return err;
}

static int gsl_gpio_idt_tp(struct i2c_client *client)
{
    int i =0;
    u32 value = 0x1;
    u8 rstate =0;

    gsl_write_TotalAdr(client,0xff000018,&value);
    value = 0x0;
    gsl_write_TotalAdr(client,0xff020000,&value);

    for(i = 0; i < GSL_I2C_RETRY; i++){
        gsl_read_TotalAdr(client,0xff020004,&value);
    }

    rstate = value & 0x1;

    if(rstate == 1){
         gsl_cfg_index  = 0;   //
    } else if (rstate == 0){
        gsl_cfg_index = 1;   //
    }

   printk("[tpd-gsl][%s] [rstate]=[%d]\n",__func__,rstate);
    return 1;
}
#endif
#ifdef GSL_TEST_TP
static ssize_t gsl_test_show(void)
{
	static int gsl_test_flag = 0; 
	char *tmp_buf;
	int err;
	int result = 0;
	printk("[%s]:enter gsl_test_show start::gsl_test_flag  = %d\n",__func__,gsl_test_flag);
	if(gsl_test_flag == 1){
		return 0;	
	}
	gsl_test_flag = 1;
    /*
	tmp_buf = kzalloc(3*1024,GFP_KERNEL);
	if(!tmp_buf){
		printk("[%s]:kzalloc kernel fail\n",__func__);
		return 0;
		}
	*/
	printk("[%s]:tp module test begin\n",__func__);
	
	err = gsl_tp_module_test();

	printk("[%s]:enter gsl_test_show end\n",__func__);
	
	if(err > 0){
		printk("[%s]:tp test pass\n",__func__);
		result = 1;

	}else{
		printk("[%s]:tp test failure\n",__func__);
		result = 0;
	}
	kfree(tmp_buf);
	gsl_test_flag = 0; 
	return result;
}

static s32 gsl_openshort_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}

//static s32 gsl_openshort_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
//struct seq_file *m,void *v
static s32 gsl_openshort_proc_read(struct seq_file *m,void *v)
{
	//char *ptr = buf;
	int test_result  = 0;
    int result =0;
	/*if(*ppos)
	{
		printk("[%s]:tp test again return\n",__func__);
		return 0;
	}
	*ppos += 16;*/
	test_result = gsl_test_show();
	//memset(buf,'\0',16);
	//count = 16;
	if(1 == test_result)
	{
        result =1;
		printk("[%s]:tp test pass\n",__func__);
		 seq_printf(m, "result=%d\n", result);
	}
	else
	{
        result =0;
		printk("[%s]:tp test failure\n",__func__);
		 seq_printf(m, "result=%d\n", result);
	}
	return 0;
}
/*
static ssize_t gsl_rawdata_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}

static ssize_t gsl_rawdata_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
//	int i,number=0;
	int i,ret;
	static short* gsl_ogv;
	ssize_t read_buf_chars = 0; 
	gsl_ogv = kzalloc(26*14*2,GFP_KERNEL);
	if(!gsl_ogv){
		return -1;
	}  

	//printk("[%s]:rawdata proc node read!\n",__func__);

#if 0
	if( number != 0 )
		return -1;
	else
		number++;
#endif	
#if 1
	if(*ppos)
	{
		printk("[%s]:tp test again return\n",__func__);
		return 0;
	}
#endif
	ret=gsl_test_show();
	gsl_obtain_array_data_ogv(gsl_ogv,26,14);

	for(i=0;i<26*14;i++)
	{
		read_buf_chars += sprintf(&(buf[read_buf_chars])," _%u_ ",gsl_ogv[i]);
		if(!((i+1)%10))
		{
			buf[read_buf_chars++] = '\n';
		}

	}

	buf[read_buf_chars-1] = '\n';

	//printk("[%s]:rawdata proc node end!\n",__func__);
	
	*ppos += count;
	//return count;
	*ppos += read_buf_chars; 


	return read_buf_chars; 
}

static const struct file_operations gsl_rawdata_procs_fops =
{
	.write = gsl_rawdata_proc_write,
	.read = gsl_rawdata_proc_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};
*/
/*
static const struct file_operations gsl_openshort_procs_fops =
{
    .write = gsl_openshort_proc_write,
    .read = gsl_openshort_proc_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};*/
static int gsl_list_open(struct inode *inode,struct file *file)
{
    return single_open(file,gsl_openshort_proc_read,NULL);
}
static const struct file_operations gsl_openshort_procs_fops = {
    .open = gsl_list_open,
    .read = seq_read,
    .release = single_release,
    .write = gsl_openshort_proc_write,
    .owner = THIS_MODULE,
};
void create_ctp_proc(void)
{
	struct proc_dir_entry *gsl_device_proc = NULL;
	struct proc_dir_entry *gsl_openshort_proc = NULL;
	//struct proc_dir_entry *gsl_rawdata_proc = NULL;

	gsl_device_proc = proc_mkdir(GSL_PARENT_PROC_NAME, NULL);
		if(gsl_device_proc == NULL)
    	{
        	printk("[%s]: create parent_proc fail\n",__func__);
        	return;
    	}

	gsl_openshort_proc = proc_create(GSL_OPENHSORT_PROC_NAME, 0666, gsl_device_proc, &gsl_openshort_procs_fops);

    	if (gsl_openshort_proc == NULL)
    	{
        	printk("[%s]: create openshort_proc fail\n",__func__);
    	}
	/*	
	gsl_rawdata_proc = proc_create(GSL_RAWDATA_PROC_NAME, 0777, gsl_device_proc, &gsl_rawdata_procs_fops);
    	if (gsl_rawdata_proc == NULL)
    	{
        	printk("[%s]: create ctp_rawdata_proc fail\n",__func__);
    	}*/
}

#endif

static int gsl_ts_power_init(struct gsl_ts_data *ts_data, bool init)
{
    int rc = 0;

    if (init) {
        gsl_data->vdd = regulator_get(&ts_data->client->dev,"vdd");
        if (IS_ERR(ts_data->vdd)) {
            rc = PTR_ERR(ts_data->vdd);
            printk("Regulator get failed vdd rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(ts_data->vdd) > 0) {
            rc = regulator_set_voltage(ts_data->vdd,
                            GSL_VTG_MIN_UV,
                            GSL_VTG_MAX_UV);
            if (rc) {
                printk("Regulator set_vtg failed vdd rc=%d\n", rc);
                goto reg_vdd_put;
            }
        }

        ts_data->vcc_i2c = regulator_get(&ts_data->client->dev, "vcc_i2c");
        if (IS_ERR(ts_data->vcc_i2c)) {
            rc = PTR_ERR(ts_data->vcc_i2c);
            printk("Regulator get failed vcc-i2c rc=%d\n", rc);
            goto reg_vdd_set_vtg;
        }

        if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
            rc = regulator_set_voltage(ts_data->vcc_i2c,
                        GSL_I2C_VTG_MIN_UV,
                        GSL_I2C_VTG_MAX_UV);
            if (rc) {
                printk("Regulator set_vtg failed vcc-i2c rc=%d\n", rc);
                goto reg_vcc_i2c_put;

            }
        }
    } else {
        if (regulator_count_voltages(ts_data->vdd) > 0)
            regulator_set_voltage(ts_data->vdd, 0, GSL_VTG_MAX_UV);

        regulator_put(ts_data->vdd);

        if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
            regulator_set_voltage(ts_data->vcc_i2c, 0, GSL_I2C_VTG_MAX_UV);

        regulator_put(ts_data->vcc_i2c);
    }

    return 0;

reg_vcc_i2c_put:
    regulator_put(ts_data->vcc_i2c);
reg_vdd_set_vtg:
    if (regulator_count_voltages(ts_data->vdd) > 0)
        regulator_set_voltage(ts_data->vdd, 0, GSL_VTG_MAX_UV);
reg_vdd_put:
   regulator_put(ts_data->vdd);
    return rc;
}

static int gsl_ts_power_on(struct gsl_ts_data *ts_data, bool on)
{

    int rc =0;

    if (!on)
        goto power_off;

    rc = regulator_enable(ts_data->vdd);
    if (rc) {
        printk("Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }

    rc = regulator_enable(ts_data->vcc_i2c);
    if (rc) {
        printk("Regulator vcc_i2c enable failed rc=%d\n", rc);
        regulator_disable(ts_data->vdd);
    }

    return rc;

power_off:
    rc = regulator_disable(ts_data->vdd);
    if (rc) {
        printk("Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }

    rc = regulator_disable(ts_data->vcc_i2c);
    if (rc) {
        printk("Regulator vcc_i2c disable failed rc=%d\n", rc);
        rc = regulator_enable(ts_data->vdd);
    }

    return rc;
}

static void gsl_sw_init(struct i2c_client *client)
{
    if(1 == gsl_data->updating_fw)
        return;
    gsl_data->updating_fw = 1;

    gpio_set_value(gsl_data->pdata->reset_gpio, 0);
    msleep(20);
    gpio_set_value(gsl_data->pdata->reset_gpio, 1);
    msleep(20);

    gsl_clear_reg(client);
    gsl_reset_core(client);
    gsl_load_fw(client,gsl_cfg_table[gsl_cfg_index].fw,
        gsl_cfg_table[gsl_cfg_index].fw_size);
    gsl_start_core(client);

    gsl_data->updating_fw = 0;
}

static void check_mem_data(struct i2c_client *client)
{

    u8 read_buf[4]  = {0};
    msleep(30);
    gsl_read_interface(client,0xb0,read_buf,4);
    if (read_buf[3] != 0x5a || read_buf[2] != 0x5a
        || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
    {
       printk("0xb4 ={0x%02x%02x%02x%02x}\n",
           read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
        gsl_sw_init(client);
    }
}

#define GSL_CHIP_NAME   "gslx68x"
static ssize_t gsl_sysfs_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    //ssize_t len=0;
    int count = 0;
    u8 buf_tmp[4];
    u32 tmp =0;
    //char *ptr = buf;
    count += scnprintf(buf,PAGE_SIZE,"sileadinc:");
    count += scnprintf(buf+count,PAGE_SIZE-count,GSL_CHIP_NAME);

#ifdef GSL_TIMER
    count += scnprintf(buf+count,PAGE_SIZE-count,":0001-1:");
#else
    count += scnprintf(buf+count,PAGE_SIZE-count,":0001-0:");
#endif

#ifdef TPD_PROC_DEBUG
    count += scnprintf(buf+count,PAGE_SIZE-count,"0002-1:");
#else
    count += scnprintf(buf+count,PAGE_SIZE-count,"0002-0:");
#endif

    count += scnprintf(buf+count,PAGE_SIZE-count,"0003-0:");

#ifdef GSL_DEBUG
    count += scnprintf(buf+count,PAGE_SIZE-count,"0004-1:");
#else
    count += scnprintf(buf+count,PAGE_SIZE-count,"0004-0:");
#endif

#ifdef GSL_ALG_ID
    tmp = gsl_version_id();
    count += scnprintf(buf+count, PAGE_SIZE - count, "%08x:",tmp);
    count += scnprintf(buf+count, PAGE_SIZE - count, "%08x:",
        gsl_cfg_table[gsl_cfg_index].data_id[0]);
#endif
    buf_tmp[0]=0x3;
    buf_tmp[1]=0;
    buf_tmp[2]=0;
    buf_tmp[3]=0;
    gsl_write_interface(gsl_data->client, 0xf0, buf_tmp, 4);
    gsl_read_interface(gsl_data->client, 0, buf_tmp, 4);
    count += scnprintf(buf+count, PAGE_SIZE - count,"%02x%02x%02x%02x\n",
        buf_tmp[3],buf_tmp[2],buf_tmp[1],buf_tmp[0]);

    return count;
}

static DEVICE_ATTR(version, 0444, gsl_sysfs_version_show, NULL);



#ifdef GSL_GESTURE
static void gsl_enter_doze(struct gsl_ts_data *ts)
{
    u8 buf[4] = {0};
    buf[0] = 0xa;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    gsl_write_interface(ts->client,0xf0,buf,4);
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0x1;
    buf[3] = 0x5a;
    gsl_write_interface(ts->client,0x8,buf,4);
    //gsl_gesture_status = GE_NOWORK;
    msleep(10);
    gsl_gesture_status = GE_ENABLE;
   printk("Enter! gsture status = %d", gsl_gesture_status);

}
static void gsl_quit_doze(struct gsl_ts_data *ts)
{
    u8 buf[4] = {0};
    //u32 tmp;

    gsl_gesture_status = GE_DISABLE;

    gpio_set_value(ts->pdata->reset_gpio,0);
    msleep(20);
    gpio_set_value(ts->pdata->reset_gpio,1);
    msleep(5);

    buf[0] = 0xa;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    gsl_write_interface(ts->client,0xf0,buf,4);
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0x5a;
    gsl_write_interface(ts->client,0x8,buf,4);
    msleep(10);

  printk("Exit! gsture status = %d", gsl_gesture_status);
}

static int gsl_gesture_init(struct gsl_ts_data *gsl_data)
{
   printk("Enter");

    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_POWER);

    /*
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_S);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_C);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_E);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_O);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_W);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_M);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_Z);
    input_set_capability(gsl_data->input_dev, EV_KEY, KEY_V);
    */
    __set_bit(KEY_POWER, gsl_data->input_dev->keybit);

    /*
    __set_bit(KEY_S, input_dev->keybit);
    __set_bit(KEY_C, input_dev->keybit);
    __set_bit(KEY_E, input_dev->keybit);
    __set_bit(KEY_O, input_dev->keybit);
    __set_bit(KEY_W, input_dev->keybit);
    __set_bit(KEY_M, input_dev->keybit);
    __set_bit(KEY_V, input_dev->keybit);
    __set_bit(KEY_Z, input_dev->keybit);
    */

   printk("Exit");

    return 0;
}
static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u32 count = 0;
    count += scnprintf(buf,PAGE_SIZE,"tp gesture is on/off:\n");
    if(gsl_gesture_flag == 1){
        count += scnprintf(buf+count,PAGE_SIZE-count,
                " on \n");
    }else if(gsl_gesture_flag == 0){
        count += scnprintf(buf+count,PAGE_SIZE-count,
                " off \n");
    }
    count += scnprintf(buf+count,PAGE_SIZE-count,"tp gesture:");
    count += scnprintf(buf+count,PAGE_SIZE-count,
            "%c\n",gsl_gesture_c);
        return count;
}
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#if 1
    if(buf[0] == '0'){
        gsl_gesture_flag = 0;
    }else if(buf[0] == '1'){
        gsl_gesture_flag = 1;
    }
#endif
    return count;
}
static DEVICE_ATTR(gesture, 0664, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);
#endif

static struct attribute *gsl_attrs[] = {

    &dev_attr_version.attr,

#ifdef GSL_GESTURE
    &dev_attr_gesture.attr,
#endif

    NULL
};
static const struct attribute_group gsl_attr_group = {
    .attrs = gsl_attrs,
};

#ifdef GSL_HAVE_TOUCH_KEY
static int gsl_report_key(struct input_dev *idev,int x,int y)
{
    int i = 0;
    for(i = 0; i < GSL_KEY_NUM; i++){
        if((x > gsl_key_data[i].x_min) &&
            (x < gsl_key_data[i].x_max) &&
            (y > gsl_key_data[i].y_min) &&
            (y < gsl_key_data[i].y_max))
        {
            gsl_data->gsl_key_state = i+1;
            input_report_key(idev, gsl_key_data[i].key, 1);
            input_sync(idev);
            return 1;
        }
    }
    return 0;
}
#endif
static void gsl_report_point(struct input_dev *idev, struct gsl_touch_info *cinfo)
{
    int i =0;
    u32 gsl_point_state = 0;
    u32 temp=0;
    printk("gsl_report_point enter!\n");
    if(cinfo->finger_num>0 && cinfo->finger_num<6)
    {
        gsl_data->gsl_up_flag = 0;
        gsl_point_state = 0;

        for(i=0;i<cinfo->finger_num;i++){
            gsl_point_state |= (0x1<<cinfo->id[i]);
            printk("id = %d, x = %d, y = %d \n",cinfo->id[i],
                cinfo->x[i],cinfo->y[i]);
    #ifdef GSL_REPORT_POINT_SLOT
            input_mt_slot(idev, cinfo->id[i] - 1);
            input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
            //input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
            input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
            input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);
            input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);

    #else
            input_report_key(idev, BTN_TOUCH, 1);
            input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
            input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
            input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
            input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);
            input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
            input_mt_sync(idev);
    #endif
        }
    }
    else if(cinfo->finger_num == 0){
        gsl_point_state = 0;
    //  ts_data->gsl_point_state = 0;
        if(1 == gsl_data->gsl_up_flag){
            return;
        }
        gsl_data->gsl_up_flag = 1;

    }

    temp = gsl_point_state & gsl_data->gsl_point_state;
    temp = (~temp) & gsl_data->gsl_point_state;

#ifdef GSL_REPORT_POINT_SLOT
    for(i=1;i<6;i++){
        if(temp & (0x1<<i)){
            input_mt_slot(idev, i-1);
            input_report_abs(idev, ABS_MT_TRACKING_ID, -1);
            //input_mt_report_slot_state(idev, MT_TOOL_FINGER, false);
        }
    }
#endif

    gsl_data->gsl_point_state = gsl_point_state;
    input_sync(idev);
}

#ifdef IRQ_WITH_WORKQUEUE
static void gsl_report_work(struct work_struct *work)
{
    int rc = 0;
    int tmp = 0;
    u8 buf[44] = {0};
    struct gsl_touch_info *cinfo = gsl_data->cinfo;
    struct i2c_client *client = gsl_data->client;
    struct input_dev *idev = gsl_data->input_dev;
    int tmp1 = 0;

    /* Added by zhangyijie for add TP gesture func for ZQ1989-93 2018-04-02 begin */
    #ifdef GSL_GESTURE
    int tmp_c;
    u8 key_data = 0;
    #endif
    /* Added by zhangyijie for add TP gesture func for ZQ1989-93 2018-04-02 end */

    if(1 == gsl_data->updating_fw)
        goto end_report_schedule;
    /*
    #ifdef TPD_PROC_DEBUG
        if(gsl_proc_flag == 1){
            return;
        }
    #endif
    */
    /* Gesture Resume */

    /* read data from DATA_REG */
    rc = gsl_read_interface(client, 0x80, buf, 44);
    if (rc < 0) {
        printk("I2C read failed\n");
        goto end_report_schedule;
    }

    if (buf[0] == 0xff) {
        goto end_report_schedule;
    }

    cinfo->finger_num = buf[0];

    for(tmp=0; tmp<(cinfo->finger_num > 10 ? 10:cinfo->finger_num); tmp++){
        cinfo->y[tmp] = (buf[tmp * 4 + 4] | ((buf[tmp * 4 + 5])<<8));
        cinfo->x[tmp] = (buf[tmp * 4 + 6] | ((buf[tmp * 4 + 7] & 0x0f)<<8));
        cinfo->id[tmp] = buf[tmp * 4 + 7] >> 4;
        printk("tp-gsl  x = %d y = %d \n",cinfo->x[tmp],cinfo->y[tmp]);
    }

    printk("111 finger_num= %d\n",cinfo->finger_num);
#ifdef GSL_ALG_ID
    cinfo->finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
    gsl_alg_id_main(cinfo);
    tmp1 = gsl_mask_tiaoping();

    printk("[tp-gsl] tmp1=%x\n",tmp1);
    if ((tmp1 > 0) && (tmp1 < 0xffffffff)) {
        buf[0]=0xa;
        buf[1]=0;
        buf[2]=0;
        buf[3]=0;
        gsl_write_interface(client, 0xf0, buf, 4);
        buf[0]=(u8)(tmp1 & 0xff);
        buf[1]=(u8)((tmp1>>8) & 0xff);
        buf[2]=(u8)((tmp1>>16) & 0xff);
        buf[3]=(u8)((tmp1>>24) & 0xff);
        printk("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n", tmp1,buf[0],buf[1],buf[2],buf[3]);
        gsl_write_interface(client, 0x8, buf, 4);
    }
#endif

/* Added by zhangyijie for add TP gesture func for ZQ1989-93 2018-04-02 begin */
#ifdef GSL_GESTURE
    //printk(KERN_ERR"gsl_gesture_status=%d,gsl_gesture_flag=%d[%d]\n",gsl_gesture_status,gsl_gesture_flag,test_count++);

    if(GE_ENABLE == gsl_gesture_status && gsl_gesture_flag == 1){
        tmp_c = gsl_obtain_gesture();
        //printk("gsl_obtain_gesture():tmp_c=0x%x[%d]\n",tmp_c,test_count++);
     printk("gsl_obtain_gesture():tmp_c=0x%x\n",tmp_c);
        switch(tmp_c){
        case (int)'.':
            key_data = KEY_POWER;
            break;/* click */
        }

        if(key_data != 0){
            gsl_gesture_c = (char)(tmp_c & 0xff);
            //gsl_gesture_status = GE_WAKEUP;
            input_report_key(idev,key_data,1);
            input_sync(idev);
            input_report_key(idev,key_data,0);
            input_sync(idev);
            msleep(400);
        }
        return;
    }
#endif
/* Added by zhangyijie for add TP gesture func for ZQ1989-93 2018-04-02 end */

    gsl_report_point(idev,cinfo);

end_report_schedule:
    enable_irq(client->irq);
}
#endif

static int gsl_request_input_dev(struct gsl_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;
    int err =0;

    /*set input parameter*/
    //input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
#ifdef GSL_REPORT_POINT_SLOT
    //input_mt_init_slots(input_dev, ts_data->pdata->num_max_touches,0);     // in case of "out of memory"
	__set_bit(EV_REP, input_dev->evbit);
	input_mt_init_slots(input_dev,5,0);
#else
    __set_bit(BTN_TOUCH, input_dev->keybit);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0,5,0,0);
#endif
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, GSL_MAX_X, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, GSL_MAX_Y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    input_dev->name = GSL_TS_NAME;      //dev_name(&client->dev)

    /*register input device*/
    err = input_register_device(input_dev);
    if (err) {
        goto err_register_input_device_fail;
    }
    return 0;
err_register_input_device_fail:
    input_free_device(input_dev);
    return err;
}
/* Modified by lihuiqin for fix ADT firmware version read  ZQ1989-155 begin */
static int gsl_read_version(void)
{
    u8 buf[4] = {0};
    int err= 0;
    buf[0] = 0x03;
    err = gsl_write_interface(gsl_data->client, 0xf0, buf, 4);
    if(err < 0) {
        return err;
    }
    gsl_read_interface(gsl_data->client, 0x04, buf, 4);
    if(err < 0) {
        return err;
    }
    gsl_data->fw_version = buf[0];
    printk("fw version = %d\n", gsl_data->fw_version);
    return 0;
}
/* Modified by lihuiqin for fix ADT firmware version read  ZQ1989-155 end */

static irqreturn_t gsl_ts_interrupt(int irq, void *dev_id)
{
    struct i2c_client *client = gsl_data->client;
    disable_irq_nosync(client->irq);

    #ifdef GSL_GESTURE
    if ((gsl_gesture_status == GE_ENABLE) && (gsl_gesture_flag==1)){
        wake_lock_timeout(&gsl_data->gsl_wake_lock, msecs_to_jiffies(2000));
    }
    #endif

    // if (!work_pending(&gsl_data->work)) {
        queue_work(gsl_data->wq, &gsl_data->work);
    // }
    //gsl_report_work();
    return IRQ_HANDLED;
}

#if defined(CONFIG_FB)
static void gsl_ts_suspend(void)
{
    u32 tmp = 0;

   printk("gslX68X_ts_suspend\n");
    /* version information */
    /*dev_info("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);*/

    if(1 == gsl_data->updating_fw)
        return;
    gsl_data->suspended = 1;
#ifdef TPD_PROC_DEBUG
        if(gsl_proc_flag == 1){
            return;
        }
#endif

#ifdef GSL_ALG_ID
    tmp = gsl_version_id();
    printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif

#ifdef GSL_TIMER
    cancel_delayed_work_sync(&gsl_timer_check_work);
#endif

/*Guesture Resume*/
#ifdef GSL_GESTURE
        if(gsl_gesture_flag == 1){
            gsl_enter_doze(gsl_data);
            return;
        }
#endif

#ifndef GSL_GESTURE
    disable_irq_nosync(gsl_data->client->irq);
    gpio_set_value(gsl_data->pdata->reset_gpio, 0);
#endif
    return;
}

static void gsl_ts_resume(void)
{
    struct i2c_client *client = gsl_data->client;

  printk("==gslX68X_ts_resume=\n");
    if(1==gsl_data->updating_fw){
        gsl_data->suspended = 0;
        return;
    }

    /* Proximity Sensor */

    /*Gesture Resume*/
    #ifdef GSL_GESTURE
        if(gsl_gesture_flag == 1){
            gsl_quit_doze(gsl_data);
        }
    #endif

    gpio_set_value(gsl_data->pdata->reset_gpio, 1);
    msleep(20);
    gsl_reset_core(client);

    /*Gesture Resume*/
    #ifdef GSL_GESTURE
        #ifdef GSL_ALG_ID
            gsl_DataInit(gsl_config_data_id_DJ);
        #endif
    #endif

    gsl_start_core(client);
    msleep(20);
    check_mem_data(client);
    enable_irq(client->irq);


#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1){
        return;
    }
#endif

#ifdef GSL_TIMER
    queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif

    gsl_data->suspended = 0;
    return;

}

static void gsl_ts_resume_work(struct work_struct *work)
{
    gsl_ts_resume();
}

static int fb_notifier_callback(struct notifier_block *self,
                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    if (evdata && evdata->data && event == FB_EVENT_BLANK ){
        blank = evdata->data;
      printk("fb_notifier_callback blank=%d\n",*blank);
        if (*blank == FB_BLANK_UNBLANK) {
            if (!work_pending(&gsl_data->resume_work)){
                schedule_work(&gsl_data->resume_work);
            }
        }else if (*blank == FB_BLANK_POWERDOWN) {
            cancel_work_sync(&gsl_data->resume_work);
            gsl_ts_suspend();
        }
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void gsl_early_suspend(struct early_suspend *handler)
{
    u32 tmp =0;
    struct i2c_client *client = gsl_data->client;
    printk("==gslX68X_ts_suspend=\n");
    //version info
    printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

    if(1==gsl_data->updating_fw)
        return;

    gsl_data->suspended = 1;

#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1){
        return;
    }
#endif

#ifdef GSL_ALG_ID
    tmp = gsl_version_id();
    printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
#ifdef GSL_TIMER
    cancel_delayed_work_sync(&gsl_timer_check_work);
#endif

    disable_irq_nosync(client->irq);
    gpio_set_value(gsl_data->pdata->reset_gpio, 0);
}

static void gsl_early_resume(struct early_suspend *handler)
{
    struct i2c_client *client = gsl_data->client;
    printk("==gslX68X_ts_resume=\n");
    if(1==gsl_data->updating_fw){
        gsl_data->suspended = 0;
        return;
    }

    gpio_set_value(gsl_data->pdata->reset_gpio, 1);
    msleep(20);
    gsl_reset_core(client);
    gsl_start_core(client);
    msleep(20);
    check_mem_data(client);
    enable_irq(client->irq);
#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1){
        return;
    }
#endif

#ifdef GSL_TIMER
    queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif

    gsl_data->suspended = 0;

}
#endif

static int gsl_pinctrl_init(struct gsl_ts_data *ts)
{
    int ret  = 0;
    struct i2c_client *client = ts->client;

   printk("Enter");
    if (NULL == ts) {
        printk("ts is NULL exit");
        return -EPERM;
    }

    ts->pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        printk("Failed to get pinctrl, please check dts");
        ret = PTR_ERR(ts->pinctrl);
        goto err_pinctrl_get;
    }

    ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
    if (IS_ERR_OR_NULL(ts->pins_active)) {
        printk("Pin state[active] not found");
        ret = PTR_ERR(ts->pins_active);
        goto err_pinctrl_lookup;
    }
    return 0;

err_pinctrl_lookup:
    if (ts->pinctrl) {
        devm_pinctrl_put(ts->pinctrl);
    }
err_pinctrl_get:
    ts->pinctrl = NULL;
    ts->pins_active = NULL;

   printk("Exit");
    return ret;

}
#if defined(CONFIG_FB)
static int gsl_get_dt_coords(struct device *dev, char *name,
                struct gsl_ts_platform_data *pdata)
{
    u32 coords[GSL_COORDS_ARR_SIZE];
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size, rc;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != GSL_COORDS_ARR_SIZE) {
        printk("invalid %s\n", name);
        return -EINVAL;
    }

    rc = of_property_read_u32_array(np, name, coords, coords_size);
    if (rc && (rc != -EINVAL)) {
        printk( "Unable to read %s\n", name);
        return rc;
    }

    if (!strcmp(name, "gsl,panel-coords")) {
        pdata->panel_minx = coords[0];
        pdata->panel_miny = coords[1];
        pdata->panel_maxx = coords[2];
        pdata->panel_maxy = coords[3];
    } else if (!strcmp(name, "gsl,display-coords")) {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    } else {
        printk("unsupported property %s\n", name);
        return -EINVAL;
    }

    return 0;
}

static int gsl_parse_dt(struct device *dev, struct gsl_ts_platform_data *pdata)
{

    int rc =0;
    struct device_node *np = dev->of_node;
    struct property *prop;
    u32 temp_val;

    printk("gsl_parse_dt Enter\n");
   printk("Enter");
    pdata->name = "gsl";
    rc = of_property_read_string(np, "gsl,name", &pdata->name);
    if (rc && (rc != -EINVAL)) {
        printk("Unable to read name\n");
        return rc;
    }

    rc = gsl_get_dt_coords(dev, "gsl,panel-coords", pdata);
    if (rc && (rc != -EINVAL))
        return rc;

    rc = gsl_get_dt_coords(dev, "gsl,display-coords", pdata);
    if (rc)
        return rc;
   printk("display");

    rc = of_property_read_u32(np, "gsl,hard-reset-delay-ms",
                            &temp_val);
    if (!rc)
        pdata->hard_reset_delay_ms = temp_val;
    else
        return rc;

    rc = of_property_read_u32(np, "gsl,post-hard-reset-delay-ms",
                            &temp_val);

    if (!rc)
        pdata->post_hard_reset_delay_ms = temp_val;
    else
        return rc;

   printk("delay");

    /*panel channel num */
    rc = of_property_read_u32(np, "gsl,panel_tx_num", &temp_val);
    if (!rc)
        pdata->panel_tx_num = temp_val;
    else
        return rc;

    rc = of_property_read_u32(np, "gsl,panel_rx_num", &temp_val);
    if (!rc)
        pdata->panel_rx_num = temp_val;
    else
        return rc;

    //("gpio");

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "gsl,reset-gpio",
                0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        return pdata->reset_gpio;
    printk("pdata->reset_gpio = %d\n",pdata->reset_gpio);

    pdata->irq_gpio = of_get_named_gpio_flags(np, "gsl,irq-gpio",
                0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        return pdata->irq_gpio;
    printk("pdata->irq_gpio = %d\n",pdata->irq_gpio);

    rc = of_property_read_u32(np, "gsl,num-max-touches", &temp_val);
    if (!rc)
        pdata->num_max_touches = temp_val;
    else
        return rc;

    prop = of_find_property(np, "gsl,button-map", NULL);
    if (prop) {
        pdata->num_buttons = prop->length / sizeof(temp_val);
        if (pdata->num_buttons > MAX_BUTTONS)
            return -EINVAL;
      printk("button");

        rc = of_property_read_u32_array(np, "gsl,button-map", pdata->button_map,
        pdata->num_buttons);
        if (rc) {
            printk("Unable to read key codes\n");
            return rc;
        }
    }
    printk("ok parse");

    return 0;
}
#else
static int gsl_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
    return -ENODEV;
}
#endif
#ifdef CONFIG_GET_HARDWARE_INFO
/* Modified by zhangyijie for fix ADT hardware_info module name ZQ1989-31 begin */
static int gsl_register_hardware_info(struct gsl_ts_data *ts_data)
{
    int ret = 0;

   printk("!!!! REGISTER !!!!  read_vendor_id");
    if (ret < 0)
        printk("get vendor id failed");

    ret = gsl_read_version();
    if (ret < 0)
        printk("get vendor id failed");

    if (gsl_cfg_index == GSL_VENDOR_ID) {
        snprintf(gsl_ctp_hardware_info, MAX_HARDWARE_INFO_LEN, "%s_%s_%s_FW_0x%02x",
                IC_VENDOR_NAME, IC_NAME, DJN_MODULE_NAME, ts_data->fw_version);
    } else if (gsl_cfg_index == GSL_VENDOR_ID2) {
        snprintf(gsl_ctp_hardware_info, MAX_HARDWARE_INFO_LEN, "%s_%s_%s_FW_0x%02x",
                IC_VENDOR_NAME, IC_NAME, HLT_MODULE_NAME, ts_data->fw_version);
    } else
        snprintf(gsl_ctp_hardware_info, MAX_HARDWARE_INFO_LEN, "unknow");

   printk("ctp hardware info:%s", gsl_ctp_hardware_info);

    ret = register_hardware_info(CTP, gsl_ctp_hardware_info);
    if (0 != ret) {
       printk("register_hardware_info failed\n");
        return ret;
    }

    return ret;
}
/* Modified by zhangyijie for fix ADT hardware_info module name ZQ1989-31 end */
#endif

static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    u8 read_buf[4]={0};
    struct input_dev *input_dev = NULL;
    struct gsl_ts_platform_data *pdata = NULL;

    printk("gsl_ts_probe enter!\n");
    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
            sizeof(struct gsl_ts_platform_data), GFP_KERNEL);
        if (!pdata) {
            printk("Failed to allocate memory\n");
            return -ENOMEM;
        }

        err = gsl_parse_dt(&client->dev, pdata);
        if (err) {
            printk("Failed to parse dt");
            err = -ENOMEM;
            goto exit_parse_dt_err;
        }
    } else {
        pdata = client->dev.platform_data;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        printk("I2c doesn't work");
        goto exit_check_functionality_failed;
    }

    gsl_data = devm_kzalloc(&client->dev,sizeof(struct gsl_ts_data), GFP_KERNEL);
    if (gsl_data==NULL) {
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    gsl_data->suspended = 0;
    gsl_data->updating_fw = 0;
    gsl_data->gsl_up_flag = 0;
    gsl_data->gsl_point_state = 0;
#ifdef GSL_HAVE_TOUCH_KEY
    gsl_data->gsl_key_state = 0;
#endif
    gsl_data->cinfo = kzalloc(sizeof(struct gsl_touch_info),GFP_KERNEL);
    if(gsl_data->cinfo == NULL) {
        err = -ENOMEM;
        goto exit_alloc_cinfo_failed;
    }

    mutex_init(&gsl_data->gsl_i2c_lock);
#ifdef GSL_GESTURE
    wake_lock_init(&gsl_data->gsl_wake_lock, 0, "gsl_wake_lock");
#endif

    /*allocate input device*/
   printk("==input_allocate_device=");

    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
       printk("input_allocate_device faild");
        goto err_allocate_input_device_fail;
    }

    input_dev->name = client->name;
    input_dev->phys = "input/gsl_ts";
    input_dev->dev.parent = &client->dev;///sss
    input_dev->id.bustype = BUS_I2C;

    gsl_data->input_dev = input_dev;
    gsl_data->client = client;
    gsl_data->pdata = pdata;
    i2c_set_clientdata(client, gsl_data);
    private_ts = gsl_data;
   // ts_data = gsl_data;
   printk("I2C addr=%x\n", client->addr);

    err = gsl_ts_power_init(gsl_data, true);
    if (err) {
        printk("Silead power init failed\n");
        goto exit_power_init_err;
    }

    err = gsl_ts_power_on(gsl_data, true);
    if (err) {
        printk("Silead power on failed\n");
        goto exit_power_on_err;
    }

    if (gpio_is_valid(pdata->irq_gpio)) {
            err = gpio_request(pdata->irq_gpio, GSL_IRQ_NAME);
            if (err) {
                printk("irq gpio request failed\n");
                goto exit_request_irq_gpio_err;
            }
            err = gpio_direction_input(pdata->irq_gpio);
            if (err) {
                printk("set_direction for irq gpio failed\n");
            goto exit_set_irq_gpio_err;
            }
        }

    if (gpio_is_valid(pdata->reset_gpio)) {
        err = gpio_request(pdata->reset_gpio, GSL_RST_NAME);
        if (err) {
            printk("reset gpio request failed");
            goto exit_request_reset_gpio_err;
        }

        err = gpio_direction_output(pdata->reset_gpio, 0);
        if (err) {
            printk("set_direction for reset gpio failed\n");
            goto exit_set_reset_gpio_err;
        }
        msleep(20);
        gpio_set_value_cansleep(pdata->reset_gpio, 1);
    }

    /* make sure CTP already finish startup process */
    msleep(100);

    err = gsl_read_version();
    if(err < 0) {
        goto exit_i2c_transfer_fail;
    }

    err = gsl_compatible_id(client);
    if(err < 0) {
        goto exit_i2c_transfer_fail;
    }
    err = gsl_pinctrl_init(gsl_data);
    if (err < 0) {
        printk("gsl_pinctrl_init faild");
    }

#ifdef GSL_COMPATIBLE_GPIO
    gsl_gpio_idt_tp(client);
#endif

    /*request input system*/
    err = gsl_request_input_dev(gsl_data);
    if(err < 0) {
        goto exit_i2c_transfer_fail;
    }

    /*register early suspend*/
  printk("==register_early_suspend ==");

    #if defined(CONFIG_FB)
        INIT_WORK(&gsl_data->resume_work, gsl_ts_resume_work);
        gsl_data->fb_notif.notifier_call = fb_notifier_callback;
        err = fb_register_client(&gsl_data->fb_notif);
        if (err)
            printk("Unable to register fb_notifier: %d\n", err);
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
        gsl_data->pm.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        gsl_data->pm.suspend = gsl_early_suspend;
        gsl_data->pm.resume = gsl_early_resume;
        register_early_suspend(&gsl_data->pm);
    #endif

    /*init work queue*/
    INIT_WORK(&gsl_data->work, gsl_report_work);
    gsl_data->wq = create_singlethread_workqueue(dev_name(&client->dev));
    if (!gsl_data->wq) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }

    /*request irq */
	client->irq = GSL_IRQ_NUM;
    printk("irq = %d",client->irq);
    err = request_irq(client->irq, gsl_ts_interrupt, IRQF_TRIGGER_RISING, client->name, gsl_data);
    if (err < 0) {
        printk("gslX68X_probe: request irq failed\n");
        goto exit_irq_request_failed;
    }

    disable_irq_nosync(client->irq);

    /*gesture resume*/
#ifdef GSL_GESTURE
    gsl_FunIICRead(gsl_read_oneframe_data);
#endif

    /*gsl of software init*/
    gsl_sw_init(client);
    msleep(20);
    check_mem_data(client);
    gsl_read_interface(client,0xb0,read_buf,4);
    printk("0xb0 after judgment = {0x%02x%02x%02x%02x}",
                read_buf[3],read_buf[2],read_buf[1],read_buf[0]);
#ifdef CONFIG_GET_HARDWARE_INFO
    err = gsl_register_hardware_info(gsl_data);
    if (err < 0) {
        printk("register hardware faild");
    }
#endif
#ifdef TOUCH_VIRTUAL_KEYS
    gsl_ts_virtual_keys_init();
#endif

#ifdef TPD_PROC_DEBUG
   proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);

  printk("GSL *********gsl_config_proc********");
/*
    if (gsl_config_proc == NULL){
        dev_info("create_proc_entry %s failed", GSL_CONFIG_PROC_FILE);
    }else{
        dev_info("create proc entry %s success", GSL_CONFIG_PROC_FILE);

    }
*/
    gsl_proc_flag = 0;
#endif

#ifdef GSL_TIMER
    INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
    gsl_timer_workqueue = create_workqueue("gsl_timer_check");
    queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif

#ifdef GSL_GESTURE
    gsl_gesture_init(gsl_data);
#endif

    err = sysfs_create_group(&client->dev.kobj, &gsl_attr_group);
    if (err < 0) {
        printk("sysfs_create_group faild");
    }

#ifdef GSL_TEST_TP
    create_ctp_proc();
#endif
    gsl_read_version();
    enable_irq(client->irq);

#ifdef GSL_GESTURE
    enable_irq_wake(client->irq);
#endif
    printk("gsl_ts_probe succeed!\n");
    printk("==probe end succeed ==");
    return 0;

exit_irq_request_failed:
#ifdef IRQ_WITH_WORKQUEUE
    cancel_work_sync(&gsl_data->work);
    destroy_workqueue(gsl_data->wq);
exit_create_singlethread:
#endif
    #if defined(CONFIG_FB)
    if (fb_unregister_client(&gsl_data->fb_notif))
        printk("Error occurred while unregistering fb_notifier.\n");

    #elif defined(CONFIG_HAS_EARLYSUSPEND)
        unregister_early_suspend(&gsl_data->pm);
    #endif

    input_unregister_device(gsl_data->input_dev);
    input_free_device(gsl_data->input_dev);
exit_i2c_transfer_fail:
exit_set_reset_gpio_err:
    if (gpio_is_valid(gsl_data->pdata->reset_gpio))
        gpio_free(gsl_data->pdata->reset_gpio);
exit_request_reset_gpio_err:
exit_set_irq_gpio_err:
    if (gpio_is_valid(gsl_data->pdata->irq_gpio))
        gpio_free(gsl_data->pdata->irq_gpio);
exit_request_irq_gpio_err:
    gsl_ts_power_on(gsl_data, false);
exit_power_on_err:
    gsl_ts_power_init(gsl_data, false);
exit_power_init_err:
err_allocate_input_device_fail:
    /* Modified by zhangyijie for fix system dump without tp ZQ1989-129 2018-04-04 begin */
    #ifdef GSL_GESTURE
    wake_lock_destroy(&gsl_data->gsl_wake_lock);
    #endif
    /* Modified by zhangyijie for fix system dump without tp ZQ1989-129 2018-04-04 end */

    i2c_set_clientdata(client, NULL);
    kfree(gsl_data->cinfo);
exit_alloc_cinfo_failed:
    /*Modified by zhangyijie for fix can't start when without tp ZQ1989-27 2018-03-26 begin*/
    devm_kfree(&client->dev, gsl_data);
    /*Modified by zhangyijie for fix can't start when without tp ZQ1989-27 2018-03-26 end*/
exit_alloc_data_failed:
exit_check_functionality_failed:
exit_parse_dt_err:
    devm_kfree(&client->dev, pdata);

    return err;
}

static int  gsl_ts_remove(struct i2c_client *client)
{
  printk("==gslX68X_ts_remove=\n");
/*
#ifdef TPD_PROC_DEBUG
    if(gsl_config_proc!=NULL)
        remove_proc_entry(GSL_CONFIG_PROC_FILE, NULL);
#endif
*/
    if (fb_unregister_client(&gsl_data->fb_notif))
          printk("Error occurred while unregistering fb_notifier.\n");

    free_irq(client->irq,gsl_data);
    input_unregister_device(gsl_data->input_dev);
    input_free_device(gsl_data->input_dev);
    gpio_free(gsl_data->pdata->reset_gpio);
    gpio_free(gsl_data->pdata->irq_gpio);

    cancel_work_sync(&gsl_data->work);
    destroy_workqueue(gsl_data->wq);
    i2c_set_clientdata(client, NULL);
    //sprd_free_gpio_irq(client->irq);
    kfree(gsl_data->cinfo);
    kfree(gsl_data);
    return 0;
}


static const struct i2c_device_id gsl_ts_id[] = {
    { GSL_TS_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, gsl_ts_id);


#if defined(CONFIG_FB)
static struct of_device_id gsl_match_table[] = {
    { .compatible = "silead,gsl-tp",},
    { },
};
#else
#define gsl_match_table NULL
#endif

static struct i2c_driver gsl_ts_driver = {
    .driver = {
        .name = GSL_TS_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = gsl_match_table,
    },
    .probe = gsl_ts_probe,
    .remove = gsl_ts_remove,
    .id_table   = gsl_ts_id,
};


module_i2c_driver(gsl_ts_driver);

MODULE_AUTHOR("sileadinc");
MODULE_DESCRIPTION("GSL Series Driver");
MODULE_LICENSE("GPL");


