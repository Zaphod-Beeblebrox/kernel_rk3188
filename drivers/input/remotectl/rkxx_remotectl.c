/*$_rbox_$_ modify _$hzb,20120522*/
/*$_rbox_$_ modify _$add this file for rk29 remotectl*/

/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/adc.h>
#include <asm/gpio.h>
#include <mach/remotectl.h>
#include <mach/iomux.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>

extern int bonovo_get_suspend_status(void);


#if 0
#define remotectl_dbg(bdata, format, arg...)		\
	dev_printk(KERN_INFO , &bdata->input->dev , format , ## arg)
#else
#define remotectl_dbg(bdata, format, arg...)	
#endif

//#define DEBUG_ZBIAO
#ifdef DEBUG_ZBIAO
#define printk_zbiao(format, arg...)	\
	printk(format, ##arg);
#else
#define printk_zbiao(format, arg...) 
#endif

extern suspend_state_t get_suspend_state(void);

struct rkxx_remotectl_suspend_data{
    int suspend_flag;
    int cnt;
    long scanTime[50];
};

struct rkxx_remotectl_drvdata {
    int state;
	int nbuttons;
	int result;
    unsigned long pre_time;
    unsigned long cur_time;
    long period;
    int scanData;
    int count;
    int keybdNum;
    int keycode;
    int press;
    int pre_press;

    struct bonovo_advance_keys * rc_using;
    struct input_dev *input;
    struct timer_list timer;
    struct tasklet_struct remote_tasklet;
    struct wake_lock remotectl_wake_lock;
    struct rkxx_remotectl_suspend_data remotectl_suspend_data;
};


//特殊功能键值定义
    //193      //photo
    //194      //video
    //195      //music
    //196      //IE
    //197      //
    //198
    //199
    //200
    
    //183      //rorate_left
    //184      //rorate_right
    //185      //zoom out
    //186      //zoom in
/*    
static struct rkxx_remote_key_table remote_key_table_meiyu_202[] = {
    {0xB0, KEY_ENTER},
    {0xA2, KEY_BACK}, 
    {0xD0, KEY_UP},
    {0x70, KEY_DOWN},
    {0x08, KEY_LEFT},
    {0x88, KEY_RIGHT},  ////////
    {0x42, KEY_HOME},     //home
    {0xA8, KEY_VOLUMEUP},
    {0x38, KEY_VOLUMEDOWN},
    {0xE2, KEY_SEARCH},     //search
    {0xB2, KEY_POWER},     //power off
    {0xC2, KEY_MUTE},       //mute
    {0xC8, KEY_MENU},

//media ctrl
    {0x78,   0x190},      //play pause
    {0xF8,   0x191},      //pre
    {0x02,   0x192},      //next

//pic
    {0xB8, 183},          //rorate left
    {0x58, 184},          //rorate right
    {0x68, 185},          //zoom out
    {0x98, 186},          //zoom in
//mouse switch
    {0xf0,388},
//display switch
    {0x82,   0x175},
};

static struct rkxx_remote_key_table remote_key_table_df[] = {
    {0xf8, KEY_REPLY},
    {0xc0, KEY_BACK}, 
    {0xf0, KEY_UP},
    {0xd8, KEY_DOWN},
    {0xd0, KEY_LEFT},
    {0xe8,KEY_RIGHT},  ////////
    {0x90, KEY_VOLUMEDOWN},
    {0x60, KEY_VOLUMEUP},
    {0x80, KEY_HOME},     //home
    {0xe0, 183},          //rorate left
    {0x10, 184},          //rorate right
    {0x20, 185},          //zoom out
    {0xa0, 186},          //zoom in
    {0x70, KEY_MUTE},       //mute
    {0x50, KEY_POWER},     //power off
    {0x40, KEY_SEARCH},     //search
};

static struct rkxx_remote_key_table remote_key_table_lenovo[] = {
    {0x7a, KEY_ENTER},
    {0x00, KEY_REPLY},
    {0x18, KEY_BACK}, 
    {0x88, KEY_UP},
    {0x10, KEY_DOWN},
    {0x90, KEY_LEFT},
    {0x28, KEY_RIGHT},  ////////
    {0x92, KEY_VOLUMEDOWN},
    {0x52, KEY_VOLUMEUP},
    {0xb8, KEY_HOMEPAGE},     //home
    {0xd8, KEY_MENU},     // menu
    {0xe2, 177},          //rorate left
    {0x50, 178},          //rorate right
    {0x20, 185},          //zoom out
    {0xa0, 186},          //zoom in
    {0xe8, KEY_MUTE},       //mute
    {0x08, KEY_POWER},     //power off
    {0xf8, KEY_SEARCH},     //search
    {0x4a, KEY_0},       // 0
    {0xc2, KEY_1},       // 1
    {0x02, KEY_2},       // 2
    {0xc8, KEY_3},       // 3
    {0xb2, KEY_4},       // 4
    {0x32, KEY_5},       // 5
    {0xd2, KEY_6},       // 6
    {0x0a, KEY_7},       // 7
    {0xf2, KEY_8},       // 8
    {0x72, KEY_9},       // 9
    {0xca, KEY_NUMERIC_STAR},       // *
    {0x8a, KEY_NUMERIC_POUND},       // #
};

extern suspend_state_t get_suspend_state(void);


static struct rkxx_remotectl_button remotectl_button[] = 
{
    {  
       .usercode = 0x202, 
       .nbuttons =  22, 
       .key_table = &remote_key_table_meiyu_202[0],
    },
    {  
       .usercode = 0xdf, 
       .nbuttons =  16, 
       .key_table = &remote_key_table_df[0],
    },
    {  
       .usercode = 0x00FF, 
       .nbuttons =  30, 
       .key_table = &remote_key_table_lenovo[0],
    },
};
*/
//#define ANDROID_IR
extern struct rkxx_remotectls bonovo_irctls;
extern struct rkxx_remotectl_button irctl_table;
extern struct bonovo_rc_buff bonovo_keys_buff;      // keys' buff.
struct input_dev *input;
struct rkxx_remotectl_drvdata *pbonovo_data = NULL;

extern void bonovo_buff_key(struct bonovo_rc_buff *buff, int addrCode, int scanCode);
extern int bonovo_is_buff_empty(struct bonovo_rc_buff * buff);
extern int bonovo_is_buff_full(struct bonovo_rc_buff * buff);
extern int bonovo_buff_add(struct bonovo_rc_buff * buff, int addrCode, int scanCode);
extern int bonovo_buff_remove_head(struct bonovo_rc_buff * buff, int *addrCode, int *scanCode);
extern int bonovo_is_custom_mode(void);
extern int bonovo_is_half_mode(void);
extern void bonovo_set_half_flag(void);
extern void bonovo_clear_half_flag(void);
extern void bonovo_custom_ir_key(int keyCode, int keyStatus);

void rc_register_buttons(struct bonovo_advance_keys * rc)
{
    int i;
    printk_zbiao("++++++++++ rc->ekey_type:%d rc->nbuttons:%d\n", rc->ekey_type, rc->nbuttons);
    for (i = 0; i < rc->nbuttons; i++) {
        unsigned int type = EV_KEY;
		printk_zbiao("++++++++++ rc->key_table[%d].keyCode:%d\n", i, rc->key_table[i].keyCode);
        if((rc->key_table[i].keyCode < KEY_BONOVO_MIN_VALUE)
            || (rc->key_table[i].keyCode > KEY_BONOVO_MAX_VALUE)){
            input_set_capability(input, type, rc->key_table[i].keyCode);
        }
    }
}

static int remotectl_keybdNum_lookup(struct rkxx_remotectl_drvdata *ddata)
{	
/*    int i;	

    for (i = 0; i < bonovo_irctls.num; i++){		
        if (remotectl_button[i].usercode == (ddata->scanData&0xFFFF)){			
            ddata->keybdNum = i;
            return 1;
        }
    }*/

    struct bonovo_advance_keys * rc_index;
    unsigned long flags = 0;

    read_lock_irqsave(&bonovo_irctls.ir_rwlock, flags);
    list_for_each_entry(rc_index, &bonovo_irctls.irctls_list, node){
        if(rc_index->usercode == (ddata->scanData&0xFFFF)){
//            ddata->keybdNum = i;
            ddata->rc_using = rc_index;
            read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
            return 1;
        }
    }
    read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
    return 0;
}


static int remotectl_keycode_lookup(struct rkxx_remotectl_drvdata *ddata)
{	
    int i;	
    unsigned char keyData = ddata->scanData & 0xFFFF;//((ddata->scanData >> 8) & 0xff);
    unsigned long flags = 0;
/*
    for (i = 0; i < remotectl_button[ddata->keybdNum].nbuttons; i++){
        if (remotectl_button[ddata->keybdNum].key_table[i].scanCode == keyData){			
            ddata->keycode = remotectl_button[ddata->keybdNum].key_table[i].keyCode;
            return 1;
        }
    }
*/
    read_lock_irqsave(&bonovo_irctls.ir_rwlock, flags);
    for (i = 0; i < ddata->rc_using->nbuttons; i++){
        if (ddata->rc_using->key_table[i].scanCode == keyData){			
            ddata->keycode = ddata->rc_using->key_table[i].keyCode;
            read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
            return 1;
        }
    }
    read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
    return 0;
}


struct bonovo_rc_key rc_key;
static void remotectl_do_something(unsigned long  data)
{
    struct rkxx_remotectl_drvdata *ddata = (struct rkxx_remotectl_drvdata *)data;
    unsigned long flags = 0;

    switch (ddata->state)
    {
    case RMC_IDLE:
        {
            ;
        }
        break;
        
    case RMC_PRELOAD:
        { 
            //printk_zbiao("ddata->period = 0x%08x\n",  ddata->period);
            if ((TIME_PRE_MIN < ddata->period) && (ddata->period < TIME_PRE_MAX)){
                
                ddata->scanData = 0;
                ddata->count = 0;
                ddata->state = RMC_USERCODE;
            }else{
                ddata->state = RMC_PRELOAD;
            }
            ddata->pre_time = ddata->cur_time;
            //mod_timer(&ddata->timer,jiffies + msecs_to_jiffies(130));
        }
        break;
        
    case RMC_USERCODE:
        {
            ddata->scanData <<= 1;
            ddata->count ++;

            if ((TIME_BIT1_MIN < ddata->period) && (ddata->period < TIME_BIT1_MAX)){
                ddata->scanData |= 0x01;
            }
            
            if (ddata->count == 0x10){//16 bit user code
                //printk("u=0x%04x\n",((ddata->scanData)&0xFFFF));
                if ((ddata->scanData&0x0ff) == ((~ddata->scanData >> 8)&0x0ff)){
                    flags = 0;
                    read_lock_irqsave(&bonovo_irctls.ir_rwlock, flags);
                    if(bonovo_is_custom_mode())
                    {
                        read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
                        bonovo_set_half_flag();
                        //printk_zbiao("u=0x%04x\n",((ddata->scanData)&0xFFFF));
                        rc_key.addrCode = ddata->scanData & 0xFFFF;
                        ddata->state = RMC_GETDATA;
                        ddata->scanData = 0;
                        ddata->count = 0;
                    }
                    else
                    {
                        read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
                        if (remotectl_keybdNum_lookup(ddata)){
                            ddata->state = RMC_GETDATA;
                            ddata->scanData = 0;
                            ddata->count = 0;
                        }else{                //user code error
                            //ddata->state = RMC_PRELOAD;
                            // new usercode
							bonovo_set_half_flag();
                        	//printk_zbiao("new usercode :0x%04x\n",((ddata->scanData)&0xFFFF));
                        	rc_key.addrCode = ddata->scanData & 0xFFFF;
                        	ddata->state = RMC_GETDATA;
                        	ddata->scanData = 0;
                        	ddata->count = 0;
                        }
                    }
                }
				else{
					ddata->state = RMC_PRELOAD;
				//	ddata->scanData = 0;
                //    ddata->count = 0;
				}
            } 
        }
        break;
        
    case RMC_GETDATA:
        {
            ddata->count ++;
            ddata->scanData <<= 1;
      
            if ((TIME_BIT1_MIN < ddata->period) && (ddata->period < TIME_BIT1_MAX)){
                ddata->scanData |= 0x01;
            }           
            if (ddata->count == 0x10){
                //printk("d=0x%04x\n",(ddata->scanData&0xFFFF));
                if ((ddata->scanData&0x0ff/*0x00f*/) == ((~ddata->scanData >> 8)&0x0ff/*0x00f*/)){
                    if(bonovo_is_half_mode()){
                        bonovo_clear_half_flag();
                        //printk_zbiao("d=0x%04x\n",(ddata->scanData&0xFFFF));
                        rc_key.scanCode = ddata->scanData & 0xFFFF;
                        //printk_zbiao("===========rc_key. addrCode:0x%04x  scanCode:0x%04x\n", 
                        //    rc_key.scanCode, rc_key.addrCode);
                        bonovo_buff_key(&bonovo_keys_buff, rc_key.addrCode, rc_key.scanCode);
						bonovo_custom_ir_key(KEY_BONOVO_CUSTOM_IR_BUTTON, 1);
						bonovo_custom_ir_key(KEY_BONOVO_CUSTOM_IR_BUTTON, 0);
                        memset(&rc_key, 0, sizeof(rc_key));
                    }
                    else
                    {
                        if (remotectl_keycode_lookup(ddata)){
                            ddata->press = 1;
							//printk_zbiao("status:%d\n", bonovo_get_suspend_status());
                            if (bonovo_get_suspend_status()==0){
								//printk_zbiao("=== send input event.\n");
                                input_event(ddata->input, EV_KEY, ddata->keycode, 1);
                                input_sync(ddata->input);
                            }else if (/*(get_suspend_state())&&*/(ddata->keycode==KEY_POWER)){
                            	//printk_zbiao("=== send input event:KEY_WAKEUP.\n");
                                input_event(ddata->input, EV_KEY, KEY_WAKEUP, 1);
                                input_sync(ddata->input);
                            }
                            //input_event(ddata->input, EV_KEY, ddata->keycode, ddata->press);
	                          //input_sync(ddata->input);
                            ddata->state = RMC_SEQUENCE;
                        }else{
                        	//printk_zbiao("------ lookup key failed.\n");
                            ddata->state = RMC_PRELOAD;
                        }
                    }
                }else{
                    ddata->state = RMC_PRELOAD;
                }
            }
        }
        break;
             
    case RMC_SEQUENCE:
        {
            //printk( "S\n");
        
            if ((TIME_RPT_MIN < ddata->period) && (ddata->period < TIME_RPT_MAX)){
                ;
            }else if ((TIME_SEQ_MIN < ddata->period) && (ddata->period < TIME_SEQ_MAX)){
                if (ddata->press == 1){
                    ddata->press = 3;
                }else if (ddata->press & 0x2){
                    ddata->press = 2;
                    //input_event(ddata->input, EV_KEY, ddata->keycode, 2);
                    //input_sync(ddata->input);
                }
                //mod_timer(&ddata->timer,jiffies + msecs_to_jiffies(130));
                //ddata->state = RMC_PRELOAD;
            }
        }
        break;
       
    default:
            break;
    }
    
    return;
}

#define MAX_ABS 40
int bonovo_deal_advance_key(int keyCode, int keyStatus)
{
    struct bonovo_rc_key rc_key;
    int i, is_found_in_list = 0;	
    unsigned long flags = 0;
    struct bonovo_advance_keys *pos;
    unsigned char src_key_type = (keyCode>>16)&0x00FF;
    eKEY_TYPE type;

    printk_zbiao("======== keyCode:0x%08X, keyStatus:%d, src_key_type:0x%04X\n",
        keyCode, keyStatus, src_key_type);

    if(keyStatus){
        keyStatus = 1;
    }else{
        keyStatus = 0;
    }

    switch(src_key_type){
        case 0x00:
        case 0x01:
            type = KEY_TYPE_PANEL;
            break;
        case 0x02:
            switch(keyCode&0x00FFFF){
                case 200:
                    input_event(input, EV_KEY, KEY_POWER, keyStatus);
                    input_sync(input);
                    printk("====== KEY_POWER keyStatus:%d\n", keyStatus);
                    break;
                case 400:
                    input_event(input, EV_KEY, KEY_HOMEPAGE, keyStatus);
                    input_sync(input);
                    break;
                case 600:
                    input_event(input, EV_KEY, KEY_BACK, keyStatus);
                    input_sync(input);
                    break;
                case 800:
                    bonovo_custom_ir_key(KEY_BONOVO_VOLUME_ADD, keyStatus);
                    break;
                case 1000:
                    bonovo_custom_ir_key(KEY_BONOVO_VOLUME_SUB, keyStatus);
                    break;
                case 1200:
                    bonovo_custom_ir_key(KEY_BONOVO_SYSTEM_MUTE, keyStatus);
                    break;
                default:
                    printk_zbiao("========default keyCode:0x%08X\n", keyCode);
                    break;
            }
            type = KEY_TYPE_PANEL;
            return 0;
            break;
        case 0x03:
            type = KEY_TYPE_IR;
            break;
        case 0x04:
        case 0x05:
            type = KEY_TYPE_BACK_ADC;
            break;
		case 0x06:		// canbus key
            type = KEY_TYPE_BACK_ADC;
            break;
        default:
            type = KEY_TYPE_CNT;
            return -1;
            break;
    }

    if(bonovo_is_custom_mode()){
        memset(&rc_key, 0, sizeof(rc_key));
        list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
            if(type == pos->ekey_type){
                rc_key.addrCode = pos->usercode;
                is_found_in_list = 1;
            }
        }
        if(is_found_in_list == 0){
            rc_key.addrCode = 0;
        }
        rc_key.scanCode = keyCode;
        bonovo_buff_key(&bonovo_keys_buff, rc_key.addrCode, rc_key.scanCode);
        bonovo_custom_ir_key(KEY_BONOVO_CUSTOM_IR_BUTTON, 1);
		bonovo_custom_ir_key(KEY_BONOVO_CUSTOM_IR_BUTTON, 0);

    }else{
        //keyCode = keyCode & 0x00FFFF;
        read_lock_irqsave(&bonovo_irctls.ir_rwlock, flags);
        list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
            if(type == pos->ekey_type){
                is_found_in_list = 1;
                for(i=0; i<pos->nbuttons; i++){
                    printk_zbiao("tyep:%d  key[%d] --- accuracy:%d scanCode:%d  keyCode:%d\n", pos->ekey_type, i, 
                        pos->accuracy, pos->key_table[i].scanCode, pos->key_table[i].keyCode);
                    if (abs(pos->key_table[i].scanCode - keyCode) <= pos->accuracy){
                        if(pos->key_table[i].keyCode == KEY_VOLUMEUP){
                            bonovo_custom_ir_key(KEY_BONOVO_VOLUME_ADD, keyStatus);
                        }else if(pos->key_table[i].keyCode == KEY_VOLUMEDOWN){
                            bonovo_custom_ir_key(KEY_BONOVO_VOLUME_SUB, keyStatus);
                        }else if(pos->key_table[i].keyCode == KEY_MUTE){
                            bonovo_custom_ir_key(KEY_BONOVO_SYSTEM_MUTE, keyStatus);
                        }else if((pos->key_table[i].keyCode >= KEY_BONOVO_MIN_VALUE)
                            && (pos->key_table[i].keyCode <= KEY_BONOVO_MAX_VALUE)){
                            bonovo_custom_ir_key(pos->key_table[i].keyCode, keyStatus);
                        }else {
                            if(get_suspend_state() && (pos->key_table[i].keyCode==KEY_POWER)){
                                input_event(input, EV_KEY, KEY_WAKEUP, keyStatus);
                                input_sync(input);
                            }else{
                                input_event(input, EV_KEY, pos->key_table[i].keyCode, keyStatus);
                                input_sync(input);
                            }
                        }
                    }
                }
            }
        }
        read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
    }
   
    return 0;
}

int  bonovo_deal_board_key(int keyCode, int keyStatus)
{
    struct bonovo_rc_key rc_key;
    int i;	
    unsigned long flags = 0;

    if(keyStatus){
        keyStatus = 1;
    }else{
        keyStatus = 0;
    }

    if(bonovo_is_custom_mode()){
        memset(&rc_key, 0, sizeof(rc_key));
        rc_key.addrCode = irctl_table.usercode;
        rc_key.scanCode = keyCode;
        bonovo_buff_key(&bonovo_keys_buff, rc_key.addrCode, rc_key.scanCode);
        bonovo_custom_ir_key(KEY_BONOVO_CUSTOM_IR_BUTTON, 1);
		bonovo_custom_ir_key(KEY_BONOVO_CUSTOM_IR_BUTTON, 0);
    }else{
        read_lock_irqsave(&bonovo_irctls.ir_rwlock, flags);
        for (i = 0; i < irctl_table.nbuttons; i++){
            if (abs(irctl_table.key_table[i].scanCode - keyCode) < MAX_ABS){			
                if (bonovo_get_suspend_status()==0){
					if(irctl_table.key_table[i].keyCode == KEY_VOLUMEUP){
						bonovo_custom_ir_key(KEY_BONOVO_VOLUME_ADD, keyStatus);
					}else if(irctl_table.key_table[i].keyCode == KEY_VOLUMEDOWN){
						bonovo_custom_ir_key(KEY_BONOVO_VOLUME_SUB, keyStatus);
					}else{
                        input_event(input, EV_KEY, irctl_table.key_table[i].keyCode, keyStatus);
                        input_sync(input);
					}
					//input_event(input, EV_KEY, irctl_table.key_table[i].keyCode, 0);
                    //input_sync(input);
                }else if (/*(get_suspend_state())&&*/(irctl_table.key_table[i].keyCode==KEY_POWER)){
                    input_event(input, EV_KEY, KEY_WAKEUP, keyStatus);
                    input_sync(input);
					//input_event(input, EV_KEY, KEY_WAKEUP, 0);
                    //input_sync(input);
                }else if((irctl_table.key_table[i].keyCode == KEY_VOLUMEUP) ||
                    (irctl_table.key_table[i].keyCode == KEY_VOLUMEDOWN)){
                    if(irctl_table.key_table[i].keyCode == KEY_VOLUMEUP){
						bonovo_custom_ir_key(KEY_BONOVO_VOLUME_ADD, keyStatus);
					}else if(irctl_table.key_table[i].keyCode == KEY_VOLUMEDOWN){
						bonovo_custom_ir_key(KEY_BONOVO_VOLUME_SUB, keyStatus);
					}
				}
				break;
            }
        }
		read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
    }
   
    return 0;
}

#ifdef CONFIG_PM
void remotectl_wakeup(unsigned long _data)
{
    struct rkxx_remotectl_drvdata *ddata =  (struct rkxx_remotectl_drvdata*)_data;
    long *time;
    int i;

    time = ddata->remotectl_suspend_data.scanTime;

    if (/*get_suspend_state()*/bonovo_get_suspend_status()){
        
        static int cnt;
       
        ddata->remotectl_suspend_data.suspend_flag = 0;
        ddata->count = 0;
        ddata->state = RMC_USERCODE;
        ddata->scanData = 0;
        
        for (i=0;i<ddata->remotectl_suspend_data.cnt;i++){
            if (((TIME_BIT1_MIN<time[i])&&(TIME_BIT1_MAX>time[i]))||((TIME_BIT0_MIN<time[i])&&(TIME_BIT0_MAX>time[i]))){
                cnt = i;
                break;;
            }
        }
        
        for (;i<cnt+32;i++){
            ddata->scanData <<= 1;
            ddata->count ++;

            if ((TIME_BIT1_MIN < time[i]) && (time[i] < TIME_BIT1_MAX)){
                ddata->scanData |= 0x01;
            }
            
            if (ddata->count == 0x10){//16 bit user code
                          
                if (ddata->state == RMC_USERCODE){
//                    printk(KERN_ERR "d=%x\n",(ddata->scanData&0xFFFF));  
                    if (remotectl_keybdNum_lookup(ddata)){
                        ddata->scanData = 0;
                        ddata->count = 0;
                        ddata->state = RMC_GETDATA;
                    }else{
                        ddata->state = RMC_PRELOAD;
                    }
                }else if (ddata->state == RMC_GETDATA){
                    if ((ddata->scanData&0x0ff) == ((~ddata->scanData >> 8)&0x0ff)){
//                        printk(KERN_ERR "d=%x\n",(ddata->scanData&0xFFFF));
                        if (remotectl_keycode_lookup(ddata)){
                             if (ddata->keycode==KEY_POWER){
                                input_event(ddata->input, EV_KEY, KEY_WAKEUP, 1);
                                input_sync(ddata->input);
                                input_event(ddata->input, EV_KEY, KEY_WAKEUP, 0);
                                input_sync(ddata->input);
                            }
                            ddata->state = RMC_PRELOAD;
                        }else{
                            ddata->state = RMC_PRELOAD;
                        }
                    }else{
                        ddata->state = RMC_PRELOAD;
                    }
                }else{
                    ddata->state = RMC_PRELOAD;
                }
            }
        }
    }
    memset(ddata->remotectl_suspend_data.scanTime,0,50*sizeof(long));
    ddata->remotectl_suspend_data.cnt= 0; 
    ddata->state = RMC_PRELOAD;
    
}

#endif


static void remotectl_timer(unsigned long _data)
{
    struct rkxx_remotectl_drvdata *ddata =  (struct rkxx_remotectl_drvdata*)_data;
    
    //printk("to\n");
    
    if(ddata->press != ddata->pre_press) {
        ddata->pre_press = ddata->press = 0;

        if (/*get_suspend_state()*/bonovo_get_suspend_status()==0){
            //input_event(ddata->input, EV_KEY, ddata->keycode, 1);
            //input_sync(ddata->input);
            input_event(ddata->input, EV_KEY, ddata->keycode, 0);
		    input_sync(ddata->input);
        }else if (/*(get_suspend_state())&&*/(ddata->keycode==KEY_POWER)){
            //input_event(ddata->input, EV_KEY, KEY_WAKEUP, 1);
            //input_sync(ddata->input);
            input_event(ddata->input, EV_KEY, KEY_WAKEUP, 0);
            input_sync(ddata->input);
        }
    }
#ifdef CONFIG_PM
    remotectl_wakeup(_data);
#endif
    ddata->state = RMC_PRELOAD;
}



static irqreturn_t remotectl_isr(int irq, void *dev_id)
{
    struct rkxx_remotectl_drvdata *ddata =  (struct rkxx_remotectl_drvdata*)dev_id;
    struct timeval  ts;

    //printk_zbiao("++++++ remotectl_isr\n");
    ddata->pre_time = ddata->cur_time;
    do_gettimeofday(&ts);
    ddata->cur_time = ts.tv_usec;

    if (ddata->cur_time && ddata->pre_time)
        ddata->period =  ddata->cur_time - ddata->pre_time;

    tasklet_hi_schedule(&ddata->remote_tasklet); 
    if ((ddata->state==RMC_PRELOAD)||(ddata->state==RMC_SEQUENCE))
    	mod_timer(&ddata->timer,jiffies + msecs_to_jiffies(130));
#ifdef CONFIG_PM
    //wake_lock_timeout(&ddata->remotectl_wake_lock, HZ);
   if ((/*get_suspend_state()*/bonovo_get_suspend_status())&&(ddata->remotectl_suspend_data.cnt<50))
       ddata->remotectl_suspend_data.scanTime[ddata->remotectl_suspend_data.cnt++] = ddata->period;
#endif

    return IRQ_HANDLED;
}


static int __devinit remotectl_probe(struct platform_device *pdev)
{
    struct RKxx_remotectl_platform_data *pdata = pdev->dev.platform_data;
    struct rkxx_remotectl_drvdata *ddata;
//    struct input_dev *input;
//    int i, j;
    int irq;
    int error = 0;
    unsigned long flags = 0;
    struct bonovo_advance_keys * rc_index;

    printk_zbiao("++++++++remotectl_probe\n");

    if(!pdata) 
        return -EINVAL;

    ddata = kzalloc(sizeof(struct rkxx_remotectl_drvdata),GFP_KERNEL);
    memset(ddata,0,sizeof(struct rkxx_remotectl_drvdata));

    ddata->state = RMC_PRELOAD;
    input = input_allocate_device();
    
    if (!ddata || !input) {
        error = -ENOMEM;
        goto fail0;
    }

    platform_set_drvdata(pdev, ddata);

    input->name = pdev->name;
    input->phys = "gpio-keys/input0";
    input->dev.parent = &pdev->dev;

    input->id.bustype = BUS_HOST;
    input->id.vendor = 0x0001;
    input->id.product = 0x0001;
    input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);
    
	ddata->nbuttons = pdata->nbuttons;
	ddata->input = input;
  wake_lock_init(&ddata->remotectl_wake_lock, WAKE_LOCK_SUSPEND, "rk29_remote");
  if (pdata->set_iomux){
  	pdata->set_iomux();
  }
  error = gpio_request(pdata->gpio, "remotectl");
	if (error < 0) {
		printk("gpio-keys: failed to request GPIO %d,"
		" error %d\n", pdata->gpio, error);
		//goto fail1;
	}
	error = gpio_direction_input(pdata->gpio);
	if (error < 0) {
		pr_err("gpio-keys: failed to configure input"
			" direction for GPIO %d, error %d\n",
		pdata->gpio, error);
		gpio_free(pdata->gpio);
		//goto fail1;
	}
    gpio_pull_updown(pdata->gpio, GPIOPullUp);
#ifdef ANDROID_IR
    irq = gpio_to_irq(pdata->gpio);
	if (irq < 0) {
		error = irq;
		pr_err("gpio-keys: Unable to get irq number for GPIO %d, error %d\n",
		pdata->gpio, error);
		gpio_free(pdata->gpio);
		goto fail1;
	}

	error = request_irq(irq, remotectl_isr,	IRQF_TRIGGER_FALLING , "remotectl", ddata);
	
	if (error) {
		pr_err("gpio-remotectl: Unable to claim irq %d; error %d\n", irq, error);
		gpio_free(pdata->gpio);
		goto fail1;
	}

    setup_timer(&ddata->timer,remotectl_timer, (unsigned long)ddata);
    tasklet_init(&ddata->remote_tasklet, remotectl_do_something, (unsigned long)ddata);
#endif
/*
    for (j=0;j<sizeof(remotectl_button)/sizeof(struct bonovo_advance_keys);j++){ 
    	printk_zbiao("remotectl probe j=0x%x\n",j);
		for (i = 0; i < remotectl_button[j].nbuttons; i++) {
			unsigned int type = EV_KEY;
	        
			input_set_capability(input, type, remotectl_button[j].key_table[i].keyCode);
		}
  }
*/
    read_lock_irqsave(&bonovo_irctls.ir_rwlock, flags);
    list_for_each_entry(rc_index, &bonovo_irctls.irctls_list, node){
        rc_register_buttons(rc_index);
    }
    read_unlock_irqrestore(&bonovo_irctls.ir_rwlock, flags);
	
	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, error: %d\n", error);
		goto fail2;
	}
    
    input_set_capability(input, EV_KEY, KEY_WAKEUP);
	
	device_init_wakeup(&pdev->dev, 1);
	pbonovo_data = ddata;

	return 0;

fail2:
    pr_err("gpio-remotectl input_allocate_device fail\n");
	input_free_device(input);
	kfree(ddata);
fail1:
    pr_err("gpio-remotectl gpio irq request fail\n");
    free_irq(gpio_to_irq(pdata->gpio), ddata);
    del_timer_sync(&ddata->timer);
    tasklet_kill(&ddata->remote_tasklet); 
    gpio_free(pdata->gpio);
fail0: 
    pr_err("gpio-remotectl input_register_device fail\n");
    platform_set_drvdata(pdev, NULL);

	return error;
}

static int __devexit remotectl_remove(struct platform_device *pdev)
{
	struct RKxx_remotectl_platform_data *pdata = pdev->dev.platform_data;
	struct rkxx_remotectl_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
    int irq;

	device_init_wakeup(&pdev->dev, 0);
    irq = gpio_to_irq(pdata->gpio);
    free_irq(irq, ddata);
    tasklet_kill(&ddata->remote_tasklet); 
    gpio_free(pdata->gpio);

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int remotectl_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct RKxx_remotectl_platform_data *pdata = pdev->dev.platform_data;
    struct rkxx_remotectl_drvdata *ddata = platform_get_drvdata(pdev);
    
    //ddata->remotectl_suspend_data.suspend_flag = 1;
    ddata->remotectl_suspend_data.cnt = 0;

	/*@
	 *  removed by bonovo zbiao
	if (device_may_wakeup(&pdev->dev)) {
		if (pdata->wakeup) {
			int irq = gpio_to_irq(pdata->gpio);
			enable_irq_wake(irq);
		}
	}*/
    
	return 0;
}

static int remotectl_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct RKxx_remotectl_platform_data *pdata = pdev->dev.platform_data;

    /*@
     * removed by bonovo zbiao
    if (device_may_wakeup(&pdev->dev)) {
        if (pdata->wakeup) {
            int irq = gpio_to_irq(pdata->gpio);
            disable_irq_wake(irq);
        }
    }*/

	return 0;
}

static const struct dev_pm_ops remotectl_pm_ops = {
	.suspend	= remotectl_suspend,
	.resume		= remotectl_resume,
};
#endif



static struct platform_driver remotectl_device_driver = {
	.probe		= remotectl_probe,
	.remove		= __devexit_p(remotectl_remove),
	.driver		= {
		.name	= "rkxx-remotectl",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
	    .pm	= &remotectl_pm_ops,
#endif
	},

};

static int  remotectl_init(void)
{
    printk_zbiao(KERN_INFO "++++++++remotectl_init\n");
    return platform_driver_register(&remotectl_device_driver);
}


static void  remotectl_exit(void)
{
	platform_driver_unregister(&remotectl_device_driver);
    printk_zbiao(KERN_INFO "++++++++remotectl_exit\n");
}

//module_init(remotectl_init);
late_initcall(remotectl_init);
module_exit(remotectl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys1");


