// ****************************************** //
// Temporary Driver for Infrared Remote Controller on Android
// 
// Title  : GPIO (for infrared remote controller)
// Target : android_box
//
// Author : zbiao
// Rewrite: 
// ****************************************** //

#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 

#include <linux/fs.h>            
#include <linux/mm.h>            
#include <linux/errno.h>         
#include <linux/types.h>         
#include <linux/fcntl.h>         
#include <linux/cdev.h>         
#include <linux/device.h>         
#include <linux/major.h> 
#include <linux/list.h>
#include <linux/input.h>
#include <linux/rwlock_types.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>  
#include <asm/io.h>
#include <mach/remotectl.h>

//#define DEBUG_ZBIAO
#ifdef DEBUG_ZBIAO
#define printk_zbiao(format, arg...)	\
	printk(format, ##arg);
#else
#define printk_zbiao(format, arg...) 
#endif

#define DEV_NAME      "bonovo_irctl"      // device name
#define CONFIG_DIR_PATH   "/data/"      // the path where the configs of ir were stored in.

#define DEBUG_ON                          // debug flag

#define DEV_MAJOR     235
#define DEV_MINOR	    0

#define IOC_MAGIC     DEV_MAJOR
#define IOCTL_BONOVO_CUSTOM_MODE	    _IO(IOC_MAGIC,   1)                                  // enter custom mode
#define IOCTL_BONOVO_NORMAL_MODE      _IO(IOC_MAGIC,   2)                                    // enter normal mode
#define IOCTL_BONOVO_READ_MODE        _IOR(IOC_MAGIC,  3, int *)                             // read the mode of infrared remote controllor
#define IOCTL_BONOVO_READ_KEY         _IOR(IOC_MAGIC, 10, struct bonovo_rc_key *)            // read one key
#define IOCTL_BONOVO_WRITE_KEY        _IOW(IOC_MAGIC, 11, struct bonovo_rc_wkey *)           // write one key
#define IOCTL_BONOVO_READ_CONFIG      _IOR(IOC_MAGIC, 12, struct rkxx_remote_key_table *)    // read the config corresponding addrcode
#define IOCTL_BONOVO_WRITE_CONFIG     _IOW(IOC_MAGIC, 13, struct rkxx_remote_key_table *)    // write the config corresponding addrcode
#define IOCTL_BONOVO_READ_ADDRCODE    _IOR(IOC_MAGIC, 14, int *)                             // read address code
#define IOCTL_BONOVO_WRITE_ADDRCODE   _IOW(IOC_MAGIC, 15, int *)                             // write address code
#define IOCTL_BONOVO_CLEAR_BUFF       _IO(IOC_MAGIC, 20)                                     // clear buff
#define IOCTL_BONOVO_CLEAR_KEYS       _IO(IOC_MAGIC, 21)                                     // clear all keys
#define IOCTL_BONOVO_READ_ACCURACY    _IO(IOC_MAGIC, 22)
#define IOCTL_BONOVO_WRITE_ACCURACY   _IO(IOC_MAGIC, 23)
#define KEY_VALUE_PAIRS_NUM           51//38                                                 // 键值对
#define IOC_MAXNR     23
#define MAX_KEYS_CNT  KEY_VALUE_PAIRS_NUM
#define DEF_OFFSET    40
#define MIN_OFFSET    20

static int      major = DEV_MAJOR; 
static dev_t    dev_no;
static struct   cdev dev_cdev;
static struct   class *dev_class;

extern int serial_send_ack(char * data, int len);

struct bonovo_key_addr {
    int ekey_type;
    int addrcode;
};

struct bonovo_advance_key_wbuf {
    struct bonovo_key_addr key_addr;
    struct rkxx_remote_key_table key_table[MAX_KEYS_CNT];
};

//atomic_t flag = ATOMIC_INIT(1);            // the flag in sign of whether the irctl_default is inserted the link list.

struct bonovo_rc_buff bonovo_keys_buff;      // keys' buff.The keys in the buff will upload to the framework and app.

struct rkxx_remotectls bonovo_irctls = 
{
	.mode = 0,
	.num  = 0,
};

static struct rkxx_remote_key_table key_table_default[] = {
    //{0x7a, KEY_BONOVO_DPAD_CENTER},
    {0x7a, KEY_SELECT},
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

static struct rkxx_remote_key_table mKeyCodeTableArry[] ={
	{0x00, 172}, //  0 - KEY_HOMEPAGE
	{0x00, 158}, //  1 - KEY_BACK
	{0x00, 11},  //  2 - KEY_0
	{0x00, 2},   //  3 - KEY_1
	{0x00, 3},   //  4 - KEY_2
	{0x00, 4},   //  5 - KEY_3
	{0x00, 5},   //  6 - KEY_4
	{0x00, 6},   //  7 - KEY_5
	{0x00, 7},   //  8 - KEY_6
	{0x00, 8},   //  9 - KEY_7
	{0x00, 9},   // 10 - KEY_8
	{0x00, 10},  // 11 - KEY_9
	{0x00, 522}, // 12 - KEY_STAR
	{0x00, 523}, // 13 - KEY_POUND
	{0x00, 103}, // 14 - KEY_UP
	{0x00, 108}, // 15 - KEY_DOWN
	{0x00, 105}, // 16 - KEY_LEFT
	{0x00, 106}, // 17 - KEY_RIGHT
	{0x00, 353}, // 18 - KEY_SELECT [KEY_BONOVO_DPAD_CENTER]
	{0x00, 115}, // 19 - KEY_VOLUMEUP
	{0x00, 114}, // 20 - KEY_VOLUMEDOWN
	{0x00, 116}, // 21 - KEY_POWER
	{0x00, 150}, // 22 - KEY_WWW
	{0x00, 155}, // 23 - KEY_MAIL
	{0x00, 127}, // 24 - KEY_COMPOSE (menu) --- also changed to 139(KEY_MENU)
	{0x00, 217}, // 25 - KEY_SEARCH
	{0x00, 164}, // 26 - KEY_PLAYPAUSE
	{0x00, 128}, // 27 - KEY_STOP
	{0x00, 163}, // 28 - KEY_NEXTSONG
	{0x00, 165}, // 29 - KEY_PREVIOUSSONG
	{0x00, 168}, // 30 - KEY_REWIND
	{0x00, 208}, // 31 - KEY_FASTFORWARD
	{0x00, 104}, // 32 - KEY_PAGEUP
	{0x00, 109}, // 33 - KEY_PAGEDOWN
	{0x00, 113}, // 34 - KEY_MUTE
	{0x00, 397}, // 35 - KEY_CALENDAR
	{0x00, 171}, // 36 - KEY_CONFIG (music)
	{0x00, 140}, // 37 - KEY_CALC (calculator)
	{0x00, KEY_BONOVO_RADIO}, // 38 - KEY_RADIO
	{0x00, KEY_BONOVO_NAVI},              // 39  0x229
	{0x00, KEY_BONOVO_DVD},               // 40  0x233
	{0x00, KEY_BONOVO_3G},                // 41  0x234
	{0x00, KEY_BONOVO_CAMERA},            // 42  0x238
	{0x00, KEY_BONOVO_BT},                // 43  0x240
	{0x00, KEY_BONOVO_BT_ANSWER},         // 44  0x241
	{0x00, KEY_BONOVO_BT_HANG_UP},        // 45  0x242
	{0x00, KEY_BONOVO_BT_ANSWER_HANG},    // 46  0x243
	{0x00, KEY_BONOVO_BT_SWITCH_AUDIO},   // 47  0x244
	{0x00, KEY_BONOVO_RADIO_TURNUP},      // 48  0x251
	{0x00, KEY_BONOVO_RADIO_TURNDOWN},    // 49  0x252
	{0x00, KEY_BONOVO_SWITCH_FMAM},       // 50  0x253
};

struct rkxx_remotectl_button irctl_default = 
{
	.usercode = 0x00FF,
	.nbuttons = 30,
	.key_table = &key_table_default[0],
};

struct rkxx_remotectl_button irctl_table = 
{
	.usercode = 0x0000, //0x00FF,
	.nbuttons = MAX_KEYS_CNT, //40, //38, //30,
	.key_table = &mKeyCodeTableArry[0],//&key_table_default[0], 
};

static unsigned int calculateSum(unsigned char* cmdBuf, int size)
{
    unsigned int temp = 0;
    int i;
    for(i=0; i < size; i++){
        temp += cmdBuf[i];
    }
    return temp;
}

void notify_mcu_power_key(unsigned int value, unsigned int type, unsigned int offset){
    unsigned int sum = 0, i;
    unsigned char data[12]={0xFA, 0xFA, 0x0C, 0x00, 0x83, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    data[6] = value & 0x00FF;
    data[7] = (value >> 8) & 0x00FF;
    data[8] = (value >> 16) & 0x00FF;
    data[9] = offset & 0x00FF;

    sum = calculateSum(data, 10);
    data[10] = sum & 0x00FF;
    data[11] = (sum >> 8) & 0x00FF;

    serial_send_ack(data, 12);
}

int bonovo_is_custom_mode(void)
{
	return (bonovo_irctls.mode & 0x01);
}

int bonovo_is_half_mode(void)
{
	return (bonovo_irctls.mode & 0x02);
}

void bonovo_set_half_flag(void)
{
	unsigned long irqflags;

	write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
	bonovo_irctls.mode |= 0x02;
	write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
}

void bonovo_clear_half_flag(void)
{
	unsigned long irqflags;

	write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
	bonovo_irctls.mode &= ~0x02;
	write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
}

int bonovo_is_buff_empty(struct bonovo_rc_buff * buff)
{
	return (buff->head == buff->tail);
}

int bonovo_is_buff_full(struct bonovo_rc_buff * buff)
{
	//printk_zbiao("=============bonovo_is_buff_full\n");
	return (buff->head == (buff->tail + 1)%BONOVO_BUFF_LEN);
}

int bonovo_buff_add(struct bonovo_rc_buff *buff, int addrCode, int scanCode)
{
	int res = 0;
	unsigned long flags = 0;

	//printk_zbiao("=============bonovo_buff_add. addrCode:0x%04x  scanCode:0x%04x\n", addrCode, scanCode);
	
	spin_lock_irqsave(&buff->lock, flags);
	if(!bonovo_is_buff_full(buff))
	{
		buff->key_table[buff->tail].addrCode = addrCode;	
		buff->key_table[buff->tail].scanCode = scanCode;
		printk_zbiao("=============buff->key_table. addrCode:0x%04x  scanCode:0x%04x\n", 
			buff->key_table[buff->tail].addrCode, buff->key_table[buff->tail].scanCode);
		buff->tail = (buff->tail + 1) % BONOVO_BUFF_LEN;
	}
	else
	{
		printk_zbiao("============= buff is full.\n");
		res = -1;
	}
	spin_unlock_irqrestore(&buff->lock, flags);
	return res;
}

void bonovo_buff_key(struct bonovo_rc_buff *buff, int addrCode, int scanCode)
{
	//printk_zbiao("=============bonovo_buff_add. addrCode:0x%04x  scanCode:0x%04x\n", addrCode, scanCode);
	buff->key_table[0].addrCode = addrCode;
	buff->key_table[0].scanCode = scanCode;
	buff->tail = 1;
	buff->head = 0;
	return;
}

int bonovo_buff_remove_head(struct bonovo_rc_buff *buff, int *addrCode, int *scanCode)
{
	int res = 0;
	unsigned long flags = 0;
	spin_lock_irqsave(&buff->lock, flags);
	if(!bonovo_is_buff_empty(buff))
	{
		if(addrCode != NULL)
			*addrCode = buff->key_table[buff->head].addrCode;
		if(scanCode != NULL)
			*scanCode = buff->key_table[buff->head].scanCode;
		buff->key_table[buff->tail].addrCode = 0;	
		buff->key_table[buff->tail].scanCode = 0;
		buff->head = (buff->head + 1) % BONOVO_BUFF_LEN;
	}
	else
	{
		res = -1;
	}
	spin_unlock_irqrestore(&buff->lock, flags);
	return res;
}

int bonovo_buff_clear(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&bonovo_keys_buff.lock, irqflags);
	memset(bonovo_keys_buff.key_table, 0, BONOVO_BUFF_LEN * sizeof(struct bonovo_rc_key));
	bonovo_keys_buff.head = bonovo_keys_buff.tail = 0;
	spin_unlock_irqrestore(&bonovo_keys_buff.lock, irqflags);

	return 0;
}

static int bonovo_init(void)
{
    unsigned long irqflags;
    int result = 0, i;
    struct bonovo_advance_keys *pos;
    struct bonovo_advance_keys *keys_cfg = NULL;
    printk_zbiao("====================== in bonovo_init\n");

    INIT_LIST_HEAD(&bonovo_irctls.irctls_list);
    rwlock_init(&bonovo_irctls.ir_rwlock);

    for(i = 0; i<KEY_TYPE_CNT; i++){
        keys_cfg = kmalloc(sizeof(struct bonovo_advance_keys), GFP_KERNEL);
        if(!keys_cfg){
            printk("can't alloc memory for keys config.\n");
            goto fail_malloc;
        }
        memset(keys_cfg, 0, sizeof(struct bonovo_advance_keys));
        keys_cfg->key_table = kmalloc(sizeof(struct rkxx_remote_key_table)*MAX_KEYS_CNT, GFP_KERNEL);
        if(!keys_cfg){
            printk("can't alloc memory for keys table.\n");
            kfree(keys_cfg);
            goto fail_malloc;
        }
        memset(keys_cfg->key_table, 0, sizeof(struct rkxx_remote_key_table)*MAX_KEYS_CNT);
        keys_cfg->usercode = 0;
        keys_cfg->nbuttons = MAX_KEYS_CNT;
        keys_cfg->ekey_type = (eKEY_TYPE)i;
        if((keys_cfg->ekey_type == KEY_TYPE_BACK_ADC)
            || (keys_cfg->ekey_type == KEY_TYPE_PANEL)){
            keys_cfg->accuracy = DEF_OFFSET;
        }else{
            keys_cfg->accuracy = 0;
        }
        printk_zbiao("++++++++ bonovo_init keys_cfg->ekey_type:%d  MAX_KEYS_CNT:%d\n", keys_cfg->ekey_type, MAX_KEYS_CNT);
        INIT_LIST_HEAD(&keys_cfg->node);
        memcpy(keys_cfg->key_table, mKeyCodeTableArry, sizeof(struct rkxx_remote_key_table)*MAX_KEYS_CNT);

        write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags); 
        list_add_tail(&keys_cfg->node, &bonovo_irctls.irctls_list);
        bonovo_irctls.num++;
        write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
    }

//	if(atomic_dec_and_test(&flag))
/*	{
    write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags); 
    list_add(&irctl_table.node, &bonovo_irctls.irctls_list);
    bonovo_irctls.num++;
    write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
    }*/
//	else
//	{
//		atomic_add(&flag);
//	}
    memset(&bonovo_keys_buff, 0, sizeof(struct bonovo_rc_buff));
    bonovo_keys_buff.key_table = kmalloc(BONOVO_BUFF_LEN * sizeof(struct bonovo_rc_key), GFP_KERNEL);
    if(!bonovo_keys_buff.key_table)
    {
        result = -ENOMEM;
        goto fail_malloc;
    }
    memset(bonovo_keys_buff.key_table, 0, BONOVO_BUFF_LEN * sizeof(struct bonovo_rc_key));
    spin_lock_init(&bonovo_keys_buff.lock);

    return result;
    
fail_malloc:
    write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
	list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
        kfree(pos->key_table);
        kfree(pos);
    }
	write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
    
    printk("bonovo_init: can't malloc memory for key buff.\n");
    return result;
}

static void bonovo_exit(void)
{
    struct bonovo_advance_keys *pos;
    unsigned long irqflags;

    write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
    list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
        kfree(pos->key_table);
        kfree(pos);
    }
    write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
    if(!bonovo_keys_buff.key_table)
    {
        kfree(bonovo_keys_buff.key_table);
    }
    return;
}

static int bonovo_irctl_open (struct inode *inode, struct file *filp)  
{
	printk_zbiao("=============bonovo_irctl_open\n");
	return 0;  
}  

static int bonovo_irctl_release (struct inode *inode, struct file *filp)  
{  
	printk_zbiao("=============bonovo_irctl_release\n");
	return 0;  
}  

static long bonovo_irctl_ioctl(struct file *filp,
                           unsigned int cmd, unsigned long arg)  
{
	unsigned long irqflags;
	int err = 0;
	struct bonovo_rc_wkey tempBuff;
    struct bonovo_advance_keys *pos;
    struct bonovo_advance_key_wbuf temp_keys;
    int i, key_type = 0;//KEY_TYPE_CNT;
	int tempAddrCode = 0;
    int is_found_in_list = 0;
    struct bonovo_key_addr tempAddr;

	printk_zbiao("=============bonovo_irctl_ioctl\n");

	if(_IOC_TYPE(cmd) != IOC_MAGIC)
	{
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > IOC_MAXNR){
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if(err)
		return -EFAULT;
	
	
	switch( cmd )  
	{
	case IOCTL_BONOVO_CUSTOM_MODE:
//		bonovo_buff_clear();
		irqflags = 0;
		write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
		bonovo_irctls.mode |= 0x01;
		write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
		
		printk_zbiao("the mode is %d.\n", bonovo_irctls.mode);
		break;
	case IOCTL_BONOVO_NORMAL_MODE:
		irqflags = 0;
		write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
		bonovo_irctls.mode &= ~0x01;
		write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);

		printk_zbiao("the mode is %d.\n", bonovo_irctls.mode);
		break;
	case IOCTL_BONOVO_READ_KEY:
		#if 0
		if(bonovo_is_buff_empty(&bonovo_keys_buff)){
			printk_zbiao("the buff is empty.\n");
			err = -EAGAIN;
		}
		else
		{
			printk_zbiao("=============buff->key_table. addrCode:0x%04x  scanCode:0x%04x\n", 
				bonovo_keys_buff.key_table[bonovo_keys_buff.head].addrCode, bonovo_keys_buff.key_table[bonovo_keys_buff.head].scanCode);
			
			if(copy_to_user((void *)arg,
				(void *)&(bonovo_keys_buff.key_table[bonovo_keys_buff.head]), sizeof(struct bonovo_rc_key)))
				err = -EFAULT;
			else
				bonovo_buff_remove_head(&bonovo_keys_buff, NULL, NULL);
		}
		#endif
		
		if(bonovo_keys_buff.tail != 1){
			printk_zbiao("the buff is empty. bonovo_keys_buff.tail:%d  bonovo_keys_buff.head:%d\n", 
               bonovo_keys_buff.tail, bonovo_keys_buff.head);
			err = -EAGAIN;
		}else{
			printk_zbiao("=============buff->key_table. addrCode:0x%04x  scanCode:0x%04x\n", 
				bonovo_keys_buff.key_table[bonovo_keys_buff.head].addrCode, bonovo_keys_buff.key_table[bonovo_keys_buff.head].scanCode);
			
			if(copy_to_user((void *)arg,
				(void *)&(bonovo_keys_buff.key_table[bonovo_keys_buff.head]), sizeof(struct bonovo_rc_key)))
				err = -EFAULT;
			else
				bonovo_keys_buff.tail = bonovo_keys_buff.head = 0;
		}
		
		break;
	case IOCTL_BONOVO_WRITE_KEY:
		memset(&tempBuff, 0, sizeof(tempBuff));
		if(copy_from_user((void *)&tempBuff, (void *)arg, sizeof(tempBuff)))
		{
			err = -EFAULT;
		}
		else
		{
			printk_zbiao("====== addrCode:0x%04x   scanCode:0x%04x   keyCode:0x%04x\n",
				tempBuff.addrCode, tempBuff.key_val.scanCode, tempBuff.key_val.keyCode);
            
            write_lock_irqsave(&bonovo_irctls.ir_rwlock, irqflags);
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if((pos->ekey_type == tempBuff.key_type)&&(pos->usercode == tempBuff.addrCode)){
                    for(i = 0; i < pos->nbuttons; i++){
                        if(pos->key_table[i].keyCode == tempBuff.key_val.keyCode){
                            pos->key_table[i].scanCode = tempBuff.key_val.scanCode;
                            err = 0;
                            is_found_in_list = 1;

                            if(pos->key_table[i].keyCode == KEY_POWER){
                                notify_mcu_power_key((unsigned int)pos->key_table[i].scanCode,
                                    pos->ekey_type, pos->accuracy);
                            }
                            printk_zbiao("=00===== addrCode:0x%04x   scanCode:0x%04x   keyCode:0x%04x\n",
						        tempBuff.addrCode, tempBuff.key_val.scanCode, tempBuff.key_val.keyCode);
                            break;
                        }
                    }
                }
            }
            write_unlock_irqrestore(&bonovo_irctls.ir_rwlock, irqflags);
            
            if(is_found_in_list == 0){
                printk_zbiao("=01=====can't find keyCode. addrCode:0x%04x scanCode:0x%04x keyCode:0x%04x\n",
				    tempBuff.addrCode, tempBuff.key_val.scanCode, tempBuff.key_val.keyCode);
                err = -ESPIPE;
            }else{
                err = 0;
            }
            
#if 0
			if(irctl_table.usercode == tempBuff.addrCode)
			{
				for(i = 0; i < irctl_table.nbuttons; i++)
				{
					if(irctl_table.key_table[i].keyCode == tempBuff.key_val.keyCode)
					{
						irctl_table.key_table[i].scanCode = tempBuff.key_val.scanCode;
						err = 0;
						printk_zbiao("=00===== addrCode:0x%04x   scanCode:0x%04x   keyCode:0x%04x\n",
							tempBuff.addrCode, tempBuff.key_val.scanCode, tempBuff.key_val.keyCode);
						break;
					}
				}
				if(i >= irctl_table.nbuttons)
				{
					err = -EINVAL;
				}
			}
			else
			{
				err = -EINVAL;
			}
#endif
		}
		break;
	case IOCTL_BONOVO_READ_ADDRCODE:
        if(copy_from_user((void*)&key_type, (void*)arg, sizeof(int))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(key_type == (int)pos->ekey_type){
                    if(copy_to_user((void*)arg, (void*)&(pos->usercode), sizeof(int))){
                        err = -EFAULT;
                    }else{
                        err = 0;
                    }
                    is_found_in_list = 1;
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
#if 0
        if(copy_to_user((void*)arg, (void*)&(irctl_table.usercode), sizeof(int)))
		{
			err = -EFAULT;
		}
#endif
		break;
	case IOCTL_BONOVO_WRITE_ADDRCODE:
        memset((void*)&tempAddr, 0, sizeof(struct bonovo_key_addr));
        if(copy_from_user((void*)&tempAddr, (void*)arg, sizeof(struct bonovo_key_addr))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(pos->ekey_type == tempAddr.ekey_type){
                    pos->usercode = tempAddr.addrcode;
                    err = 0;
                    is_found_in_list = 1;
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
#if 0
		tempAddrCode = 0;
		if(copy_from_user((void *)&tempAddrCode, (void *)arg, sizeof(tempAddrCode)))
		{
			err = -EFAULT;
		}
		else
		{
			printk_zbiao("========== address code: 0x%04x\n", tempAddrCode);
			//if((tempAddrCode&0x0ff) == ((~tempAddrCode >> 8)&0x0ff))
				irctl_table.usercode = tempAddrCode;
			//else
			//	err = -EINVAL;
		}
#endif
		break;
    case IOCTL_BONOVO_READ_ACCURACY:
        if(copy_from_user((void*)&key_type, (void*)arg, sizeof(int))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(key_type == (int)pos->ekey_type){
                    if(copy_to_user((void*)arg, (void*)&(pos->accuracy), sizeof(int))){
                        err = -EFAULT;
                    }else{
                        err = 0;
                    }
                    is_found_in_list = 1;
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
		break;
	case IOCTL_BONOVO_WRITE_ACCURACY:
        memset((void*)&tempAddr, 0, sizeof(struct bonovo_key_addr));
        if(copy_from_user((void*)&tempAddr, (void*)arg, sizeof(struct bonovo_key_addr))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(pos->ekey_type == tempAddr.ekey_type){
                    if((tempAddr.ekey_type == KEY_TYPE_BACK_ADC
                        || tempAddr.ekey_type == KEY_TYPE_PANEL)
                        && pos->accuracy < MIN_OFFSET){
                        pos->accuracy = MIN_OFFSET;
                    }else{
                        pos->accuracy= tempAddr.addrcode;
                    }

                    for(i=0; i< pos->nbuttons; i++){
                        if((pos->key_table[i].keyCode == KEY_POWER) 
                            && (pos->key_table[i].scanCode != -1)){
                            notify_mcu_power_key((unsigned int)pos->key_table[i].scanCode, pos->ekey_type, pos->accuracy);
                            break;
                        }
                    }
                    err = 0;
                    is_found_in_list = 1;
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
		break;
	case IOCTL_BONOVO_READ_CONFIG:
        if(copy_from_user((void*)&key_type, (void*)arg, sizeof(int))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(pos->ekey_type == key_type){
                    if(copy_to_user((void *)arg, (void *)pos->key_table,
                        sizeof(struct rkxx_remote_key_table)*(pos->nbuttons))){
            			err = -EFAULT;
                    }else{
                        err = 0;
                    }
                    is_found_in_list = 1;
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
#if 0
		if(copy_to_user((void *)arg, (void *)mKeyCodeTableArry, sizeof(mKeyCodeTableArry)))
			err = -EFAULT;
#endif
		break;
	case IOCTL_BONOVO_WRITE_CONFIG:
        if(copy_from_user((void*)&temp_keys, (void*)arg, sizeof(struct bonovo_advance_key_wbuf))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(pos->ekey_type == temp_keys.key_addr.ekey_type){
                    memcpy((void *)pos->key_table, (void *)&temp_keys.key_table,
                        sizeof(struct rkxx_remote_key_table)*(pos->nbuttons));
                    pos->usercode = temp_keys.key_addr.addrcode;
                    is_found_in_list = 1;

                    for(i=0; i< pos->nbuttons; i++){
                        if((pos->key_table[i].keyCode == KEY_POWER) 
                            && (pos->key_table[i].scanCode != -1)){
                            notify_mcu_power_key((unsigned int)pos->key_table[i].scanCode, pos->ekey_type, pos->accuracy);
                            break;
                        }
                    }
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
#if 0
		if(copy_from_user((void *)mKeyCodeTableArry, (void *)arg, sizeof(mKeyCodeTableArry)))
			err = -EFAULT;
#endif
		break;
	case IOCTL_BONOVO_CLEAR_BUFF:
		bonovo_buff_clear();
		break;
	case IOCTL_BONOVO_CLEAR_KEYS:
        if(copy_from_user((void*)&key_type, (void*)arg, sizeof(int))){
            err = -EFAULT;
        }else{
            list_for_each_entry(pos, &bonovo_irctls.irctls_list, node){
                if(pos->ekey_type == key_type){
                    for(i=0; i < pos->nbuttons; i++){
                        pos->key_table[i].scanCode = 0;
                    }
                    err = 0;
                    is_found_in_list = 1;
                    break;
                }
            }
            if(0 == is_found_in_list){
                err = -ESPIPE;
            }
        }
#if 0
		for(i=0; i < irctl_table.nbuttons; i++)
		{
			irctl_table.key_table[i].scanCode = 0;
		}
#endif
		break;
	case IOCTL_BONOVO_READ_MODE:
		if(copy_to_user((void *)arg, (void *)&(bonovo_irctls.mode), sizeof(bonovo_irctls.mode)))
			err = -EFAULT;
		else
			bonovo_buff_remove_head(&bonovo_keys_buff, NULL, NULL);
		break;
	default :
		err = -EINVAL;
		break;
	}
	return err;  
}  
  
static struct file_operations irctl_fops =  
{  
	.owner    = THIS_MODULE,  
	.unlocked_ioctl    = bonovo_irctl_ioctl,  
	.open     = bonovo_irctl_open,       
	.release  = bonovo_irctl_release,    
};

//  ************************************************************ //
//  Device Init : register char driver
//
//  ************************************************************ //
static int __init bonovo_irctl_init(void)  
{  
	int result;  
	
	printk_zbiao("=============bonovo_irctl driver loaded\n");

	if (0 == major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev_no, 0, 1, DEV_NAME);
		major = MAJOR(dev_no);
	}
	else
	{
		dev_no = MKDEV(major, 0);
		result = register_chrdev_region(dev_no, 1, DEV_NAME);
	}
	
	if (result)
	{
		printk("register_chrdev_region error!\n");
		goto fail_reg;
	}
	
	memset(&dev_cdev, 0, sizeof(dev_cdev));

	/* initialize our char dev data */
	cdev_init(&dev_cdev, &irctl_fops);

	/* register char dev with the kernel */
	result = cdev_add(&dev_cdev, dev_no, 1);

	printk_zbiao("=============bonovo_irctl register char dev complete. %d\n", result);
	
	if (0 != result)
	{
		printk("Error registrating bonovo_irctl device object with the kernel\n");
		goto fail_add;
	}
	dev_class = class_create(THIS_MODULE, DEV_NAME);
	device_create(dev_class, NULL, MKDEV(major, MINOR(dev_no)), NULL,
                  DEV_NAME);

	if(!bonovo_init())                                                 // 初始化。主要工作：将默认的遥控器键值表加入到链表中
	{
		goto fail_add;
	}
	
	return 0;
	
fail_add:
	unregister_chrdev_region(dev_no, 1);
fail_reg:
	return result;	  
}  

//  ************************************************************ //
//  Device Exit :
//
//  ************************************************************ //
static void __exit bonovo_irctl_exit(void)  
{
	bonovo_exit();
	device_destroy(dev_class, MKDEV(major, 0));
	class_destroy(dev_class);

	cdev_del(&dev_cdev);
	unregister_chrdev_region(dev_no, 1);
	
	printk_zbiao("=============handle driver unloaded.\n");
}  
  
module_init(bonovo_irctl_init);  
module_exit(bonovo_irctl_exit);  
  
MODULE_LICENSE("Dual BSD/GPL");  
