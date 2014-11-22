// ****************************************** //
// Temporary Driver for HANDLE on Android
// 
// Title  : UART2 and GPIO (HANDLE_ON)
// Target : TCC8801
//
// Author : wchao
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
#include <linux/slab.h>
#include <linux/sched.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>  
#include <asm/io.h>  
#include <linux/list.h>
#include <linux/sched.h>

//#define DEBUG_ZBIAO
#ifdef DEBUG_ZBIAO
#define printk_zbiao(format, arg...)	\
	printk(format, ##arg);
#else
#define printk_zbiao(format, arg...) 
#endif
  
#define HANDLE_DEV_NAME      "bonovo_handle"   // device name
//#define DEBUG_ON_HANDLE                          // debug falg

#define HANDLE_CTRL_DEV_MAJOR		230
#define HANDLE_CTRL_DEV_MINOR		0

#define IOCTL_HANDLE_GET_BRIGHTNESS		    _IO(HANDLE_CTRL_DEV_MAJOR, 1)
#define IOCTL_HANDLE_GET_WINCEVOLUME		_IO(HANDLE_CTRL_DEV_MAJOR, 2)
#define IOCTL_HANDLE_GET_SYS_VOL            _IO(HANDLE_CTRL_DEV_MAJOR, 10)
// radio
#define IOCTL_HANDLE_GET_RADIO_STATUS       _IO(HANDLE_CTRL_DEV_MAJOR, 3)
#define IOCTL_HANDLE_START_RADIO_SEARCH     _IO(HANDLE_CTRL_DEV_MAJOR, 4)
#define IOCTL_HANDLE_STOP_RADIO_SEARCH      _IO(HANDLE_CTRL_DEV_MAJOR, 5)
#define IOCTL_HANDLE_GET_RADIO_CURR_FREQ    _IO(HANDLE_CTRL_DEV_MAJOR, 6)
#define IOCTL_HANDLE_GET_RADIO_FREQ         _IO(HANDLE_CTRL_DEV_MAJOR, 7)
#define IOCTL_HANDLE_CLEAR_RADIO_BUFF       _IO(HANDLE_CTRL_DEV_MAJOR, 8)
// bt
#define IOCTL_HANDLE_BT_POWER_ON            _IO(HANDLE_CTRL_DEV_MAJOR, 20)
#define IOCTL_HANDLE_BT_POWER_OFF           _IO(HANDLE_CTRL_DEV_MAJOR, 21)
#define IOCTL_HANDLE_BT_CLEAR_BUF           _IO(HANDLE_CTRL_DEV_MAJOR, 22)
#define IOCTL_HANDLE_BT_SET_CONNECT         _IO(HANDLE_CTRL_DEV_MAJOR, 23)
#define IOCTL_HANDLE_BT_SET_DISCONNECT      _IO(HANDLE_CTRL_DEV_MAJOR, 24)
#define IOCTL_HANDLE_BT_GET_STATUS          _IO(HANDLE_CTRL_DEV_MAJOR, 25)

#define BT_STATUS_POWER_OFF          0   // bt power off
#define BT_STATUS_POWER_ON           1   // bt power on but not connect
#define BT_STATUS_CONNECT            2   // bt connectting

static int bt_status = BT_STATUS_POWER_OFF;

// codec
#define IOCTL_HANDLE_CODEC_SWITCH_SRC       _IO(HANDLE_CTRL_DEV_MAJOR, 30)
#define IOCTL_HANDLE_CODEC_RECOVER_SRC      _IO(HANDLE_CTRL_DEV_MAJOR, 31)
#define IOCTL_HANDLE_CODEC_INIT             _IO(HANDLE_CTRL_DEV_MAJOR, 32)
#define IOCTL_HANDLE_CODEC_GET_CURRENT_SRC  _IO(HANDLE_CTRL_DEV_MAJOR, 33)

// power key
#define IOCTL_HANDLE_SEND_POWER_KEY         _IO(HANDLE_CTRL_DEV_MAJOR, 11)

#define CODEC_SET_CMD       0xA0
#define CMD_CODEC_AUDIO_SRC_SELECT  0               // select external analog audio source commander
#define CMD_CODEC_MUX_SRC_SELECT    1               // select MUX audio source commander
#define CMD_CODEC_MIXER_SRC_SELECT  2               // select Mixer source commander
#define CMD_CODEC_MIXER_DIG_VOLUME  3               // set digital audio volume commander
#define CMD_CODEC_MIXER_ANA_VOLUME  4               // set analog audio volume commander
#define CMD_CODEC_LOUT1_VOLUME      5               // set LOUT1 volume commander
#define CMD_CODEC_ROUT1_VOLUME      6               // set ROUT1 volume commander
#define CMD_CODEC_LOUT2_VOLUME      7               // set LOUT2 volume commander
#define CMD_CODEC_ROUT2_VOLUME      8               // set ROUT2 volume commander
#define CMD_CODEC_OUTPUT_VOLUME     9               // set Total volume commander
#define CMD_CODEC_MUTE              10              // mute commander
#define CMD_CODEC_STEREO_STRENGTHEN 11              // 3D stereo strength commander

#define SET_CODEC_ANALOG_SRC_CLOSE  0               // cut analog audio 
#define SET_CODEC_ANALOG_SRC_AVIN   1               // select av_in as anlog audio
#define SET_CODEC_ANALOG_SRC_DVB    2               // select DVB/ISDBT as anlog audio
#define SET_CODEC_ANALOG_SRC_DVD    3               // select DVD as anlog audio
#define SET_CODEC_ANALOG_SRC_RADIO  4               // select radio as anlog audio
#define SET_CODEC_ANALOG_SRC_BT     5               // select bluetooth as anlog audio

#define SET_CODEC_MIXER_SRC_ANALOG  0               // Analog audio as mixer source
#define SET_CODEC_MIXER_SRC_DIGITAL 1               // Digital audio as mixer source
#define SET_CODEC_MIXER_SRC_DA      2               // Analog and digital as mixer source
// mcu status
#define IOCTL_HANDLE_MCU_LIGHT_STATUS       _IO(HANDLE_CTRL_DEV_MAJOR, 40)
#define IOCTL_HANDLE_MCU_REVERSE_STATUS       _IO(HANDLE_CTRL_DEV_MAJOR, 41)

#define ASTERN_STATE     		 0			    //mcu astern status
#define LIGHT_STATE			 1			    //mcu light status

typedef enum
{
    MUX_ONLY_ANALOG = 0,
    MUX_ONLY_DIGIT = 1,
    MUX_ANALOG_DIGIT = 2,
    MUX_MODEL_COUNT
}MUX_Model;

// codec status
typedef enum
{
	CODEC_LEVEL_NO_ANALOG = 0,
    CODEC_LEVEL_BT_MUSIC = 1,
    CODEC_LEVEL_AV_IN = 2,
    CODEC_LEVEL_DVB = 3,
    CODEC_LEVEL_DVD = 4,
    CODEC_LEVEL_RADIO = 5,
    CODEC_LEVEL_BT_TEL = 6,
    CODEC_LEVEL_COUNT
}CODEC_Level;
#define CODEC_DEFAULT_SOURCE         CODEC_LEVEL_NO_ANALOG

struct audio_node
{
    pid_t id;
	CODEC_Level level;
	struct list_head node;
};

struct codec_dev
{
	struct semaphore codec_sem;
	CODEC_Level current_src;    // current analog audio source
	pid_t turn_pid;
	struct list_head list;      // list head
//	int pre_idx;
//	CODEC_Level previous[CODEC_LEVEL_COUNT];
};
struct codec_dev *codec_src;

static int      handle_major = HANDLE_CTRL_DEV_MAJOR; 
static dev_t    dev;
static struct   cdev handle_cdev;
static struct   class *handle_class;
static int g_bonovo_brightness=150;
static int g_bonovo_wince_volume = 0;
static int g_bonovo_sys_vol = 10;
static struct   fasync_struct *async_queue;   // async struct point for read 
static int bonovo_fasync(int fd, struct file *filp, int mode)
{
    return fasync_helper(fd, filp, mode, &async_queue);
}

extern int serial_send_ack(char * data, int len);

// about radio ----------------------------------------------------
#define RADIO_BUF_LEN    20
struct radio_freq
{
	unsigned char freq[2];
	unsigned char is_valid;
};

struct radio_status
{
	spinlock_t radio_lock;                        // spin lock of radio (perservation)
	unsigned char search_status;                  // searching status(1:searching  0:no searching)
	unsigned char buf_head_idx;                   // radio frequence buffer head index
	unsigned char buf_tail_idx;                   // radio frequence buffer tail index
	struct radio_freq radio_freqs[RADIO_BUF_LEN]; // radio frequence buffer
};
struct radio_status *radio_buf;
#define RADIO_NO_ACTION    0
#define RADIO_SEARCHING    1
#define RADIO_START_SEARCH 2

// about bluetooth -------------------------------------------------
#define BLUETOOTH_BUF_LEN  4096
struct bluetooth_buff
{
    spinlock_t bluetooth_lock;
//	struct semaphore sem;
	wait_queue_head_t bt_r_wait;
	wait_queue_head_t bt_w_wait;
    unsigned int buf_head_idx;
	unsigned int buf_tail_idx;
	char buff[BLUETOOTH_BUF_LEN];
};
static struct bluetooth_buff *bt_dev;

// about mcu_status -------------------------------------------------
struct mcu_status
{
	unsigned int light_status;
	unsigned int astern_status;
}mcu_st;

////////////////////////////////////////////////////////////////////

unsigned int checkSum(unsigned char* cmdBuf, int size)
{
	unsigned int temp = 0;
	int i;
	for(i=0; i < size; i++)
	{
		temp += cmdBuf[i];
	}
	return temp;
}

static int codecMuxSelectSrc(MUX_Model toSrc)
{
    int ret = -1;
    unsigned int sum;
    unsigned char cmdMuxModel[9] = {0xFA, 0xFA, 0x09, 0x00, CODEC_SET_CMD, 0x01};
    
    switch(toSrc){
    case MUX_ONLY_ANALOG:
    case MUX_ONLY_DIGIT:
    case MUX_ANALOG_DIGIT:
        cmdMuxModel[6] = toSrc;
        sum = checkSum(cmdMuxModel, 7);
		cmdMuxModel[7] = sum & 0x00FF;
		cmdMuxModel[8] = (sum >> 8) & 0x00FF;
		ret = serial_send_ack(cmdMuxModel, 9);
		if(ret < 0){
			ret = -EAGAIN;
		}else{
			ret = 0;
            
		}
        printk_zbiao("++++++++++++ codecMuxSelectSrc  toSrc:%d  ret:%d\n", toSrc, ret);
        break;
    default:
        break;
    }
    return ret;
}

static int codecSwitchAnalogSrc(CODEC_Level switchToSrc)
{
    int ret = 0;
	unsigned int sum;
	unsigned char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, CODEC_SET_CMD, 0x00};
	struct audio_node *pnode;

	printk_zbiao("[%s] switchToSrc:%d currentSrc:%d turn_pid:%d current:%d\n",
		__FUNCTION__, switchToSrc, codec_src->current_src, codec_src->turn_pid, current->pid);
	
    if(switchToSrc == codec_src->current_src){
		return 0;
    }
	down(&codec_src->codec_sem);
	if(codec_src->current_src != CODEC_LEVEL_BT_TEL){
		pnode = (struct audio_node *)kmalloc(sizeof(struct audio_node), GFP_KERNEL);
		pnode->id = codec_src->turn_pid = current->pid;
		pnode->level = codec_src->current_src;
		list_add(&pnode->node, &codec_src->list);
		codec_src->current_src = switchToSrc;
		btCmd[6] = codec_src->current_src;
		up(&codec_src->codec_sem);

		switch(btCmd[6]){
		case CODEC_LEVEL_NO_ANALOG:
			btCmd[6] = SET_CODEC_ANALOG_SRC_CLOSE;
			break;
		case CODEC_LEVEL_BT_MUSIC:
			btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
			break;
        case CODEC_LEVEL_AV_IN:
			btCmd[6] = SET_CODEC_ANALOG_SRC_AVIN;
			break;
        case CODEC_LEVEL_DVB:
			btCmd[6] = SET_CODEC_ANALOG_SRC_DVB;
			break;
        case CODEC_LEVEL_DVD:
			btCmd[6] = SET_CODEC_ANALOG_SRC_DVD;
			break;
        case CODEC_LEVEL_RADIO:
			btCmd[6] = SET_CODEC_ANALOG_SRC_RADIO;
			break;
        case CODEC_LEVEL_BT_TEL:
			btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
            codecMuxSelectSrc(MUX_ONLY_ANALOG);
			break;
		default:
			return -EINVAL;
				break;
		}
		sum = checkSum((unsigned char*)btCmd, 7);
		btCmd[7] = sum & 0x00FF;
		btCmd[8] = (sum >> 8) & 0x00FF;
		ret = serial_send_ack(btCmd, 9);
		if(ret < 0){
			ret = -EAGAIN;
		}else{
			ret = 0;
            
		}
    }else{
		pnode = (struct audio_node *)kmalloc(sizeof(struct audio_node), GFP_KERNEL);
		pnode->id = current->pid;
		pnode->level = switchToSrc;
		list_add(&pnode->node, &codec_src->list);
		up(&codec_src->codec_sem);
    }
	return ret;
}

static int codecRecoverPrevSrc(CODEC_Level currSrc)
{
    int ret = 0;
	unsigned int sum;
    CODEC_Level oldLevel = CODEC_LEVEL_COUNT;
	char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, CODEC_SET_CMD, 0x00};
	struct list_head *pos, *n;
	struct audio_node *pnode;
	
	printk_zbiao("[%s] codec_src->turn_id:%d currentSrc:%d current_id:%d currSrc:%d\n",
		__FUNCTION__, codec_src->turn_pid, codec_src->current_src, current->pid, currSrc);
	down(&codec_src->codec_sem);
//	if(codec_src->pre_idx >= 0){
	if(!list_empty(&codec_src->list)){
		if((codec_src->current_src == currSrc) && (codec_src->turn_pid == current->pid)){
			pnode = list_entry(codec_src->list.next, struct audio_node, node);
            oldLevel = codec_src->current_src;
			codec_src->current_src = pnode->level;
			codec_src->turn_pid = pnode->id;
			list_del(&pnode->node);
			kfree(pnode);
			pnode = NULL;
            printk_zbiao("oldLevel:%d\n", oldLevel);
			btCmd[6] = codec_src->current_src;
			up(&codec_src->codec_sem);
			switch(btCmd[6]){
			case CODEC_LEVEL_NO_ANALOG:
				btCmd[6] = SET_CODEC_ANALOG_SRC_CLOSE;
				break;
			case CODEC_LEVEL_BT_MUSIC:
				btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
				break;
            case CODEC_LEVEL_AV_IN:
				btCmd[6] = SET_CODEC_ANALOG_SRC_AVIN;
				break;
            case CODEC_LEVEL_DVB:
				btCmd[6] = SET_CODEC_ANALOG_SRC_DVB;
				break;
            case CODEC_LEVEL_DVD:
				btCmd[6] = SET_CODEC_ANALOG_SRC_DVD;
				break;
            case CODEC_LEVEL_RADIO:
				btCmd[6] = SET_CODEC_ANALOG_SRC_RADIO;
				break;
            case CODEC_LEVEL_BT_TEL:
				btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
				break;
			default:
				return -EINVAL;
				break;
			}
            if(oldLevel == CODEC_LEVEL_BT_TEL){
                codecMuxSelectSrc(MUX_ANALOG_DIGIT);
            }
			sum = checkSum((unsigned char*)btCmd, 7);
			btCmd[7] = sum & 0x00FF;
			btCmd[8] = (sum >> 8) & 0x00FF;
			ret = serial_send_ack(btCmd, 9);
			if(ret < 0){
                printk_zbiao("serial_send_ack failed. ret:%d\n", ret);
				ret = -EAGAIN;
			}else{
			    ret = 0;
			}
		}else{
			list_for_each_safe(pos, n, &codec_src->list){
				pnode = list_entry(pos, struct audio_node, node);
				if((pnode->id == current->pid) && (pnode->level == currSrc)){
					list_del(pos);
					kfree(pnode);
					pnode = NULL;
					break;
				}
			}
			up(&codec_src->codec_sem);
		}
	}else{
	    printk_zbiao("codecRecoverPrevSrc list is empty.\n");
		up(&codec_src->codec_sem);
		ret = -EINVAL;
	}

	return ret;
}

static int codecClear(void)
{
    // need to realize
    struct list_head *pos, *n;
	struct audio_node *pnode;
	int ret = 0;
	char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, CODEC_SET_CMD, 0x00};
	unsigned int sum;
    down(&codec_src->codec_sem);
	list_for_each_safe(pos, n, &codec_src->list){
		list_del(pos);
		pnode = list_entry(pos, struct audio_node, node);
		kfree(pnode);
	}
	codec_src->current_src = CODEC_DEFAULT_SOURCE;
	codec_src->turn_pid = 0;
	up(&codec_src->codec_sem);

	btCmd[6] = SET_CODEC_ANALOG_SRC_CLOSE;
	sum = checkSum((unsigned char*)btCmd, 7);
	btCmd[7] = sum & 0x00FF;
	btCmd[8] = (sum >> 8) & 0x00FF;
	ret = serial_send_ack(btCmd, 9);
	if(ret < 0){
		ret = -EAGAIN;
	}else{
	    ret = 0;
	}
    codecMuxSelectSrc(MUX_ANALOG_DIGIT);
	return ret;
}

static int codecInit(void)
{
	int ret = 0;
	char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, CODEC_SET_CMD, 0x00};
	unsigned int sum;
	sema_init(&codec_src->codec_sem, 1);
	//down(&codec_src->codec_sem);
	//codec_src->pre_idx = -1;
	//memset(codec_src->previous, 0 , sizeof(codec_src->previous));
	codec_src->current_src = CODEC_DEFAULT_SOURCE;
	codec_src->turn_pid = 0;
	INIT_LIST_HEAD(&codec_src->list);
	//up(&codec_src->codec_sem);
	codecMuxSelectSrc(MUX_ANALOG_DIGIT);
	
	btCmd[6] = CODEC_DEFAULT_SOURCE;
	switch(btCmd[6]){
	case CODEC_LEVEL_NO_ANALOG:
		btCmd[6] = SET_CODEC_ANALOG_SRC_CLOSE;
		break;
	case CODEC_LEVEL_BT_MUSIC:
		btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
		break;
    case CODEC_LEVEL_AV_IN:
		btCmd[6] = SET_CODEC_ANALOG_SRC_AVIN;
		break;
    case CODEC_LEVEL_DVB:
		btCmd[6] = SET_CODEC_ANALOG_SRC_DVB;
		break;
    case CODEC_LEVEL_DVD:
		btCmd[6] = SET_CODEC_ANALOG_SRC_DVD;
		break;
    case CODEC_LEVEL_RADIO:
		btCmd[6] = SET_CODEC_ANALOG_SRC_RADIO;
		break;
    case CODEC_LEVEL_BT_TEL:
		btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
		break;
	default:
		return -EINVAL;
		break;
	}
	sum = checkSum((unsigned char*)btCmd, 7);
	btCmd[7] = sum & 0x00FF;
	btCmd[8] = (sum >> 8) & 0x00FF;
	ret = serial_send_ack(btCmd, 9);
	if(ret < 0){
		ret = -EAGAIN;
	}else{
	    ret = 0;
	}
	return ret;
}

static int clearBTBuff(void)
{
    spin_lock(&bt_dev->bluetooth_lock);
	memset(bt_dev->buff, 0, BLUETOOTH_BUF_LEN);
	bt_dev->buf_head_idx = bt_dev->buf_tail_idx = 0;
	spin_unlock(&bt_dev->bluetooth_lock);
	return 0;
}

static int btClearCodecCache(void)
{
	int ret = 0;
	unsigned int sum;
	char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, CODEC_SET_CMD, 0x00};
	struct audio_node *pnode;
	struct list_head *pos, *n;
	
	down(&codec_src->codec_sem);
	list_for_each_safe(pos, n, &codec_src->list){
		pnode = list_entry(pos, struct audio_node, node);
		if((pnode->level == CODEC_LEVEL_BT_MUSIC)||(pnode->level == CODEC_LEVEL_BT_TEL)){
			list_del(pos);
			kfree(pnode);
			pnode = NULL;
			break;
		}
	}

	if((codec_src->current_src == CODEC_LEVEL_BT_MUSIC)||(codec_src->current_src == CODEC_LEVEL_BT_TEL)){
		pnode = list_entry(codec_src->list.next, struct audio_node, node);
		codec_src->current_src = pnode->level;
		codec_src->turn_pid = pnode->id;
		list_del(&pnode->node);
		kfree(pnode);
		pnode = NULL;	
		btCmd[6] = codec_src->current_src;
		up(&codec_src->codec_sem);
		switch(btCmd[6]){
		case CODEC_LEVEL_NO_ANALOG:
			btCmd[6] = SET_CODEC_ANALOG_SRC_CLOSE;
			break;
		case CODEC_LEVEL_BT_MUSIC:
			btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
			break;
		case CODEC_LEVEL_AV_IN:
			btCmd[6] = SET_CODEC_ANALOG_SRC_AVIN;
			break;
		case CODEC_LEVEL_DVB:
			btCmd[6] = SET_CODEC_ANALOG_SRC_DVB;
			break;
		case CODEC_LEVEL_DVD:
			btCmd[6] = SET_CODEC_ANALOG_SRC_DVD;
			break;
		case CODEC_LEVEL_RADIO:
			btCmd[6] = SET_CODEC_ANALOG_SRC_RADIO;
			break;
		case CODEC_LEVEL_BT_TEL:
			btCmd[6] = SET_CODEC_ANALOG_SRC_BT;
			break;
		default:
			return -EINVAL;
			break;
		}
		sum = checkSum((unsigned char*)btCmd, 7);
		btCmd[7] = sum & 0x00FF;
		btCmd[8] = (sum >> 8) & 0x00FF;
		ret = serial_send_ack(btCmd, 9);
		if(ret < 0){
			ret = -EAGAIN;
		}else{
		    ret = 0;
		}
        codecMuxSelectSrc(MUX_ANALOG_DIGIT);
	}
	up(&codec_src->codec_sem);
    return 0;
}

static int btPowerCtl(int offOn)
{
    char btCmd[9] = {0xFA, 0xFA, 0x09, 0x00, 0x83, 0x01};
	unsigned int sum, ret;

	if(offOn){
		btCmd[6] = 0x01;
	}else{
		btCmd[6] = 0x00;
	}

	sum = checkSum((unsigned char*)btCmd, 7);
	btCmd[7] = sum & 0x00FF;
	btCmd[8] = (sum >> 8) & 0x00FF;
	ret = serial_send_ack(btCmd, 9);
	if(ret > 0){
        ret = 0;
		printk_zbiao("+++++++++++++++ btPowerCtl(%d)+++++++++\n", offOn);
		if(offOn){
			bt_status = BT_STATUS_POWER_ON;
			clearBTBuff();
		}else{
			bt_status = BT_STATUS_POWER_OFF;
			btClearCodecCache();
		}
	}
	return ret;
}

static int isBTBuffEmpty(void)
{
    return (bt_dev->buf_head_idx == bt_dev->buf_tail_idx);
}

static int isBTBuffFull(void)
{
    return ((bt_dev->buf_head_idx + 1)%BLUETOOTH_BUF_LEN == bt_dev->buf_tail_idx);
}

int bonovo_write_bt_buff(char* data, int size)
{
	int i = 0, res;

	
	if(isBTBuffFull()/* || (bt_status == BT_STATUS_POWER_OFF)*/)
	{
        printk("bluetooth buffer is full when be writted!   bt_status:%d\n", bt_status);
		res = -ENOMEM;
		//wake_up_interruptible(&bt_dev->bt_r_wait);
		goto fail_buff_full;
	}
#if 0
	if(bt_dev->buf_head_idx >= bt_dev->buf_tail_idx)
	{
		usable_buff_size = BLUETOOTH_BUF_LEN - bt_dev->buf_head_idx + bt_dev->buf_tail_idx; 
	}else{
		usable_buff_size = bt_dev->buf_tail_idx - bt_dev->buf_head_idx; 
	}
	if(size > usable_buff_size)
	{
		printk("the usable buffer of bluetooth is too small.\n");
		spin_unlock(&bt_dev->bluetooth_lock);
		return -1;
	}
#endif
	spin_lock(&bt_dev->bluetooth_lock);
	for(i=0; (i<size)&&(!isBTBuffFull()); i++)
	{
		bt_dev->buff[bt_dev->buf_head_idx] = data[i];
		bt_dev->buf_head_idx = (bt_dev->buf_head_idx + 1)%BLUETOOTH_BUF_LEN;
	}
	spin_unlock(&bt_dev->bluetooth_lock);
	wake_up_interruptible(&bt_dev->bt_r_wait);
	return 0;
fail_buff_full:
	return res;
	
}
EXPORT_SYMBOL(bonovo_write_bt_buff);
#if 0
int bonovo_read_bt_buff(char *data, int size)
{
	int i = 0;
	if(isBTBuffEmpty())
	{
		printk("bluetooth buffer is empty when be read!\n");
		return -1;
	}

	spin_lock(&bt_dev->bluetooth_lock);
	for(i=0;(i < size) && (!isBTBuffEmpty()); i++)
	{
		data[i] = bt_dev->buff[bt_dev->buf_tail_idx];
		bt_dev->buf_tail_idx = (bt_dev->buf_tail_idx+1)%BLUETOOTH_BUF_LEN;
	}
	spin_unlock(&bt_dev->bluetooth_lock);
	return 0;
}
#endif

static ssize_t bonovo_read_bt_buff(struct file *fp, char __user *buff, size_t count, loff_t *offset)
{
	int i, res, size;
	char *temp_buf;
	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&bt_dev->bt_r_wait, &wait);

	while(isBTBuffEmpty() || (bt_status == BT_STATUS_POWER_OFF))
	{
		if(fp->f_flags & O_NONBLOCK)
		{
			//printk("bluetooth buffer is empty when be read!\n");
			res = -EAGAIN;
			goto fail_buff_empty;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if(signal_pending(current))
		{
			res = -ERESTARTSYS;
			goto fail_buff_empty;
		}
	}
#if 0	
	if(isBTBuffEmpty())
	{
		printk("++++ bluetooth buffer is empty when be read!\n");
		res = -EAGAIN;
		return res;
	}
#endif
	temp_buf = kmalloc((size_t)BLUETOOTH_BUF_LEN, GFP_KERNEL);
	if(!temp_buf)
	{
		return -ENOMEM;
	}
	memset(temp_buf, 0, BLUETOOTH_BUF_LEN);
	size = 0;
	spin_lock(&bt_dev->bluetooth_lock);
	for(i=0; (i < count) && (!isBTBuffEmpty()); i++)
	{
		temp_buf[i] = bt_dev->buff[bt_dev->buf_tail_idx];
		bt_dev->buf_tail_idx = (bt_dev->buf_tail_idx+1)%BLUETOOTH_BUF_LEN;
		size++;
	}
	spin_unlock(&bt_dev->bluetooth_lock);

	if(copy_to_user(buff, temp_buf, size)){
		res = -EFAULT;
	}else{
		res = size;
	}
	kfree(temp_buf);
//	wake_up_interruptible(bt_dev->bt_w_wait);

fail_buff_empty:
	remove_wait_queue(&bt_dev->bt_r_wait, &wait);
	set_current_state(TASK_RUNNING);
	return res;
}

//----------------------------------------------------------------
// radio
int isRadioBuffEmpty(void)
{
    return (radio_buf->buf_head_idx == radio_buf->buf_tail_idx);
}

int isRadioBuffFull(void)
{
    return ((radio_buf->buf_head_idx + 1)%RADIO_BUF_LEN == radio_buf->buf_tail_idx);
}

int clearRadioBuff(void)
{
	spin_lock(&radio_buf->radio_lock);
	memset(radio_buf->radio_freqs, 0, sizeof(struct radio_freq)*RADIO_BUF_LEN);
	radio_buf->buf_head_idx = radio_buf->buf_tail_idx = 0;
	radio_buf->search_status = 0;
	spin_unlock(&radio_buf->radio_lock);
	return 0;
}

int bonovo_write_radio_buff(unsigned char freq_low, unsigned char freq_high, unsigned char valid)
{
    if(isRadioBuffFull())
    {
#ifdef DEBUG_ON_HANDLE
        printk("radio buffer is full!\n");
#endif
		return -1;
    }
	spin_lock(&radio_buf->radio_lock);
	radio_buf->radio_freqs[radio_buf->buf_head_idx].freq[0] = freq_low;
	radio_buf->radio_freqs[radio_buf->buf_head_idx].freq[1] = freq_high;
	radio_buf->radio_freqs[radio_buf->buf_head_idx].is_valid = valid;
	radio_buf->buf_head_idx = (radio_buf->buf_head_idx + 1)%RADIO_BUF_LEN;
	spin_unlock(&radio_buf->radio_lock);
	return 0;
}
EXPORT_SYMBOL(bonovo_write_radio_buff);

int bonovo_read_radio_buff(unsigned char __user * buff, int len)
{
    //int freq;
	int res;
	unsigned char temp[3] = {0};
    if(isRadioBuffEmpty())
	{
#ifdef DEBUG_ON_HANDLE
        printk("radio buffer is empty!\n");
#endif
        return -1;
    }
	if(len < sizeof(struct radio_freq))
	{
#ifdef DEBUG_ON_HANDLE
		printk("radio buffer is less than size of struct radio_freq!\n");
#endif
		return -2;
	}
	spin_lock(&radio_buf->radio_lock);
	temp[0] = radio_buf->radio_freqs[radio_buf->buf_tail_idx].freq[0];
	temp[1] = radio_buf->radio_freqs[radio_buf->buf_tail_idx].freq[1];
	temp[2] = radio_buf->radio_freqs[radio_buf->buf_tail_idx].is_valid;
	radio_buf->buf_tail_idx = (radio_buf->buf_tail_idx + 1)%RADIO_BUF_LEN;
	spin_unlock(&radio_buf->radio_lock);

	res = copy_to_user(buff, temp, sizeof(struct radio_freq));
	return res;
}

int bonovo_read_current_freq(unsigned char __user * buff, int len)
{
	int res;
	unsigned char temp[3] = {0};
	if(isRadioBuffEmpty())
	{
#ifdef DEBUG_ON_HANDLE
		printk("radio buffer is empty!\n");
#endif
		return -1;
	}
	if(len < sizeof(struct radio_freq))
	{
#ifdef DEBUG_ON_HANDLE
		printk("radio buffer is less than size of struct radio_freq!\n");
#endif
		return -2;
	}
	spin_lock(&radio_buf->radio_lock);
	temp[0] = radio_buf->radio_freqs[radio_buf->buf_head_idx].freq[0];
	temp[1] = radio_buf->radio_freqs[radio_buf->buf_head_idx].freq[1];
	temp[2] = radio_buf->radio_freqs[radio_buf->buf_head_idx].is_valid;
	spin_unlock(&radio_buf->radio_lock);

	res = copy_to_user(buff, temp, sizeof(struct radio_freq));
	return res;
}

int bonovo_set_radio_status(unsigned char status)
{
	spin_lock(&radio_buf->radio_lock);
    if(((RADIO_START_SEARCH == status)&&(radio_buf->search_status == RADIO_NO_ACTION))
		||((RADIO_START_SEARCH != status)))
    {
		radio_buf->search_status = status;
	}
	spin_unlock(&radio_buf->radio_lock);
	
	return 0;
}
EXPORT_SYMBOL(bonovo_set_radio_status);

void bonovo_set_brightness_leven(int leven)
{
	g_bonovo_brightness = leven;
}
EXPORT_SYMBOL(bonovo_set_brightness_leven);

void bonovo_set_sys_vol(int vol)
{
    g_bonovo_sys_vol = vol;
}
EXPORT_SYMBOL(bonovo_set_sys_vol);

void bonovo_set_wince_volume(int volume)
{
	g_bonovo_wince_volume = volume;
	if(async_queue)
        kill_fasync(&async_queue, SIGIO, POLL_IN);
	/*
	if(g_bonovo_wince_volume == 0)
	{
		g_bonovo_wince_volume = 1;
	}
	else
	{
		g_bonovo_wince_volume = 0;
	}
	*/
#ifdef DEBUG_ON_HANDLE
	printk("--bonovo_set_wince_volume = %d --\n",g_bonovo_wince_volume);
#endif
}
EXPORT_SYMBOL(bonovo_set_wince_volume);

void bonovo_mcu_status(char* data, int size)
{
//    printk(KERN_INFO "myu mcu len = %d data[0] = %d data[1] = %d data[2] = %d\n",size,data[0],data[1],data[2]);
    switch(data[0]){
	case ASTERN_STATE:
		mcu_st .astern_status = (data[1] & 0x00FF) + ((data[2]<<8) & 0x00FF);
		break;
	case LIGHT_STATE:
		mcu_st .light_status = (data[1] & 0x00FF) + ((data[2]<<8) & 0x00FF);
		break;
	default:
		printk(KERN_INFO "Nothing to do myu\n");
		break;
	}
}
EXPORT_SYMBOL(bonovo_mcu_status);


static int bonovo_handle_open (struct inode *inode, struct file *filp)  
{

#ifdef DEBUG_ON_HANDLE
	printk("bonovo_handle_open\n");
#endif
	return 0;  
}  

static int bonovo_handle_release (struct inode *inode, struct file *filp)  
{
    bonovo_fasync(-1, filp, 0);
#ifdef DEBUG_ON_HANDLE
	printk("bonovo_handle_release\n");
#endif
	return 0;  
}  

extern void rk29_send_power_key(int state);
static long bonovo_handle_ioctl (struct file *filp,
                           unsigned int cmd, unsigned long arg)  
{  	
    int ret = 0;
	switch( cmd )  
	{  
	case IOCTL_HANDLE_GET_BRIGHTNESS:
#ifdef DEBUG_ON_HANDLE
		printk("IOCTL_HANDLE_GET_BRIGHTNESS\n");
#endif
		return (g_bonovo_brightness);
		
    case IOCTL_HANDLE_GET_SYS_VOL:
        return (g_bonovo_sys_vol);
        break;

	case IOCTL_HANDLE_GET_WINCEVOLUME:
#ifdef DEBUG_ON_HANDLE
		printk("IOCTL_HANDLE_GET_WINCEVOLUME\n");
#endif
		return (g_bonovo_wince_volume);

	case IOCTL_HANDLE_GET_RADIO_STATUS:
		return radio_buf->search_status;
		break;
	case IOCTL_HANDLE_START_RADIO_SEARCH:
		bonovo_set_radio_status(RADIO_START_SEARCH);
		break;
	case IOCTL_HANDLE_STOP_RADIO_SEARCH:
		bonovo_set_radio_status(RADIO_NO_ACTION);
		break;
	case IOCTL_HANDLE_GET_RADIO_CURR_FREQ:
		return bonovo_read_current_freq((unsigned char*)arg, sizeof(struct radio_freq));
		break;
	case IOCTL_HANDLE_GET_RADIO_FREQ:
		return bonovo_read_radio_buff((unsigned char*)arg, sizeof(struct radio_freq));
		break;
	case IOCTL_HANDLE_CLEAR_RADIO_BUFF:
		clearRadioBuff();
		break;
	case IOCTL_HANDLE_BT_POWER_ON:
		return btPowerCtl(1);
		break;
	case IOCTL_HANDLE_BT_POWER_OFF:
		return btPowerCtl(0);
		break;
	case IOCTL_HANDLE_BT_CLEAR_BUF:
		clearBTBuff();
		break;
	case IOCTL_HANDLE_BT_SET_CONNECT:
		bt_status = BT_STATUS_CONNECT;
		break;
	case IOCTL_HANDLE_BT_SET_DISCONNECT:
		bt_status = BT_STATUS_POWER_ON;
		break;
	case IOCTL_HANDLE_BT_GET_STATUS:
		return (bt_status);
		break;
	case IOCTL_HANDLE_CODEC_SWITCH_SRC:
		if((arg >= 0)&&(arg <= CODEC_LEVEL_BT_TEL)){
			return codecSwitchAnalogSrc((CODEC_Level)arg);
		}else{
			return -EINVAL;
		}
		break;
	case IOCTL_HANDLE_CODEC_RECOVER_SRC:
//        if(arg != NULL){
            ret = (int)arg;
    		if((arg >= 0)&&(arg <= CODEC_LEVEL_BT_TEL)){
    			return codecRecoverPrevSrc((CODEC_Level)arg);
    		}
//        }else {
//			return codecRecoverPrevSrc((CODEC_Level)0);
//		}
		break;
	case IOCTL_HANDLE_CODEC_INIT:
		return codecClear();
		break;
    case IOCTL_HANDLE_CODEC_GET_CURRENT_SRC:
              return copy_to_user((CODEC_Level*)arg, &(codec_src->current_src), sizeof(CODEC_Level));
	case IOCTL_HANDLE_MCU_LIGHT_STATUS:
		//printk("IOCTL_HANDLE_MCU_LIGHT_STATUS(myu)\n");
		return mcu_st.light_status;
    case IOCTL_HANDLE_MCU_REVERSE_STATUS:
        return mcu_st.astern_status;
        break;
    case IOCTL_HANDLE_SEND_POWER_KEY:
        rk29_send_power_key(1);
        rk29_send_power_key(0);
        break;
	default :
		return -EINVAL;
		break;
	}
	return 0;  
}  

static struct file_operations handle_fops =  
{  
	.owner    = THIS_MODULE,  
	.unlocked_ioctl    = bonovo_handle_ioctl,  
	.open     = bonovo_handle_open,       
	.release  = bonovo_handle_release,
	.read     = bonovo_read_bt_buff,
	.fasync   = bonovo_fasync,
};  

//  ************************************************************ //
//  Device Init : register char driver
//
//  ************************************************************ //
static int __init bonovo_handle_init(void)  
{  
	int result;  
	
#ifdef DEBUG_ON_HANDLE
		printk("bonovo_handle driver loaded\n");
#endif
	if (0 == handle_major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev, 0, 1, HANDLE_DEV_NAME);
		handle_major = MAJOR(dev);
	}
	else
	{
		/* use load time defined major number */
		dev = MKDEV(handle_major, 0);
		result = register_chrdev_region(dev, 1, HANDLE_DEV_NAME);
	}
	
	if (result)
	{
		printk("register_chrdev_region error!\n");
		return result;
	}
	
	bt_dev = kmalloc(sizeof(struct bluetooth_buff), GFP_KERNEL);
	if(!bt_dev)
	{
		printk("Can't allocate memory for BT of bonovo_handle\n");
		result = -ENOMEM;
		goto fail_bt_mem;
	}
	radio_buf = kmalloc(sizeof(struct radio_status), GFP_KERNEL);
	if(!radio_buf)
	{
		printk("Can't allocate memory for RADIO of bonovo_handle\n");
		result = -ENOMEM;
		goto fail_radio_mem;
	}

	codec_src = kmalloc(sizeof(struct codec_dev), GFP_KERNEL);
	if(!codec_src){
		printk("Can't allocate memory for RADIO of bonovo_handle\n");
		result = -ENOMEM;
		goto fail_codec_mem;
	}
	codecInit();
//	memset(codec_src, 0 , sizeof(struct codec_dev));
//  sema_init(&codec_src->codec_sem, 1);
//	codec_src->current_src = CODEC_DEFAULT_SOURCE;
//	codec_src->pre_idx = -1;
	
	memset(bt_dev, 0, sizeof(struct bluetooth_buff));
	memset(radio_buf, 0 , sizeof(struct radio_status));
	spin_lock_init(&radio_buf->radio_lock);
	spin_lock_init(&bt_dev->bluetooth_lock);
	//init_MUTEX(&bt_dev->sem);
	init_waitqueue_head(&bt_dev->bt_r_wait);
	init_waitqueue_head(&bt_dev->bt_w_wait);
	
	memset(&handle_cdev, 0, sizeof(handle_cdev));

	/* initialize our char dev data */
	cdev_init(&handle_cdev, &handle_fops);

	/* register char dev with the kernel */
	result = cdev_add(&handle_cdev, dev, 1);
    
	if (0 != result)
	{
		unregister_chrdev_region(dev, 1);
		printk("Error registrating mali device object with the kernel\n");
	}

    handle_class = class_create(THIS_MODULE, HANDLE_DEV_NAME);
    device_create(handle_class, NULL, MKDEV(handle_major, MINOR(dev)), NULL,
                  HANDLE_DEV_NAME);

	return 0;
fail_codec_mem:
	kfree(radio_buf);
fail_radio_mem:
	kfree(bt_dev);
fail_bt_mem:
	return result;
}  

//  ************************************************************ //
//  Device Exit :
//
//  ************************************************************ //
static void __exit bonovo_handle_exit(void)  
{
    kfree(codec_src);
	kfree(radio_buf);
	kfree(bt_dev);
	
	device_destroy(handle_class, MKDEV(handle_major, 0));
	class_destroy(handle_class);

	cdev_del(&handle_cdev);
	unregister_chrdev_region(dev, 1);
#ifdef DEBUG_ON_HANDLE
	printk("handle driver unloaded");
#endif
}  
  
module_init(bonovo_handle_init);  
module_exit(bonovo_handle_exit);  
  
MODULE_LICENSE("Dual BSD/GPL");  
