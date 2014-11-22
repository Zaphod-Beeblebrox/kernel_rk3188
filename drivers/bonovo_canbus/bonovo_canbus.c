// ****************************************** //
// Driver for CAN bus data read and write
// 
// Target : RK3188
//
// Author : dzwei, 2014-8-25
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
//#include <net/sock.h>

//#include <asm/mach-types.h>
#include <asm/uaccess.h>  
#include <asm/io.h>  
#include <linux/circ_buf.h>


#define DEBUG_CANBUS_CDEV	0
#if DEBUG_CANBUS_CDEV
#define printk_canbus(format, arg...)	\
	printk(format, ##arg);
#else
#define printk_canbus(format, arg...) 
#endif

#define CANBUS_CDEV_NAME      "bonovo_canbus"   // device name
#define DEV_MAJOR			236
#define DEV_MINOR			0

static int      canbus_cdev_major = DEV_MAJOR;
static struct   class *canbus_class;

#define CANBUS_IOCTL_KEY_INFO		_IO(DEV_MAJOR, 0)
#define CANBUS_BUF_SIZE			64
#define FLAG_USE_CIRC_BUF		1

#if (FLAG_USE_CIRC_BUF)
#define CANBUS_CIRC_BUF_SIZE		PAGE_SIZE
struct canbus_cdev_t
{
    unsigned char is_opened;
    spinlock_t lock;
	wait_queue_head_t r_wait;
	wait_queue_head_t w_wait;
	struct cdev cdev;
	struct circ_buf *circ;
	struct file *fp;
};
#else
#define CANBUS_BUF_NUM			32
struct canbus_buf_t
{
	int w_idx;			// indicate the buffer that is written now
	int r_idx;			// indicate the buffer that is read now
	int valid_buf_num;	// indicate how many buffers that contain the valid data
	int idx_in_buf;		// the index of byte in the current buffer
	unsigned int buf_len[CANBUS_BUF_NUM];
	unsigned char buf[CANBUS_BUF_NUM][CANBUS_BUF_SIZE];
};

struct canbus_cdev_t
{
    unsigned char is_opened;
    spinlock_t lock;
	wait_queue_head_t r_wait;
	wait_queue_head_t w_wait;
	struct cdev cdev;
	struct canbus_buf_t canbus_buf;
	struct file *fp;
};
#endif

static struct canbus_cdev_t *canbus_cdev;
static unsigned char uart3_frame_buf[CANBUS_BUF_SIZE+8] =
{
	0xFA, 0xFA, 0x0C, 0x00, 0xA8, 0x00
};
extern unsigned int calculateSum(unsigned char* cmdBuf, int size);
extern int serial_send_ack(char * data, int len);
extern int bonovo_deal_advance_key(int keyCode, int keyStatus);


int calculate_canbus_frame_checksum(unsigned char *buf, int data_num, unsigned char *checksum)
{
	int i;
	
	*checksum = 0;
	
	if (!buf || data_num <= 0)
	{
		printk("calculate_canbus_frame_checksum error\r\n");
		return -1;
	}
	for (i=0; i<data_num; i++)
	{
		*checksum += buf[i];
	}
	*checksum ^= 0xFF;
	
	return 0;
}

// this function will be called from rk_serial.c.
// when we received canbus data frame, we call this function
// to fill the frame data to the canbus buffer.
int fill_canbus_buf(unsigned char *buf, unsigned int buf_len)
{
	//unsigned char *ptr;
	struct circ_buf *circ;
	int c;
	int ret_val = 0;

#if 0
    // add by zbiao for debug
	int i;
	printk_canbus("=== get can data. Data Len:%d  Data:\n", buf_len);
	for(i = 0; i<buf_len; i++) {
		printk_canbus(" 0x%02X", buf[i]);
	}
	printk_canbus("\n");
#endif

	if (!buf || !canbus_cdev)
	{
		printk("=====fill_canbus_buf=====, input buf or canbus_cdev is NULL \r\n");
		return -1;
	}
	if (buf_len > CANBUS_BUF_SIZE)
	{
		printk("=====fill_canbus_buf=====, input buffer is too large! \r\n");
		return -2;
	}

    if(!canbus_cdev->is_opened){
        printk("=====fill_canbus_buf=====, canbus_cdev has not been opened!\n");
        return -3;
    }

#if (FLAG_USE_CIRC_BUF)
    if(!(canbus_cdev->circ)){
        printk("=====fill_canbus_buf=====, canbus_cdev->circ is NULL \r\n");
        return -1;
    }
	circ = canbus_cdev->circ;
	if (!circ->buf)
		return 0;
	
	spin_lock(&canbus_cdev->lock);
	while (1) {
		c = CIRC_SPACE_TO_END(circ->head, circ->tail, CANBUS_CIRC_BUF_SIZE);
		if (buf_len < c)
			c = buf_len;
		if (c <= 0)
			break;
		memcpy(circ->buf + circ->head, buf, c);
		circ->head = (circ->head + c) & (CANBUS_CIRC_BUF_SIZE - 1);
		buf += c;
		buf_len -= c;
		ret_val += c;
	}
	spin_unlock(&canbus_cdev->lock);
	
#else
    if(!canbus_cdev->canbus_buf){
        printk("=====fill_canbus_buf=====, canbus_cdev->canbus_buf is NULL \r\n");
        return -1;
    }
	if (canbus_cdev->canbus_buf.valid_buf_num >= CANBUS_BUF_NUM)
	{
		printk("=====fill_canbus_buf=====, circle buffer is full! \r\n");
		return -3;
	}
	
	ptr = canbus_cdev->canbus_buf.buf[canbus_cdev->canbus_buf.w_idx];
	memcpy(ptr, buf, buf_len);
	canbus_cdev->canbus_buf.w_idx += 1;
	canbus_cdev->canbus_buf.w_idx &= CANBUS_BUF_NUM-1;
	spin_lock(&canbus_cdev->lock);
	canbus_cdev->canbus_buf.valid_buf_num += 1;
	spin_unlock(&canbus_cdev->lock);
#endif
	
	if (!(canbus_cdev->fp->f_flags & O_NONBLOCK))
	{
		wake_up_interruptible(&canbus_cdev->r_wait);
	}

	return ret_val;
}

static int canbus_open (struct inode *inode, struct file *filp)  
{
	// send can bus data request frame
	unsigned char canbus_frame_buf[5] = 
	{
		0x2E, 0x81, 0x01, 0x01, 0x00
	};
	unsigned int checksum;
	
	printk_canbus("canbus_open\r\n");
	canbus_cdev->fp = filp;			// save fp. fp will be used in fill_canbus_buf() 
	canbus_cdev->is_opened = 1;
	
	calculate_canbus_frame_checksum(canbus_frame_buf, 4, &canbus_frame_buf[4]);
	
	memcpy(&uart3_frame_buf[5], canbus_frame_buf, 5);
	checksum = calculateSum(uart3_frame_buf, 10);
	uart3_frame_buf[10] = checksum&0xFF;
	uart3_frame_buf[11] = (checksum>>8)&0xFF;

	serial_send_ack(uart3_frame_buf, 12);
	
	return 0;  
}  

static ssize_t canbus_read(struct file *fp, char __user *buf, size_t count, loff_t *offset)
{
	int ret_val = 0; 
	//int size;
	//char *ptr;
	struct circ_buf *circ;
	int c;

	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&canbus_cdev->r_wait, &wait);

#if (FLAG_USE_CIRC_BUF)
	circ = canbus_cdev->circ;
	if (!circ->buf)
		return 0;

	while (circ->tail == circ->head)
	{
		if (fp->f_flags & O_NONBLOCK)
		{
			printk("=====canbus_read=====, canbus buffer is empty!\r\n");
			ret_val = -EAGAIN;
			goto canbus_buf_empty;
		}
		
		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (signal_pending(current))
		{
			ret_val = -ERESTARTSYS;
			goto canbus_buf_empty;
		}
	}
	
	spin_lock(&canbus_cdev->lock);
	while (1) 
	{
		c = CIRC_CNT_TO_END(circ->head, circ->tail, CANBUS_CIRC_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		if (copy_to_user(buf, circ->buf + circ->tail, c))
		{
			printk("=====canbus_read=====, cannot copy all data to user space!\r\n");
			break;
		}
		circ->tail = (circ->tail + c) & (CANBUS_CIRC_BUF_SIZE - 1);
		buf += c;
		count -= c;
		ret_val += c;
	}
	spin_unlock(&canbus_cdev->lock);
#else
	while (!canbus_cdev->canbus_buf.valid_buf_num)
	{
		if (fp->f_flags & O_NONBLOCK)
		{
			printk("=====canbus_read=====, canbus buffer is empty!\r\n");
			ret_val = -EAGAIN;
			goto canbus_buf_empty;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (signal_pending(current))
		{
			ret_val = -ERESTARTSYS;
			goto canbus_buf_empty;
		}
	}

	ptr = canbus_cdev->canbus_buf.buf[canbus_cdev->canbus_buf.r_idx];
	size = canbus_cdev->canbus_buf.buf_len[canbus_cdev->canbus_buf.r_idx];

	if (copy_to_user(buf, ptr, size))
	{
		ret_val = -EFAULT;
	}
	else
	{
		ret_val = size;
	}
	canbus_cdev->canbus_buf.r_idx += 1;
	canbus_cdev->canbus_buf.r_idx &= CANBUS_BUF_NUM-1;

	// when this routine can reach here, canbus_cdev->canbus_buf.valid_buf_num
	// must great than 0. so we do not need check this value.
	//if (canbus_cdev->canbus_buf.valid_buf_num)
	{
		spin_lock(&canbus_cdev->lock);
		canbus_cdev->canbus_buf.valid_buf_num -= 1;
		spin_unlock(&canbus_cdev->lock);
	}
#endif

canbus_buf_empty:
	remove_wait_queue(&canbus_cdev->r_wait, &wait);
	set_current_state(TASK_RUNNING);
	
	return ret_val;
}

static ssize_t canbus_write(struct file *fp, const char __user *buf,
						size_t count, loff_t *ppos)
{
	unsigned int checksum;
	if (count > CANBUS_BUF_SIZE)
	{
		printk("=====canbus_write=====, data is too large!\r\n");
		return -1;
	}
	if (copy_from_user(&uart3_frame_buf[5], buf, count))
	{
		return -EFAULT;
	}
	uart3_frame_buf[2] = count+7;
	uart3_frame_buf[3] = 0x00;

	checksum = calculateSum(uart3_frame_buf, count+5);
	uart3_frame_buf[count+5] = checksum&0xFF;
	uart3_frame_buf[count+6] = (checksum>>8)&0xFF;

	return serial_send_ack(uart3_frame_buf, count+7);
}

static long canbus_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int kbuf[3];
	int count;
	
    switch (cmd)
    {
    case CANBUS_IOCTL_KEY_INFO:
		count = copy_from_user(kbuf, (void *)arg, sizeof(kbuf));
		if (count)
		{
			printk("=====canbus_ioctl=====, can not copy parameter from user space !\r\n");
			break;
		}
		
		if (kbuf[0] != 2)
		{
			printk("=====canbus_ioctl=====, wrong parameter number !\r\n");
			break;
		}
		bonovo_deal_advance_key(0x060000 + kbuf[1], kbuf[2]);
		break;
	default:
		return -EINVAL;
		break;
    }
    return 0;
}


static int canbus_release (struct inode *inode, struct file *filp)  
{  
	// canbus data stop command frame
	unsigned char canbus_frame_buf[5] = 
	{
		0x2E, 0x81, 0x01, 0x00, 0x00
	};
	unsigned int checksum;

    canbus_cdev->is_opened = 0;
    canbus_cdev->fp = NULL;			// save fp. fp will be used in fill_canbus_buf() 
	
	printk_canbus("canbus_release\r\n");
	calculate_canbus_frame_checksum(canbus_frame_buf, 4, &canbus_frame_buf[4]);
	
	memcpy(&uart3_frame_buf[5], canbus_frame_buf, 5);
	checksum = calculateSum(uart3_frame_buf, 10);
	uart3_frame_buf[10] = checksum&0xFF;
	uart3_frame_buf[11] = (checksum>>8)&0xFF;

	serial_send_ack(uart3_frame_buf, 12);
	
	return 0;  
}  

static struct file_operations canbus_fops =  
{  
	.owner    = THIS_MODULE,
	.open	  = canbus_open,
	.read	  = canbus_read,
	.write	  = canbus_write,
	.unlocked_ioctl	  = canbus_ioctl,
	.release  = canbus_release,
};  

//  ************************************************************ //
//  Device Init : register char driver
//
//  ************************************************************ //
static int __init bonovo_canbus_init(void)  
{  
	int result;  
	dev_t dev_no;
	
	printk_canbus("bonovo_canbus_cdev driver loaded\r\n");

	if (0 == canbus_cdev_major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev_no, 0, 1, CANBUS_CDEV_NAME);
		canbus_cdev_major = MAJOR(dev_no);
	}
	else
	{
		/* use load time defined major number */
		dev_no = MKDEV(canbus_cdev_major, DEV_MINOR);
		result = register_chrdev_region(dev_no, 1, CANBUS_CDEV_NAME);
	}

	if (result)
	{
		printk("register_canbus_cdev_region error!\r\n");
		return result;
	}
	canbus_cdev = kmalloc(sizeof(struct canbus_cdev_t), GFP_KERNEL);
	if (!canbus_cdev)
	{
		printk("Can't allocate memory for canbus cdev\r\n");
		return -ENOMEM;
	}
	memset(canbus_cdev, 0, sizeof(struct canbus_cdev_t));

    canbus_cdev->circ = kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
    if(!canbus_cdev->circ)
    {
        kfree(canbus_cdev);
        printk("Can't allocate memory for canbus->cir\r\n");
        return -ENOMEM;
    }
    memset(canbus_cdev->circ, 0, sizeof(struct circ_buf));

	canbus_cdev->circ->buf = kmalloc(CANBUS_CIRC_BUF_SIZE, GFP_KERNEL);
	if (!canbus_cdev->circ->buf)
	{
	    kfree(canbus_cdev->circ);
		kfree(canbus_cdev);
		printk("Can't allocate memory for canbus cdev\r\n");
		return -ENOMEM;
	}
	spin_lock_init(&canbus_cdev->lock);
	init_waitqueue_head(&canbus_cdev->r_wait);
	init_waitqueue_head(&canbus_cdev->w_wait);
	memset(&canbus_cdev->cdev, 0, sizeof(struct cdev));
	/* initialize our char dev data */
	cdev_init(&canbus_cdev->cdev, &canbus_fops);

	/* register char dev with the kernel */
	result = cdev_add(&canbus_cdev->cdev, dev_no, 1);
	if (0 != result)
	{
		unregister_chrdev_region(dev_no, 1);
		kfree(canbus_cdev->circ->buf);
        kfree(canbus_cdev->circ);
		kfree(canbus_cdev);
		printk("Error registrating mali device object with the kernel\r\n");
		return -1;
	}
    canbus_class = class_create(THIS_MODULE, CANBUS_CDEV_NAME);
    device_create(canbus_class, NULL, dev_no, NULL, CANBUS_CDEV_NAME);
	return 0;
}  

//  ************************************************************ //
//  Device Exit :
//
//  ************************************************************ //
static void __exit bonovo_canbus_exit(void)  
{
	dev_t dev_no = MKDEV(canbus_cdev_major, DEV_MINOR);
	// delete device file
	device_destroy(canbus_class, dev_no);
	class_destroy(canbus_class);
	
	cdev_del(&canbus_cdev->cdev);
	unregister_chrdev_region(dev_no, 1);
	
	kfree(canbus_cdev->circ->buf);
	kfree(canbus_cdev);

	printk_canbus("handle driver unloaded\r\n");
}

module_init(bonovo_canbus_init);  
module_exit(bonovo_canbus_exit);  
  
MODULE_LICENSE("Dual BSD/GPL");  
