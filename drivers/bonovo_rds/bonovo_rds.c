// ****************************************** //
// Driver for RDS data read and write
// 
// Target : RK3188
//
// Author : dzwei, 2014-12-1
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
#include <asm/uaccess.h>  
#include <asm/io.h>  
#include <linux/circ_buf.h>


#define DEBUG_RDS_CDEV	0
#if DEBUG_RDS_CDEV
#define printk_rds(format, arg...)	\
	printk(format, ##arg);
#else
#define printk_rds(format, arg...) 
#endif

#define RDS_CDEV_NAME      "bonovo_rds"   // device name
#define DEV_MAJOR			237
#define DEV_MINOR			0

static int rds_cdev_major = DEV_MAJOR;
static struct class *rds_class;

#define RDS_IOCTL_START_DATA	_IO(DEV_MAJOR, 0)		// request rds data
#define RDS_IOCTL_STOP_DATA		_IO(DEV_MAJOR, 1)		// stop transfer rds data to android

#define FLAG_USE_CIRC_BUF		1

#define RDS_CIRC_BUF_SIZE		PAGE_SIZE
struct rds_cdev_t
{
    unsigned char is_opened;
    spinlock_t lock;
	wait_queue_head_t r_wait;
	wait_queue_head_t w_wait;
	struct cdev cdev;
	struct circ_buf *circ;
	struct file *fp;
};

static struct rds_cdev_t *rds_cdev;
extern unsigned int calculateSum(unsigned char* cmdBuf, int size);
extern int serial_send_ack(char * data, int len);



// this function will be called from rk_serial.c.
// when we received rds data frame, we call this function
// to fill the frame data to the rds circle buffer.
// buf			--- the start address of rda data
// buf_len		--- the valid data number in the buffer
int fill_rds_buf(unsigned char *buf, unsigned int buf_len)
{
	struct circ_buf *circ;
	int c;
	int ret_val = 0;

#if 0
    // add by zbiao for debug
	int i;
	printk_rds("=== get can data. Data Len:%d  Data:\n", buf_len);
	for(i = 0; i<buf_len; i++) {
		printk_rds(" 0x%02X", buf[i]);
	}
	printk_rds("\n");
#endif

	if (!buf || !rds_cdev)
	{
		printk("=====fill_rds_buf=====, input buf or rds_cdev is NULL \r\n");
		return -1;
	}

    if(!rds_cdev->is_opened){
        printk("=====fill_rds_buf=====, rds_cdev has not been opened!\n");
        return -3;
    }

    if(!(rds_cdev->circ)){
        printk("=====fill_rds_buf=====, rds_cdev->circ is NULL \r\n");
        return -1;
    }
	circ = rds_cdev->circ;
	if (!circ->buf)
		return 0;
	
	spin_lock(&rds_cdev->lock);
	while (1) {
		c = CIRC_SPACE_TO_END(circ->head, circ->tail, RDS_CIRC_BUF_SIZE);
		if (buf_len < c)
			c = buf_len;
		if (c <= 0)
			break;
		memcpy(circ->buf + circ->head, buf, c);
		circ->head = (circ->head + c) & (RDS_CIRC_BUF_SIZE - 1);
		buf += c;
		buf_len -= c;
		ret_val += c;
	}
	spin_unlock(&rds_cdev->lock);
	
	if (!(rds_cdev->fp->f_flags & O_NONBLOCK))
	{
		wake_up_interruptible(&rds_cdev->r_wait);
	}

	return ret_val;
}

static int rds_open (struct inode *inode, struct file *filp)  
{
	printk_rds("rds_open\r\n");
	rds_cdev->fp = filp;			// save fp. fp will be used in fill_rds_buf() 
	rds_cdev->is_opened = 1;
	
	return 0;  
}  

static ssize_t rds_read(struct file *fp, char __user *buf, size_t count, loff_t *offset)
{
	int ret_val = 0; 
	struct circ_buf *circ;
	int c;

	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&rds_cdev->r_wait, &wait);

	circ = rds_cdev->circ;
	if (!circ->buf)
		return 0;

	while (circ->tail == circ->head)
	{
		if (fp->f_flags & O_NONBLOCK)
		{
			printk("=====rds_read=====, rds buffer is empty!\r\n");
			ret_val = -EAGAIN;
			goto rds_buf_empty;
		}
		
		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (signal_pending(current))
		{
			ret_val = -ERESTARTSYS;
			goto rds_buf_empty;
		}
	}
	
	spin_lock(&rds_cdev->lock);
	while (1) 
	{
		c = CIRC_CNT_TO_END(circ->head, circ->tail, RDS_CIRC_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		if (copy_to_user(buf, circ->buf + circ->tail, c))
		{
			printk("=====rds_read=====, cannot copy all data to user space!\r\n");
			break;
		}
		circ->tail = (circ->tail + c) & (RDS_CIRC_BUF_SIZE - 1);
		buf += c;
		count -= c;
		ret_val += c;
	}
	spin_unlock(&rds_cdev->lock);

rds_buf_empty:
	remove_wait_queue(&rds_cdev->r_wait, &wait);
	set_current_state(TASK_RUNNING);
	
	return ret_val;
}

static ssize_t rds_write(struct file *fp, const char __user *buf,
						size_t count, loff_t *ppos)
{
	return 0;
}

static long rds_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	// rds data stop command frame
	unsigned char uart3_frame_buf[10] = 
	{
		0xFA, 0xFA, 0x0A, 0x00, 0x83, 0x0A, 0x00, 0x00
	};
	unsigned int checksum;
    switch (cmd)
    {
	case RDS_IOCTL_START_DATA:
		uart3_frame_buf[6] = 0x01;
		checksum = calculateSum(uart3_frame_buf, 8);
		uart3_frame_buf[8] = checksum&0xFF;
		uart3_frame_buf[9] = (checksum>>8)&0xFF;
		serial_send_ack(uart3_frame_buf, 10);
		break;
	case RDS_IOCTL_STOP_DATA:
		uart3_frame_buf[6] = 0x00;
		checksum = calculateSum(uart3_frame_buf, 8);
		uart3_frame_buf[8] = checksum&0xFF;
		uart3_frame_buf[9] = (checksum>>8)&0xFF;
		serial_send_ack(uart3_frame_buf, 10);
		break;
	default:
		return -EINVAL;
		break;
    }
	return 0;
}


static int rds_release (struct inode *inode, struct file *filp)  
{  
    rds_cdev->is_opened = 0;
    rds_cdev->fp = NULL;			// save fp. fp will be used in fill_rds_buf() 
	
	printk_rds("rds_release\r\n");
	
	return 0;  
}  

static struct file_operations rds_fops =  
{  
	.owner    = THIS_MODULE,
	.open	  = rds_open,
	.read	  = rds_read,
	.write	  = rds_write,
	.unlocked_ioctl	  = rds_ioctl,
	.release  = rds_release,
};  

//  ************************************************************ //
//  Device Init : register char driver
//
//  ************************************************************ //
static int __init bonovo_rds_init(void)  
{  
	int result;  
	dev_t dev_no;
	
	printk_rds("bonovo_rds_cdev driver loaded\r\n");

	if (0 == rds_cdev_major)
	{
		/* auto select a major */
		result = alloc_chrdev_region(&dev_no, 0, 1, RDS_CDEV_NAME);
		rds_cdev_major = MAJOR(dev_no);
	}
	else
	{
		/* use load time defined major number */
		dev_no = MKDEV(rds_cdev_major, DEV_MINOR);
		result = register_chrdev_region(dev_no, 1, RDS_CDEV_NAME);
	}

	if (result)
	{
		printk("register_rds_cdev_region error!\r\n");
		return result;
	}
	rds_cdev = kmalloc(sizeof(struct rds_cdev_t), GFP_KERNEL);
	if (!rds_cdev)
	{
		printk("Can't allocate memory for rds cdev\r\n");
		return -ENOMEM;
	}
	memset(rds_cdev, 0, sizeof(struct rds_cdev_t));

    rds_cdev->circ = kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
    if(!rds_cdev->circ)
    {
        kfree(rds_cdev);
        printk("Can't allocate memory for rds->cir\r\n");
        return -ENOMEM;
    }
    memset(rds_cdev->circ, 0, sizeof(struct circ_buf));

	rds_cdev->circ->buf = kmalloc(RDS_CIRC_BUF_SIZE, GFP_KERNEL);
	if (!rds_cdev->circ->buf)
	{
	    kfree(rds_cdev->circ);
		kfree(rds_cdev);
		printk("Can't allocate memory for rds cdev\r\n");
		return -ENOMEM;
	}
	spin_lock_init(&rds_cdev->lock);
	init_waitqueue_head(&rds_cdev->r_wait);
	init_waitqueue_head(&rds_cdev->w_wait);
	memset(&rds_cdev->cdev, 0, sizeof(struct cdev));
	/* initialize our char dev data */
	cdev_init(&rds_cdev->cdev, &rds_fops);

	/* register char dev with the kernel */
	result = cdev_add(&rds_cdev->cdev, dev_no, 1);
	if (0 != result)
	{
		unregister_chrdev_region(dev_no, 1);
		kfree(rds_cdev->circ->buf);
        kfree(rds_cdev->circ);
		kfree(rds_cdev);
		printk("Error registrating mali device object with the kernel\r\n");
		return -1;
	}
    rds_class = class_create(THIS_MODULE, RDS_CDEV_NAME);
    device_create(rds_class, NULL, dev_no, NULL, RDS_CDEV_NAME);
	return 0;
}  

//  ************************************************************ //
//  Device Exit :
//
//  ************************************************************ //
static void __exit bonovo_rds_exit(void)  
{
	dev_t dev_no = MKDEV(rds_cdev_major, DEV_MINOR);
	// delete device file
	device_destroy(rds_class, dev_no);
	class_destroy(rds_class);
	
	cdev_del(&rds_cdev->cdev);
	unregister_chrdev_region(dev_no, 1);
	
	kfree(rds_cdev->circ->buf);
	kfree(rds_cdev);

	printk_rds("handle driver unloaded\r\n");
}

module_init(bonovo_rds_init);  
module_exit(bonovo_rds_exit);  
  
MODULE_LICENSE("Dual BSD/GPL");  
