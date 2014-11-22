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
#include <net/sock.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>  
#include <asm/io.h>  

#define DEBUG_ZBIAO
#ifdef DEBUG_ZBIAO
#define printk_zbiao(format, arg...)	\
	printk(format, ##arg);
#else
#define printk_zbiao(format, arg...) 
#endif

#define HANDLE_DEV_NAME      "bonovo_virtual_serial"   // device name
//#define OBD_WRITE_BLOCK                     // write block flag

#define DEV_MAJOR		232
#define DEV_MINOR		0

static int      handle_major = DEV_MAJOR;
static dev_t    dev;
static struct   cdev handle_cdev;
static struct   class *handle_class;

#define IOCTL_CLEAR_BUF           _IO(DEV_MAJOR, 0)
#define OBD_BUF_LEN                   200

struct SObdDev
{
    spinlock_t lock;
//	struct semaphore sem;
	wait_queue_head_t r_wait;
	wait_queue_head_t w_wait;
    unsigned int buf_head_idx;
	unsigned int buf_tail_idx;
	char buff[OBD_BUF_LEN];
};
static struct SObdDev *obd_dev;

////////////////////////////////////////////////////////////////////

static int isDevBuffEmpty(void)
{
    return (obd_dev->buf_head_idx == obd_dev->buf_tail_idx);
}

static int isDevBuffFull(void)
{
    return ((obd_dev->buf_head_idx + 1)%OBD_BUF_LEN == obd_dev->buf_tail_idx);
}

static int clearDevBuff(void)
{
    spin_lock(&obd_dev->lock);
	memset(obd_dev->buff, 0, OBD_BUF_LEN);
	obd_dev->buf_head_idx = obd_dev->buf_tail_idx = 0;
	spin_unlock(&obd_dev->lock);
	return 0;
}

#ifdef OBD_WRITE_BLOCK
static ssize_t virtual_serial_write_buff(struct file *fp, const char __user *data, size_t size, loff_t *offset)
#else
int virtual_serial_write_buff(char* data, int size)
#endif
{
	int i = 0, res;
	
#ifdef OBD_WRITE_BLOCK
	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&obd_dev->w_wait, &wait);
	
	while(isDevBuffFull())
	{
		if(fp->f_flags & O_NONBLOCK)
		{
			res = -EAGAIN;
			goto fail_buff_full;
		}

		__set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if(signal_pending(current))
		{
			res = -ERESTARTSYS;
			goto fail_buff_full;
		}
	}
#else	
	if(isDevBuffFull())
	{
        printk("obd buffer is full when be writted!\n");
		res = -ENOMEM;
		//wake_up_interruptible(&obd_dev->r_wait);
		goto fail_buff_full;
	}
#endif

#if 0
	if(obd_dev->buf_head_idx >= obd_dev->buf_tail_idx)
	{
		usable_buff_size = OBD_BUF_LEN - obd_dev->buf_head_idx + obd_dev->buf_tail_idx; 
	}else{
		usable_buff_size = obd_dev->buf_tail_idx - obd_dev->buf_head_idx; 
	}
	if(size > usable_buff_size)
	{
		printk("the usable buffer of bluetooth is too small.\n");
		spin_unlock(&obd_dev->lock);
		return -1;
	}
#endif
	spin_lock(&obd_dev->lock);
	for(i=0; (i<size)&&(!isDevBuffFull()); i++)
	{
		obd_dev->buff[obd_dev->buf_head_idx] = data[i];
		obd_dev->buf_head_idx = (obd_dev->buf_head_idx + 1)%OBD_BUF_LEN;
	}
	spin_unlock(&obd_dev->lock);
	wake_up_interruptible(&obd_dev->r_wait);
	res = size;
	
fail_buff_full:
#ifdef OBD_WRITE_BLOCK
	remove_wait_queue(&obd_dev->w_wait, &wait);
	set_current_state(TASK_RUNNING);
#endif
	return res;
	
}
EXPORT_SYMBOL(virtual_serial_write_buff);


static ssize_t obd_read_buff(struct file *fp, char __user *buff, size_t count, loff_t *offset)
{
	int i, res, size;
	char *temp_buf;

	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&obd_dev->r_wait, &wait);

	while(isDevBuffEmpty())
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
	if(isDevBuffEmpty())
	{
		printk("++++ bluetooth buffer is empty when be read!\n");
		res = -EAGAIN;
		return res;
	}
#endif
	temp_buf = kmalloc((size_t)OBD_BUF_LEN, GFP_KERNEL);
	if(!temp_buf)
	{
		return -ENOMEM;
	}
	memset(temp_buf, 0, OBD_BUF_LEN);
	size = 0;
	spin_lock(&obd_dev->lock);
	for(i=0; (i < count) && (!isDevBuffEmpty()); i++)
	{
		temp_buf[i] = obd_dev->buff[obd_dev->buf_tail_idx];
		obd_dev->buf_tail_idx = (obd_dev->buf_tail_idx+1)%OBD_BUF_LEN;
		size++;
	}
	spin_unlock(&obd_dev->lock);

	if(copy_to_user(buff, temp_buf, size)){
		res = -EFAULT;
	}else{
		res = size;
	}
	kfree(temp_buf);
	
#ifdef OBD_WRITE_BLOCK
	wake_up_interruptible(&obd_dev->w_wait);  // Happened Null point
#endif

fail_buff_empty:
	remove_wait_queue(&obd_dev->r_wait, &wait);
	set_current_state(TASK_RUNNING);
	return res;
}

static unsigned int obd_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask = 0;

    poll_wait(filp, &obd_dev->r_wait, wait);
    poll_wait(filp, &obd_dev->w_wait, wait);

    if(!isDevBuffEmpty()){
        mask |= POLLIN | POLLRDNORM;
    }
    if(!isDevBuffFull()){
        mask |= POLLOUT | POLLWRNORM;
    }
    return mask;
}

static long obd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch( cmd )
    {
    case IOCTL_CLEAR_BUF:
		clearDevBuff();
		break;
	default:
		return -EINVAL;
		break;
    }
    return 0;
}

static int obd_open (struct inode *inode, struct file *filp)  
{
	printk_zbiao("obd_open\n");
	return 0;  
}  

static int obd_release (struct inode *inode, struct file *filp)  
{  
	printk_zbiao("obd_release\n");
	return 0;  
}  

static struct file_operations handle_fops =  
{  
	.owner    = THIS_MODULE,
	.unlocked_ioctl = obd_ioctl,
	.open     = obd_open,
	.release  = obd_release,
	.read     = obd_read_buff,
	.poll     = obd_poll,
#ifdef OBD_WRITE_BLOCK
	.write    = virtual_serial_write_buff,
#endif
};  

//  ************************************************************ //
//  Device Init : register char driver
//
//  ************************************************************ //
static int __init obd_init(void)  
{  
	int result;  
	
	printk_zbiao("bonovo_obd driver loaded\n");

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
	
	obd_dev = kmalloc(sizeof(struct SObdDev), GFP_KERNEL);
	if(!obd_dev)
	{
		printk("Can't allocate memory for BT of bonovo_bt\n");
		result = -ENOMEM;
		goto fail_bt_mem;
	}
	
	memset(obd_dev, 0, sizeof(struct SObdDev));
	spin_lock_init(&obd_dev->lock);
	//init_MUTEX(&obd_dev->sem);
	init_waitqueue_head(&obd_dev->r_wait);
	init_waitqueue_head(&obd_dev->w_wait);
	
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

fail_bt_mem:
	return result;
}  

//  ************************************************************ //
//  Device Exit :
//
//  ************************************************************ //
static void __exit obd_exit(void)  
{
	kfree(obd_dev);
	
	device_destroy(handle_class, MKDEV(handle_major, 0));
	class_destroy(handle_class);

	cdev_del(&handle_cdev);
	unregister_chrdev_region(dev, 1);

	printk_zbiao("handle driver unloaded");
}

module_init(obd_init);  
module_exit(obd_exit);  
  
MODULE_LICENSE("Dual BSD/GPL");  
