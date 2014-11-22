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

  
#define HANDLE_DEV_NAME      "bonovo_bt"   // device name
//#define DEBUG_ON_BT                          // debug falg

#define BT_CTRL_DEV_MAJOR		231
#define BT_CTRL_DEV_MINOR		0

static int      handle_major = BT_CTRL_DEV_MAJOR; 
static dev_t    dev;
static struct   cdev handle_cdev;
static struct   class *handle_class;

#define IOCTL_HANDLE_BT_CLEAR_BUF           _IO(BT_CTRL_DEV_MAJOR, 20)

// about bluetooth -------------------------------------------------
#define BLUETOOTH_BUF_LEN  200
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

////////////////////////////////////////////////////////////////////

static int isBTBuffEmpty(void)
{
    return (bt_dev->buf_head_idx == bt_dev->buf_tail_idx);
}

static int isBTBuffFull(void)
{
    return ((bt_dev->buf_head_idx + 1)%BLUETOOTH_BUF_LEN == bt_dev->buf_tail_idx);
}

static int clearBTBuff(void)
{
    spin_lock(&bt_dev->bluetooth_lock);
	memset(bt_dev->buff, 0, BLUETOOTH_BUF_LEN);
	bt_dev->buf_head_idx = bt_dev->buf_tail_idx = 0;
	spin_unlock(&bt_dev->bluetooth_lock);
	return 0;
}

int bt_write_buff(char* data, int size)
{
	int i = 0, res;

	
	if(isBTBuffFull())
	{
        printk("bluetooth buffer is full when be writted!\n");
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
EXPORT_SYMBOL(bt_write_buff);


static ssize_t bt_read_buff(struct file *fp, char __user *buff, size_t count, loff_t *offset)
{
	int i, res, size;
	char *temp_buf;
	DECLARE_WAITQUEUE(wait, current);   // 定义等待队列
	add_wait_queue(&bt_dev->bt_r_wait, &wait);

	while(isBTBuffEmpty())
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

static long bonovo_bt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch( cmd )
    {
    case IOCTL_HANDLE_BT_CLEAR_BUF:
		clearBTBuff();
		break;
	default:
		return -EINVAL;
		break;
    }
    return 0;
}

static int bonovo_bt_open (struct inode *inode, struct file *filp)  
{

#ifdef DEBUG_ON_BT
	printk("bonovo_bt_open\n");
#endif
	return 0;  
}  

static int bonovo_bt_release (struct inode *inode, struct file *filp)  
{  
#ifdef DEBUG_ON_BT
	printk("bonovo_bt_release\n");
#endif
	return 0;  
}  

static struct file_operations handle_fops =  
{  
	.owner    = THIS_MODULE,
	.unlocked_ioctl = bonovo_bt_ioctl,
	.open     = bonovo_bt_open,
	.release  = bonovo_bt_release,
	.read     = bt_read_buff,
};  

//  ************************************************************ //
//  Device Init : register char driver
//
//  ************************************************************ //
static int __init bonovo_bt_init(void)  
{  
	int result;  
	
#ifdef DEBUG_ON_BT
		printk("bonovo_bt driver loaded\n");
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
		printk("Can't allocate memory for BT of bonovo_bt\n");
		result = -ENOMEM;
		goto fail_bt_mem;
	}
	
	memset(bt_dev, 0, sizeof(struct bluetooth_buff));
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

fail_bt_mem:
	return result;
}  

//  ************************************************************ //
//  Device Exit :
//
//  ************************************************************ //
static void __exit bonovo_bt_exit(void)  
{
	kfree(bt_dev);
	
	device_destroy(handle_class, MKDEV(handle_major, 0));
	class_destroy(handle_class);

	cdev_del(&handle_cdev);
	unregister_chrdev_region(dev, 1);
#ifdef DEBUG_ON_BT
	printk("handle driver unloaded");
#endif
}

module_init(bonovo_bt_init);  
module_exit(bonovo_bt_exit);  
  
MODULE_LICENSE("Dual BSD/GPL");  
