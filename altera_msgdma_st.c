/*
 * Copyright (C) 2018 Pavel Fiala / main driver /
 *               2015 Intel (Altera) Corporation / headers only
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place, Suite 330, Boston, MA 02111-1307 USA
 */

 // System Linux includes ...
 // -------------------------
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>

// ioctl declaration - use also for user space application ...
#include "altera_msgdma_ioctl.h"

// Altera MSGDMA register maps includes ...
// ----------------------------------------
// Follow this url:
// https://www.intel.com/content/www/us/en/programmable/documentation/sfo1400787952932/lro1402196946061/lro1402202415704.html

#include "my_altera_msgdma_descriptor_regs.h"
#include "my_altera_msgdma_csr_regs.h"

// Defines & enums
// ---------------
#define CONST_DEBUG_MODE						// Turns on DEBUG verbose mode / enabled by default ...
#define CONST_DEBUG_EXTENDED_MODE		// Turns on EXTENDED DEBUG verbose mode / disabled by default ...
#define CONST_KMALLOC_MULTIPLE 128  // Default value - 128 ...
#define CONST_NAME_BUF_SIZE 32			// Default value - 32 ...
#define CONST_MIN_BLOCK_SIZE 1024	  // Default value - 1024 ...

#define CSR_STATUS_REG			(ALTERA_MSGDMA_CSR_STATUS_REG << 2)
#define CSR_CONTROL_REG			(ALTERA_MSGDMA_CSR_CONTROL_REG << 2)
#define CSR_DESCRIPTOR_FILL_LEVEL_REG	(ALTERA_MSGDMA_CSR_DESCRIPTOR_FILL_LEVEL_REG << 2)
#define CSR_RESPONSE_FILL_LEVEL_REG	(ALTERA_MSGDMA_CSR_RESPONSE_FILL_LEVEL_REG << 2)

#define DESC_READ_ADDRESS_REG		(ALTERA_MSGDMA_DESCRIPTOR_READ_ADDRESS_REG << 2)
#define DESC_WRITE_ADDRESS_REG		(ALTERA_MSGDMA_DESCRIPTOR_WRITE_ADDRESS_REG << 2)
#define DESC_LENGTH_REG			(ALTERA_MSGDMA_DESCRIPTOR_LENGTH_REG << 2)
#define DESC_CONTROL_REG		(ALTERA_MSGDMA_DESCRIPTOR_CONTROL_STANDARD_REG << 2)

#define START_DMA_MASK	( \
		ALTERA_MSGDMA_DESCRIPTOR_CONTROL_GO_MASK | \
		ALTERA_MSGDMA_DESCRIPTOR_CONTROL_TRANSFER_COMPLETE_IRQ_MASK)

#define MSGDMA_CSR_STAT_MASK GENMASK(9, 0)

typedef enum{
  MSGDMA_READ,
  MSGDMA_WRITE
}MSGDMA_ENUM;

// Global definitions ...
// ----------------------
static int g_dev_index;
static int g_dev_read_index;
static int g_dev_write_index;
static struct platform_driver msgdma_platform_driver;
static struct list_head g_dev_list;
static struct semaphore g_dev_probe_sem;

static spinlock_t g_irq_lock;

// struct altera_msgdma_dev
// ------------------------
struct altera_msgdma_dev{
  int id;     /* msgdma id in list */
  int type;   /* msgdma read = 0, msgdma write = 1 */
  int opened; /* msgdma closed = 0, msgdma opened = 1 */
	size_t g_buffer_size; /*msgdma buffer size */
	size_t g_min_interval_size; /*msgdma min. interval size */
  char name[CONST_NAME_BUF_SIZE]; /* device name */
  int g_sgdma_csr_addr; /* csr register base address*/
  int g_sgdma_csr_size; /* csr register region size*/
  int g_sgdma_desc_addr; /* desc register base address*/
  int g_sgdma_desc_size; /* desc register region size*/
  void __iomem *g_ioremap_csr_addr; /* csr remap addres */
  void __iomem *g_ioremap_desc_addr; /* csr remap addres */
  int g_sgdma_irq; /* sgdma irq */
  void *g_kmalloc_ptr; /* driver kmalloc allocated buffer */
  struct device *pdev_dev; /* msgdma device structure */
  struct semaphore dev_sem; /* msgdma semaphore structure */
  wait_queue_head_t wait_queue; /*msgdma waitqueue */
  struct miscdevice miscdev;  /* msgdma misc (char) device */
  struct list_head dev_list;  /* msgdma list_head structure */
};

// get_dma_fill_level function - param (struct altera_msgdma_dev *) ...
// --------------------------------------------------------------------
static inline uint32_t get_dma_fill_level(const struct altera_msgdma_dev *g_altera_msgdma_dev){

  uint32_t msgdma_fill_level = 0;
  uint32_t msgdma_read_fill_level = 0;
  uint32_t msgdma_write_fill_level = 0;
  uint32_t msgdma_max_fill_level = 0;

  msgdma_fill_level = ioread32(g_altera_msgdma_dev->g_ioremap_csr_addr +
                               CSR_DESCRIPTOR_FILL_LEVEL_REG);

  msgdma_read_fill_level = msgdma_fill_level;
  msgdma_read_fill_level &= ALTERA_MSGDMA_CSR_READ_FILL_LEVEL_MASK;
  msgdma_read_fill_level >>= ALTERA_MSGDMA_CSR_READ_FILL_LEVEL_OFFSET;

  msgdma_write_fill_level = msgdma_fill_level;
  msgdma_write_fill_level &= ALTERA_MSGDMA_CSR_WRITE_FILL_LEVEL_MASK;
  msgdma_write_fill_level >>= ALTERA_MSGDMA_CSR_WRITE_FILL_LEVEL_OFFSET;

  msgdma_max_fill_level = (msgdma_write_fill_level > msgdma_read_fill_level) ?
       msgdma_write_fill_level : msgdma_read_fill_level;

  return msgdma_max_fill_level;
}

// get_dma_busy function - param (struct altera_msgdma_dev *) ...
// --------------------------------------------------------------
static inline uint32_t get_dma_busy(const struct altera_msgdma_dev *g_altera_msgdma_dev){

  uint32_t msgdma_status = 0;

  msgdma_status = ioread32(g_altera_msgdma_dev->g_ioremap_csr_addr +
                           CSR_STATUS_REG);

  msgdma_status &= ALTERA_MSGDMA_CSR_BUSY_MASK;
  msgdma_status >>= ALTERA_MSGDMA_CSR_BUSY_OFFSET;

  return msgdma_status;
}

// altera_msgdma_stop function - param (struct altera_msgdma_dev *) ...
// --------------------------------------------------------------------
static int altera_msgdma_stop(const struct altera_msgdma_dev *g_altera_msgdma_dev){

	int ret = 0;
	uint32_t val = 0;

	iowrite32(ALTERA_MSGDMA_CSR_STOP_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr +
						CSR_CONTROL_REG);
	/**
	* readl_poll_timeout - Periodically poll an address until a condition is met or a timeout occurs
	* @addr: Address to poll
	* @val: Variable to read the value into
	* @cond: Break condition (usually involving @val)
	* @sleep_us: Maximum time to sleep between reads in uS (0 tight-loops)
	* @timeout_us: Timeout in uS, 0 means never timeout
	*
	* Returns 0 on success and -ETIMEDOUT upon a timeout. In either
	* case, the last read value at @addr is stored in @val. Must not
	* be called from atomic context if sleep_us or timeout_us are used.
	*/

	ret = readl_poll_timeout(g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_STATUS_REG, val,
										      (val & ALTERA_MSGDMA_CSR_STOP_STATE_MASK) != 0, 1, 10000);

	if(ret) return -ETIMEDOUT;

	return 0;
}


// altera_msgdma_reset function - param (struct altera_msgdma_dev *) ...
// ---------------------------------------------------------------------
static int altera_msgdma_reset(const struct altera_msgdma_dev *g_altera_msgdma_dev){

	int ret = 0;
	uint32_t val = 0;

	/* Reset mSGDMA */
	iowrite32(MSGDMA_CSR_STAT_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr +
						CSR_STATUS_REG);

	iowrite32(ALTERA_MSGDMA_CSR_RESET_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr +
						CSR_CONTROL_REG);
	/**
	* readl_poll_timeout - Periodically poll an address until a condition is met or a timeout occurs
	* @addr: Address to poll
	* @val: Variable to read the value into
	* @cond: Break condition (usually involving @val)
	* @sleep_us: Maximum time to sleep between reads in uS (0 tight-loops)
	* @timeout_us: Timeout in uS, 0 means never timeout
	*
	* Returns 0 on success and -ETIMEDOUT upon a timeout. In either
	* case, the last read value at @addr is stored in @val. Must not
	* be called from atomic context if sleep_us or timeout_us are used.
	*/

	ret = readl_poll_timeout(g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_STATUS_REG, val,
										      (val & ALTERA_MSGDMA_CSR_RESET_STATE_MASK) == 0, 1, 10000);

	if(ret) return -ETIMEDOUT;

	/* Clear all status bits */
	iowrite32(MSGDMA_CSR_STAT_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr +
						CSR_STATUS_REG);

	/* Enable the DMA controller including interrupts */
	iowrite32(ALTERA_MSGDMA_CSR_GLOBAL_INTERRUPT_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr +
						CSR_CONTROL_REG);

	return 0;
}

// Platform altera_msgdma_open function ...
// ----------------------------------------
static int altera_msgdma_open(struct inode *ip,struct file *fp){

  struct altera_msgdma_dev *g_altera_msgdma_dev = NULL;
  struct list_head *next_list_entry = NULL;

  int found_this = 0;
  uint32_t minor_this = 0;
  uint32_t access_mode = 0;

  /* acquire the probe lock */
  if(down_interruptible(&g_dev_probe_sem)){
    return -ERESTARTSYS;
  }

  // Get  minor number ...
  // ---------------------
  minor_this = iminor(ip);

  /*
  #define list_entry(ptr, type, member) \
  ((type *)((char *)(ptr) – (unsigned long)(&((type *)0)->member)))
  */

  list_for_each(next_list_entry,&g_dev_list){
    g_altera_msgdma_dev = list_entry(next_list_entry,struct altera_msgdma_dev,dev_list);
    if(minor_this == g_altera_msgdma_dev->miscdev.minor){
       found_this = 1; 	// Item was found in linked list - exit loop ...
       break;
     }
  }

  up(&g_dev_probe_sem);

  if(found_this == 0) return -ENXIO;                   /* No such device or address - 6 */
  if(g_altera_msgdma_dev->opened == 1) return -EBUSY;  /* Device or resource busy - 16 */

  access_mode = (fp->f_flags & O_ACCMODE);

  // sgdma write mode ...
  // --------------------
  if(g_altera_msgdma_dev->type == 0 && access_mode == O_RDONLY){ // READ only...
     g_altera_msgdma_dev->opened = 1; 	// 1 --> opened ...
  }else if(g_altera_msgdma_dev->type == 1 && access_mode == O_WRONLY){ // WRITE only ...
     g_altera_msgdma_dev->opened = 1;   // 1 --> opened ...
  }else{
     g_altera_msgdma_dev->opened = 0;   // 0 --> closed ...
     return -EACCES;          				  /* Permission denied - 13 */
  }

#ifdef CONST_DEBUG_EXTENDED_MODE
	if(g_altera_msgdma_dev->type == 0){
		 pr_info("Opened Altera MSGDMA device with ID %d (in READ mode) ... \n",g_altera_msgdma_dev->id);
	}else if(g_altera_msgdma_dev->type == 1){
		 pr_info("Opened Altera MSGDMA device with ID %d (in WRITE mode) ... \n",g_altera_msgdma_dev->id);
	}
#endif

  // Set private data ...
  // --------------------
  fp->private_data = g_altera_msgdma_dev;

  return 0;
}

// Platform altera_msgdma_release function ...
// -------------------------------------------
static int altera_msgdma_release(struct inode *ip,struct file *fp){

  struct altera_msgdma_dev *g_altera_msgdma_dev = (struct altera_msgdma_dev *)fp->private_data;
  g_altera_msgdma_dev->opened = 0;  // 0 --> closed ...

#ifdef CONST_DEBUG_EXTENDED_MODE
	if(g_altera_msgdma_dev->type == 0){
		 pr_info("Closed Altera MSGDMA device with ID %d (in READ mode) ... \n",g_altera_msgdma_dev->id);
	}else if(g_altera_msgdma_dev->type == 1){
		 pr_info("Closed Altera MSGDMA device with ID %d (in WRITE mode) ... \n",g_altera_msgdma_dev->id);
	}
#endif

	// Unset private data ...
	// ----------------------
  // fp->private_data = 0;

  return 0;
}

// Platform altera_msgdma_ioctl function ...
// -----------------------------------------
static long altera_msgdma_ioctl(struct file *fp, unsigned int cmd, unsigned long arg){

	struct altera_msgdma_dev *g_altera_msgdma_dev = (struct altera_msgdma_dev *)fp->private_data;

	size_t new_buffer_size = 0;
	size_t new_interval_size = 0;

	int altera_msgdma_ret = 0;

	/* acquire the probe lock */
	if(down_interruptible(&g_altera_msgdma_dev->dev_sem)){
		pr_info("altera_msgdma_ioctl sem interrupted - exit \n");
		return -ERESTARTSYS;
	}

	// check get_dma_busy (altera_msgdma_dev *) state - disabled ...
	/*
	while(get_dma_busy(g_altera_msgdma_dev)!=0){
			  if(wait_event_interruptible(g_altera_msgdma_dev->wait_queue,
					 													get_dma_busy(g_altera_msgdma_dev)==0)){
					up(&g_altera_msgdma_dev->dev_sem);
					pr_info("altera_msgdma_ioctl wait interrupted - exit \n");
					return -ERESTARTSYS;
			  }
	}
	*/

	switch (cmd) {
		case IO_ALTERA_MSGDMA_SET_BUFFER_SIZE: // SET_BUFFER_SIZE ...
				if(get_user(new_buffer_size, (size_t *)arg) < 0){
						up(&g_altera_msgdma_dev->dev_sem);
						pr_info("altera_msgdma_ioctl get_user - exit \n");
						return -EFAULT;
				}

				// realloc memory - prev. allocated via kmalloc ...
				// -----------------------------------------------------
				// http://einon.net/DocBook/kernel-api/API-krealloc.html
				if(new_buffer_size > 0){
					 g_altera_msgdma_dev->g_buffer_size = new_buffer_size;
					 g_altera_msgdma_dev->g_kmalloc_ptr = krealloc(g_altera_msgdma_dev->g_kmalloc_ptr,g_altera_msgdma_dev->g_buffer_size,GFP_KERNEL);
				}
				// check newly allocated memory ...
				// --------------------------------
				if(g_altera_msgdma_dev->g_kmalloc_ptr == 0){
					 up(&g_altera_msgdma_dev->dev_sem);
					 return -ENOMEM; /* Out of memory - 12 */
				}
#ifdef CONST_DEBUG_EXTENDED_MODE
				pr_info("IOCTL IO_ALTERA_MSGDMA_SET_BUFFER_SIZE value is: %d",new_buffer_size);
#endif
				break;
		 case IO_ALTERA_MSGDMA_SET_MIN_BLOCK_SIZE:	// SET_MIN_BLOCK_SIZE ...
				if(get_user(new_interval_size, (size_t *)arg) < 0){
						up(&g_altera_msgdma_dev->dev_sem);
						pr_info("altera_msgdma_ioctl get_user - exit \n");
						return -EFAULT;	/* Bad address - 14 */
				}

				if(new_interval_size > 0)
					 g_altera_msgdma_dev->g_min_interval_size = new_interval_size;
#ifdef CONST_DEBUG_EXTENDED_MODE
		    pr_info("IOCTL IO_ALTERA_MSGDMA_SET_MIN_BLOCK_SIZE value is: %d",new_interval_size);
#endif
				break;
			case IO_ALTERA_MSGDMA_STOP:				// IO_ALTERA_MSGDMA_STOP ...
						 altera_msgdma_ret = altera_msgdma_stop(g_altera_msgdma_dev);
#ifdef CONST_DEBUG_EXTENDED_MODE
				pr_info("IOCTL IO_ALTERA_MSGDMA_STOP return value is: %d",altera_msgdma_ret);
#endif
				break;
			case IO_ALTERA_MSGDMA_RESET:			// IO_ALTERA_MSGDMA_RESET ...
					 altera_msgdma_ret = altera_msgdma_reset(g_altera_msgdma_dev);
#ifdef CONST_DEBUG_EXTENDED_MODE
				pr_info("IOCTL IO_ALTERA_MSGDMA_RESET return value is: %d",altera_msgdma_ret);
#endif
				break;
			case IO_ALTERA_MSGDMA_STOP_RESET:		// IO_ALTERA_MSGDMA_STOP_RESET ...
					 altera_msgdma_ret = altera_msgdma_stop(g_altera_msgdma_dev);
#ifdef CONST_DEBUG_EXTENDED_MODE
				   pr_info("IOCTL IO_ALTERA_MSGDMA_STOP_RESET (STOP action) return value 0 is: %d",altera_msgdma_ret);
#endif
					 if(altera_msgdma_ret == 0)
					    altera_msgdma_ret = altera_msgdma_reset(g_altera_msgdma_dev);
#ifdef CONST_DEBUG_EXTENDED_MODE
					pr_info("IOCTL IO_ALTERA_MSGDMA_STOP_RESET (RESET action) return value 1 is: %d",altera_msgdma_ret);
#endif
				break;
			default:
				up(&g_altera_msgdma_dev->dev_sem);
				return -ENOTTY;	 /* 25 - Not a typewriter */
		}

		up(&g_altera_msgdma_dev->dev_sem);

#ifdef CONST_DEBUG_EXTENDED_MODE
			if(g_altera_msgdma_dev->type == 0){
				 pr_info("IOCTL succeeded for Altera MSGDMA device with ID %d (in READ mode) ... \n",g_altera_msgdma_dev->id);
			}else if(g_altera_msgdma_dev->type == 1){
				 pr_info("IOCTL succeeded for Altera MSGDMA device with ID %d (in WRITE mode) ... \n",g_altera_msgdma_dev->id);
			}
#endif

		return 0;
}

// Platform altera_msgdma_write function ...
// -----------------------------------------
static ssize_t altera_msgdma_write(struct file *fp, const char __user *user_bufffer,
																	 size_t count,loff_t *offset){

	struct altera_msgdma_dev *g_altera_msgdma_dev = (struct altera_msgdma_dev *)fp->private_data;

	size_t temp_count = 0;
	size_t this_loop_count = 0;
	size_t last_loop_count = 0;
	uint32_t next_io_buffer_offset = 0; // Init value 0 ...

	dma_addr_t dma_handle = 0;
	dma_addr_t last_dma_handle_0 = 0;
	dma_addr_t last_dma_handle_1 = 0;
	dma_addr_t last_dma_handle_2 = 0;
	dma_addr_t last_dma_handle_3 = 0;

	//  Only for MSGDMA (ST) in write mode !!!
	// ---------------------------------------
  if(g_altera_msgdma_dev->type == 0){
		return -EACCES;          				  /* Permission denied - 13 */
	}

	/* acquire the probe lock */
	if(down_interruptible(&g_altera_msgdma_dev->dev_sem)){
		pr_info("altera_msgdma_write sem interrupted - exit \n");
		return -ERESTARTSYS;
	}

	if(count > g_altera_msgdma_dev->g_buffer_size){
		 count -= (count - g_altera_msgdma_dev->g_buffer_size);
	}

	if(count < g_altera_msgdma_dev->g_min_interval_size){
		// count is smaller than (struct altera_msgdma_dev *)->g_min_interval_size ...
		// ---------------------------------------------------------------------------
		this_loop_count = count;
	}else{
		// Set this_loop_count --> (struct altera_msgdma_dev *)->g_min_interval_size ...
		// -----------------------------------------------------------------------------
		this_loop_count = g_altera_msgdma_dev->g_min_interval_size;
	}

	// Set last_loop_count --> this_loop_count ...
	// -------------------------------------------
	last_loop_count = this_loop_count;

	// Set temp_count --> count ...
	// ----------------------------
	temp_count = count;

	while(temp_count > 0){
		/*
		Write Fill Level
		----------------
		Indicates number of valid descriptor entries in the write command fifo that
		are yet to be executed.
		The fill level read back of 0 indicates that the write master is either idle
		or operating on the last descriptor.
		*/

		while(get_dma_fill_level(g_altera_msgdma_dev) > 2){
					if(wait_event_interruptible(g_altera_msgdma_dev->wait_queue,
																			get_dma_fill_level(g_altera_msgdma_dev)<=2)){
						up(&g_altera_msgdma_dev->dev_sem);
						pr_info("altera_msgdma_write wait interrupted (get_dma_fill_level)- exit \n");
						return -ERESTARTSYS;
					}
		}

		if(last_dma_handle_3 != 0){
			 dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_3,
											  g_altera_msgdma_dev->g_min_interval_size, DMA_TO_DEVICE);
			 last_dma_handle_3 = 0;
		}

		/*
		copy_from_user — copy data from user space ...
		----------------------------------------------
		Returns number of bytes that could not be copied. On success, this will be zero.
		--------------------------------------------------------------------------------
		unsigned long copy_from_user (void * to, const void __user *from, unsigned long n);
		*/

		if(copy_from_user(g_altera_msgdma_dev->g_kmalloc_ptr + next_io_buffer_offset,
											user_bufffer, this_loop_count)){
			 up(&g_altera_msgdma_dev->dev_sem);
			 pr_info("altera_msgdma_write copy_from_user error - exit \n");
			 return -EFAULT; /* Bad address - 14 */
		}

		/*
		https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
		dma_handle = dma_map_single(dev, addr, size, direction);
		*/

		dma_handle = dma_map_single(g_altera_msgdma_dev->pdev_dev,
																g_altera_msgdma_dev->g_kmalloc_ptr + next_io_buffer_offset,
																this_loop_count, DMA_TO_DEVICE);

		if(dma_mapping_error(g_altera_msgdma_dev->pdev_dev,dma_handle)){
			 up(&g_altera_msgdma_dev->dev_sem);
			 pr_info("altera_msgdma_write dma mapping error - exit \n");
			 return -EBUSY; /* Device or resource busy - 16 */
		}

		last_dma_handle_3 = last_dma_handle_2;
		last_dma_handle_2 = last_dma_handle_1;
		last_dma_handle_1 = last_dma_handle_0;
		last_dma_handle_0 = dma_handle;

		// Altera MSGDMA AVALON ST {streaming} dma interface ...
		// -----------------------------------------------------
		/*
		Memory-Mapped to Streaming [MM --> ST]
		--------------------------------------
    - write address must not be written / or 0 ...
		- using standard descriptor format
		*/
		iowrite32(dma_handle,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_READ_ADDRESS_REG); // dma_handle = bus address
		iowrite32(0,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_WRITE_ADDRESS_REG); 				// 0
		iowrite32(this_loop_count,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_LENGTH_REG);
		iowrite32(START_DMA_MASK,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_CONTROL_REG);

		temp_count -= this_loop_count;

		if(temp_count > 0){
			 // Set next_io_buffer_offset (prev)next_io_buffer_offset + this_loop_count(old) ...
			 // --------------------------------------------------------------------------------
			 next_io_buffer_offset+=this_loop_count;
			 user_bufffer+=this_loop_count;

			 if(temp_count < g_altera_msgdma_dev->g_min_interval_size){
					this_loop_count = temp_count;
			 }else{
					this_loop_count = g_altera_msgdma_dev->g_min_interval_size;
			 }

			 // Set last_loop_count --> this_loop_count ...
			 // -------------------------------------------
			 last_loop_count = this_loop_count;
		}
	}


	/* check get_dma_busy(altera_msgdma_dev *) state */
	while(get_dma_busy(g_altera_msgdma_dev)!=0){
			  if(wait_event_interruptible(g_altera_msgdma_dev->wait_queue,
					 													get_dma_busy(g_altera_msgdma_dev)==0)){
					up(&g_altera_msgdma_dev->dev_sem);
					pr_info("altera_msgdma_write wait interrupted (get_dma_busy) - exit \n");
					return -ERESTARTSYS;
			  }
	}

	if(last_dma_handle_3 != 0){
		 dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_3,
											g_altera_msgdma_dev->g_min_interval_size, DMA_TO_DEVICE);
		 last_dma_handle_3 = 0;
	}

	if(last_dma_handle_2 != 0){
		 dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_2,
											g_altera_msgdma_dev->g_min_interval_size, DMA_TO_DEVICE);
		 last_dma_handle_2 = 0;
	}

	if(last_dma_handle_1 != 0){
		 dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_1,
											g_altera_msgdma_dev->g_min_interval_size, DMA_TO_DEVICE);
		 last_dma_handle_1 = 0;
	}

	if(last_dma_handle_0 != 0){
		 dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_0,
											last_loop_count, DMA_TO_DEVICE); // last_loop_count ...
		 last_dma_handle_0 = 0;
	}

	up(&g_altera_msgdma_dev->dev_sem);

	return count;
}

// Platform altera_msgdma_read function ...
// ----------------------------------------
static ssize_t altera_msgdma_read(struct file *fp, char __user *user_bufffer,
																	 size_t count,loff_t *offset){

	struct altera_msgdma_dev *g_altera_msgdma_dev = (struct altera_msgdma_dev *)fp->private_data;

	size_t temp_user_count = 0;
	size_t temp_dma_count = 0;

	size_t this_loop_count = 0;
	size_t prev_loop_count = 0;
	size_t prev_loop_count_1 = 0;

	uint32_t next_user_io_buffer_offset = 0; // Init value 0 ...
	uint32_t next_dma_io_buffer_offset = 0;  // Init value 0 ...

	dma_addr_t dma_handle = 0;
	dma_addr_t last_dma_handle_0 = 0;
	dma_addr_t last_dma_handle_1 = 0;

	//  Only for MSGDMA (ST) in read mode !!!
	// -------------------------------------
	if(g_altera_msgdma_dev->type == 1){
		return -EACCES;          		 /* Permission denied - 13 */
	}

  /* acquire the probe lock */
  if(down_interruptible(&g_altera_msgdma_dev->dev_sem)){
		 pr_info("altera_msgdma_read sem interrupted - exit \n");
		 return -ERESTARTSYS;
	}

	if(count > g_altera_msgdma_dev->g_buffer_size){
		 count -= (count - g_altera_msgdma_dev->g_buffer_size);
	}

	if(count < g_altera_msgdma_dev->g_min_interval_size){
		// count is smaller than (struct altera_msgdma_dev *)->g_min_interval_size ...
		// ---------------------------------------------------------------------------
	  this_loop_count = count;
	}else{
		// Set this_loop_count --> (struct altera_msgdma_dev *)->g_min_interval_size ...
	  // -----------------------------------------------------------------------------
		this_loop_count = g_altera_msgdma_dev->g_min_interval_size;
  }

	// Set prev_loop_count & prev_loop_count_1 --> this_loop_count ...
	// ---------------------------------------------------------------
	prev_loop_count_1 = this_loop_count;
  prev_loop_count = this_loop_count;

  // Set temp_user_count & temp_dma_count --> count ...
	// --------------------------------------------------
	temp_user_count = count;
	temp_dma_count = count;

	/*
	https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
	dma_handle = dma_map_single(dev, addr, size, direction);
	*/

	dma_handle = dma_map_single(g_altera_msgdma_dev->pdev_dev,
															g_altera_msgdma_dev->g_kmalloc_ptr + next_dma_io_buffer_offset,
															this_loop_count, DMA_FROM_DEVICE);

	if(dma_mapping_error(g_altera_msgdma_dev->pdev_dev,dma_handle)){
		 up(&g_altera_msgdma_dev->dev_sem);
		 pr_info("altera_msgdma_read dma mapping error - exit \n");
		 return -EBUSY; /* Device or resource busy - 16 */
	}

	last_dma_handle_1 = last_dma_handle_0;
	last_dma_handle_0 = dma_handle;

	// Altera MSGDMA AVALON ST {streaming} dma interface ...
	// -----------------------------------------------------
	/*
	Memory-Mapped to Streaming [ST --> MM]
	--------------------------------------
	- read address must not be written / or 0 ...
	- using standard descriptor format
	*/

	iowrite32(0,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_READ_ADDRESS_REG); 						// 0 ...
	iowrite32(dma_handle,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_WRITE_ADDRESS_REG);	// dma_handle = bus address ...
	iowrite32(this_loop_count,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_LENGTH_REG);
	iowrite32(START_DMA_MASK,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_CONTROL_REG);

	temp_dma_count-= this_loop_count;

	// Set prev_loop_count --> this_loop_count ...
	// -------------------------------------------
	prev_loop_count_1 = prev_loop_count;
	prev_loop_count = this_loop_count;

	if(temp_dma_count > 0){
	 	 next_dma_io_buffer_offset+=this_loop_count;

		 if(temp_dma_count < g_altera_msgdma_dev->g_min_interval_size){
				this_loop_count = temp_dma_count;
		 }else{
				this_loop_count = g_altera_msgdma_dev->g_min_interval_size;
		 }
	 }

	 // temp_user_count
	 while(temp_user_count > 0){
		 // temp_dma_count is always smaller than temp_user_count ...
		 if(temp_dma_count > 0){
			 /*
			 https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
			 dma_handle = dma_map_single(dev, addr, size, direction);
			 */

			 dma_handle = dma_map_single(g_altera_msgdma_dev->pdev_dev,
																	 g_altera_msgdma_dev->g_kmalloc_ptr + next_dma_io_buffer_offset,
																	 this_loop_count, DMA_FROM_DEVICE);

			 if(dma_mapping_error(g_altera_msgdma_dev->pdev_dev,dma_handle)){
					up(&g_altera_msgdma_dev->dev_sem);
					pr_info("altera_msgdma_read dma mapping error - exit \n");
					return -EBUSY; /* Device or resource busy - 16 */
			 }

			 last_dma_handle_1 = last_dma_handle_0;
			 last_dma_handle_0 = dma_handle;

			 // Altera MSGDMA AVALON ST {streaming} dma interface ...
			 // -----------------------------------------------------
			 /*
			 Memory-Mapped to Streaming [ST --> MM]
			 --------------------------------------
			 - read address must not be written / or 0 ...
			 - using standard descriptor format
			 */

			 iowrite32(0,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_READ_ADDRESS_REG); // 0
			 iowrite32(dma_handle,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_WRITE_ADDRESS_REG); // dma_handle - bus address ...
			 iowrite32(this_loop_count,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_LENGTH_REG);
			 iowrite32(START_DMA_MASK,g_altera_msgdma_dev->g_ioremap_desc_addr + DESC_CONTROL_REG);

			 temp_dma_count-= this_loop_count;

			 // Set prev_loop_count & prev_loop_count_1--> this_loop_count ...
			 // --------------------------------------------------------------
			 prev_loop_count_1 = prev_loop_count;
			 prev_loop_count = this_loop_count;

			 if(temp_dma_count > 0){
					next_dma_io_buffer_offset+=this_loop_count;

					if(temp_dma_count < g_altera_msgdma_dev->g_min_interval_size){
						 this_loop_count = temp_dma_count;
					}else{
						 this_loop_count = g_altera_msgdma_dev->g_min_interval_size;
					}
				}
				// END OF IF SECTIOn - TRUE ...
		 }else{
			 // Set prev_loop_count_1--> this_loop_count ...
			 // --------------------------------------------
			 prev_loop_count_1 = prev_loop_count;

			 /*check get_dma_busy(altera_msgdma_dev *) state */
			 while(get_dma_busy(g_altera_msgdma_dev)!=0){
				 	if(wait_event_interruptible(g_altera_msgdma_dev->wait_queue,
																			get_dma_busy(g_altera_msgdma_dev)==0)){
						 up(&g_altera_msgdma_dev->dev_sem);
						 pr_info("altera_msgdma_read wait interrupted (get_dma_busy) - exit \n");
						 return -ERESTARTSYS;
					}
			 }
			 // END OF IF SECTIOn - FALSE (ELSE) ...
		 }

		 /*
		 ---------------
		 Read Fill Level
		 ---------------
		 Indicates number of valid descriptor entries in the read command fifo
		 that are yet to be executed.
		 The fill level read back of 0 indicates that the read master is either
		 idle or operating on the last descriptor.
		 */

		 while(get_dma_fill_level(g_altera_msgdma_dev) > 0){
					 if(wait_event_interruptible(g_altera_msgdma_dev->wait_queue,
																			 get_dma_fill_level(g_altera_msgdma_dev)==0)){
						 up(&g_altera_msgdma_dev->dev_sem);
						 pr_info("altera_msgdma_read wait interrupted (get_dma_fill_level) - exit \n");
						 return -ERESTARTSYS;
					 }
		 }

		 if(last_dma_handle_1 != 0){
				dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_1,
												 prev_loop_count_1, DMA_FROM_DEVICE);
				last_dma_handle_1 = 0;
		 } else if(last_dma_handle_0 != 0){
				dma_unmap_single(g_altera_msgdma_dev->pdev_dev, last_dma_handle_0,
												 prev_loop_count_1, DMA_FROM_DEVICE);
				last_dma_handle_0 = 0;
		 }

		 /*
		 copy_to_user — copy a block of data into user space  ...
		 --------------------------------------------------------
		 Returns number of bytes that could not be copied. On success, this will be zero.
		 --------------------------------------------------------------------------------
		 unsigned long copy_to_user (void __user *to,const void *from,unsigned long n);
		 */

		 if(copy_to_user(user_bufffer,g_altera_msgdma_dev->g_kmalloc_ptr +
			 							next_user_io_buffer_offset, prev_loop_count_1)){
				up(&g_altera_msgdma_dev->dev_sem);
				pr_info("altera_msgdma_read copy_to_user error - exit \n");
				return -EFAULT; /* Bad address - 14 */
		 }

		 if(temp_user_count > 0){
			  temp_user_count-=prev_loop_count_1;
			  user_bufffer+=prev_loop_count_1;
			  next_user_io_buffer_offset+= prev_loop_count_1;
		 }
	 }

	 up(&g_altera_msgdma_dev->dev_sem);

	 return count;
}

// file_operations altera_msgdma_fops (file operation) structure ...
// -----------------------------------------------------------------
static const struct file_operations altera_msgdma_fops = {
	.owner = THIS_MODULE,
	.open = altera_msgdma_open,
	.release = altera_msgdma_release,
	.read = altera_msgdma_read,
	.write = altera_msgdma_write,
	.unlocked_ioctl = altera_msgdma_ioctl,
};

// Platform interrupt (isr) function ...
// -------------------------------------
irqreturn_t sgdma_driver_interrupt_handler(int irq, void *dev_id){

 	struct altera_msgdma_dev *g_altera_msgdma_dev = (struct altera_msgdma_dev *)dev_id;

  spin_lock(&g_irq_lock);

  if(g_altera_msgdma_dev->g_sgdma_irq != irq){
     spin_unlock(&g_irq_lock);
     return IRQ_NONE;
  }

  iowrite32(ALTERA_MSGDMA_CSR_IRQ_SET_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_STATUS_REG);
  spin_unlock(&g_irq_lock);

  // Wake up event ...
  // -----------------
  wake_up_interruptible(&g_altera_msgdma_dev->wait_queue);

  return IRQ_HANDLED;
}

// Platform_probe function ...
// ---------------------------
static int platform_probe(struct platform_device *pdev){

  int ret_val = 0;
  int irq = 0;

  uint32_t dma_status = 0;
  uint32_t dma_control = 0;

  char sgdma_name_csr_region[CONST_NAME_BUF_SIZE];
  char sgdma_name_desc_region[CONST_NAME_BUF_SIZE];
  const char *string_property = NULL;

  struct resource *r = 0;
  struct resource *altera_sgdma_csr_mem_region = 0;
  struct resource *altera_sgdma_desc_mem_region = 0;
  struct altera_msgdma_dev *g_altera_msgdma_dev = 0;

  ret_val = -EBUSY;   /* Device or resource busy - 16 */

  /* acquire the probe lock */
  if(down_interruptible(&g_dev_probe_sem)){
    return -ERESTARTSYS;
  }

  /* allocate mem for altera_msgdma_dev struct */
  g_altera_msgdma_dev = kzalloc(sizeof(struct altera_msgdma_dev),GFP_KERNEL);
  if(g_altera_msgdma_dev == NULL){
    pr_err("error - kzalloc failed - could not allocate memory \n");
    ret_val = -ENOMEM;    /* Out of memory - 12 */
    goto bad_exit_kfree_0;
  }

  /* get msgdma string property - msgdma type - read or write
  string_property = "msgdma_read"
  or
  string_property = "msgdma_write"
  */

  scnprintf(sgdma_name_csr_region,CONST_NAME_BUF_SIZE,"sgdma_csr_hw_region%d",
            g_dev_index);

  if(of_property_read_string(pdev->dev.of_node,"string-property",&string_property)!=0){
    pr_err("error - DT string_property does not exist \n");
    ret_val = -ENODEV;   /* No such device - 19 */
    goto bad_exit_return;
  }

  if(!strcmp(string_property,"msgdma_read")){
    g_altera_msgdma_dev->type = MSGDMA_READ;
    scnprintf(g_altera_msgdma_dev->name,CONST_NAME_BUF_SIZE,"altera_msgdma_rd%d",
              g_dev_read_index);
  }else if(!strcmp(string_property,"msgdma_write")){
    g_altera_msgdma_dev->type = MSGDMA_WRITE;
    scnprintf(g_altera_msgdma_dev->name,CONST_NAME_BUF_SIZE,"altera_msgdma_wr%d",
              g_dev_write_index);
  }else{
    pr_err("error - DT string_property does not contain right string ...\n");
    ret_val = -ENODEV;   /* No such device - 19 */
    goto bad_exit_return;
  }

  scnprintf(sgdma_name_csr_region,CONST_NAME_BUF_SIZE,"sgdma_csr_hw_region%d",
            g_dev_index);
  scnprintf(sgdma_name_desc_region,CONST_NAME_BUF_SIZE,"sgdma_desc_hw_region%d",
            g_dev_index);

  g_altera_msgdma_dev->id = g_dev_index;
  g_altera_msgdma_dev->opened = 0;        // 0 --> closed ...

  ret_val = -EINVAL;  /* Invalid argument - 22 */

  // ------------------------------------------------------
  // Get resource 0 - SGDMA CSR registers memory region ...
  // ------------------------------------------------------
  r = platform_get_resource(pdev,IORESOURCE_MEM,0);
  if(r == NULL){
    pr_err("error - IORESOURCE_MEM 0 does not exist \n");
    goto bad_exit_return;
  }

  g_altera_msgdma_dev->g_sgdma_csr_addr = r->start;         /* csr register base address*/
  g_altera_msgdma_dev->g_sgdma_csr_size = resource_size(r); /* csr register region size*/

  ret_val = -EBUSY;  /* Device or resource busy - 16 */

  // Reserve CSR memory region ...
  // -----------------------------
  altera_sgdma_csr_mem_region = request_mem_region(g_altera_msgdma_dev->g_sgdma_csr_addr,
                                                   g_altera_msgdma_dev->g_sgdma_csr_size,
                                                   sgdma_name_csr_region); // "sgdma_csr_hw_region%d"
  if(altera_sgdma_csr_mem_region == NULL){
     pr_err("error - request_mem_region failed - altera_sgdma_csr_mem_region \n");
     goto bad_exit_return;
  }

  // ioremap CSR memory region ...
  // -----------------------------
  g_altera_msgdma_dev->g_ioremap_csr_addr = ioremap(g_altera_msgdma_dev->g_sgdma_csr_addr,
                                                    g_altera_msgdma_dev->g_sgdma_csr_size);

  if(g_altera_msgdma_dev->g_ioremap_csr_addr == NULL){
     pr_err("error - ioremap failed - altera_sgdma_csr_mem_region \n");
     goto bad_exit_release_mem_region_csr;
  }

  // Initialize the DMA controller ...
  // ---------------------------------
	dma_status = ioread32(g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_STATUS_REG);

  if ((dma_status & (ALTERA_MSGDMA_CSR_BUSY_MASK |
                     ALTERA_MSGDMA_CSR_STOP_STATE_MASK |
			               ALTERA_MSGDMA_CSR_RESET_STATE_MASK |
			               ALTERA_MSGDMA_CSR_IRQ_SET_MASK)) != 0) {
  		pr_err("error - initial dma status set unexpected: 0x%08X\n",dma_status);
  		goto bad_exit_release_mem_region_csr;
	}

  if ((dma_status & ALTERA_MSGDMA_CSR_DESCRIPTOR_BUFFER_EMPTY_MASK) == 0) {
      pr_err("error - initial dma status cleared unexpected: 0x%08X\n",dma_status);
      goto bad_exit_release_mem_region_csr;
  }

  dma_control = ioread32(g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_CONTROL_REG);

  if ((dma_control & (ALTERA_MSGDMA_CSR_STOP_MASK |
                      ALTERA_MSGDMA_CSR_RESET_MASK |
                      ALTERA_MSGDMA_CSR_STOP_ON_ERROR_MASK |
                      ALTERA_MSGDMA_CSR_STOP_ON_EARLY_TERMINATION_MASK |
                      ALTERA_MSGDMA_CSR_GLOBAL_INTERRUPT_MASK |
                      ALTERA_MSGDMA_CSR_STOP_DESCRIPTORS_MASK)) != 0) {
      pr_err("error - initial dma control set unexpected: 0x%08X\n",dma_control);
      goto bad_exit_release_mem_region_csr;
  }

  ret_val = -EINVAL;  /* Invalid argument - 22 */

  // -------------------------------------------------------
  // Get resource 1 - SGDMA DESC registers memory region ...
  // -------------------------------------------------------
  r = platform_get_resource(pdev,IORESOURCE_MEM,1);
  if(r == NULL){
    pr_err("error - IORESOURCE_MEM 1 does not exist \n");
    goto bad_exit_iounmap_csr;
  }

  g_altera_msgdma_dev->g_sgdma_desc_addr = r->start;         /* desc register base address*/
  g_altera_msgdma_dev->g_sgdma_desc_size = resource_size(r); /* desc register region size*/

  ret_val = -EBUSY; /* Device or resource busy - 16 */

  // Reserve DESC memory region ...
  // -----------------------------
  altera_sgdma_desc_mem_region = request_mem_region(g_altera_msgdma_dev->g_sgdma_desc_addr,
                                                    g_altera_msgdma_dev->g_sgdma_desc_size,
                                                    sgdma_name_desc_region); // "sgdma_name_desc_region%d"
  if(altera_sgdma_desc_mem_region == NULL){
     pr_err("error - request_mem_region failed - altera_sgdma_desc_mem_region \n");
     goto bad_exit_iounmap_csr;
  }

  // ioremap DESC memory region ...
  // -----------------------------
  g_altera_msgdma_dev->g_ioremap_desc_addr = ioremap(g_altera_msgdma_dev->g_sgdma_desc_addr,
                                                     g_altera_msgdma_dev->g_sgdma_desc_size);

  if(g_altera_msgdma_dev->g_ioremap_desc_addr == NULL){
     pr_err("error - ioremap failed - altera_sgdma_desc_mem_region \n");
     goto bad_exit_release_mem_region_desc;
  }

  // --------------------------
  // Get IRQ SGDMA resource ...
  // --------------------------
  irq = platform_get_irq(pdev,0); // 0 - get first interrupt ...
  if(irq < 0){
     pr_err("error - irq not available \n");
     goto bad_exit_iounmap_desc;
  }

  g_altera_msgdma_dev->g_sgdma_irq = irq; // Set irq ...

  ret_val = -ENOMEM;    /* Out of memory - 12 */

  // ----------------------------------------
  // Allocate memory buffer using kmalloc ...
  // ----------------------------------------
	g_altera_msgdma_dev->g_buffer_size = CONST_KMALLOC_MULTIPLE*PAGE_SIZE; // Set initial buffer size ...
	g_altera_msgdma_dev->g_min_interval_size = CONST_MIN_BLOCK_SIZE;
  g_altera_msgdma_dev->g_kmalloc_ptr = kmalloc(g_altera_msgdma_dev->g_buffer_size,GFP_KERNEL);

  if(g_altera_msgdma_dev->g_kmalloc_ptr == NULL){
     pr_err("error - kmalloc failed \n");
     goto bad_exit_iounmap_desc;
  }

  g_altera_msgdma_dev->pdev_dev = &pdev->dev; // Set pdev_dev ...

  ret_val = -EBUSY; /* Device or resource busy - 16 */

  // ----------------------------------------------------------------
  // Register interrupt driver + spinlock + semaphore + waitqueue ...
  // ----------------------------------------------------------------
  if(g_dev_index == 0) spin_lock_init(&g_irq_lock); // if g_dev_index == 0 --> first device
  sema_init(&g_altera_msgdma_dev->dev_sem,1);
  init_waitqueue_head(&g_altera_msgdma_dev->wait_queue);

  ret_val = request_irq(g_altera_msgdma_dev->g_sgdma_irq, /* irq number */
                        sgdma_driver_interrupt_handler,   /* irq function */
                        0,                                /* irq flags - 0 - no flags */
                        g_altera_msgdma_dev->name,        /* irq name */
                        g_altera_msgdma_dev);             /* void data - using altera_msgdma_dev */

  if(ret_val){
     pr_err("error - request given irq failed \n");
     goto bad_exit_kfree_1;
  }

  // Enable Altera IP global isr mask ...
  // ------------------------------------
  iowrite32(ALTERA_MSGDMA_CSR_GLOBAL_INTERRUPT_MASK,g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_CONTROL_REG);

  // ----------------------------------------------------------------------
  // Register misc (character) device - MAJOR == 10, with dynamic MINOR ...
  // ----------------------------------------------------------------------
  g_altera_msgdma_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
  g_altera_msgdma_dev->miscdev.name = g_altera_msgdma_dev->name;
  g_altera_msgdma_dev->miscdev.fops = &altera_msgdma_fops;

  ret_val = misc_register(&g_altera_msgdma_dev->miscdev);

  if(ret_val){
     pr_err("error - could not register misc device %s ... \n",g_altera_msgdma_dev->name);
     goto bad_exit_freeirq;
  }

  // ------------------------------
  // Add device to list finally ...
  // ------------------------------
  INIT_LIST_HEAD(&g_altera_msgdma_dev->dev_list);
  list_add(&g_altera_msgdma_dev->dev_list,&g_dev_list);

  if(!strcmp(string_property,"msgdma_read")){
    g_dev_read_index++;
  }else if(!strcmp(string_property,"msgdma_write")){
    g_dev_write_index++;
  }

  g_dev_index++;

  // ---------------------
  // Set platform data ...
  // ---------------------
  platform_set_drvdata(pdev,g_altera_msgdma_dev);

#ifdef CONST_DEBUG_MODE
	if(!strcmp(string_property,"msgdma_read")){
		 pr_info("Registered Altera MSGDMA device %d (in READ mode) ... \n",g_dev_index-1);
	}else if(!strcmp(string_property,"msgdma_write")){
		pr_info("Registered Altera MSGDMA device %d (in WRITE mode) ... \n",g_dev_index-1);
	}
#endif

  up(&g_dev_probe_sem);
  // Return success ...
  // ------------------
  return 0;

  // ----------------------------
  // Error handling goes here ...
  // ----------------------------
  bad_exit_freeirq:
    free_irq(g_altera_msgdma_dev->g_sgdma_irq,g_altera_msgdma_dev);
  bad_exit_kfree_1:
    kfree(g_altera_msgdma_dev->g_kmalloc_ptr);
  bad_exit_iounmap_desc:
    iounmap(g_altera_msgdma_dev->g_ioremap_desc_addr);
  bad_exit_release_mem_region_desc:
    release_mem_region(g_altera_msgdma_dev->g_sgdma_desc_addr,
                       g_altera_msgdma_dev->g_sgdma_desc_size);
  bad_exit_iounmap_csr:
    iounmap(g_altera_msgdma_dev->g_ioremap_csr_addr);
  bad_exit_release_mem_region_csr:
    release_mem_region(g_altera_msgdma_dev->g_sgdma_csr_addr,
                       g_altera_msgdma_dev->g_sgdma_csr_size);
  bad_exit_return:
    kfree(g_altera_msgdma_dev);
  bad_exit_kfree_0:
    up(&g_dev_probe_sem);	/* up(&g_altera_msgdma_dev->dev_sem); */

    return ret_val;
}

// Platform_remove function ...
// ---------------------------
static int platform_remove(struct platform_device *pdev){

  struct altera_msgdma_dev *g_altera_msgdma_dev = 0;

  /* acquire the probe lock */
  if(down_interruptible(&g_dev_probe_sem)){
    return -ERESTARTSYS;
  }

  g_altera_msgdma_dev = (struct altera_msgdma_dev *)platform_get_drvdata(pdev);
  list_del_init(&g_altera_msgdma_dev->dev_list);
  misc_deregister(&g_altera_msgdma_dev->miscdev);

  // Disable Altera IP global isr mask ...
  // -------------------------------------
  iowrite32(0,g_altera_msgdma_dev->g_ioremap_csr_addr + CSR_CONTROL_REG);

  free_irq(g_altera_msgdma_dev->g_sgdma_irq,g_altera_msgdma_dev);
  kfree(g_altera_msgdma_dev->g_kmalloc_ptr);
  iounmap(g_altera_msgdma_dev->g_ioremap_desc_addr);
  release_mem_region(g_altera_msgdma_dev->g_sgdma_desc_addr,
                     g_altera_msgdma_dev->g_sgdma_desc_size);
  iounmap(g_altera_msgdma_dev->g_ioremap_csr_addr);
  release_mem_region(g_altera_msgdma_dev->g_sgdma_csr_addr,
                     g_altera_msgdma_dev->g_sgdma_csr_size);

  switch(g_altera_msgdma_dev->type){
    case MSGDMA_READ: g_dev_read_index--;break;
    case MSGDMA_WRITE: g_dev_write_index--;break;
    default: break;
  }

  g_dev_index--;

  kfree(g_altera_msgdma_dev);

  up(&g_dev_probe_sem);
  return 0;
}

// Device Tree compatibility altr,altera-msgdma-st-1.0 ...
// -------------------------------------------------------
static struct of_device_id driver_dt_ids[] ={
  {
    .compatible = "altr,altera-msgdma-st-1.0"},
  { /* END of the DT table */}
};

MODULE_DEVICE_TABLE(of,driver_dt_ids);

static struct platform_driver msgdma_platform_driver = {
  .probe = platform_probe,    // Platform probe ...
  .remove = platform_remove,  // Platform remove ...
  .driver = {
            .name = "altera_msgdma_st",
            .owner = THIS_MODULE,
            .of_match_table = driver_dt_ids,
          },
};

static int altera_msgdma_init(void){

  int ret_val = 0;

  // Init list ...
  INIT_LIST_HEAD(&g_dev_list);

  /* Init semaphore as MUTEX ...
   Using semaphore as a MUTEX, the value of semaphore is initialized to 1.
   So at any give time only one process can execute the critical section ...
  */
  sema_init(&g_dev_probe_sem,1);

  g_dev_index = 0;
  g_dev_read_index=0; g_dev_write_index=0;

  // Register platform driver ...
  // ----------------------------
  ret_val = platform_driver_register(&msgdma_platform_driver);
  if(ret_val != 0){
    pr_err("error - platform_driver_register returned %d \n",ret_val);
    return ret_val;
  }

	return 0;
}

static void altera_msgdma_exit(void){

  // Unregister platform driver ...
  // ------------------------------
  platform_driver_unregister(&msgdma_platform_driver);

}

module_init(altera_msgdma_init);
module_exit(altera_msgdma_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pavel Fiala <pavelfpl@gmail.com");
MODULE_DESCRIPTION("Intel / Altera Modular SG DMA driver - Avalon streaming");
MODULE_VERSION("1.3");
