#include <linux/module.h>
#include <linux/platform_device.h>
#include <rtdm/driver.h>
#include <xenomai/rtdm/uapi/rtdm.h>
#include <xenomai/rtdm/uapi/serial.h>
#include <linux/serial_reg.h>

#include "simple_ring_buffer.h"

#define SOC_PRCM_REGS                   (0x44E00000)
#define SOC_PRCM_SIZE                   (0x400 )
#define CM_PER_UART4_CLKCTRL			(1 << 1)

#define UART_OMAP_TLR	7

#define BBB_RX_BUF_SIZ 512
#define BBB_TX_BUF_SIZ 512


struct bbb_uart_device {
	char *name;
	struct rtdm_device rtdm_dev;
	int line;
	int uartclk;
	int	regshift;
	rtdm_irq_t irq_handle;
	unsigned char __iomem	*membase;				/* read/write[bwl] */
	int irq;	
	struct pinctrl		*pins;
	struct simple_ring_buffer rx_buf;	
	struct simple_ring_buffer tx_buf;
	
	uint8_t rx_buf_raw[BBB_RX_BUF_SIZ];
	uint8_t tx_buf_raw[BBB_TX_BUF_SIZ];
		
	rtdm_lock_t rx_lock;	
	rtdm_lock_t tx_lock;	
	
	rtdm_task_t	tx_task;	
	rtdm_event_t tx_event;
};

static inline unsigned int serial_in(struct bbb_uart_device *up, int offset)//read
{
		offset <<= up->regshift;
		return readw(up->membase + offset);
}

static inline void serial_out(struct bbb_uart_device *up, int offset, int value)//write
{
	offset <<= up->regshift;
	writew(value, up->membase + offset);
}


static void bbb_tx_handler(void *arg)
{
	struct bbb_uart_device *bdev = arg;

	u8 w;
    while (!rtdm_task_should_stop()) {
		if (rtdm_event_wait(&bdev->tx_event) < 0)
		    break;
		rtdm_lock_get(&bdev->tx_lock);
		
		while (!simple_ring_buffer_is_empty(&bdev->tx_buf)) {
			w = simple_ring_buffer_get(&bdev->tx_buf);
			//rtdm_printk("@ting: tx %c\n", c);			
			serial_out(bdev, UART_TX, w);
		}
		//rtdm_printk("@ting: tx buffer empty.\n");
		rtdm_lock_put(&bdev->tx_lock);
    }
}

int bbb_uart_open(struct rtdm_fd *fd, int oflags)
{
	// struct bbb_uart_device *bdev = rtdm_fd_to_context(fd)->device->device_data;
	int retval = 0;
	
	rtdm_printk("%s\n", __func__);
	

	
	return retval;
}

void bbb_uart_close(struct rtdm_fd *fd)
{
	// struct bbb_uart_device *bdev = rtdm_fd_to_context(fd)->device->device_data;

	rtdm_printk("%s\n", __func__);
}

ssize_t bbb_uart_read(struct rtdm_fd *fd,
		   void __user *buf, size_t size) 
{
	struct bbb_uart_device *bdev = rtdm_fd_to_context(fd)->device->device_data;
	rtdm_lockctx_t lock_ctx;
	
	if (size > BBB_RX_BUF_SIZ)
		size = BBB_RX_BUF_SIZ;
	
	if (rtdm_fd_is_user(fd) && !rtdm_read_user_ok(fd, buf, size))
		return -EFAULT;

	size = simple_ring_buffer_get_n(&bdev->rx_buf, bdev->rx_buf_raw, size);
	rtdm_safe_copy_to_user(fd, buf, bdev->rx_buf_raw, size);

	rtdm_lock_get_irqsave(&bdev->rx_lock, lock_ctx);

	//rtdm_printk("%s\n", __func__);

	rtdm_lock_put_irqrestore(&bdev->rx_lock, lock_ctx);

	return size;
}

ssize_t bbb_uart_write(struct rtdm_fd *fd,
			const void __user *buf, size_t size)
{
	struct bbb_uart_device *bdev = rtdm_fd_to_context(fd)->device->device_data;
	if (size > BBB_RX_BUF_SIZ)
		size = BBB_RX_BUF_SIZ;
	
	if (rtdm_fd_is_user(fd) && !rtdm_read_user_ok(fd, buf, size))
		return -EFAULT;

	rtdm_lock_get(&bdev->tx_lock);

	// rtdm_printk("%s\n", __func__);

	rtdm_safe_copy_from_user(fd, bdev->tx_buf_raw, buf, size);
	simple_ring_buffer_npush_back(&bdev->tx_buf, bdev->tx_buf_raw, size);
		
	rtdm_event_signal(&bdev->tx_event);
	rtdm_lock_put(&bdev->tx_lock);
	
	return size;
}

static struct rtdm_driver bbb_uart_driver = {
	.profile_info		= RTDM_PROFILE_INFO(bbb_uart,
						    RTDM_CLASS_SERIAL,
						    RTDM_SUBCLASS_16550A,
						    RTSER_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 8,
	.context_size		= sizeof(struct bbb_uart_device),
	.ops = {
		.open		= bbb_uart_open,
		.close		= bbb_uart_close,
		.read_rt	= bbb_uart_read,
		.write_rt	= bbb_uart_write,
	},
};

static int bbb_uart_clock_enable(void)
{
	void __iomem			 *mem;

	mem = ioremap(SOC_PRCM_REGS, SOC_PRCM_SIZE);
	if(!mem)
	{
			printk (KERN_ERR "HI: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
			return -1;
	}
	iowrite32(CM_PER_UART4_CLKCTRL , mem + 0x78);
	return 0;
}

int bbb_uart_irq_handler(rtdm_irq_t *irq_handle)
{
	struct bbb_uart_device *bdev = rtdm_irq_get_arg(irq_handle,
		struct bbb_uart_device);
	
	unsigned int iir, lsr, type;

	// rtdm_irq_disable(irq_handle);
	u8 w;
	
	//rtdm_printk("@ting: %s\n", __func__);
	/* note: << 1
	Other combinations never occur:
	0h = Modem interrupt. Priority = 4.
	1h = THR interrupt. Priority = 3.
	2h = RHR interrupt. Priority = 2.
	3h = Receiver line status error. Priority = 1.
	4h = Reserved
	5h = Reserved
	6h = Rx timeout. Priority = 2.
	7h = Reserved
	8h = Xoff/special character. Priority = 5.
	*/
	rtdm_lock_get(&bdev->rx_lock);
	do {
		iir = serial_in(bdev, UART_IIR);
		lsr = serial_in(bdev, UART_LSR);
		//if (iir & UART_IIR_NO_INT)
		//	break;
		type = iir & 0x3e;
		switch (type) {	
			case UART_IIR_RX_TIMEOUT:
			case UART_IIR_RDI:
				w = serial_in(bdev, UART_RX);
				simple_ring_buffer_push_back(&bdev->rx_buf, w);
				// rtdm_printk("@ting: %s, received: %c.\n", __func__, w);
				break;
		}
	} while (!(iir & UART_IIR_NO_INT));
	rtdm_lock_put(&bdev->rx_lock);
	
	return RTDM_IRQ_HANDLED;
}

static void bbb_uart_clock_disable(void)
{
	// TODO
}

static void bbb_uart_init(struct bbb_uart_device *bdev)
{	
	rtdm_printk(KERN_INFO "@ting: %s\n", __func__);
	
	// disable hw&sw flow control
	serial_out(bdev, UART_LCR, UART_LCR_CONF_MODE_B);	// for accessing efr
	serial_out(bdev, UART_EFR, (1 << 4));				// efr[4] = 1
	serial_out(bdev, UART_LCR, UART_LCR_CONF_MODE_A);	// for accessing mcr
	serial_out(bdev, UART_MCR, (1 << 6));				// mcr[6] = 1

	serial_out(bdev, UART_TCR, 0x0f);					// no meaning
	// stick[0] - dont't configure EFR yet, cause we need keep in TCR_TLR mode
	
	// configure FIFO&DMA
	// -- modeA, and still in TCR_TLR, no need to switch again
	// -- we can access TLR,SCR,FCR
	// -SCR: DMAMODECTL=0, the DMA mode is set with FCR[3]
	serial_out(bdev, UART_SCR, (1 << 7)|(1 << 6));	// [7]RX_TRIG_GRANU1 [6]TX_TRIG_GRANU1 [0]DMA_MODE_CTL
	// 0FCR: no DMA, DMA_MODE = 0, [0]FIFO_EN -- 1, [7:6]RX_FIFO_TRIG,[5:4]TX_FIFO_TRIG -- LSB	

	serial_out(bdev, UART_FCR, (3 << 6) | (3 << 4) | 1);
	// TLR: [7:4]RX_FIFO_TRIG_DMA,[3:0]TX_FIFO_TRIG_DMA -- MSB
	serial_out(bdev, (UART_OMAP_TLR >> bdev->regshift), 0xff);
	rtdm_printk(KERN_INFO "@ting: FIFO&DMA has configured.\n");

	// configure protocl and data format
	// --modeA, and still in TCR_TLR mode, no need to switch again
	// -- we can access DLL,DLH,FCR,MDR1,IER,LCR	
	serial_out(bdev, UART_OMAP_MDR1, 0x7);	// disable UART first
	
	//
	serial_out(bdev, UART_LCR, 0x0);					// set to operation mode for accessing IER
	serial_out(bdev, UART_IER, 0x0);					// set [4]SLEEP_MODE to 0, for accessing LCR
	serial_out(bdev, UART_LCR, UART_LCR_CONF_MODE_B);	// set to modeB for accesing DLL,DLH
	serial_out(bdev, UART_DLL, 0x1);					// set baud rate with 3M, 48M/16/0 = 3M
	serial_out(bdev, UART_DLM, 0);						// set baud rate with 3M
	//

	// -stick[0] - stick EFR here, because this place in operation mode
	serial_out(bdev, UART_EFR, 0);				// UART_EFR[7] AUTO_CTS_EN=0 AUTO_RTS_EN=0 SW_FLOW_CONTROL=0
	// -staick[0] !
	
	// -slected when LCR[7] == 0	
	serial_out(bdev, UART_LCR, 0x0);				// set to operation mode for accessing IER
	serial_out(bdev, UART_IER, 0x1);				// [0]RHRIT [1]THRIT, enable tx,rx interrupts
	//serial_out(bdev, UART_IER, 0x0);				// [0]RHRIT [1]THRIT, enable tx,rx interrupts

	// MODE_SELECT x16, uart enabled
	serial_out(bdev, UART_OMAP_MDR1, UART_OMAP_MDR1_16X_MODE);
	
	// set format, uart start to work
	serial_out(bdev, UART_LCR, 0x3);		// [1:0]CHAR_LENGTH = 0x3(8bit), [2]NB_STOP = 0(1stop), [3]PARITY_EN = 0(No parity)

	/*
	serial_out(bdev, UART_TX, 'h');
	serial_out(bdev, UART_TX, 'e');
	serial_out(bdev, UART_TX, 'l');
	serial_out(bdev, UART_TX, 'l');
	serial_out(bdev, UART_TX, 'o');
	*/
	
}

static void bbb_uart_destroy(struct bbb_uart_device *bdev)
{
	rtdm_printk("@ting: %s\n", __func__);
	// TODO

}

static int bbb_uart_probe(struct platform_device *pdev)
{
	struct bbb_uart_device *bdev;
	struct resource *mem, *irq;
	char *name;
	int retval;	

	rtdm_printk("@ting: %s\n", __func__);
	
	bdev = devm_kzalloc(&pdev->dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;	
	bdev->regshift = 2;
	platform_set_drvdata(pdev, bdev);
	// TODO 
	// get irq
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	// get mem
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	
	
	// set line uartclk
	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				     &bdev->uartclk);
		bdev->line = of_alias_get_id(pdev->dev.of_node, "serial");
		rtdm_printk("pdev->dev.of_node, bdev->line: %d, bdev->uartclk: %d\n", bdev->line, bdev->uartclk);	
	} else {
		bdev->uartclk = 48000000;
		bdev->line = pdev->id;
	}
	rtdm_printk("actually, bdev->line: %d, bdev->uartclk: %d\n", bdev->line, bdev->uartclk);	
	bbb_uart_clock_enable();
	bdev->pins = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(bdev->pins)) {
		rtdm_printk("@ting: fuck pins.\n");
	}
	// [0] ioremap
	if (!devm_request_mem_region(&pdev->dev, mem->start, 
				resource_size(mem), pdev->dev.driver->name))
	{
		goto failed;
	}
	bdev->membase = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!bdev->membase) {
		dev_err(&pdev->dev, "can't ioremap UART\n");
		retval = -ENOMEM;
		goto failed;
	}
	// [0] !

	bbb_uart_init(bdev);
	
	bdev->rtdm_dev.device_data = bdev;
	bdev->rtdm_dev.driver = &bbb_uart_driver;
	bdev->rtdm_dev.label = "rtser%d";	
	name = (char *)(&bdev->rtdm_dev + 1);
	ksformat(name, RTDM_MAX_DEVNAME_LEN, 
		bdev->rtdm_dev.label, bdev->line);
	bdev->name = name;
	rtdm_printk("bdev->name: %s\n", bdev->name);
	retval = rtdm_dev_register(&bdev->rtdm_dev);
	if (retval) {
		rtdm_printk("error occur while rtdm_dev_register, ret=%d\n", retval);
		goto failed;
	}

	retval = simple_ring_buffer_init(&bdev->rx_buf, BBB_RX_BUF_SIZ);
	if (retval) {
		goto failed1;
	}
	
	retval = simple_ring_buffer_init(&bdev->tx_buf, BBB_TX_BUF_SIZ);
	if (retval) {
		goto failed2;
	}

	rtdm_lock_init(&bdev->rx_lock);
	rtdm_lock_init(&bdev->tx_lock);

	// [1] irq_request	
	bdev->irq = irq->start;
	retval = rtdm_irq_request(&bdev->irq_handle,bdev->irq,
		bbb_uart_irq_handler,0,bdev->name, bdev);
	if(retval) {
		rtdm_printk("error in requesting irq\n");
		dev_err(&pdev->dev, "failure requesting irq %i\n", bdev->irq);
		goto failed3;
	}	

	rtdm_event_init(&bdev->tx_event, 0);
	retval = rtdm_task_init(&bdev->tx_task, bdev->name, bbb_tx_handler,
		bdev, RTDM_TASK_HIGHEST_PRIORITY, 0);
	if (retval) {
		rtdm_printk("@ting: error in initiating tx task.\n");
		goto failed4;
	}
	// [1] !
	
	return 0;

failed4:
	rtdm_irq_disable(&bdev->irq_handle);
	rtdm_irq_free(&bdev->irq_handle);
	
failed3:	
	rtdm_dev_unregister(&bdev->rtdm_dev);
	bbb_uart_destroy(bdev);

failed2:
	simple_ring_buffer_destroy(&bdev->tx_buf);

failed1:
	simple_ring_buffer_destroy(&bdev->rx_buf);
		
failed:
	bbb_uart_clock_disable();
	
	return retval;
}

static int bbb_uart_remove(struct platform_device *pdev)
{
	struct bbb_uart_device *bdev = platform_get_drvdata(pdev);

	rtdm_printk("@ting: %s\n", __func__);

	rtdm_task_destroy(&bdev->tx_task);
	
	rtdm_irq_disable(&bdev->irq_handle);
	rtdm_irq_free(&bdev->irq_handle);

	rtdm_dev_unregister(&bdev->rtdm_dev);
	bbb_uart_destroy(bdev);

	simple_ring_buffer_destroy(&bdev->tx_buf);
	
	simple_ring_buffer_destroy(&bdev->rx_buf);
	
	bbb_uart_clock_disable();

	return 0;
}

static const struct of_device_id bbb_uart_dt_ids[] = {
		{ .compatible = "xeno_bbb_uart" },
};

struct platform_driver bbb_uart_platform_driver = {
	.probe = bbb_uart_probe,
	.remove = bbb_uart_remove,
	.driver = {
		.name = "bbb_uart",
		.of_match_table = bbb_uart_dt_ids,
	},
};

module_platform_driver(bbb_uart_platform_driver);

MODULE_AUTHOR("Jadyn Chou <Jadyn@kuibudynamics.com>");
MODULE_DESCRIPTION("Xenomai UART driver for BBB ");
MODULE_LICENSE("GPL");
MODULE_ALIAS("XENO_BBB_UART");
