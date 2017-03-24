obj-m		:= bbb_uart.o
KDIR := <YOUR_PATH>/linux-4.4-xenomai
PWD		:= $(shell pwd)
EXTRA_CFLAGS    := -I$(KDIR)/include/xenomai/ -I$(PWD)

all:
	make -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean
