# Description
The RTDM uart driver for Bealgebone Black. This driver is tested under [kernel from offical repository which already patched with xenomai 3](https://github.com/beagleboard/linux/tree/4.4-xenomai/).

Both of the baudrate and the data format are non configurable, they are fixed to be 3000000bps and 8N1 respectively.

The driver inside only enable the uart4 clock, so uart1, uart2, uart3 and uart5 are unsupported yet.

# Device tree example
To make the driver recognized by the kernel, you need modify the device tree at kernel compile time.
`vim arch/arm/boot/dts/am335x-boneblack.dts`
````
&am33xx_pinmux {
  ...
  <some other things>
  ...
  uart4_pins: pinmux_uart4_pins {
		pinctrl-single,pins = <
			AM33XX_IOPAD(0x870, PIN_INPUT_PULLUP | MUX_MODE6)	/* gpmc_wait0.uart4_rxd */
			AM33XX_IOPAD(0x874, PIN_OUTPUT_PULLDOWN | MUX_MODE6)	/* gpmc_wpn.uart4_txd */
		>;
	};
};

&uart4 {
	compatible = "xeno_bbb_uart"
	pinctrl-names = "default";
	pinctrl-0 = <&uart4_pins>;

	status = "okay";
};
```

