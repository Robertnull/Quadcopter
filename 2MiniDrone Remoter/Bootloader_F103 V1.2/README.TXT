
Hardware Resources£º
	1,MCU:STM32F103C8T6 (FLAH:64K, RAM:20K, System Clock:72MHz) 
	2,Button (KEY_L:PB11)
	3,Running Indicator LED (BLUE_LED1:PB3)

Observed Behavior£º
	Press the KEY_L button on the remote controller to power on, the device enters Bootloader mode when the running indicator LED (BLUE_LED1) starts blinking.
	
Notes:
	Before code download and debugging, set the programmer switch to STM32 mode.
	Bootloader start address (BOOTLOADER_START_ADDR): 0x08000000
	Firmware start address (FIRMWARE_START_ADDR): 0x08002400






































