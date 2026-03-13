
Hardware Resources£º
	1,MCU:STM32F103C8T6 (FLAH:64K, RAM:20K, System Clock:72MHz) 
	2,Wireless Communication NRF24L01+(CE:PA3, CSN:PA4, SCK:PA5, MISO:PA5, MOSI:PA6, IRQ:PC13) 
	3,Throttle Joystick£¨ADC_YAW:PB0, ADC_THRUST: PB1£©
	4,Direction Joystick£¨ADC_ROLL:PA1, ADC_PITCH: PA2£©
	5,OLED(MOSI:PB15, RST:PB14, CLK:PB13, DC:PB12)
	6,Buttons(KEY_J1£ºPB10, KEY_J2£ºPA8, KEY_L:PB11, KEY_R:PC15)
	7,Communication LEDs(RED_LED0:PB7, BLUE_LED1:PB3)
	8,Buzzer£¨PC14£©

Observed Behavior£º
	After powering on the remote controller, the main interface is displayed. The buzzer emits a single ¡°beep,¡± and the quadcopter is ready to fly.
	
	LED Indicators£º
	    RED_LED0 ON: Communication with the quadcopter failed
	    BLUE_LED1 ON: Communication with the quadcopter successful

Notes:
	Before code download and debugging, set the programmer switch to STM32 mode.
	Bootloader start address (BOOTLOADER_START_ADDR): 0x08000000
	Firmware start address (FIRMWARE_START_ADDR): 0x08002400






































