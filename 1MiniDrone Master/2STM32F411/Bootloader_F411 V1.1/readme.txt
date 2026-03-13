
MiniDrone structure:

         HEAD
	  M4  ¡ü  M1
	   \     /
		\   /
		 \ /
		 / \
		/   \
	   /     \
	  M3     M2
	
Hardware Resources:
	1. MCU: STM32F411CEU6 (Flash: 512 KB, RAM: 128 KB, System Clock: 96 MHz)
	2. 9-axis IMU: MPU9250 connected to I2C1(IMU_SCL:PB8, IMU_SDA:PB9, Communication: Software I2C (bit-banging)) 
	3. Barometer: BMP280 connected to the auxiliary I2C bus of MPU9250 (AUX_DA, AUX_CL)
	4. Wireless communication module: NRF51822 connected to UART2(NRF_RX:PA2, NRF_TX:PA3, NRF_FLOW_CTRL:PA0) 
	5. Motor1: TIM4_CH2 (PB7)
	6. Motor2: TIM4_CH1 (PB6)
	7. Motor3: TIM2_CH3 (PB10)
	8. Motor4: TIM2_CH1 (PA5)
	9. LED_BLUE_L: PB12	(Blue LED corresponding to MOTOR3, Active high)
	10. LED_GREEN_L: PA6	(Green LED corresponding to MOTOR4, Active low)
	11. LED_RED_L: PA7		(Red LED corresponding to MOTOR4, Active low)
	12. LED_GREEN_R: PC13	(Green LED corresponding to MOTOR1, Active low)
	13. LED_RED_R: PC14	(Red LED corresponding to MOTOR1, Active low)
	14. Extended I2C interface(SDA:PB4, SCL:PB5) 
	15. Extended SPI2 interface(SCK:PB13, MISO:PB14, MOSI:PB15)  
	16. Extended UART1 interface(RX1:PB3, TX1:PA15, Used for external camera module)  
	17. Extended GPIO(CS0:PC15, CS1:PB0, CS2:PB1, CS3:PA8). 	
	18. USB Slave interface(USB_ID:PA10, USB_DM:PA11, USB_DP:PA12)

Experimental Behavior:
	After power-on, MiniDrone performs the following motor test: MOTOR1¨CMOTOR4 rotate sequentially at 20% duty cycle for 50 ms, then all motors stop.
	
	LED Indicators:
		LED_BLUE_L: Charging indicator, blinks with a 1-second period while charging, Solid ON after charging stops;
		LED_GREEN_L: Wireless communication data reception indicator, flashes once when one packet is received;
		LED_RED_L: Wireless communication data transmission indicator, flashes once when one packet is sent;
		LED_GREEN_R: Sensor calibration indicator
					 Slow blink (2 s period) ¡ú sensors not calibrated
					 Fast blink (0.5 s period) ¡ú calibration completed;
		LED_RED_R: Low battery indicator, Solid ON indicates low battery level, stop flying and recharge the battery;
		LED_RED_LºÍLED_RED_R both solid ON, Indicates MiniFly has entered an error state; 

Notes:
	Before downloading or debugging the firmware: Set the programmer switch to STM32 mode.
	Bootloader start address (BOOTLOADER_ADDR) 0x08000000;
	Firmware start address (FIRMWARE_START_ADDR) 	0x08008000;