
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


Firmware Update History:
	Firmware V1.0 Release	
	
	Firmware V1.1 Release
		1.MCU communicates with sensors via hardware I2C, and sensor data is read using interrupts, improving data acquisition speed.
		2.Added sensor task sensorsTask to read and process sensor data. Processed data is placed into a queue for other tasks to consume, reducing the load of stabilizerTask.
		3.Gyroscope and accelerometer use second-order low-pass filtering, reducing vibration and improving flight stability.
		4.Added extension module management task expModuleMgtTask and file group EXP_MODULES.
		5.Added WiFi aerial photography module ATK-WIFI-MODULE, allowing smartphones/tablets to control MiniFly via WiFi (flight, photo, video).
		6.Added RGB LED ring module ATK-LED-RING with 10 lighting effects for testing. Users can implement custom lighting effects.
		7.Each extension module has a unique ID. Added module_detect.c under the HARDWARE group for module detection.
		8.Refer to the module user manual for details on extension modules.


	Firmware V1.2 Release
		1.Modified attitude estimation: velocity and position estimated from acceleration; corrected using barometer or laser data (when using optical flow module).
		2.Improved altitude hold control using cascade PID (position loop + velocity loop) for better stability and faster response. Default one-key takeoff altitude: 80 cm.
		3.Improved one-key landing: automatic landing and motor shutdown after touchdown.
		4.Added optical flow position hold and laser altitude hold (within 2 m). Requires the optical flow module connected via SPI2 and I2C.
		5.Added support for SPL06 barometer, auto-detecting BMP280 or SPL06.
		6.Added filter2.c and maths.c under COMMON group.
		7.Improved throw-to-fly feature: horizontal throw launch with automatic stabilization.
		8.Extension module detection is now real-time, enabling plug-and-play (only one module supported simultaneously).
		9.Control loop frequencies:
			- Remote control: 100 Hz
			- Optical flow data: 100 Hz
			- Attitude fusion and angle PID: 250 Hz
			- Flip detection and control output: 500 Hz
		10.Fixed bug where throw-to-fly could activate when motors were not armed.
		11.Fixed bug where PID parameters were not saved after modification. Now automatically saved 1.5 s after motors stop.
		12.Refer to module user manuals for extension module details.


	Firmware V1.2.1 Release
		1.Disabled motor thrust battery compensation (motors.h : ENABLE_THRUST_BAT_COMPENSATED) to improve flip stability.

	
	Firmware V1.3 Release
		1.Ported iNAV Flight position-hold algorithm, fusing optical flow and accelerometer data for improved stability and disturbance rejection.
		2.Supports 4 m optical flow module and automatically detects 2 m or 4 m versions.
		3.Allows users to select laser altitude hold or barometer altitude hold when optical flow module is installed.
		4.Added trim data storage, allowing one remote controller to control multiple drones without recalibration.
		5.Added USERDATA upload function, enabling real-time waveform display on the ground station for debugging.
		6.Fixed incorrect initialization of attitude PID parameters.
		7.Fixed unstable behavior in one-key landing.
		
	
	Firmware V1.4 Release
		1.Changed the programming/debugging adapter to ATK Mini-CMSIS-DAP.

					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
















