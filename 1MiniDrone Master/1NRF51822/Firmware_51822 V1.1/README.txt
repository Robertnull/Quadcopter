
Hardware Resources£º
	1. MCU:NRF51822 (FLAH:256K, RAM:16K, System Clock:16MHz) 
	2. Wireless Communication Pins: PIN19,PIN20 
	3. Blue Power LED: LED_BLUE_R connected to PIN8
	4. Power Button: connected to PIN17
	5. LDO Power Control Pin: connected to PIN29
	6. Battery Charging Status Detection Pin: PIN1
	7. Battery Voltage Measurement Pins: PIN4, PIN5
	8. STM32 BOOT0 Control Pin: PIN9
	9. STM32 NRST Control Pin: PIN3
	10.UART Communication Pins: PIN0 (RX), PIN2 (RTS), PIN30 (TX)
	11.USB Insertion Detection Pin: PIN7

Observed Behavior:
	Short press the power button: turns on the device. After boot, the M2 arm power indicator LED (LED_BLUE_R) stays on.
	Long press while powered on: wait until the power LED blinks, and the wireless communication configuration parameters will reset to default.
	Long press while powered off: wait until the power LED blinks, and the STM32 enters bootloader mode.


Notes:
	The NRF51822 currently mainly handles wireless communication and power management.
	To use Bluetooth functionality, must continue development.
	Before code download and debugging, set the programmer switch to the NRF51xx mode.
