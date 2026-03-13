#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "pinout.h"
#include "button.h"
#include "systick.h"

/********************************************************************************	 
 * Button driver
********************************************************************************/

#define BUTTON_LONGPRESS_TICK 1000 /*定义判断为长按时间*/

static buttonEvent_e state;

/*按键初始化*/
void buttonInit(buttonEvent_e init)
{
	nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	state = init;
}

/*按键扫描处理*/
void buttonProcess()
{
	static unsigned int lastTick;
	static unsigned int pressedTick;
	static bool pressed = false;

	if (lastTick != systickGetTick())
	{
		lastTick = systickGetTick();

		if(pressed==false && BUTTON_READ()==BUTTON_PRESSED)
		{
			pressed = true;
			pressedTick = systickGetTick();
		} 
		else if(pressed==true) 
		{
			if(BUTTON_READ()==BUTTON_RELEASED)
				pressed = false;
			if ((systickGetTick()-pressedTick) > BUTTON_LONGPRESS_TICK) 
				state = buttonLongPress;
			else if(BUTTON_READ()==BUTTON_RELEASED)
				state = buttonShortPress;
		}
	}
}

/*获取按键状态*/
buttonEvent_e buttonGetState()
{
	buttonEvent_e currentState = state;
	state = buttonIdle;
	return currentState;
}



