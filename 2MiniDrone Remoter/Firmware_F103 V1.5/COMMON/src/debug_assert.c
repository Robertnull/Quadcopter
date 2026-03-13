#include <stdint.h>
#include "debug_assert.h"
#include "led.h"
#include "sys.h"
#include "usart.h"
/********************************************************************************	 
 * Assert driver
********************************************************************************/

void assertFail(char *exp, char *file, int line)
{
	//printf("Assert failed %s:%d\n", file, line);
	while (1);
}


