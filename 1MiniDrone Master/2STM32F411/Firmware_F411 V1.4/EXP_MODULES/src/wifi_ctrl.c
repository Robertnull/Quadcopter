#include "string.h"
#include <math.h>
#include "wifi_ctrl.h"
#include "uart1.h"
#include "config_param.h"
#include "commander.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * Mobile WiFi Control Driver	
********************************************************************************/

typedef enum
{
	waitForStart,
	waitForData,
	waitForChksum,
	waitForEnd
} WifilinkRxState;

TaskHandle_t wifCtrlTaskHandle = NULL;

static bool isInit = false;
static u8 rawWifiData[8];
static ctrlVal_t wifiCtrl;/*发送到commander姿态控制数据*/

/*wifi电源控制*/
void wifiPowerControl(bool state)
{
	if(state == true)
		WIFI_POWER_ENABLE = true;
	else
		WIFI_POWER_ENABLE = false;
}

/*wifi模块初始化*/
void wifiModuleInit(void)
{
	if(!isInit)	/*首次插上wifi摄像头*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		
		/* 配置wifi电源控制脚输出 */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
				
		wifiPowerControl(true);	
		vTaskDelay(50);
		uart1Init(19200);	/*串口1初始化，波特率固定19200*/
		
		xTaskCreate(wifiLinkTask, "WIFILINK", 150, NULL, 4, &wifCtrlTaskHandle);		/*创建通信连接任务*/
		
		isInit = true;
	}
	else
	{
		wifiPowerControl(true);
		vTaskDelay(50);
		vTaskResume(wifCtrlTaskHandle);
	}	
}

static bool wifiDataCrc(u8 *data)
{
	u8 temp=(data[1]^data[2]^data[3]^data[4]^data[5])&0xff;
	if(temp==data[6])
		return true;
	return false;
}

/*命令解析*/
static void wifiCmdProcess(u8 data)
{
	wifiCmd_t wifiCmd = *(wifiCmd_t*)&data;
	
	if(getCommanderCtrlMode() == true)/*当前四轴飞行为定高模式*/
	{
		if(wifiCmd.keyFlight) /*一键起飞*/
		{
			setCommanderKeyFlight(true);
			setCommanderKeyland(false);	
		}
		if(wifiCmd.keyLand) /*一键降落*/
		{
			setCommanderKeyFlight(false);
			setCommanderKeyland(true);
		}
		if(wifiCmd.emerStop) /*紧急停机*/
		{
			setCommanderKeyFlight(false);
			setCommanderKeyland(false);
			setCommanderEmerStop(true);
		}else
		{
			setCommanderEmerStop(false);
		}
	}
	else/*当前四轴飞行为手动飞模式*/
	{
		setCommanderCtrlMode(0);
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}

	setCommanderFlightmode(wifiCmd.flightMode);
	
	if(wifiCmd.flipOne) /*固定方向翻滚*/
	{
	}
	if(wifiCmd.flipFour) /*4D翻滚*/
	{
	}
	if(wifiCmd.ledControl) /*灯光控制*/
	{		
	}
	if(wifiCmd.gyroCalib) /*陀螺校准*/
	{
	}
}

static void wifiDataHandle(u8 *data)
{
	static u16 lastThrust;
	
	wifiCtrl.roll   = ((float)data[1]-(float)0x80)*0.25f;	/*roll: ±9.5 ±19.2 ±31.7*/
	wifiCtrl.pitch  = ((float)data[2]-(float)0x80)*0.25f;	/*pitch:±9.5 ±19.2 ±31.7*/
	wifiCtrl.yaw    = ((float)data[4]-(float)0x80)*1.6f;	/*yaw : ±203.2*/				
	wifiCtrl.thrust = (u16)data[3] << 8;					/*thrust :0~63356*/
	
	if(wifiCtrl.thrust==32768 && lastThrust<10000)/*手动飞切换到定高*/
	{
		setCommanderCtrlMode(1);
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}
	else if(wifiCtrl.thrust==0 && lastThrust>256)/*定高切换成手动飞*/
	{
		setCommanderCtrlMode(0);
		wifiCtrl.thrust = 0;
	}
	lastThrust = wifiCtrl.thrust;

	wifiCmdProcess(data[5]);/*位标志命令解析*/
	flightCtrldataCache(WIFI, wifiCtrl);
}

void wifiLinkTask(void *param)
{
	u8 c;
	u8 dataIndex=0;
	WifilinkRxState rxState=waitForStart;

	while(1)
	{
		if(getModuleID() != WIFI_CAMERA)	/*移除wifi摄像头*/
		{
			vTaskSuspend(wifCtrlTaskHandle);
		}
		
		if (uart1GetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStart:
					if(c == 0x66)					/*起始符正确*/
					{
						dataIndex=1;
						rawWifiData[0] = c;
						rxState = waitForData;
					} else							/*起始符错误*/
					{
						rxState = waitForStart;
					}
					break;				
				case waitForData:
					rawWifiData[dataIndex] = c;
					dataIndex++;
					if (dataIndex == 6)				/*数据接收完成，校验*/
					{
						rxState = waitForChksum;
					}
					break;
				case waitForChksum:
					rawWifiData[6] = c;
					if (wifiDataCrc(rawWifiData))	/*校验正确，判断结束符*/
					{
						rxState = waitForEnd;
					} else
					{
						rxState = waitForStart;		/*校验错误*/
					}
					break;
				case waitForEnd:
					if (c == 0x99)					/*结束符正确*/
					{
						rawWifiData[7] = c;
						wifiDataHandle(rawWifiData);/*处理接收到的数据*/
					} else
					{
						rxState = waitForStart;		/*结束符错误*/
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStart;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*超时处理*/
		{
			rxState = waitForStart;
		}
	}
}






