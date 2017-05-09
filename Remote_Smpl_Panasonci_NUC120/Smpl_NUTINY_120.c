/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//说明：采集的数据采用PDMA0通道从UART1中发送出去

//---------------Include files-------------------------//
#include "NUC1xx.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvTIMER.h"
#include "Driver\DrvPDMA.h"
#include "Driver\DrvUSB.h"
#include "Driver\DrvPWM.h"

//---------------Defines-------------------------//
#define REMOTE_DEVICE_ID		0x12
//#define _NETWORK_MODE_
#define _DEFAULT_MODE_
//#define _CHANGFENG_MODE_

#define TIMER0_FREQ         2500   //改进前最高支持7KHz，改进后最高支持10KHz
#define UART0_BAUD          115200  //最高支持921600
#define UART0_DMA_TX_MAX    80
#define PWM_Frequence       1000
//---------------Type define-------------------------//
struct PinSet
{ 
    E_DRVGPIO_PORT  Port;
    int32_t         Num;
};


//---------------Function Prototype-------------------------//

void InitSystem(void); 
void TIMER_Configuration(void);
void UART_Configuration(void);
void Timer0_Callback(void);
void ReadRTS0(int i);
//void SendMIDIDdata(int key, int d, int p);
void PWM_Configuration(int frequence , int pulseratio);

//---------------Variable-------------------------//
uint8_t  buffer[4] = {0X11,0x12,0x13,0x14};

struct PinSet pupin[18]={
// Input GPIO  
{E_GPD,1},{E_GPD,2},{E_GPD,3},{E_GPA,5},
{E_GPA,4},{E_GPA,3},{E_GPA,2},{E_GPA,1},{E_GPA,0},
{E_GPA,12},{E_GPA,13},
//foot botton
{E_GPC,7},{E_GPC,6},{E_GPA,15},
// Output GPIO
{E_GPC,8},
{E_GPC,12},
{E_GPA,9},
{E_GPB,8}
};

void delay_loop(void)
 {
    uint32_t j;
 		for(j=0;j<60;j++);		

 }
 
/*  
 * === FUNCTION ========================================= 
 * Name         : main 
 * Description  : 系统主函数
 * ===================================================== 
 */
int main(void)
{ 
	InitSystem();	
	while(1);
	
}


/*  
 * === FUNCTION ========================================= 
 * Name         : InitSystem 
 * Description  : 系统初始化函数
 * ===================================================== 
 */
void InitSystem(void)
{
    uint32_t i;	
    UNLOCKREG();
    SYSCLK->PWRCON.XTL12M_EN = 1;   //设定12M外部晶振
    DrvSYS_Delay(5000);             //等待时钟就绪
    DrvSYS_SelectPLLSource(E_SYS_EXTERNAL_12M);   //选择12MHz为PLL输入
    DrvSYS_Open(50000000);          //打开50MHz
    LOCKREG();
		  

    /* 下面对GPIO管脚进行配置 */
    for(i=0;i<14;i++)
    {
        DrvGPIO_Open(pupin[i].Port, pupin[i].Num, E_IO_OUTPUT);  //配置输入管脚
    }
    while(i<18)
    {
        DrvGPIO_Open(pupin[i].Port, pupin[i].Num, E_IO_OUTPUT); //配置输出管脚
        i++;
    }	
		
		GPD_0 = 0;
		GPD_1 = 0;	 
		GPD_2 = 0;
		GPD_3 = 0;
		GPB_8 = 0;

		GPC_0 = 0;	
		GPC_1 = 0;
		GPC_2 = 0;
		
		GPC_8 = 1;
		GPC_12 = 0;
		
    UART_Configuration();   //初始化UART
    TIMER_Configuration();  //初始化TIMER    
		PWM_Configuration( PWM_Frequence , 25 );	
}

void PWM_Configuration(int frequence , int pulseratio)
{
S_DRVPWM_TIME_DATA_T sPt;
/* PWM Timer property */ 
sPt.u8Mode = DRVPWM_AUTO_RELOAD_MODE; /*自动重载模式*/ 
sPt.u32Frequency = frequence; // 100 /*PWM 频率 为100HZ即10000us为一周期*/ 
sPt.u8HighPulseRatio = pulseratio; //测试发现实际有误差矫正系数 //25; /* 高脉冲宽度时间所占周期的百分比: 25%*/ 
sPt.i32Inverter = 0; /*反向关闭*/
/* Enable PWM clock */
DrvPWM_Open(); //打开 PWM 时钟并且复位PWM
/* Select PWM engine clock */
//DrvPWM_SelectClockSource(DRVPWM_TIMER0, DRVPWM_EXT_12M);//设置PWM 定时器0 为外部12 MHz crystal 时钟
DrvSYS_SelectIPClockSource(E_SYS_PWM01_CLKSRC,0); //使用外设时注意必须设置该外设的时钟 设置PWM01的时钟源为外部12MHZ 
/* Set PWM Timer0 Configuration */
DrvPWM_SetTimerClk(DRVPWM_TIMER0, &sPt); //配置PWM 定时器0的一些参数 如配置频率/脉冲/模式/逆转功能
/* Enable Output for PWM Timer0 */
DrvPWM_SetTimerIO(DRVPWM_TIMER0, 1); //使能或关闭PWM定时器0对应的IO口输出使能
/* Set PWM pins */
DrvGPIO_InitFunction(E_FUNC_PWM01); //指定多功能引脚 即 PA12，PA13为PWM0和PWM1
/* Enable the PWM Timer 0 */
DrvPWM_Enable(DRVPWM_TIMER0, 1); //使能/关闭PWM定时器0 
}


/*  
 * === FUNCTION ========================================= 
 * Name         : TIMER_Configuration 
 * Description  : TIMER配置函数
 * =====================================================                                                                                                                                                                                         													fdDDFFFFFFFS
 */
void TIMER_Configuration(void)
{
    DrvTIMER_Init();  //初始化定时器
    DrvSYS_SelectIPClockSource(E_SYS_TMR0_CLKSRC,0x00);   //设定TIMER0的时钟源为外部12MHZ 
    DrvTIMER_Open(E_TMR0,TIMER0_FREQ,E_PERIODIC_MODE);  //设定定时器timer0的tick周期，并且启动定时器：TIMER0_FREQ，周期模式
    DrvTIMER_SetTimerEvent(E_TMR0, 1, (TIMER_CALLBACK)Timer0_Callback, 0);  //安装一个定时处理事件到 timer0通道
    DrvTIMER_EnableInt(E_TMR0);   //使能定时器中断 TIMER0->TCSR.IE = 1
    DrvTIMER_Start(E_TMR0); //定时器timer0开始计数 TIMER0->TCSR.CEN = 1;
}

/*  
 * === FUNCTION ========================================= 
 * Name         : UART_Configuration 
 * Description  : UART配置函数
 * ===================================================== 
 */
void UART_Configuration(void)
{
    STR_UART_T param;
  
    /* Select UART Clock Source From 12MHz */
    DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC, 0x00);  //使能UART时钟

    param.u32BaudRate = UART0_BAUD; //波特率
    param.u8cDataBits = DRVUART_DATABITS_8; //数据位
    param.u8cStopBits = DRVUART_STOPBITS_1; //停止位
    param.u8cParity = DRVUART_PARITY_NONE;  //校验位
    param.u8cRxTriggerLevel = DRVUART_FIFO_4BYTES;   //FIFO存储深度63字节
    param.u8TimeOut = 0;  //FIFO超时设定
 
    DrvGPIO_InitFunction(E_FUNC_UART1);   //复用功能引脚设置
    DrvUART_Open(UART_PORT1, &param);   //串口usart1开启、结构体整体赋值
}


/*  
 * === FUNCTION ========================================= 
 * Name         : Timer0_Callback 
 * Description  : Timer0回调函数
 * ===================================================== 
 */
void Timer0_Callback(void)
{	
	
#ifdef _DEFAULT_MODE_
	
	uint8_t command[6]={0x0,0x0,0x0,0x0,0x0,0x0}; //0 0xff 1 命令区分停止和速度 2开关 或者速度 4 0xff
//	DrvUART_Write(UART_PORT1,buffer,4);	
	GPD_0 = 0;   //485收到状态复位

	DrvUART_Read(UART_PORT1,command,4);	
	
//	GPD_0 = 1;
//	DrvUART_Write(UART_PORT1,command,4);
	
	if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 1))         // ff000100   录像打开
  {
		GPC_12 = 1;	//开关管为0 停止录像
	}else if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 0))   //ff000000     录像关闭
	{  
		GPC_12 = 0;	//开关管为1 开始录像
	}  
	
	if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 0))       //ff010030       缩放开始 
	{
		PWM_Configuration( PWM_Frequence , command[3]*3/5 );   //第二次调节占空比则不准确了，使用比例系数实际只能0~~~~60变化		
	 
		// other mode 
		if(command[3] <= 0x20)
		{
			GPC_0 = 1;	//开关管为1 开始录像
			GPC_1 = 0;
			GPC_2 = 0;
		}
		else if((command[3] < 0x30)&&((command[3] > 0x20)))
		{
			GPC_0 = 0;	//开关管为1 开始录像
			GPC_1 = 0;
			GPC_2 = 0;
    }
		else if(command[3] > 0x30)
		{
			GPC_0 = 0;	//开关管为1 开始录像
			GPC_1 = 0;
			GPC_2 = 1;
		}
  }

	#if 0
	//另一种工作模式定速定阻
	if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 2))
	{
		GPD_1 = 1;	//开关管为1 开始录像
		GPD_2 = 0;
		GPD_3 = 0;
		
	} 
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 3))
	{
		GPD_1 = 0;	//开关管为1 开始录像
		GPD_2 = 1;
		GPD_3 = 0;
	} 
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 4))
	{
		GPD_1 = 0;	//开关管为1 开始录像
		GPD_2 = 0;
		GPD_3 = 1;
	} 
	#endif 
#endif 
	
#ifdef _CHANGFENG_MODE_
  uint8_t command[7]={0x0,0x0,0x0,0x0,0x0,0x0,0x0}; //0 0xff 1 命令区分停止和速度 2开关 或者速度 4 0xff
	DrvUART_Read(UART_PORT1,command,7);	
	
		// pelco-d 协议
	if((command[0] == 0xff)&&(command[1] == 0x02)&&(command[2] == 0x00)) 
	{
		if((command[3] == 0x40)&&(command[4] == 0x00)&&(command[5] == 0x00)&&(command[6] == 0x42)) 
		{
				PWM_Configuration( PWM_Frequence , 15 );  
				GPC_0 = 1;	//
			  GPC_1 = 0;
			  GPC_2 = 0;
		}
			
		if((command[3] == 0x20)&&(command[4] == 0x00)&&(command[5] == 0x00)&&(command[6] == 0x22)) 
		{
				PWM_Configuration( PWM_Frequence , 50 );  
		    GPC_0 = 0;	//
			  GPC_1 = 0;
			  GPC_2 = 0;
		}
			
		if((command[3] == 0x00)&&(command[4] == 0x00)&&(command[5] == 0x00)&&(command[6] == 0x02)) 
		{
				PWM_Configuration( PWM_Frequence , 35 );  
				GPC_0 = 0;	//
			  GPC_1 = 0;
			  GPC_2 = 1;
		}
	}		
#endif 
	
#ifdef _NETWORK_MODE_
	uint8_t command[6]={0x0,0x0,0x0,0x0,0x0,0x0}; //0 0xff 1 命令区分停止和速度 2开关 或者速度 4 0xff

	GPD_0 = 0;   //485收到状态复位

	DrvUART_Read(UART_PORT1,command,5);	

	if((command[0] == 0xff)&&(command[4] == REMOTE_DEVICE_ID))  //comand 5bit is device ID number.
	{	

		
		if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 1))         // ff000100   录像打开
		{
			GPC_12 = 1;	//开关管为0 停止录像
		}else if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 0))   //ff000000     录像关闭
		{  
			GPC_12 = 0;	//开关管为1 开始录像
		}else  
		
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 0))       //ff010030       缩放开始 
		{
			PWM_Configuration( PWM_Frequence , command[3]*3/5 );   //第二次调节占空比则不准确了，使用比例系数实际只能0~~~~60变化		
		}
		
		//另一种工作模式定速定阻
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 2))
		{
			GPD_1 = 1;	//开关管为1 开始录像
			GPD_2 = 0;
			GPD_3 = 0;
			
		} 
			if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 3))
		{
			GPD_1 = 0;	//开关管为1 开始录像
			GPD_2 = 1;
			GPD_3 = 0;
		} 
			if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 4))
		{
			GPD_1 = 0;	//开关管为1 开始录像
			GPD_2 = 0;
			GPD_3 = 1;
		} 
		
	}
#endif 	

		
	GPC_8 = 0;
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	delay_loop();
	GPC_8 = 1;  //LED闪烁1次 收到数据
		 
}


