/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//˵�����ɼ������ݲ���PDMA0ͨ����UART1�з��ͳ�ȥ

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

#define TIMER0_FREQ         2500   //�Ľ�ǰ���֧��7KHz���Ľ������֧��10KHz
#define UART0_BAUD          115200  //���֧��921600
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
 * Description  : ϵͳ������
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
 * Description  : ϵͳ��ʼ������
 * ===================================================== 
 */
void InitSystem(void)
{
    uint32_t i;	
    UNLOCKREG();
    SYSCLK->PWRCON.XTL12M_EN = 1;   //�趨12M�ⲿ����
    DrvSYS_Delay(5000);             //�ȴ�ʱ�Ӿ���
    DrvSYS_SelectPLLSource(E_SYS_EXTERNAL_12M);   //ѡ��12MHzΪPLL����
    DrvSYS_Open(50000000);          //��50MHz
    LOCKREG();
		  

    /* �����GPIO�ܽŽ������� */
    for(i=0;i<14;i++)
    {
        DrvGPIO_Open(pupin[i].Port, pupin[i].Num, E_IO_OUTPUT);  //��������ܽ�
    }
    while(i<18)
    {
        DrvGPIO_Open(pupin[i].Port, pupin[i].Num, E_IO_OUTPUT); //��������ܽ�
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
		
    UART_Configuration();   //��ʼ��UART
    TIMER_Configuration();  //��ʼ��TIMER    
		PWM_Configuration( PWM_Frequence , 25 );	
}

void PWM_Configuration(int frequence , int pulseratio)
{
S_DRVPWM_TIME_DATA_T sPt;
/* PWM Timer property */ 
sPt.u8Mode = DRVPWM_AUTO_RELOAD_MODE; /*�Զ�����ģʽ*/ 
sPt.u32Frequency = frequence; // 100 /*PWM Ƶ�� Ϊ100HZ��10000usΪһ����*/ 
sPt.u8HighPulseRatio = pulseratio; //���Է���ʵ����������ϵ�� //25; /* ��������ʱ����ռ���ڵİٷֱ�: 25%*/ 
sPt.i32Inverter = 0; /*����ر�*/
/* Enable PWM clock */
DrvPWM_Open(); //�� PWM ʱ�Ӳ��Ҹ�λPWM
/* Select PWM engine clock */
//DrvPWM_SelectClockSource(DRVPWM_TIMER0, DRVPWM_EXT_12M);//����PWM ��ʱ��0 Ϊ�ⲿ12 MHz crystal ʱ��
DrvSYS_SelectIPClockSource(E_SYS_PWM01_CLKSRC,0); //ʹ������ʱע��������ø������ʱ�� ����PWM01��ʱ��ԴΪ�ⲿ12MHZ 
/* Set PWM Timer0 Configuration */
DrvPWM_SetTimerClk(DRVPWM_TIMER0, &sPt); //����PWM ��ʱ��0��һЩ���� ������Ƶ��/����/ģʽ/��ת����
/* Enable Output for PWM Timer0 */
DrvPWM_SetTimerIO(DRVPWM_TIMER0, 1); //ʹ�ܻ�ر�PWM��ʱ��0��Ӧ��IO�����ʹ��
/* Set PWM pins */
DrvGPIO_InitFunction(E_FUNC_PWM01); //ָ���๦������ �� PA12��PA13ΪPWM0��PWM1
/* Enable the PWM Timer 0 */
DrvPWM_Enable(DRVPWM_TIMER0, 1); //ʹ��/�ر�PWM��ʱ��0 
}


/*  
 * === FUNCTION ========================================= 
 * Name         : TIMER_Configuration 
 * Description  : TIMER���ú���
 * =====================================================                                                                                                                                                                                         													fdDDFFFFFFFS
 */
void TIMER_Configuration(void)
{
    DrvTIMER_Init();  //��ʼ����ʱ��
    DrvSYS_SelectIPClockSource(E_SYS_TMR0_CLKSRC,0x00);   //�趨TIMER0��ʱ��ԴΪ�ⲿ12MHZ 
    DrvTIMER_Open(E_TMR0,TIMER0_FREQ,E_PERIODIC_MODE);  //�趨��ʱ��timer0��tick���ڣ�����������ʱ����TIMER0_FREQ������ģʽ
    DrvTIMER_SetTimerEvent(E_TMR0, 1, (TIMER_CALLBACK)Timer0_Callback, 0);  //��װһ����ʱ�����¼��� timer0ͨ��
    DrvTIMER_EnableInt(E_TMR0);   //ʹ�ܶ�ʱ���ж� TIMER0->TCSR.IE = 1
    DrvTIMER_Start(E_TMR0); //��ʱ��timer0��ʼ���� TIMER0->TCSR.CEN = 1;
}

/*  
 * === FUNCTION ========================================= 
 * Name         : UART_Configuration 
 * Description  : UART���ú���
 * ===================================================== 
 */
void UART_Configuration(void)
{
    STR_UART_T param;
  
    /* Select UART Clock Source From 12MHz */
    DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC, 0x00);  //ʹ��UARTʱ��

    param.u32BaudRate = UART0_BAUD; //������
    param.u8cDataBits = DRVUART_DATABITS_8; //����λ
    param.u8cStopBits = DRVUART_STOPBITS_1; //ֹͣλ
    param.u8cParity = DRVUART_PARITY_NONE;  //У��λ
    param.u8cRxTriggerLevel = DRVUART_FIFO_4BYTES;   //FIFO�洢���63�ֽ�
    param.u8TimeOut = 0;  //FIFO��ʱ�趨
 
    DrvGPIO_InitFunction(E_FUNC_UART1);   //���ù�����������
    DrvUART_Open(UART_PORT1, &param);   //����usart1�������ṹ�����帳ֵ
}


/*  
 * === FUNCTION ========================================= 
 * Name         : Timer0_Callback 
 * Description  : Timer0�ص�����
 * ===================================================== 
 */
void Timer0_Callback(void)
{	
	
#ifdef _DEFAULT_MODE_
	
	uint8_t command[6]={0x0,0x0,0x0,0x0,0x0,0x0}; //0 0xff 1 ��������ֹͣ���ٶ� 2���� �����ٶ� 4 0xff
//	DrvUART_Write(UART_PORT1,buffer,4);	
	GPD_0 = 0;   //485�յ�״̬��λ

	DrvUART_Read(UART_PORT1,command,4);	
	
//	GPD_0 = 1;
//	DrvUART_Write(UART_PORT1,command,4);
	
	if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 1))         // ff000100   ¼���
  {
		GPC_12 = 1;	//���ع�Ϊ0 ֹͣ¼��
	}else if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 0))   //ff000000     ¼��ر�
	{  
		GPC_12 = 0;	//���ع�Ϊ1 ��ʼ¼��
	}  
	
	if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 0))       //ff010030       ���ſ�ʼ 
	{
		PWM_Configuration( PWM_Frequence , command[3]*3/5 );   //�ڶ��ε���ռ�ձ���׼ȷ�ˣ�ʹ�ñ���ϵ��ʵ��ֻ��0~~~~60�仯		
	 
		// other mode 
		if(command[3] <= 0x20)
		{
			GPC_0 = 1;	//���ع�Ϊ1 ��ʼ¼��
			GPC_1 = 0;
			GPC_2 = 0;
		}
		else if((command[3] < 0x30)&&((command[3] > 0x20)))
		{
			GPC_0 = 0;	//���ع�Ϊ1 ��ʼ¼��
			GPC_1 = 0;
			GPC_2 = 0;
    }
		else if(command[3] > 0x30)
		{
			GPC_0 = 0;	//���ع�Ϊ1 ��ʼ¼��
			GPC_1 = 0;
			GPC_2 = 1;
		}
  }

	#if 0
	//��һ�ֹ���ģʽ���ٶ���
	if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 2))
	{
		GPD_1 = 1;	//���ع�Ϊ1 ��ʼ¼��
		GPD_2 = 0;
		GPD_3 = 0;
		
	} 
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 3))
	{
		GPD_1 = 0;	//���ع�Ϊ1 ��ʼ¼��
		GPD_2 = 1;
		GPD_3 = 0;
	} 
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 4))
	{
		GPD_1 = 0;	//���ع�Ϊ1 ��ʼ¼��
		GPD_2 = 0;
		GPD_3 = 1;
	} 
	#endif 
#endif 
	
#ifdef _CHANGFENG_MODE_
  uint8_t command[7]={0x0,0x0,0x0,0x0,0x0,0x0,0x0}; //0 0xff 1 ��������ֹͣ���ٶ� 2���� �����ٶ� 4 0xff
	DrvUART_Read(UART_PORT1,command,7);	
	
		// pelco-d Э��
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
	uint8_t command[6]={0x0,0x0,0x0,0x0,0x0,0x0}; //0 0xff 1 ��������ֹͣ���ٶ� 2���� �����ٶ� 4 0xff

	GPD_0 = 0;   //485�յ�״̬��λ

	DrvUART_Read(UART_PORT1,command,5);	

	if((command[0] == 0xff)&&(command[4] == REMOTE_DEVICE_ID))  //comand 5bit is device ID number.
	{	

		
		if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 1))         // ff000100   ¼���
		{
			GPC_12 = 1;	//���ع�Ϊ0 ֹͣ¼��
		}else if((command[0] == 0xff)&&(command[1] == 0)&&(command[2] == 0))   //ff000000     ¼��ر�
		{  
			GPC_12 = 0;	//���ع�Ϊ1 ��ʼ¼��
		}else  
		
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 0))       //ff010030       ���ſ�ʼ 
		{
			PWM_Configuration( PWM_Frequence , command[3]*3/5 );   //�ڶ��ε���ռ�ձ���׼ȷ�ˣ�ʹ�ñ���ϵ��ʵ��ֻ��0~~~~60�仯		
		}
		
		//��һ�ֹ���ģʽ���ٶ���
		if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 2))
		{
			GPD_1 = 1;	//���ع�Ϊ1 ��ʼ¼��
			GPD_2 = 0;
			GPD_3 = 0;
			
		} 
			if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 3))
		{
			GPD_1 = 0;	//���ع�Ϊ1 ��ʼ¼��
			GPD_2 = 1;
			GPD_3 = 0;
		} 
			if((command[0] == 0xff)&&(command[1] == 1)&&(command[2] == 4))
		{
			GPD_1 = 0;	//���ع�Ϊ1 ��ʼ¼��
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
	GPC_8 = 1;  //LED��˸1�� �յ�����
		 
}


