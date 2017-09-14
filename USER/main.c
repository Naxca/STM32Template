#include "sys.h"
#include "delay.h"
#include "usart.h"

#define testReportCycles 1000

static volatile uint8_t SPI_RX_BUFFER[10] = {0x00};
static volatile uint16_t testReportCounter = 0;

// static GPIO_TypeDef* TDC_CS_Port;
// static uint16_t TDC_CS_Pin;
/* 方法1:使用函数传递参数
TDC7200_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
*/

/* 方法2:使用宏定义
*/
#define TDC_CS_Port GPIOB
#define TDC_CS_Pin GPIO_Pin_1

void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler =psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
}
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
		// LED1=!LED1;
	}
}

//TIM3 PWM部分初始化
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟

	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5

   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM3 Channel2 PWM模式	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器

	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

void testIOInit(void)
{
	/* PB_6 */
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
}

void intInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

void KEY_Init(void) //IO初始化
{
 	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //pull up
 	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//初始化 WK_UP-->GPIOA.0	  下拉输入
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 Input pull down	
	GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA.0
}

void oscInit(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		FLASH_SetLatency(FLASH_Latency_2);
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08);
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC , ENABLE);
}

void uartInit(void)
{
	uart_init(921600);
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据					    
}

/* PB13,14,15 SPI2 */
/* SPI CLK div 2, 36MHz->18MHz */
void SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTB时钟使能 
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );//SPI2时钟使能 	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15复用推挽输出 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB

	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);  //PB13/14/15上拉

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI2, ENABLE); //使能SPI外设

	SPI2_ReadWriteByte(0xff);//启动传输
}   
//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2
//SPI_BaudRatePrescaler_8
//SPI_BaudRatePrescaler_16
//SPI_BaudRatePrescaler_256
  
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1&=0XFFC7;
	SPI2->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
	SPI_Cmd(SPI2,ENABLE); 
}

void spiCSInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	GPIO_SetBits(GPIOC, GPIO_Pin_14);
}

void spiInit(void)
{
	SPI2_Init();
	SPI2_SetSpeed(SPI_BaudRatePrescaler_2);
	spiCSInit();
}

void keyInit()
{
    /* Confinguring P1.1 & P1.4 as an input and enabling interrupts */
}

void triggerInit(void)
{
	TIM3_PWM_Init(35999,0);	// 72,000,000 / 72,000 = 1000
	TIM_SetCompare2(TIM3,2);
}

void MCO_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	// 开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	// 选择GPIO8引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	//设置为复用功能推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//设置IO的翻转速率为50M
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// 初始化GPIOA8
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void oscOutInit(void)
{
	MCO_GPIO_Config();
	// GPIO_InitTypeDef GPIO_InitStructure;
	// RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	// GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	#define RCC_MCO_NoClock                  ((uint8_t)0x00)
	#define RCC_MCO_SYSCLK                   ((uint8_t)0x04)
	#define RCC_MCO_HSI                      ((uint8_t)0x05)
	#define RCC_MCO_HSE                      ((uint8_t)0x06)
	#define RCC_MCO_PLLCLK_Div2              ((uint8_t)0x07)
	*/
	
	// RCC_MCOConfig(RCC_MCO_PLLCLK_Div2);
	RCC_MCOConfig(RCC_MCO_HSE);
}

void EXTIX_Init(void)
{

 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

    KEY_Init();	 //	按键端口初始化

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

    //GPIOE.2 中断线以及中断初始化配置   下降沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);

  	EXTI_InitStructure.EXTI_Line = EXTI_Line2;	//KEY2
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

   //GPIOA.0	  中断线以及中断初始化配置 上升沿触发 PA0  WK_UP
 	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键WK_UP所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2，
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

// NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//使能按键KEY2所在的外部中断通道
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2，
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级2
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
// 	NVIC_Init(&NVIC_InitStructure);
}

//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
	// delay_ms(10);//消抖
	uint8_t i;
	uint8_t txCmd=0x00;
	if( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1 )	 	 //WK_UP按键
	{
		for (i = 0; i < 10; i++)
		{
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);
			SPI_RX_BUFFER[i] = SPI2_ReadWriteByte(txCmd);
			SPI_RX_BUFFER[i] = SPI2_ReadWriteByte(0xff);
			GPIO_SetBits(GPIOC,GPIO_Pin_13);
			txCmd++;
		}
		// GPIO_ResetBits(GPIOC,GPIO_Pin_14);
		// SPI_RX_BUFFER[0] = SPI2_ReadWriteByte(0xff);
		// SPI_RX_BUFFER[1] = SPI2_ReadWriteByte(0xff);
		// SPI_RX_BUFFER[2] = SPI2_ReadWriteByte(0xff);
		// GPIO_SetBits(GPIOC,GPIO_Pin_14);
		if ( testReportCounter == testReportCycles-1 )
		{
			testReportCounter = 0;
			// printf("EXTI0\n");
			for (i = 0; i < 10; i++)
			{
				printf("%c",SPI_RX_BUFFER[i]);
			}
			printf("\n");
		}
		else
		{
			testReportCounter++;
		}
		
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位
}

// void USART1_IRQHandler(void)
// {
// 	// this function defined in usart.c
// 	printf("USART1_IRQ!\n");
// }

void TDC7200_Init()
{
	// GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	// SPI2_ReadWriteByte(0x40);
	// SPI2_ReadWriteByte(0x20);
	// GPIO_SetBits(GPIOC,GPIO_Pin_13);

	GPIO_ResetBits(TDC_CS_Port, TDC_CS_Pin);
	SPI2_ReadWriteByte(0x40);
	SPI2_ReadWriteByte(0x20);
	GPIO_SetBits(TDC_CS_Port, TDC_CS_Pin);

// 	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
// 	SPI2_ReadWriteByte(0x42);
// 	SPI2_ReadWriteByte(0x00);
// 	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}

void sysInit(void)
{
	intInit();
	// oscInit();
	SystemInit();	//CMSIS, main clock set to 72MHz
	uartInit();
	keyInit();
	triggerInit();
	testIOInit();
	delay_init();
	spiInit();
	oscOutInit();
	// TDC7200_Init();
	EXTIX_Init();
}

void sysCheck(void)
{
	int i=0;
	for(;i<10;i++){
	}
}

void mainLoop(void)
{
	u8 t=3;
	u8 i=0;
	t=3;
	i=0;
	while(1)
	{
		
	}
}

int oldmain(void)
{
	u8 t=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	delay_init();	    	 //延时函数初始化	
	uart_init(115200);	 //串口初始化为115200
	while(1)
	{
		printf("t:%d\r\n",t);
		delay_ms(500);
		t++;
	}	
}

int main(void)
{
	// oldmain();
	sysInit();
    // sysCheck();
    mainLoop();
}
