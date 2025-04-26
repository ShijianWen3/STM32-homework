



## 《嵌入式系统及应用》综合实验报告

------



时间：*23/4/2025*

*powered by Markdown*

### 摘要：

​	本实验通过理论与实践结合，使学生掌握STM32F103 单片机的综合开
发技能，理解嵌入式系统的核心设计理念，并为后续复杂项目（如物联网终
端、工业控制器）的开发奠定坚实基础。
​	在硬件调试（如ADC 噪声抑制、PWM 波形验证）和软件调试（如中断
冲突、内存溢出）中，积累嵌入式系统调试经验，掌握逻辑分析仪、串口调
试工具的使用方法。
​	培养工程文档编写与团队协作意识。通过撰写实验报告、记录测试用
例、维护代码注释，规范技术文档编写习惯；通过差异化任务分工，理解团
队协作在复杂项目中的重要性。



------

### 目录：

#### 1.[实验目的](###1 实验目的)

#### 2.[实验原理](###2 实验原理)
- #### 2.1 [所用资源及外设](####1.所用资源及外设)
- #### 2.2 [各模块原理简述](####2.各模块原理简述)
- #### 2.3 [硬件原理框图/流程图](####3.硬件原理框图/流程图)
- #### 2.4 [开发工具](####4.开发工具)

#### 3.[实验内容](###3 实验内容)
- #### 3.1[USART](####1.USART:)
- #### 3.2[ADC](####2.ADC)
- #### 3.3[LED(GPIO/PWM)](####3.LED(GPIO/PWM))
- #### 3.4[EXTI](####4.EXTI)
- #### 3.5[TIMER](####5.TIMER)
- #### 3.6[主程序及程序流程图](####6.主程序及程序流程图)

#### 4.[实验步骤](###4 实验步骤)

#### 5.[实验结果与分析](###5 实验结果与分析)

#### 6.[总结与心得体会](###6 总结与心得体会)

#### 7.[改进建议](###7 改进建议)

#### 8.[发布地址](###8 发布地址)

------

  



### 1 实验目的
i.**掌握STM23F103核心外设的应用** - 通过实际操作”GPIO、ADC、定时器、
中断、串口通信” 等模块，深入理解单片机外设的工作原理及配置方法，培
养对硬件资源的直接控制能力。
ii.**培养嵌入式系统全流程开发能力** - 从硬件连接（传感器、LED、按键）到软
件编程（驱动开发、协议解析），完成完整的嵌入式系统设计流程，提升系
统级工程思维。
iii.**学习多模块协同与系统调试技巧** - 实现”ADC 采集、PWM 输出、串口通信、
中断响应” 等任务的协同工作和相关程序调试技巧。

### 2 实验原理

#### 1.所用资源及外设

|    模块名    |             作用             | 对应资源  |
| :----------: | :--------------------------: | :-------: |
| STM32F03C8T6 |           主控MCU            |  核心板   |
|    USART     |        与上位机PC通信        | PA9 PA10  |
|     ADC      | 对温度传感器的模拟输出量采集 |    PA7    |
|  LED(GPIO)   |            指示灯            |  PA1 PA2  |
|     EXTI     |         按键外部中断         |    PA0    |
|    TIMER     |    执行定时任务和产生PWM     | TIM3 TIM2 |
|     LM35     |          温度传感器          |    PA7    |

#### 2.各模块原理简述
- #### STM32F103：
STM32F103是基于ARM Cortex-M3的32位MCU，具有一定大小的Flash用于存储程序，一定大小的SRAM用于存储运行数据，具有GPIO，Timer，ADC/DAC，USART，SPI，I2C，USB，CAN等外设，通过HSI（内部高速时钟），HSE（外部高速时钟），PLL（锁相环倍频）等配置系统时钟

- #### USART：
即“Universal Synchronous Asynchronous Receiver Transmitter”，是一种全双工串行通信协议，通过TX RX两条数据线便可实现通信，发送的消息逐位发送，故为串行协议

- #### ADC：
即“模数转换器”， 通过电路将模拟信号转换为MCU可以进行运算的数字信号，通常有SAR（逐位逼近）型ADC和积分型ADC，前者通过预设数字量去和输入比较，然后不断调整数字量以逼近输入值；后者通过积分电路，比较输入和参考电压的积分时间，从而推出输入值

- #### LED(GPIO)：
通过配置MCU的GPIO外设为推挽输出模式（PUSH PULL），可以控制与该引脚相连的发光二极管的亮灭

- #### EXTI：
即“外部中断”，是MCU或者说整个嵌入式系统中的重要功能，大幅提升了系统的响应速度和资源利用效率，避免了使用“轮询”这种低效的方式应对时间发生

- #### TIMER：
MCU的定时器/计数器，可以用于周期性执行定时任务或者产生PWM等，虽然叫它定时器/计数器，但它本质上是个计数器，只不过它计的是时间基准数，从而达到计时的效果

- #### LM35：
  一款温度传感器，根据数据手册[^1]的描述：
  > LM35系列产品是高精度集成电路温度器件，其输出电压与摄氏温度成线性正比关系

  可以知道，只需要从LM35输出端用ADC获取输出电压，便可以根据线性关系获得温度值
#### 3.硬件原理框图/流程图

![硬件框图](C:\Users\lenovo\Desktop\markdown\pictures\硬件框图.png)




#### 4.开发工具
- #### Keil：一款ARM公司开发的嵌入式IDE，核心功能有编辑，编译，调试等

  ![keil](C:\Users\lenovo\Desktop\markdown\pictures\keil.png)

- #### SCB[^2]：本人基于Qt6开发的集成UART TCP/UDP BLE/Bluetooth通信的上位机助手

  ![tool](C:\Users\lenovo\Desktop\markdown\pictures\tool.png)
  
### 3 实验内容

#### 1.USART:

```c
//初始化串口
······
//串口发送字符串函数
void USART_SendString(USART_TypeDef* USARTx, uint8_t *str)
{
    while (*str)
    {
        USART_SendData(USARTx, *str);
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
        str++;
    }
}
//串口中断函数
/**stm32f10x_it.c**/
#define BUFFER_SIZE 32
char rx_buffer[BUFFER_SIZE];  // 用于存储接收到的数据
uint8_t rx_index = 0;         // 当前缓冲区索引

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t received_byte = USART_ReceiveData(USART1);

        // 如果接收到换行符（\n），则表示一条消息的结束
        if (received_byte == '\n')
        {
            rx_buffer[rx_index] = '\0';  // 在缓冲区末尾添加字符串结束符
            rx_index = 0;  // 重置缓冲区索引

            // 判断接收到的命令是否是 "07"
            if (strcmp(rx_buffer, "07") == 0)
            {
                char buffer[32];
                sprintf(buffer, "Threshold: %.1f C\n", temperature_threshold);
                USART_SendString(USART1, (uint8_t*)buffer);
            }
            else
            {
                USART_SendString(USART1, (uint8_t*)"invalid instruction.\n");
            }
        }
        else
        {
            // 如果接收到的字符不是换行符，保存字符到缓冲区
            if (rx_index < BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = received_byte;
            }
            else
            {
                // 缓冲区已满，丢弃当前字符
                rx_index = 0;
            }
        }
    }
}
```

#### 2.ADC

```c
// 初始化 ADC
void ADC_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE); // 开启 GPIOA 和 ADC1 时钟

    // 配置 PA7 为模拟输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 模拟输入模式
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ADC 初始化配置
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;              // 独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                   // 禁用扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;             // 单次转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;          // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                          // 转换通道数1
    ADC_Init(ADC1, &ADC_InitStructure);

    // 配置 ADC 通道（PA7 -> ADC1_IN7）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_55Cycles5);

    // 启用 ADC
    ADC_Cmd(ADC1, ENABLE);

    // 校准 ADC（上电后只需要一次）
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

//获取ADC值并根据线性关系转换为温度值
float Temperature_Sensor(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // 开始转换

    // 等待转换完成
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    uint16_t ADC_Value = ADC_GetConversionValue(ADC1); // 返回 12 位 ADC 值（0~4095）
   
    float voltage = (ADC_Value / 4096.0) * 3.3;
    float temperature = voltage * 100;  // LM35 输出为 10mV/°C
	
	temperature = voltage;
	
	return temperature;
}
```

#### 3.LED(GPIO/PWM)

```c
//PA2的LED由GPIO控制
void LED_Init(void)
{
    // 初始化 LED 引脚 PA2
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}
//PA1的LED由PWM控制
void PWM_Init(void)
{
    // 1. 配置 GPIO 引脚 PA1 为输出模式（PWM 输出）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // 使能 GPIOA 时钟

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;  // PA1 为 PWM 输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. 配置定时器 TIM2 产生 PWM 信号
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // 使能 TIM2 时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;  // 设置频率，1000 为周期
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;  // 1ms 周期
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM2, ENABLE);  // 启动定时器

    // 3. 配置 PWM 输出通道
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);  // 使用 TIM2 通道 2
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 4. 启用 PWM 输出
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
```

#### 4.EXTI

```c
// 外部中断处理函数，务必配置外部中断优先级高于定时器
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 按键切换阈值
        if (temperature_threshold == 25.0)
		{
			 temperature_threshold = 30.0;
		}
           
        else if (temperature_threshold == 30.0)
		{
			temperature_threshold = 35.0;
		}
        else
		{
			temperature_threshold = 25.0;
		} 
        
        // 发送当前温度阈值
        char buffer[32];
        sprintf(buffer, "New Threshold: %.1f C\n", temperature_threshold);
        USART_SendString(USART1, (uint8_t*)buffer);

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
```
#### 5.TIMER

```c
//定时器用于周期执行定时的任务，200ms进行一次数据发送和5ms进行一次数据采集
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        counter_5ms++;
		counter_200ms++;
		if(5 == counter_5ms)
		{
			current_temperature = Temperature_Sensor();
			counter_5ms = 0;
		}
		else if(200 == counter_200ms)
		{
			send_temperature(current_temperature);
			counter_200ms = 0;
		}
        
    }
}
//数据发送
void send_temperature(float temperature)
{
	char buffer[32];
    sprintf(buffer, "Temp: %.1f C\n", temperature);
    USART_SendString(USART1, (uint8_t*)buffer);
}
/**数据采集函数已在前文给出**/
```

#### 6.主程序及程序流程图

```c
int main(void) 
{
	USART_Config();
	
	ADC_Config();
	
	EXTI_Config();
	
	LED_Init();
	
	delay_init();
	
	PWM_Init();

	TIM3_Init();//周期1ms，用于执行200ms和5ms的周期任务
    
	temperature_threshold = 25.00;//温度阈值

	GPIO_SetBits(GPIOA, GPIO_Pin_2);//启动时LED1闪烁两次后常亮
	delay_ms(500);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	delay_ms(500);
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
    
	while(1)
	{
		if (current_temperature > temperature_threshold)  // 超温
		{
			Update_PWM_Duty();  // 每次调用更新占空比
		}
		else  // 温度正常
		{
			TIM_SetCompare2(TIM2, 0);  // 停止 PWM，关闭 LED
		}
		delay_ms(100);//每次更新pwm花费0.1s，流水灯一个周期需要更新20次，则T=0.1 * 20 = 2s，则f = 1/T = 0.5Hz
	} 
}
```

![程序流程图](C:\Users\lenovo\Desktop\markdown\pictures\程序流程图.png)

### 4 实验步骤

#### 1.烧录代码至单片机
- #### 在Keil上配置DAP-Link的Debug设置[^3]，便可通过Keil直接将程序download至Flash

  ![DAP-Link](C:\Users\lenovo\Desktop\markdown\pictures\DAP-Link.png)
  
#### 2.观察串口是否正常打印输出

![usart](C:\Users\lenovo\Desktop\markdown\pictures\usart.png)

- #### 串口正常输出，且根据时间戳判断周期确为200ms，说明USART与TIMER工作正常

#### 3.检验ADC所测值与LM35输出脚电压值是否相等

![ADC电压](C:\Users\lenovo\Desktop\markdown\pictures\ADC电压.png)



- #### 使用万用表测量电压，编写程序直接将电压值通过串口发送出来，确保二者一致



#### 4.LED是否正常工作(LED1是否正常闪烁，呼吸灯现象是否正常)

- #### 由于电脑没有后置摄像头，同时手机摄像无法保证实时性，故使用DroidCam[^4]将手机摄像头视频流通过局域网投放至电脑上

  ![LED1](C:\Users\lenovo\Desktop\markdown\pictures\LED1.png)



- #### 编写一个串口控制灯亮灭的程序，发送“LED1 ON”，LED1被成功点亮(上图)

  ![LED2](C:\Users\lenovo\Desktop\markdown\pictures\LED2.png)
  
- #### 由于加热设备缺失的原因，直接在程序中改变温度值，发现超温时LED1成功熄灭，LED2呼吸灯成功运行(上图)



### 5 实验结果与分析

- #### ==系统分析==：当拿到了实验要求后，我们对要求进行分析：首先实验要求中需要使用温度传感器LM35，根据数据手册[^1]可以知道该传感器的输出量为电压，所以需要用ADC对传感器的输出电压进行采集;同时实验要求中要通过USART与上位机进行通信，所以需要编写发送与接收逻辑的代码；其次实验有执行定时任务和产生PWM的需求，所以需要使用定时器(TIMER)；最后实验有外部中断产生以及对LED的控制，所以需要使用GPIO

- #### ==硬件搭建==：使用STM32F103C8T6核心板作为主控，与面包板一起通过杜邦线连接各种线路(VCC GND等)以及各种外设

- #### ==软件编写==：在Keil上先分模块编写代码并分别测试，然后将所有模块加入总程序中确定各模块在系统功能实现时依旧正常运行

- #### ==系统调试==：针对前面对实验的分析，进一步对系统功能进行调试，观察系统在极端情况下是否依旧能正常工作

- #### ==功能优化==：在系统满足实验要求后，对功能依照自己的理解进行优化

- #### ==遇到的BUG==：本次实验，个人遇到最大的BUG就是串口的使用，我的问题分两个：1.在将核心板插上DAP-Link，这时再单独拿一个串口转TTL的模块与核心板的TX/RX引脚连接，发现串口无法通信，后续使用DAP-Link模块上的TX/RX引脚进行连接便可以正常通信，在这之后询问AI得知插上DAP-Link时串口是Debug-Serial模式，所以自能用DAP-Link自带串口进行通信 2.当使用串口发送字符串时，如果字符串没有以'\n'结尾，那么上位机的串口助手无法正常接收和显示数据，怀疑上位机软件编写时的错误，后续会对上位机的源码进行复盘

### 6 总结及心得体会

*本次实验，我的最大收获可能就是熟悉了STM32的标准库和加强了Markdown的使用：文档的编写在我看来是一项很重要的能力，使用Word等办公软件编写文档自由度与个性化度不高，采用LaTex语法的学习曲线较陡，故使用程序员都喜欢使用的Markdown进行编写；得益于芯片系列的覆盖面广和技术资料充足的原因，STM32在现在嵌入式行业的应用十分广泛，熟悉STM32芯片的适用有益于后续转向其他MCU(甚至SoC)的使用，但是目前我的学习仍有不足之处，例如对时钟树的启动过程还不够熟悉，对很多常用的工业控制(CAN 、ModBus等)还未了解*

### 7 改进建议

*希望以后这种类似的实验能够加入更多的控制相关的东西，MCU的中文名叫什么呢？“微控制器”，这足以说明MCU的应用场景。以我所见，现在的大学教育最不足的一点就是不能结合以后学生的工作场景进行教学，总是讲一些空中楼阁，这不是培养工程师的路子。我知道教学与行业应用是两个东西，但是至少应该向行业靠拢，而不是远离行业*

### 8 发布地址

Github仓库地址：




#### 参考资料及其他URL

[^1]:<a href="https://www.ti.com/cn/lit/ds/symlink/lm35.pdf" target="_blank">LM35数据手册(德州仪器)</a>
[^2]:<a href="https://github.com/ShijianWen3/SCB/releases/tag/v1.0" target="_blank">SCB github仓库</a>
[^3]:<a href="https://blog.csdn.net/weixin_53786693/article/details/124367512" target="_blank">DAP-Link参考设置(CSDN)</a>
[^4]:<a href="https://droidcam.en.lo4d.com/windows" target="_blank">DroidCam官网</a>
