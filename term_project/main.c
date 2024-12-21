#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"

#include "misc.h"

/* function prototype */
void RCC_Configure(void); //RCC 설정
void GPIO_Configure(void); //GPIO 설정
void NVIC_Configure(void); //NVIC 설정
// ADC 관련
void ADC_Configure(void); 
void ADC1_2_IRQHandler(void);
void DMA_Configure(void); //DMA 설정

// 진동센서
void read_vibration_sensor(void);

// 초음파 감지 센서
void check_for_object(void);

// 온습도 센서
void read_temperature_humidity(void)

// bluetooth 관련
void USART1_Init(void); //USART1(putty) 설정 
void USART2_Init(void);  //USART2(bluetooth) 설정
void send_msg_to_bluetooth(char* buf); //USART1(putty)로 문자열 보냄
void send_msg_to_putty(char* buf); // USART2(bluetooth)로 문자열 보냄

void delay(); //딜레이

// 거리 센서 작동시 온습도 센서 값 읽도록 구현

/*
void feed();
void start_laser();
void stop_laser();
void print_status();

char usart1_msg[50]; // usart1(putty)에서 메시지를 받을 때 메세지를 저장할 버퍼이다.
char usart2_msg[50]; // usart2(bluetooth)에서 메시지를 받을 때 메시지를 저장할 버퍼이다.
int usart1_index = 0;//usart1_msg 버퍼에 다음으로 문자가 들어갈 인덱스이다.
int usart2_index = 0;//usart2_msg 버퍼에 다음으로 문자가 들어갈 인덱스이다.
*/

int bluetooth_connected = 0;
int menu_printed = 0;
int print_vibration = 0;
volatile uint32_t ADC_Value[2];// 진동 센서, 온습도 값 저장

// 진동 센서
#define VIBRATION_THRESHOLD 1000 // 진동 센서 값의 임계값

// 초음파 감지 센서
#define SOUND_SPEED 343.0 // 속도 (m/s)

// 온습도 센서 관련 값 (ADC 값에서 변환)
#define ADC_MAX_VALUE 4095.0
#define TEMP_SENSOR_CALIBRATION 3.3 // 센서의 보정값, 실제 센서에 따라 달라질 수 있음
#define TEMP_SENSOR_RESOLUTION 0.01 // 온도 변화에 대한 해상도


void RCC_Configure(void)
{  
    // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
	
	/* USART1, USART2 TX/RX port clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Port A
	
	/* USART1, USART2 clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
	/* Alternate Function IO clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        
        // ADC1 port clock enable
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  
        
        // DMA clock enable
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // Port D
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
   	// TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'

    /* USART1 pin setting */
        //TX
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA9
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function output Push-pull
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        
	//RX
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA10
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with pull-up
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
    /* USART2 pin setting */
         //TX
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function output Push-pull
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        
	//RX
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //PA3
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with pull-up
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        
        // ADC  pc0, pc1
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        // Set Pin Mode General Output Push-Pull
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        // Set Pin Speed Max : 50MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        
        // check connected
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
        // Set Pin Speed Max : 50MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        // 초음파 감지 센서
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // PA4
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 출력 모드 (Push-pull)
        GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC;
    ADC_DeInit(ADC1);
    // ADC1 Configuration
    ADC.ADC_Mode = ADC_Mode_Independent ;
    ADC.ADC_ContinuousConvMode = ENABLE;
    ADC.ADC_DataAlign = ADC_DataAlign_Right;
    ADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC.ADC_NbrOfChannel = 2;
    ADC.ADC_ScanConvMode = ENABLE;
    
    ADC_Init(ADC1, &ADC);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);  //진동
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);  //온습도
   // ADC_ITConfig(ADC1,  ADC_IT_EOC, ENABLE );  // interrupt enable
    ADC_Cmd(ADC1, ENABLE);  // ADC1 enable
    ADC_DMACmd(ADC1,ENABLE);
    
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// ADC는 인터럽트 베이스이므로 핸들러 작성 필요, 정의된 이름을 그대로 사용해야 함.
void ADC1_2_IRQHandler(void) {
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC)!=RESET){
       // value = ADC_GetConversionValue(ADC1);
  
        ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
    }
}

void DMA_Configure(void) {
	DMA_InitTypeDef DMA_Instructure;
        DMA_DeInit(DMA1_Channel1);
	DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // 어디에 있는걸 가져올지
	DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Value; // 가져온걸 어디에 쓸지
	DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_Instructure.DMA_BufferSize = 2; 
	DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Enable;;
	DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Instructure.DMA_Priority = DMA_Priority_High;
        DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_Instructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);
}


void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

	// Enable the USART1 peripheral
	USART_Cmd(USART1, ENABLE);
	
	// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
	USART1_InitStructure.USART_BaudRate = 9600;
        USART1_InitStructure.USART_StopBits = USART_StopBits_1_5;
        USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART1_InitStructure.USART_Parity = USART_Parity_No;
        USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART1_InitStructure.USART_Mode= USART_Mode_Rx| USART_Mode_Tx;
        USART_Init(USART1, &USART1_InitStructure);
	
	// TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Receive Data register not empty interrupt
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

	// Enable the USART2 peripheral
	USART_Cmd(USART2, ENABLE);
	
	// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
	USART2_InitStructure.USART_BaudRate = 9600;
        USART2_InitStructure.USART_StopBits = USART_StopBits_1_5;
        USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART2_InitStructure.USART_Parity = USART_Parity_No;
        USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART2_InitStructure.USART_Mode= USART_Mode_Rx| USART_Mode_Tx;
	USART_Init(USART2, &USART2_InitStructure);
        
	// TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Receive Data register not empty interrupt
	
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
	
    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // USART1
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO  // 우선순위 설정
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



void USART1_IRQHandler() { // putty -> phone
    uint16_t word;
 
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
      
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement
        while ((USART1->SR & USART_SR_TXE) == 0);
        USART_SendData(USART2, word); // 데이터 입력 시 usart1을 통해 보드로 전송
        
        // clear 'Read data register not empty' flag
    	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void USART2_IRQHandler() { // phone -> putty
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){

        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);
      
        // TODO implement
        while ((USART2->SR & USART_SR_TXE) == 0);
        USART_SendData(USART1, word); // 데이터 입력 시 보드를 통해 usart1으로 전송
        
        // clear 'Read data register not empty' flag
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
        
}


// 진동 센서 값을 읽고 블루투스로 전송
void read_vibration_sensor(void) {
    uint32_t vibration_value = ADC_Value[0];  // 진동 센서의 값이 첫 번째 채널에 저장됨
   
    if (vibration_value < VIBRATION_THRESHOLD && print_vibration == 0) {
        char vibration_msg[50];
        sprintf(vibration_msg, "\r\nVibration Detected! ADC Value: %d\r\n", vibration_value);
        send_msg_to_putty(vibration_msg); // 푸티로 메시지 전송
        send_msg_to_bluetooth(vibration_msg);  // 진동 감지 시 핸드폰에 메시지 전송
        print_vibration = 1;
    }
    
    if (vibration_value > VIBRATION_THRESHOLD  && print_vibration == 1) {
      print_vibration = 0;
    }
}


// 초음파 감지 센서 (물체 감지)
void check_for_object(void)
{
    // Echo 핀 상태 변화를 감지하여 물체를 감지
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == SET) {  // Echo 핀 HIGH로 변하면
        char msg[] = "Object detected! Close proximity.";
        send_msg_to_bluetooth(msg);  // 블루투스로 전송
        send_msg_to_putty(msg);      // 푸티로 전송
        
        read_temperature_humidity();  // 온습도 센서 값 읽기
    }
}

// 온습도 센서 값을 읽고 변환하여 출력
void read_temperature_humidity(void) {
    uint32_t adc_value = ADC_Value[1];  // 온습도 센서의 값이 두 번째 채널에 저장됨
    float voltage = (adc_value / ADC_MAX_VALUE) * TEMP_SENSOR_CALIBRATION;
    float temperature = voltage / TEMP_SENSOR_RESOLUTION;  // 센서에서 읽은 값에 따른 온도 계산 (보정 필요)

    char temp_msg[50];
    sprintf(temp_msg, "\r\nTemperature: %.2f°C\r\n", temperature);
    send_msg_to_putty(temp_msg);   // 푸티로 메시지 전송
    send_msg_to_bluetooth(temp_msg); // 블루투스로 메시지 전송
}


// 인자의 문자열을 블루투스로 전송
void send_msg_to_bluetooth(char* buf){
    for (int i=0;; i++) {
        if (buf[i] == '\0')             // 문자열의 끝이라면 무한 반복 종료
            break;
        USART_SendData(USART2, buf[i]); // 한글자씩 전송
        for(int k=0;k<50000;++k);       // 전송 후 조금 대기
    }
}


// 인자의 문자열을 PUTTY로 전송
void send_msg_to_putty(char* buf){
    for (int i=0;; i++) {
        if (buf[i] == '\0')             // 문자열의 끝이라면 무한 반복 종료
            break;
        USART_SendData(USART1, buf[i]); // 한글자씩 전송
        for(int k=0;k<50000;++k);       // 전송 후 조금 대기
    }
}


void delay() {
    for (int i=0; i<1000000; i++);
}

int main(void)
{
    char msg[] = "\r\nWelcome to the cat feed system:\r\n1. Feed \r\n2. Start Laser\r\n3. Stop Laser \r\n4. Print Status\r\nPlease choose an option by entering the number.\r\n";
    
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();      // pc
    USART2_Init();      // bluetooth
    NVIC_Configure();
    ADC_Configure();
    DMA_Configure();
    
    
    while (1) { 
      
      if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) != 0x00 && !menu_printed){
        send_msg_to_bluetooth(msg);
        menu_printed = 1;
      }
      
      /* 메뉴 출력 부분 이렇게 수정해본다면?
      if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET && !menu_printed){
        send_msg_to_bluetooth(msg);
        menu_printed = 1;
      }
      */
      
     // 진동 센서 값 읽기
      read_vibration_sensor();
      
      // 초음파 센서로 물체 감지
      check_for_object();

      
      delay();
      
      
     
    }

    
  
}
