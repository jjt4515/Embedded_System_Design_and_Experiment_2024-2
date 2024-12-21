
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

// ADC 관련
void ADC_Configure(void); 
void ADC1_2_IRQHandler(void);
void DMA_Configure(void); //DMA 설정
volatile uint32_t ADC_Value[2];

// bluetooth 관련
void USART1_Init(void); //USART1(putty) 설정 
void USART2_Init(void);  //USART2(bluetooth) 설정
/*
void end_usart1(); //USART1(putty)에서 받은 문자열 처리
void end_usart2(); // USART2(bluetooth)에서 받은 문자열 처리
*/

void send_msg_to_bluetooth(char* buf); //USART1(putty)로 문자열 보냄

void send_msg_to_putty(char* buf); // USART2(bluetooth)로 문자열 보냄

void delay(int); //딜레이

void NVIC_Configure(void); //NVIC 설정
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
        
        
        // ADC
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        
        // Set Pin Mode General Output Push-Pull
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        
        // Set Pin Speed Max : 50MHz
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
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
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);
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
        value = ADC_GetConversionValue(ADC1);
  
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
      /* 
         if (word == 1) {
            send_msg_to_bluetooth("Option 1 selected\r\n");
            
        } else if (word == 2) {
            send_msg_to_bluetooth("Option 2 selected\r\n");
            
        } else if (word == '3') {
            send_msg_to_bluetooth("Option 3 selected\r\n");
            
        } else if (word == '4') {
            send_msg_to_bluetooth("Option 3 selected\r\n");
            
        } else {
            send_msg_to_bluetooth("Invalid option\r\n");
        }
*/
        // TODO implement
        while ((USART2->SR & USART_SR_TXE) == 0);
        USART_SendData(USART1, word); // 데이터 입력 시 보드를 통해 usart1으로 전송

        
        // clear 'Read data register not empty' flag
    	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
        
              
    
    }
        
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

void delay(int delay_value) {
    for (int i=0; i<delay_value; i++) {}
}

int main(void)
{
    char msg[] = "\r\nWelcome to the cat feed system:\r\n1. Feed \r\n2. Start Laser\r\n3. Stop Laser \r\n4. Print Status\r\nPlease choose an option by entering the number.\r\n";

    unsigned int i;
  
    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    USART1_Init();      // pc
    
    USART2_Init();      // bluetooth

    NVIC_Configure();
    
    ADC_Configure();

    DMA_Configure();
    
    delay(10000000);

    send_msg_to_putty(msg);
    
      
    while (1) { 
      
      if(bluetooth_connected) {
         delay(10000000);
            send_msg_to_putty("abc");
        send_msg_to_bluetooth(msg);
           send_msg_to_putty("dvsa");
         delay(10000000);
        menu_printed = 1;

      }
     if(menu_printed==1) break;
     
    }
    while(1);
    
    return 0;
}
