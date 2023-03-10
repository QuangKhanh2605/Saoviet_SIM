#ifndef __UART_SIM_
#define __UART_SIM_

#include "main.h"
#include "string.h"
#include "stm32l1xx_hal_uart.h"

typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t buffer;
	uint16_t countBuffer;
	char sim_rx[5000];
}UART_BUFFER;



void Setup_On_Off_Sim(GPIO_TypeDef* GPIO1, uint16_t GPIO_Pin_On_Off_Sim, 
                      GPIO_TypeDef* GPIO2, uint16_t GPIO_Pin_PWKEY,  
                      GPIO_TypeDef* GPIO3, uint16_t GPIO_Pin_RESET);

int8_t Sim_SendCommand(UART_BUFFER *rx_uart1, UART_BUFFER *rx_uart3,char* command,char* response);
																 
int8_t Compare_Uart1_RX_Uart3_TX(UART_BUFFER *rx_uart1, UART_BUFFER *rx_uart3,char* response);
																 
int8_t Receive_SMS_Sim(char* command,char* response);

void Config_Uart_Sim(UART_BUFFER *rx_uart1, UART_BUFFER *rx_uart3);

void Check_Status_Config_Sim(UART_BUFFER *rx_uart1, UART_BUFFER *rx_uart3, char* command,char* response);
														 
void Transmit_Data_Uart3(UART_HandleTypeDef huart, char* command);
void Delete_Buffer(char* buf, uint16_t *size_buf);
void Display_Uart1(UART_HandleTypeDef huart,void* data);
void Send_Command_Uart1_Uart3(char* command,char* response);
void Send_SMS_Sim(void);

#endif
