#include "user_interface.h"

char * tx_buffer;
uint8_t rx_buffer[6];

extern UART_HandleTypeDef huart2;

//void USART2_IRQHandler()
//{
//  
//}

void test_uart()
{
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
  tx_buffer = "Potato\n\r";
  HAL_UART_Receive_IT(&huart2, rx_buffer, 6);
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)rx_buffer, 6);
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}