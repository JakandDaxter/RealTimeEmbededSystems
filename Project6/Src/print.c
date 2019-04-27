#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"

// #define RXIT true // uncomment this line to capture keystrokes from console

void newline_task(void* argument);  
SemaphoreHandle_t  transmit_mutex;  // protects UART transmitter resource

 //
 // * prints the string, blocks if UART busy, thus safe from multiple threads
 //
 // * initializes everything needed for printing
 // * starts newline_task

void print_task_init() {
  transmit_mutex = xSemaphoreCreateMutex();     // create mutex to protect UART transmitter resource
#ifdef RXIT
  receive_mutex = xSemaphoreCreateMutex();      // create mutex to protect UART receiver resource
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);    // one time, kick off receive interrupt (repeated from within rx callback)
  if (pdPASS != xTaskCreate (newline_task,	"print", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
#endif
}

#ifdef RXIT
SemaphoreHandle_t  receive_mutex;   // protects UART receiver resource
uint8_t rx_buffer[20];  // Shared buffer between foreground and UART RX
uint8_t rx_byte;        // the currently received byte
uint8_t rx_index = 0;   // pointer into the rx_buffer

 //
 // * newline task prints lines of text at the moment the user types <CR>
 // *  - blocks on taking receive_mutex
 // *  - when mutex acquired, a newline has been received.  it is printed.
 // *  - receive_mutex is then given up (so RX_ISR can acquire more characters)


void newline_task(void* argument) {
  uint32_t random;

  while(1) {
    // wait until something has arrived over UART (e.g. a user command)
    if(rx_buffer[0]!=0 && pdTRUE == xSemaphoreTake(receive_mutex, ~0)) {  // when we Take mutex, ISR stops receiving chars
      HAL_RNG_GenerateRandomNumber(&hrng, &random);
      vPrintString((char *)rx_buffer);   // something arrived, stop receiving chars, print what we have.
      rx_buffer[0] = 0;  // flag ourself to NOT try to Take mutex until after ISR has received a new char (and owns mutex)
      xSemaphoreGive(receive_mutex);
    }
  }
}
    
 //
 // * overrides _weak HAL receiver callback
 // * - called when byte received
 // * - the received byte is buffered.
 // * - if the received byte=='\r', ISR gives up mutex (so foreground can take it)
 // * - finally, the RX interrupt is re-enabled


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xTaskWoken = pdFALSE;
  static BaseType_t i_have_receive_mutex = pdFALSE;
  if(huart->Instance == USART2) {

    // if received byte is a newline, give the received buffer to the forground
    if(rx_byte == '\r') {
      xSemaphoreGiveFromISR(receive_mutex, &xTaskWoken);
      i_have_receive_mutex = pdFALSE;     // We don't have the mutex anymore
      rx_index = 0;                       // Next time around, queue data from start of buffer
    }
    
    // buffer all characters
    else {
      // acquire receive_mutex once
      if(!i_have_receive_mutex) {
        xSemaphoreTakeFromISR(receive_mutex,  &xTaskWoken);
        i_have_receive_mutex = pdTRUE;    // don't need to ask to Take again
      }
      
      // buffer all other characters
      rx_buffer[rx_index++] = rx_byte;    // buffer the byte
      rx_buffer[rx_index] = 0;            // keep string NULL terminated
      if(rx_index >= sizeof(rx_buffer))
        rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // re-enable receive interrupt
  }
}
#endif
