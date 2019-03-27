//
//  main.c
//  Threading_Tutorial
//
//  Created by JakandDaxter on 3/21/19.
//  Copyright © 2019 RealTime. All rights reserved.
//

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

//configMAX_PRIORITIES  = 7

// If you need an int use
//
//     int bla = RNG_DR % 14 + 1 (yields a number between 0 and 15 (not including 0 and 15));
//
// RNG_DR is the Data register of the RNG.
//
//     You will need to check for the data ready flag to get valid values.

each new custoemr arrives between 1 to 4 minutues
    
    1 minute equals 100ms
        
        4 minutues = 400ms
            
            30 seconds = 50ms
                
generate a random number between 100ms and 400ms  
                
each customer needs about 3 seconds to 8 minutes of time with the teller 
                
                1 minute equals 100ms
                                
                    30 seconds = 50ms
                                    
                            8 minutes =  800 ms

generate a random number between 50ms and  800ms           

//this creates a customer between
BaseType_t xTaskCreate (TaskFunction_t Customer_gen, //function that will create the customer and the time they need
                        const char * const Customer, //name for the task for debugging
                        uint16 usSrackDepth,           
                        void *Customer_gen
                        UBaseType_t configMAX_PRIORITIES - 7),
                        TaskHandle_t *Customer_Gen. 
                    );
    
//     //int *p = (int*)parameter;
//
//     for(int j = 0;j<21;j++){
//
//         int Customer_GenTime =rand()%801; //this is the random amount of time that the program will create a new customer
//
//         usleep(Customer_GenTime); //using that random generation time to genetate a new customer at a random time
//
//         for(int i = 0; i<21; i++){ //then we get here and go ahead and create a customer with a random transation time they need
//             int Y = rand()%211+30;     //this creates a random number between 30 and 240
//                 printf("Customer_%d\n",i);
//                     printf("Customer Transation Time needed %d\n\n", Y);
//                         //sleep(1);
//         }
//     }
//     return 0;
// }

int main(int argc, const char * argv[]) {

    printf("***Threading Tutorial***\n\n\n");
    
    pthread_t thread_handle; //thread handle
    
    int ret = pthread_create(&thread_handle,0, Customer_gen, 0); //save the result
    
    if(ret != 0){
        printf("Failed To Create Thread: %d \r", ret);
        return(1);
    }
    
    pthread_join(thread_handle,0);
    
    return 0;
}



//make sure to set up the random number generator for the customers
RNG -> CR |= RNG_CR_RNGEN; //enable the generator
RNG -> CR |= RNG_CR_IE;  //enable the interuuprt to let me know no errors occurred & data is ready

const TickType_t Customer_Arrival = pdMS_TO_TICKS(RNG_DR%301 + 100); //creats a random number between 100ms and 400 ms for their arrival
 
// TT = Teller Time
int Customer_TT = pdMS_TO_TICKS(RNG_DR%751 + 50);

//const TickType_t xMaxDelay = pdMS_TO_TICKS( 4000UL );
//const TickType_t xMinDelay = pdMS_TO_TICKS( 200UL );

void vCustomer_Gen( void *pvParameters )
{
    
const char *pcTaskMessage = "Customers Arriving\r\n";

const char *pcTaskName = "Customers_\r\n";

volatile uint32_t ul;  // volatile to ensure ul is not optimized away. */
 //    /* As per most tasks, this task is implemented in an infinite loop.
for( ;; ) {
         // Print out the name of this task.
        vPrintString( pcTaskMessage);
        
        vPrintStringAndNumber(pcTaskName, )
         // Delay for a period.
        for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
        {
        //  This loop is just a very crude delay implementation.  There is
        // nothing to do in here.  Later examples will replace this crude
        // loop with a proper delay/sleep function.
        } }
}

void Customer_IRQHandler(){
    
    if()
}

int main( void )
{
  //   Create one of the two tasks.  Note that a real application should check
// the return value of the xTaskCreate() call to ensure the task was created
// successfully.
xTaskCreate(
            vCustomer_Gen,   // Pointer to the function that implements the task.
                "Customer", // Text name for the task.  This is to facilitate
 //                             debugging only.(will appear on screen)
                    1000,     // Stack depth - small microcontrollers will use much
 //                             less stack than this.
                    NULL,     // This example does not use the task parameter.
                    0,        // This task will run at priority 1.
                    NULL );   // This example does not use the task handle. */
                    
 //    /* Create the other task in exactly the same way and at the same priority.
    //*****xTaskCreate( vTask2, "Task 2", 1000, NULL, 1, NULL );***********
    
     // Start the scheduler so the tasks start executing.
    vTaskStartScheduler();
    //  If all is well then main() will never reach here as the scheduler will
    // now be running the tasks.  If main() does reach here then it is likely that
    // there was insufficient heap memory available for the idle task to be created.
    // Chapter 2 provides more information on heap memory management.
    for( ;; );
}
