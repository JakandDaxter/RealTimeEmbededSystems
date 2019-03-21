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

struct node
{
    int position;
    struct node *next;
};

struct node *front = NULL;
struct node *rear = NULL;


//this creates a customer between
void* Customer_gen(void* parameter){
    
    //int *p = (int*)parameter;
    
    for(int j = 0;j<21;j++){
        
        int Customer_GenTime =rand()%801; //this is the random amount of time that the program will create a new customer
        
        usleep(Customer_GenTime); //using that random generation time to genetate a new customer at a random time
        
        for(int i = 0; i<21; i++){ //then we get here and go ahead and create a customer with a random transation time they need
            int Y = rand()%211+30;
                printf("Customer_%d\n",i);
                    printf("Customer Transation Time needed %d\n\n", Y);
                        //sleep(1);
        }
    }
    return 0;
}

int main(int argc, const char * argv[]) {
    //this creates a random number between 30 and 240
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
