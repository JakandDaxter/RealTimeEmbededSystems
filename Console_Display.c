#include "Console_Display.h"
#include "Timer_2_Input_Capture.h"
#include "LED.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>


char FaileMessage [] = "!!The Post has failed due to no pulse seen in 100ms!!\r\n";
char reruntheprogram [] = "Would You Like To Rerun the Program? (Y or N):\r\n";
extern int POST_FLAG;
unsigned char UserInput[4];

const int LOWER_BOUND = 950;
const int UPPER_BOUND = 1050;

uint32_t buckets[101];
uint32_t histogram[101];

extern uint32_t delta_time[ARRAY_SIZE];
uint8_t buffer[BufferSize];

// This is the POST console function. Prints to the console to let the user know the post test failed
// lets the user rerun the post test if need be
int POSTFAIL(void){
	char UserInput;
	USART_Write(USART2,(uint8_t *)FaileMessage, strlen(FaileMessage));
	USART_Write(USART2,(uint8_t *)reruntheprogram, strlen(reruntheprogram));
	UserInput = USART_Read(USART2);
	//Loop until valid input is registered
	while(1)
	{
		if(UserInput == 'Y' || UserInput == 'y')break;
		else if(UserInput == 'N' || UserInput == 'n'){ // User wants to exit the program
			USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n", 17);
			return (0); //offcially exit the program
		}
		else{//this is the case if the user inputted an invalied answer
			USART_Write(USART2, (uint8_t *)"Invalid Response Was Entered\r\n", 30);
			UserInput = USART_Read(USART2);
			//return FAIL();
			//return POSTFAIL();
		}
	}
}

//This is the post function that will tell the user if the test failed or succeded 
int POST(void)
{
	int lower_bound = WELCOMEMESSAGE();
	int test_input = Verify_Input(lower_bound);
	
	if(test_input == -1)
	{
		Print("Your lower bound is out of range\n\r");
		Print("Press enter to restart.....\n\r");
		char rxbyte = USART_Read(USART2);
		while(rxbyte != '\r')
		{
			rxbyte = USART_Read(USART2);
		}//wait until enter is pushed
		return 1;
	}
	else
	{
		if(test_input == 1)
		{
			lower_bound = LOWER_BOUND;
		}
	}
	Print("Press enter to start.....\n\r");
	while(USART_Read(USART2) != '\r'){}//wait until enter is pushed
	//start the timer
	Start_Timer2(); 
	//start of the test time 
	while(POST_FLAG == 0 && delta_time[999] == 0)
	{
		Print(".");
	}
	Print("\n\r");
	if(POST_FLAG == -1)
	{
		// failed the timing requirements, edge was not seen in the time
		Stop_Timer2(); //Turn off timer 2
		return POSTFAIL();
	}
	Stop_Timer2();
	Print("POST test Passed!!\r\n");
	Print("Here is a Histogram of the data\r\n\n");	
	Pop_Histogram(lower_bound);
	Print(reruntheprogram);
	while(1)
	{
		char UserInput = USART_Read(USART2);
		if(UserInput == 'Y' || UserInput == 'y')
		{
			delta_time[999] = 0;
			return 1;
		}
		else if(UserInput == 'N' || UserInput == 'n'){ // User wants to exit the program
			USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n", 17);
			return (0); //offcially exit the program
		}
		else{//this is the case if the user inputted an invalied answer
			USART_Write(USART2, (uint8_t *)"Invalid Response Was Entered\r\n", 30);
		}
	}
}
//***************************************Welcome Message**********************************************//

int WELCOMEMESSAGE(void)
{
	//uint8_t *input_value;
	
	int j = 0;
	Print("Welcome to the Input Capture system\r\n");
	Print("The system generates a histogram of the period of a refrence clock. \n\r");
	Print("To use the system, enter a lower bound into the terminal below or hit ENTER to accept the default value of 950us.\n\r");
	USART_Write(USART2,(uint8_t*)"Enter lower bound: ",19);
	unsigned char USART_char = '0';
	uint8_t * ptr_USART_char = &USART_char;
	while(USART_char != '\r')
	{
		USART_char = USART_Read(USART2);
		USART_Write(USART2,ptr_USART_char,1);
		if(isdigit(*ptr_USART_char))
		{
			UserInput[j] = *ptr_USART_char;
			j++;
		}
	}
	Print("\n\r");
	return Parse_Str_To_Int(UserInput,j);
}
//***************************************************************************************************//
//parse User_Input to ints to create lower limit
int Parse_Str_To_Int(unsigned char * arrayStart, int sizeOfArray)
{
	int output = 0;
	for(int i = 0; i < sizeOfArray; i++)
	{
		 output += (*(arrayStart+i) - '0') * pow(10,(sizeOfArray-1)-i);
	}
	return output;
}
//***************************************************************************************************//
//verify input
int Verify_Input(int input)
{
	if(input >= 50 && input <= 9950)return 0;
	else if(input == 0){return 1;}
	else return -1;
}
//***************************************************************************************************//
//this is a functions that goes through the array of the times and sorts them from lowest to greatest
//***************************************************************************************************//	
	
void SORTER(uint32_t A[], int size)
{
	for(int i=0; i<size; i++)
	{
		for(int j=0; j<size-1; j++)
		{
			if( A[j] > A[j+1] )
			{
				int temp = A[j];
				A[j] = A[j+1];
				A[j+1] = temp;
			}
		}
	}
}
//***************************************************************************************************//
//Populate a histogram array which is the # of times a specific value has been seen
//***************************************************************************************************//
void Pop_Histogram (int lower_bound)
{
	int *histogram = malloc(10*sizeof(int));
	int *buckets = malloc(10*sizeof(int));
	uint32_t tmp = 0;
	int j = 0;
	int n;
	int upper_bound = 100 + lower_bound;
	SORTER(delta_time,1000);
	for(int i = 0; i < 1000; i++)
	{
		if(get_delta_time(i) > lower_bound && get_delta_time(i) < upper_bound)
		{
			if(i == 0)
			{
				j = 0;
				tmp = get_delta_time(i);
				histogram[j] = 1;
				buckets[j] = tmp;
			}
			else if(tmp == get_delta_time(i))
			{
				histogram[j] = histogram[j] + 1;
			}
			else
			{
				tmp = get_delta_time(i);
				j++;
				histogram[j] = 1;
				buckets[j] = tmp;
			}
		}
	}
	for(int i = 0; i < 10; i++)
	{
		if(histogram[i] != 0)
		{
			n = sprintf((char *)buffer, "%d\t", buckets[i]);
			USART_Write(USART2, buffer, n);
			n = sprintf((char *)buffer, "%d\r\n", histogram[i]);
			USART_Write(USART2, buffer, n);
			
		}
	}
	memset(buckets,0,10*sizeof(int));
	memset(histogram,0,10*sizeof(int));
	free(histogram);
	free(buckets);
	Print("\n");
}
// this is a function to Show the elements that were loaed into the aray 
//***************************************************************************************************//
// void histogram( void ){
//
//     int sizeOfArray = sizeof(time_stamp)/sizeof(time_stamp[0]); // Size of array
//     int Sample; // value of bucket
//     int indexi;    // index of bucket
//     int x;               // index of loop for bucket
//     int y;
//     quicksort(time_stamp); // calling the ssort funtnction
//     USART_Write(USART2, (uint8_t *)" Number || Count\r\n\n", 20);
//     USART_Write(USART2, (uint8_t *)"==================\r\n\r\n", 19);
//     for ( i = 0; i < sizeOfArray; i++ ){ // Loop through entire array and print out sample and the enumeration of the element
//         Sample = 0;
//         Sample = time_stamp[i]; //make sure sample is equal to the array to manulate it
//         if ( Sample >= lowestboundary && Sample <= highestboundary ){ // Sample is within defined bounds
//             if ( i == 0 ){ // First element in array
//                 Sample = count(time_stamp, Sample, time_stamp);
//                 n = sprintf((char *)buffer, "%d || %d\r\n", Sample, indexi);
//                 USART_Write(USART2, buffer, n);
//             }
//             else{ // if we are not at the first bucket
//                 if (Sample == time_stamp[i-1] ){ // if that number in that bukcket is the same, then we must skip over it
//                     ;
//                 }
//                 else{ // unique number
//                     Sample = count(time_stamp, Sample, sizeOfArray);
//                     n = sprintf((char *)buffer, "%d || %d\r\n", Sample, Sample);
//                     USART_Write(USART2, buffer, n);
//                 }
//             }
//         }
//         else{ //skip samples we dont care about
//             ;
//         }
//     }
//
// 	return rerunFunc();
// }

//***************************************************************************************************//	
//***************************************************************************************************//	
//***************************************************************************************************//	