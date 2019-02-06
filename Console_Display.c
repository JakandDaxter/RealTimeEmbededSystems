#include "Console_Display.h"
#include "Timer_2_Input_Capture.h"
#include "LED.h"

uint8_t buffer[BufferSize];
uint8_t bounds[BufferSize];
uint8_t nbounds[BufferSize];
char boundBuff[5]; //because the number they input can not be more than 5 digits

char FaileMessage [] = " !!The Post has failed due to no pulse seen in 100ms!!\r\n";
char reruntheprogram [] = " Would You Like To Rerun the Program? (Y or N):\r\n";
char originalbounds [] = "!! You are currently using the default bounds of ** 950 microseconds & 1050 microseconds ** !!\r\n";
char changebounds [] = "Would you like to change the bounds? (Y or N) \r\n";
int lowestboundary = 950; //default value for the lowest boundary
int highestboundary = 1050; //default value for the higest boundary

extern int POST_FLAG;

// This is the POST console function. Prints to the console to let the user know the post test failed
// lets the user rerun the post test if need be
int POSTFAIL(void){
	char UserInput;
	USART_Write(USART2,(uint8_t *)FaileMessage, strlen(FaileMessage));
	USART_Write(USART2,(uint8_t *)reruntheprogram, strlen(reruntheprogram));
	UserInput = USART_Read(USART2);
	if(UserInput == 'N' || UserInput == 'n'){ // User wants to exit the program
		USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n\r\n", 14);
		return (0); //offcially exit the program
	}
	else if(UserInput == 'Y' || UserInput == 'y'){// User wants to rerun the program
		USART_Write(USART2, (uint8_t *)"Rerunning the POST test......\r\n\r\n", 20);
		Red_LED_Off();
		POST_FLAG = 0;
		return POST();
	}
	else{//this is the case if the user inputted an invalied answer
		USART_Write(USART2, (uint8_t *)"Invalid Response Was Entered\r\n\r\n", 22);
		//return FAIL();
		return -1;
	}
}

//This is the post function that will tell the user if the test failed or succeded 
int POST(void)
{
	//start the timer
	Start_Timer2(); 
	//start of the test time 
	while(POST_FLAG == 0)
	{
		//do nothing
	}
	if (POST_FLAG == 1) {//the edge has been sucessfully seen
			USART_Write(USART2,(uint8_t *)"POST test Passed!! Moving onto next Program.......\r\n\r\n\n",20);
			//TIM2->CR1 &= ~(TIM_CR1_CEN); //Turn off timer 2 //somehow make this a fcuntion
			return 1;
	}
	else
	{// failed the timing requirements, edge was not seen in the time
		USART_Write(USART2, (uint8_t *)"!!POST HAS FAILED!!\r\n\r\n",17);
		TIM2->CR1 &= ~(TIM_CR1_CEN); //Turn off timer 2 //somehow make this a fcuntion
		return POSTFAIL();
	}
}
//***************************************************************************************************//

//this is for when the user wants ot edit the bounds	
void reruns(void){
int h;
char UserInput;
int x;
}
//***************************************************************************************************//	
//this is a functions that goes through the array of the times and sorts them from lowest to greatest

void quicksort(int number[25],int first,int last){
   int i, j, pivot, temp;

   if(first<last){
      pivot=first;
      i=first;
      j=last;

      while(i<j){
         while(number[i]<=number[pivot]&&i<last)
            i++;
         while(number[j]>number[pivot])
            j--;
         if(i<j){
            temp=number[i];
            number[i]=number[j];
            number[j]=temp;
         }
      }

      temp=number[pivot];
      number[pivot]=number[j];
      number[j]=temp;
      quicksort(number,first,j-1);
      quicksort(number,j+1,last);

   }
}

