#include "Console_Display.h"
#include "Timer_2_Input_Capture.h"
#include "LED.h"

uint8_t buffer[BufferSize];
uint8_t bounds[BufferSize];
uint8_t nbounds[BufferSize];
char boundBuff[4]; //because the number they input can not be more than 5 digits

char FaileMessage [] = " !!The Post has failed due to no pulse seen in 100ms!!\r\n\r\n\r\n\n\n";
char reruntheprogram [] = " Would You Like To Rerun the Program? (Y or N):\r\n\r\n\r\n\n\n";
char originalbounds [] = "!! You are currently using the default bounds of ** 950 microseconds & 1050 microseconds ** !!\r\n\r\n\n\n\r\n";
char changebounds [] = "Would you like to change the bounds? (Y or N) \r\n\r\n\r\n\n\n";
char starttheprogram [] = "Enter 'Y' To be Brought to the POST test, Enter 'N' to end the program completly\r\n\r\n\n\n";
int lowestboundary = 950; //default value for the lowest boundary
int highestboundary = 1050; //default value for the higest boundary
int STARTPRG = 0;
extern int POST_FLAG;

// This is the POST console function. Prints to the console to let the user know the post test failed
// lets the user rerun the post test if need be
int POSTFAIL(void){
	char UserInput;
	USART_Write(USART2,(uint8_t *)FaileMessage, strlen(FaileMessage));
	USART_Write(USART2,(uint8_t *)reruntheprogram, strlen(reruntheprogram));
	UserInput = USART_Read(USART2);
	if(UserInput == 'N' || UserInput == 'n'){ // User wants to exit the program
		USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n\r\n\n\n", 20);
		return (0); //offcially exit the program
	}
	else if(UserInput == 'Y' || UserInput == 'y'){// User wants to rerun the program
		USART_Write(USART2, (uint8_t *)"Rerunning the POST test......\r\n\r\n\n\n", 20);
		Red_LED_Off();
		POST_FLAG = 0;
		return POST();
	}
	else{//this is the case if the user inputted an invalied answer
		USART_Write(USART2, (uint8_t *)"Invalid Response Was Entered\r\n\r\n\n\n", 22);
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
			USART_Write(USART2,(uint8_t *)"POST test Passed!! Moving onto next Program.......\r\n\r\n\n\n\n",20);
			//TIM2->CR1 &= ~(TIM_CR1_CEN); //Turn off timer 2 //somehow make this a fcuntion
			return 1;
	}
	else
	{// failed the timing requirements, edge was not seen in the time
		USART_Write(USART2, (uint8_t *)"!!POST HAS FAILED!!\r\n\r\n\n\n",17);
		TIM2->CR1 &= ~(TIM_CR1_CEN); //Turn off timer 2 //somehow make this a fcuntion
		return POSTFAIL();
	}
}
//***************************************Welcome Message**********************************************//

int WELCOMEMESSAGE(void)
{
	char UserInput;
	char input_value[4];
	static int j = 0;
	//
	if(STARTPRG == 0)
	{

		USART_Write(USART2, (uint8_t *)"!!Welcome To The Main Menue!!\r\n\n\n\n", 32);
		USART_Write(USART2, (uint8_t *)"Would You Like To Start The Program?\r\n\n\n\n", 37);
		unsigned char USART_char = USART_Read(USART2);
		while(USART_char != '\r')
		{
			USART_char = USART_Read(USART2);
			input_value[j] = USART_char;
			j++;
		}
		
		USART_Write(USART2,(uint8_t *)starttheprogram, strlen(starttheprogram)); //telling the use what would happened based on their input
		if(UserInput == 'N' || UserInput == 'n'){ // User wants to exit the program
			USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n\r\n\n\n", 22);
			return(0);
		}
		else if(UserInput == 'Y' || UserInput == 'y'){// User wants to run the program
			STARTPRG = 1;
			return(1);
		}
		else{//this is the case if the user inputted an invalied answer
			USART_Write(USART2, (uint8_t *)"Invalid Response Was Entered\r\n\r\n\n\n", 28);
			return WELCOMEMESSAGE();
			//return -1;
		}
		
		
	}	
	
}
//***************************************************************************************************//
	//have a global variable that will be set to a 1 if the person entered a Y to start the program but return 0 if they entered a N

//***************************************************************************************************//
//this is for when the user wants ot edit the bounds	
int reruns(void){
int h;
char UserInput;
int index = 0;
int x;

	USART_Write(USART2,(uint8_t *)originalbounds, strlen(originalbounds));
    USART_Write(USART2,(uint8_t *)changebounds, strlen(changebounds));
	UserInput = USART_Read(USART2);
	if(UserInput == 'N' || UserInput == 'n'){ // User wants to use the original bounds of the program
        n = sprintf((char *)bounds, "Running with [%d] and [%d] Which Are the Original Bounds\r\n\n\n", lowestboundary, highestboundary);
        USART_Write(USART2, bounds, n);
	    else if (rxByte == 'Y' || rxByte == 'y'){ //
	        USART_Write(USART2, (uint8_t *)"Changing Bounds\r\n\n\n", 17);
	        USART_Write(USART2, (uint8_t *)"Enter A New Lower Bound: ", 24);
	        rxByte = USART_Read(USART2);
	        for (j = 0; j < sizeof(boundBuff)/sizeof(boundBuff[0]); j++){ // Populate array with null terminators
	            boundBuff[j] = '\0';
	        }
	        while(rxByte != 0xD){ // No carriage return seen - oxD is hex ascii equivalent of \r - keep appending input to buffer
	            memset( buffer, '\0', sizeof(buffer)); // Null terminator to indicate end of string
	            sprintf((char *)buffer, "%c", rxByte);
	            USART_Write(USART2, buffer, sizeof(buffer));
	            boundBuff[index] = rxByte;
	            index++;
	            rxByte = USART_Read(USART2);
	        }
	        sscanf(boundBuff, "%d", &lowestboundary);
	        if ( lowestboundary < 50 || lowestboundary > 9950 ) { // Ensure that the new lower bound satisfies requirements (between 50 and 9950 micro)
	            USART_Write(USART2, (uint8_t *)"\r\nPlease enter a number between 50 and 9950\r\n\r\n", 47);
	            run();
	        }
	        defaultHigh = lowestboundary + 100; // Upper bound must be 100 more than the lower bound
	        n = sprintf((char *)nbounds, "\r\nNow running with [%d] and [%d]\r\n\r\n", lowestboundary, highestboundary);
	        USART_Write(USART2, nbounds, n); 
	    }
	    else {
	        USART_Write(USART2, (uint8_t *)"Invalid Response\r\n\r\n", 22);
	        run();
	    }
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

