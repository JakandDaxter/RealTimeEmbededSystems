#include "customer.h"

//typedef struct Customer_Ts 
//{

//	int customer_ET; //arrival time
//	int customer_LT; //leave queue time
//	int customer_MT; //hold the max value of that

//} Customer_Ts;

//struct Customer_Ts customer;

//////////
//void updateEnter(struct Customer_Ts time, int customer_arival)
//{
//	time.customer_ET = customer_arival;
//}
///////////
//void updateLeave(struct Customer_Ts time, int customer_left)
//{
//	time.customer_LT = customer_left;
//}
/////////
//void Customer_Maxupdate(struct Customer_Ts time)
//{
//	int temp;
//	
//	temp = abs( (time.customer_LT) - (time.customer_ET) );
//	
//	if(( time.customer_MT) < temp)
//	{
//	time.customer_MT = temp;
//	}

//}
///////////
//int getMaxTime(struct Customer_Ts time)
//{	
//	return time.customer_MT ;
//}
