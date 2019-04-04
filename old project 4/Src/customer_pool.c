#include "customer_pool.h"

struct Customer_Ts* customer_pool[POOL_SIZE];

void generate_pool()
{
	for(int i = 0; i < POOL_SIZE; i++)
	{
		struct Customer_Ts* customer = (struct Customer_Ts*)malloc(sizeof(struct Customer_Ts));
		customer->pool_index = i;
		customer->isFree = true;
		customer_pool[i] = customer;
	}
}

struct Customer_Ts* find_free_customer()
{
	for(int i = 0; i < POOL_SIZE; i++)
	{
		if(customer_pool[i]->isFree)
		{
			customer_pool[i]->isFree = false;
			return customer_pool[i];
		}
	}
}

void free_customer(int index)
{
	customer_pool[index]->isFree = true;
}