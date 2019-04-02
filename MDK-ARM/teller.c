#include "teller.h"
/////
struct Teller * createTeller(int id)
{
	struct Teller* teller = (struct Teller*)malloc(sizeof(struct Teller));
	teller->id = id;
	teller->customers_served = 0;
	teller->total_time_serving_customers = 0;
	teller->busy_tick_count = 0;
	teller->idle_tick_count = 0;
	teller->prev_busy_tick_count = 0;
	teller->max_wait_time_t = 0;
	return teller;
}
//////
void destroyTeller(struct Teller* hTeller)
{
	free(hTeller);
}

////
void updateTeller(struct Teller* hTeller, uint32_t customer_tt)
{
	hTeller->total_time_serving_customers += customer_tt;
	hTeller->customers_served++;
	hTeller->busy_tick_count = xTaskGetTickCount();
	hTeller->idle_tick_count = (hTeller->busy_tick_count - hTeller->prev_busy_tick_count - customer_tt); 
	hTeller->prev_busy_tick_count = hTeller->busy_tick_count;
	if(hTeller->idle_tick_count > hTeller->max_wait_time_t)
	{
		hTeller->max_wait_time_t = hTeller->idle_tick_count;
	}
}
////
int getTellerCS(struct Teller* hTeller)
{
	return hTeller->customers_served;
}
/////
int getTellerTSC(struct Teller *hTeller)
{
	return hTeller->total_time_serving_customers;
}

int getTellerMaxWaitTime(struct Teller *hTeller)
{
	return hTeller->max_wait_time_t;
}