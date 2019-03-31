#include "teller.h"
/////
struct Teller * createTeller(int id)
{
	struct Teller* teller = (struct Teller*)malloc(sizeof(struct Teller));
	teller->id = id;
	teller->customers_served = 0;
	teller->total_time_serving_customers = 0;
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