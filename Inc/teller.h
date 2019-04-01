#ifndef __TELLER_H
#define __TELLER_H

#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"

struct Teller
{
	int id;
	int customers_served;
	int total_time_serving_customers;
};
struct Teller * createTeller(int id);
void destroyTeller(struct Teller* hTeller);
void updateTeller(struct Teller* hTeller, uint32_t customer_tt);
int getTellerCS(struct Teller* hTeller);
int getTellerTSC(struct Teller *hTeller);
#endif /*__TELLER_H*/