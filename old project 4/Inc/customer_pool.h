#ifndef __CUSTOMER_POOL_H
#define __CUSTOMER_POOL_H

#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "bank_simulation.h"

#define POOL_SIZE 5

void generate_pool(void);
struct Customer_Ts* find_free_customer(void);
void free_customer(int index);
#endif /*__CUSTOMER_POOL_H*/