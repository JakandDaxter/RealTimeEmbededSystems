# RealTimeEmbededSystems
Design and implement on the STM32L476 discovery board an embedded, stand-alone program to
simulate the workflow in a typical banking environment -- single queue with queuing to a multi-threaded
server or three separate server processes. This is a software only project - you will not need anything
other than the board and development platform.


Customers enter the bank to transact business on a regular basis. Each new customer arrives
every one to four minutes, based on a uniform random distribution. Each new customer enters a
single queue of all customers.
• Three tellers are available to service customers in the queue. As tellers become available,
customers leave the queue, approach the teller and conduct their business. Each customer
requires between30 seconds and 8 minutes for their transaction with the teller. The time required
for each transaction is based on a uniform random distribution.
• The bank is open for business between the hours of 9:00am and 4:00pm. Customers begin
entering when the bank opens in the morning, and stop entering when the bank closes in the
afternoon. Customers in the queue at closing time remain in the queue until tellers are available to
complete their transactions.

