#ifndef _DRIVER_QUEUE_H_
#define _DRIVER_QUEUE_H_

#include "main.h"
#define UART_RXSIZE_QUEUE    256
typedef struct
{
	volatile uint16_t qin; 
	volatile uint16_t qout;
	volatile uint16_t bufsize;
	volatile uint8_t *data;
} Queue;

void Queue_Init(Queue *Q, uint8_t *bufin, uint16_t bz);
int Queue_read(Queue *Q);
void Queue_wirte(Queue *Q, uint8_t x);

#endif /* DRIVERS_DRIVER_QUEUE_H_ */

