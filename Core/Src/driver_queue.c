#include "driver_queue.h"
#include "string.h"

void Queue_Init(Queue *Q, uint8_t *bufin, uint16_t bz)
{
	Q->qin = 0;
	Q->qout = 0;
	Q->bufsize = bz;
	Q->data = bufin;
}

int Queue_read(Queue *Q)
{
	int x;
	
	if(Q->qin != Q->qout)
	{
		x = Q->data[Q->qout];
		Q->qout++;
		if(Q->qout >= Q->bufsize)
		{
			Q->qout = 0;
		}
	}
	else
	{
		x = -1;
	}

	return x;
}
void Queue_wirte(Queue *Q, uint8_t x)
{
	uint16_t xt;
	
	xt = Q->qin + 1; 
	if(xt >= Q->bufsize) 
	{
		xt = 0;
	}
	
	if(xt == Q->qout)  //Buffer full
	{
	 	Q->qout++;
		if(Q->qout >= Q->bufsize) 
		{
			Q->qout = 0;
		}
	}
	
	Q->data[Q->qin] = x;
	Q->qin++;
		if(Q->qin >= Q->bufsize) 
		{
			Q->qin=0;
		}
}

