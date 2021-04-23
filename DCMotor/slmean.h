#ifndef __SLMENA_H__
#define __SLMEAN_H__

#include "math_macro.h"

#define SLMEAN_MAX_LEN	64

typedef struct
{
	//internals
	int len;
	int ptr;
	double d[SLMEAN_MAX_LEN];
	//output variables
	double y;
} slmean_t;


static inline void slmean_init(slmean_t *inst, double u, int len)
{
	inst->len = CLIP(len, 0, SLMEAN_MAX_LEN);
	inst->ptr = 0;
	inst->y = u;
		
	for(int i = 0; i < inst->len; i++)
	{
		inst->d[i] = u;
	}
}

static inline double slmean_process(slmean_t *inst, double u)
{
	double y = 0;
	
	if(inst->len < 2)
	{
		inst->y = u;
		return inst->y;
	}
	
	inst->d[inst->ptr] = u;
	for(int i = 0, p = inst->ptr; i < inst->len; i++, p = (p + 1) % inst->len)
	{
		y += inst->d[p];
	}
	inst->ptr = (inst->ptr + 1) % inst->len;
	inst->y = y / (double)inst->len;
	return inst->y;
}








#endif
