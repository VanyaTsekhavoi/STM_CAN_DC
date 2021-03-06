#ifndef __PPROF_H__
#define __PPROF_H__

#include <stdbool.h>
#include "math_macro.h"

typedef struct
{
	double pd;
	double p;
	double v;
	double a;
} pprof_t;


static inline void pprof_init(pprof_t *p)
{
    p->pd = 0;
    p->p = 0;
    p->v = 0;
    p->a = 0;
}

static inline void pprof_process(pprof_t *p, double pd, double vm, double am, double dt)
{
	vm = fabs(vm);
	am = fabs(am);
	p->pd = pd;
    double pl = (SQ(p->v) / 2 / am) * SIGN(p->v);
    
    // a @ dE/da = 0, where E = (dp + v*t + a*t^2/2)^2 + (v+a*t)^2 is
    // summ of squared position and velocity errors and dp = p - pd
    double amx = ABS(((2.0 * dt + 4.0 / dt) * ABS(p->v) + 
            2.0 * ABS(p->p - pd)) / (SQ(dt) + 4.0));
    
    double ansig = SIGN(pd - p->p - pl);
 	double an = ansig * CLIP(amx, -am, am);
 	
 	double dv = vm - fabs(p->v + an * dt);
 	if(dv < 0.0)
 	{
 		an = CLIP(SIGN(p->v) * dv / dt, -am, am);
 	}
    double vn = p->v + an * dt;
    double pn = p->p + vn * dt;
    
    p->a = an;
    p->v = vn;
    p->p = pn;
}

static inline void pprof_set(pprof_t *p, double pd, double v)
{
	p->pd = pd;
    p->p = pd;
    p->v = v;
    p->a = 0;
}

static inline bool pprof_is_running(pprof_t *p)
{
	return /*(p->a != 0) || (p->v != 0) ||*/ (fabs(p->p - p->pd) > 1.0e-6);
}

#endif

