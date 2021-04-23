#ifndef __MATH_MACRO_H__
#define __MATH_MACRO_H__

#include <math.h>

#ifndef PI
#define PI M_PI
#endif

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define SQ(a)			 	((a) * (a))
#define ABS(a) 				( ((a) < 0) ? -(a) : (a) )
#define DBAND(a, l, h) 		( (((a) <= (h)) && ((a) >= (l))) ? 0 : (a) )
#define CLIP(a, l, h) 		( MAX((MIN((a), (h))), (l)) )
#define CLIPS(a, v) 		CLIP(a, -(v), (v))
#define CLIPL(a, l) 		( ((a) < (l)) ? (l) : (a))
#define CLIPH(a, h) 		( ((a) > (h)) ? (h) : (a))
#define SIGN(a) 			( ((a) > 0) ? 1 : (((a) == 0) ? 0 : -1) )
#define DIRECTION(a) 		( ((a) >= 0) ? 1 : -1 )
#define INRANGE(a, l, h) 	( (((a) > (l)) && ((a) <= (h))) || ((a) == (l)) )
#define ROUND(a) 			( ABS((a) - ((int)(a))) >= 0.5 ? ((int)(a)) + SIGN(a) : (int)(a) )
#define DEG_TO_RAD(a)		((a) * PI / 180.0)
#define RAD_TO_DEG(a)		((a) * 180.0 / PI)
#define LOGIC_XOR(a, b)		(((a) || (b)) && !((a) && (b)))

//MIX result = (1-k) * a + k * b
#define MIX(a, b, k)		( (k) * ((b) - (a)) + (a) )

//puts angle difference to {-pi, pi} interval
#define ANG_DELTA_FIX(x) (fabs(x) > PI ? (x) - 2.0 * PI * SIGN(x) : (x))

/*
DECAY
           _______
  1       /
  0   ___/
        l  h
*/
#define DECAY(a, l, h)     	CLIP((((a) - (l)) / ((h) - (l))), 0.0, 1.0)

/*
INV_DECAY
      ___
  1      \
  0       \_______
        l  h
*/
#define INV_DECAY(a, l, h) 	CLIP((((h) - (a)) / ((h) - (l))), 0.0, 1.0)



#endif
