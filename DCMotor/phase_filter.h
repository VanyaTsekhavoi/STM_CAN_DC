#ifndef PHASE_FILTER_H_
#define PHASE_FILTER_H_

#include "math_macro.h"
#include "fmath.h"

#include <stdint.h>


typedef struct
{
	float ki;
	float kp;
	float w_max;
} phase_filter_cfg_t;

typedef struct
{
	//velocity integrator
	float wa;
	//estimated velocity
	float we;
	//estimated single turn position
	float pe;
} phase_filter_t;

typedef struct
{
	//velocity integrator
	float wa;
	//fixed point position integrator
	int64_t peq;
	//fixed point unwrapped position integrator
	int64_t pequ;
	//estimated velocity
	float we;
	//estimated single turn position
	int64_t pe;
	//unwrapped estimated position
	int64_t peu;
} phase_unwrap_filter_t;

static inline float fit_phase(float theta)
{
	theta -= 2.0f * PI * (int32_t)(theta / PI / 2.0f);
	theta += (theta < 0) ? 2.0f * PI : 0.0f;
	return theta;
}

static inline void phase_filter_reset(phase_filter_t *inst)
{
	inst->wa = 0;
	inst->pe = 0;
}

static inline void phase_filter_init(phase_filter_t *inst)
{
	phase_filter_reset(inst);
}


static inline void phase_filter_update(phase_filter_t *inst, phase_filter_cfg_t *cfg, float inp, float dt)
{
	float ph_error;

	inst->pe = _MOD_(inst->pe + inst->we * dt, 2.0 * PI);
	ph_error = inp - inst->pe;

	if(ABS(ph_error) > PI)
	{
		ph_error -= 2.0f * PI * SIGN(ph_error);
	}

    inst->wa = CLIP(inst->wa + ph_error * cfg->ki * dt, 
			-cfg->w_max, cfg->w_max) ;
    inst->we = inst->wa + ph_error * cfg->kp;
}

static inline void phase_unwrap_filter_reset(phase_unwrap_filter_t *inst)
{
	inst->wa = 0;
	inst->pe = 0;
	inst->peu = 0;
	inst->peq = 0;
}

static inline void phase_unwrap_filter_init(phase_unwrap_filter_t *inst)
{
	phase_unwrap_filter_reset(inst);
}

//fractional part Q=15 limits actual output unwrapped resulution to signed 48 bit value
static inline void phase_unwrap_filter_update(phase_unwrap_filter_t *inst, phase_filter_cfg_t *cfg, 
		int64_t inp, int64_t max, float dt)
{
	const int Q = 15;
	int64_t err;
	int64_t maxq = max << Q;
	int64_t hmax = max / 2;

	int64_t delta = (inst->we * dt) * (float)(1ll << Q);

	inst->peq += delta;
	inst->pequ += delta;

	if(inst->peq >= maxq)
	{
		inst->peq -= maxq;
	}
	if(inst->peq < 0)
	{
		inst->peq += maxq;
	}

	inst->pe = inst->peq >> Q;
	inst->peu = inst->pequ >> Q;

	err = inp - inst->pe;

	if(err > hmax)
	{
		err -= max;
	}

	if(err < -hmax)
	{
		err += max;
	}

    inst->wa = CLIPS(inst->wa + err * cfg->ki * dt, cfg->w_max);
    inst->we = inst->wa + err * cfg->kp;
}

#endif /* PHASE_FILTER_H_ */
