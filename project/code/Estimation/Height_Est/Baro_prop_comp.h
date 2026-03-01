#ifndef BARO_PROP_COMP_H_
#define BARO_PROP_COMP_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float bias_on_pa;
    float tau_on_s;
    float tau_off_s;
    float ground_effect_height_cm;
    float ground_effect_scale;
    uint8 enable;
} BaroPropCompParam_t;

extern float g_baro_prop_bias_hat_pa;
extern float g_baro_pressure_comp_pa;

void Baro_PropComp_Init(void);
void Baro_PropComp_Reset(void);
void Baro_PropComp_SetEnable(uint8 enable);
void Baro_PropComp_SetParams(const BaroPropCompParam_t *param);
float Baro_PropComp_Apply(float pressure_pa, uint8 prop_spinning, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* BARO_PROP_COMP_H_ */
