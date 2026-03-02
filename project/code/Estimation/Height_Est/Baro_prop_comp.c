#include "Baro_prop_comp.h"
#include "TOF_data.h"

#define BARO_PROP_COMP_DEFAULT_ENABLE            (1U)
#define BARO_PROP_COMP_DEFAULT_BIAS_ON_PA        (6.00f)
#define BARO_PROP_COMP_DEFAULT_TAU_ON_S          (1.20f)
#define BARO_PROP_COMP_DEFAULT_TAU_OFF_S         (0.50f)
#define BARO_PROP_COMP_DEFAULT_GND_EFFECT_CM     (30.0f)
#define BARO_PROP_COMP_DEFAULT_GND_EFFECT_SCALE  (0.30f)
#define BARO_PROP_COMP_MIN_TAU_S                 (0.01f)

float g_baro_prop_bias_hat_pa = 0.0f;
float g_baro_pressure_comp_pa = 0.0f;

static BaroPropCompParam_t s_param = {
    BARO_PROP_COMP_DEFAULT_BIAS_ON_PA,
    BARO_PROP_COMP_DEFAULT_TAU_ON_S,
    BARO_PROP_COMP_DEFAULT_TAU_OFF_S,
    BARO_PROP_COMP_DEFAULT_GND_EFFECT_CM,
    BARO_PROP_COMP_DEFAULT_GND_EFFECT_SCALE,
    BARO_PROP_COMP_DEFAULT_ENABLE,
};

static float Baro_PropComp_Clampf(float x, float x_min, float x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    if (x > x_max)
    {
        return x_max;
    }
    return x;
}

static float Baro_PropComp_GetGroundScale(void)
{
    float h_cm;
    float ratio;
    float smooth_ratio;
    float ge_h_cm = s_param.ground_effect_height_cm;

    if ((0U == g_tof_fused_valid) || (ge_h_cm <= 0.0f))
    {
        return 1.0f;
    }

    h_cm = 0.1f * (float)g_tof_fused_height_mm;
    ratio = Baro_PropComp_Clampf(h_cm / ge_h_cm, 0.0f, 1.0f);
    smooth_ratio = ratio * ratio * (3.0f - 2.0f * ratio);

    return s_param.ground_effect_scale +
           (1.0f - s_param.ground_effect_scale) * smooth_ratio;
}

void Baro_PropComp_Init(void)
{
    g_baro_prop_bias_hat_pa = 0.0f;
    g_baro_pressure_comp_pa = 0.0f;
}

void Baro_PropComp_Reset(void)
{
    g_baro_prop_bias_hat_pa = 0.0f;
}

void Baro_PropComp_SetEnable(uint8 enable)
{
    s_param.enable = (0U != enable) ? 1U : 0U;
}

void Baro_PropComp_SetParams(const BaroPropCompParam_t *param)
{
    if (0 == param)
    {
        return;
    }

    s_param.bias_on_pa = param->bias_on_pa;
    s_param.tau_on_s = Baro_PropComp_Clampf(param->tau_on_s, BARO_PROP_COMP_MIN_TAU_S, 30.0f);
    s_param.tau_off_s = Baro_PropComp_Clampf(param->tau_off_s, BARO_PROP_COMP_MIN_TAU_S, 30.0f);
    s_param.ground_effect_height_cm = Baro_PropComp_Clampf(param->ground_effect_height_cm, 0.0f, 300.0f);
    s_param.ground_effect_scale = Baro_PropComp_Clampf(param->ground_effect_scale, 0.0f, 1.0f);
    s_param.enable = (0U != param->enable) ? 1U : 0U;
}

float Baro_PropComp_Apply(float pressure_pa, uint8 prop_spinning, float dt_s)
{
    float target_bias;
    float tau_s;
    float alpha;

    g_baro_pressure_comp_pa = pressure_pa;

    if (0U == s_param.enable)
    {
        g_baro_prop_bias_hat_pa = 0.0f;
        return pressure_pa;
    }

    if (0U != prop_spinning)
    {
        target_bias = s_param.bias_on_pa * Baro_PropComp_GetGroundScale();
    }
    else
    {
        target_bias = 0.0f;
    }

    tau_s = (target_bias >= g_baro_prop_bias_hat_pa) ? s_param.tau_on_s : s_param.tau_off_s;
    tau_s = Baro_PropComp_Clampf(tau_s, BARO_PROP_COMP_MIN_TAU_S, 30.0f);

    if (dt_s <= 0.0f)
    {
        alpha = 0.0f;
    }
    else
    {
        alpha = dt_s / (tau_s + dt_s);
    }
    alpha = Baro_PropComp_Clampf(alpha, 0.0f, 1.0f);

    g_baro_prop_bias_hat_pa += alpha * (target_bias - g_baro_prop_bias_hat_pa);

    g_baro_pressure_comp_pa = pressure_pa - g_baro_prop_bias_hat_pa;
    return g_baro_pressure_comp_pa;
}

