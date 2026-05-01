#include "Height_Est.h"
#include "zf_common_headfile.h"
#include "filter.h"
#include <math.h>

float g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
float g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
float g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
uint8 g_tof_fused_valid = 0U;
float g_tof_fused_vz_mps = 0.0f;
float g_height_fused_vz_mps = 0.0f;
extern volatile uint32 tick_1000us_cnt;

#define HEIGHT_EST_TOF1_INDEX           0U
#define HEIGHT_EST_TOF2_INDEX           1U
#define HEIGHT_EST_TOF3_INDEX           2U
#define HEIGHT_EST_TOF4_INDEX           3U
#define HEIGHT_EST_DT_S                 0.01f
#define HEIGHT_EST_MEDIAN_WIN           3U
#define HEIGHT_EST_STEP_LIMIT_MM        30.0f
#define HEIGHT_EST_RESIDUAL_GATE_MM     120.0f
#define HEIGHT_EST_AB_ALPHA             0.21f
#define HEIGHT_EST_AB_BETA              0.012f
#define HEIGHT_EST_PREDICT_HOLD_CNT     15U
#define HEIGHT_EST_VEL_DECAY            0.95f
#define HEIGHT_EST_WEIGHT_EPS           0.001f
#define HEIGHT_EST_HUBER_K_MM           20.0f
#define HEIGHT_EST_TOF_PITCH_ARM_MM     65.40f
#define HEIGHT_EST_TOF_ROLL_ARM_MM      84.81f

static Median_t s_tof_median[VL53L1X_SENSOR_COUNT];
static StepLim_t s_tof_step[VL53L1X_SENSOR_COUNT];
static float s_height_est_mm = (float)VL53L1X_VALID_RANGE_MAX;
static float s_height_est_vz_mps = 0.0f;
static uint8 s_height_est_ready = 0U;
static uint8 s_predict_hold_cnt = 0U;
static const float s_tof_pitch_sign[VL53L1X_SENSOR_COUNT] = {1.0f, -1.0f, 1.0f, -1.0f};
static const float s_tof_roll_sign[VL53L1X_SENSOR_COUNT] = {1.0f, 1.0f, -1.0f, -1.0f};

static float HeightEst_ClampHeightMm(float height_mm)
{
    if (height_mm < 0.0f)
    {
        return 0.0f;
    }
    if (height_mm > (float)VL53L1X_VALID_RANGE_MAX)
    {
        return (float)VL53L1X_VALID_RANGE_MAX;
    }
    return height_mm;
}

static void HeightEst_ResetChannel(uint8 index)
{
    Median_Reset(&s_tof_median[index]);
    StepLim_Reset(&s_tof_step[index]);
}

static void HeightEst_ResetAll(void)
{
    uint8 index;

    for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
    {
        Median_Init(&s_tof_median[index], HEIGHT_EST_MEDIAN_WIN);
        StepLim_Init(&s_tof_step[index], HEIGHT_EST_STEP_LIMIT_MM);
    }

    s_height_est_mm = (float)VL53L1X_VALID_RANGE_MAX;
    s_height_est_vz_mps = 0.0f;
    s_height_est_ready = 0U;
    s_predict_hold_cnt = 0U;

    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    g_tof_fused_valid = 0U;
    g_tof_fused_vz_mps = 0.0f;
    g_height_fused_vz_mps = 0.0f;
}

static float HeightEst_ProcessChannel(uint8 index, uint16 distance_mm)
{
    float height_mm = (float)distance_mm * g_euler.cos_pitch * g_euler.cos_roll;

    height_mm += s_tof_pitch_sign[index] * HEIGHT_EST_TOF_PITCH_ARM_MM * g_euler.sin_pitch;
    height_mm += s_tof_roll_sign[index] * HEIGHT_EST_TOF_ROLL_ARM_MM * g_euler.sin_roll * g_euler.cos_pitch;

    height_mm = HeightEst_ClampHeightMm(height_mm);
    height_mm = Median_Update(&s_tof_median[index], height_mm);
    height_mm = StepLim_Update(&s_tof_step[index], height_mm);

    return HeightEst_ClampHeightMm(height_mm);
}

static float HeightEst_ResidualWeight(uint8 valid, float residual_mm)
{
    float abs_residual_mm;

    if (0U == valid)
    {
        return 0.0f;
    }

    abs_residual_mm = fabsf(residual_mm);
    if (abs_residual_mm >= HEIGHT_EST_RESIDUAL_GATE_MM)
    {
        return 0.0f;
    }

    return 1.0f - abs_residual_mm / HEIGHT_EST_RESIDUAL_GATE_MM;
}

static float HeightEst_MedianMm(float *values, uint8 count)
{
    uint8 i;
    uint8 j;

    for (i = 0U; i < count; i++)
    {
        for (j = i + 1U; j < count; j++)
        {
            if (values[j] < values[i])
            {
                float temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }

    if (0U != (count & 1U))
    {
        return values[count >> 1U];
    }

    return 0.5f * (values[(count >> 1U) - 1U] + values[count >> 1U]);
}

void TOF_Init(void)
{
    HeightEst_ResetAll();
    VL53L1X_Init();
}

void TOF_update_100HZ(void)
{
    const VL53L1X_data_struct *tof_data = 0;
    uint8 index = 0U;
    uint8 sample_count = 0U;
    uint8 valid_count = 0U;
    uint8 meas_valid = 0U;
    uint8 hard_relock = 0U;
    float h_pred_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float v_pred_mps = 0.0f;
    float weighted_sum = 0.0f;
    float weight_sum = 0.0f;
    float center_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float tof_height_mm[VL53L1X_SENSOR_COUNT] = {
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM,
        (float)VL53L1X_INVALID_DISTANCE_MM
    };
    float q[VL53L1X_SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    float center_buf[VL53L1X_SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8 tof_valid[VL53L1X_SENSOR_COUNT] = {0U, 0U, 0U, 0U};
    float z_meas_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof1_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof2_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof3_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
    float log_tof4_height_mm = (float)VL53L1X_VALID_RANGE_MAX;

    VL53L1X_Update();
    tof_data = VL53L1X_GetData();

    g_tof1_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof2_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof3_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof4_height_mm = (float)VL53L1X_INVALID_DISTANCE_MM;
    g_tof_fused_valid = 0U;

    if (0U != s_height_est_ready)
    {
        v_pred_mps = s_height_est_vz_mps;
        h_pred_mm = HeightEst_ClampHeightMm(s_height_est_mm + v_pred_mps * HEIGHT_EST_DT_S * 1000.0f);
    }

    if (0 != tof_data)
    {
        if (0U != tof_data->valid[HEIGHT_EST_TOF1_INDEX])
        {
            tof_height_mm[HEIGHT_EST_TOF1_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF1_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF1_INDEX]);
            tof_valid[HEIGHT_EST_TOF1_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF1_INDEX);
        }

        if (0U != tof_data->valid[HEIGHT_EST_TOF2_INDEX])
        {
            tof_height_mm[HEIGHT_EST_TOF2_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF2_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF2_INDEX]);
            tof_valid[HEIGHT_EST_TOF2_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF2_INDEX);
        }

        if (0U != tof_data->valid[HEIGHT_EST_TOF3_INDEX])
        {
            tof_height_mm[HEIGHT_EST_TOF3_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF3_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF3_INDEX]);
            tof_valid[HEIGHT_EST_TOF3_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF3_INDEX);
        }

        if (0U != tof_data->valid[HEIGHT_EST_TOF4_INDEX])
        {
            tof_height_mm[HEIGHT_EST_TOF4_INDEX] = HeightEst_ProcessChannel(HEIGHT_EST_TOF4_INDEX, tof_data->distance_mm[HEIGHT_EST_TOF4_INDEX]);
            tof_valid[HEIGHT_EST_TOF4_INDEX] = 1U;
        }
        else
        {
            HeightEst_ResetChannel(HEIGHT_EST_TOF4_INDEX);
        }
    }
    else
    {
        HeightEst_ResetChannel(HEIGHT_EST_TOF1_INDEX);
        HeightEst_ResetChannel(HEIGHT_EST_TOF2_INDEX);
        HeightEst_ResetChannel(HEIGHT_EST_TOF3_INDEX);
        HeightEst_ResetChannel(HEIGHT_EST_TOF4_INDEX);
    }

    g_tof1_height_mm = tof_height_mm[HEIGHT_EST_TOF1_INDEX];
    g_tof2_height_mm = tof_height_mm[HEIGHT_EST_TOF2_INDEX];
    g_tof3_height_mm = tof_height_mm[HEIGHT_EST_TOF3_INDEX];
    g_tof4_height_mm = tof_height_mm[HEIGHT_EST_TOF4_INDEX];

    if (0U != s_height_est_ready)
    {
        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            if (0U != tof_valid[index])
            {
                q[index] = HeightEst_ResidualWeight(1U, tof_height_mm[index] - h_pred_mm);
                valid_count++;
                if (q[index] > 0.0f)
                {
                    center_buf[sample_count++] = tof_height_mm[index];
                }
            }
        }

        if (sample_count >= 2U)
        {
            center_mm = HeightEst_MedianMm(center_buf, sample_count);

            for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
            {
                if (q[index] > 0.0f)
                {
                    float deviation_mm = fabsf(tof_height_mm[index] - center_mm);
                    float robust_weight = 1.0f;

                    if (deviation_mm > HEIGHT_EST_HUBER_K_MM)
                    {
                        robust_weight = HEIGHT_EST_HUBER_K_MM / deviation_mm;
                    }

                    q[index] *= robust_weight;
                    weighted_sum += q[index] * tof_height_mm[index];
                    weight_sum += q[index];
                }
            }
        }
    }
    else
    {
        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            if (0U != tof_valid[index])
            {
                center_buf[sample_count++] = tof_height_mm[index];
                valid_count++;
            }
        }

        if (sample_count > 0U)
        {
            center_mm = HeightEst_MedianMm(center_buf, sample_count);

            for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
            {
                if (0U != tof_valid[index])
                {
                    float deviation_mm = fabsf(tof_height_mm[index] - center_mm);
                    float robust_weight = 1.0f;

                    if (deviation_mm > HEIGHT_EST_HUBER_K_MM)
                    {
                        robust_weight = HEIGHT_EST_HUBER_K_MM / deviation_mm;
                    }

                    weighted_sum += robust_weight * tof_height_mm[index];
                    weight_sum += robust_weight;
                }
            }
        }
    }

    if ((weight_sum <= HEIGHT_EST_WEIGHT_EPS) &&
        (valid_count >= 2U) &&
        (s_predict_hold_cnt >= HEIGHT_EST_PREDICT_HOLD_CNT))
    {
        hard_relock = 1U;
        weighted_sum = 0.0f;
        weight_sum = 0.0f;
        sample_count = 0U;

        for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
        {
            if (0U != tof_valid[index])
            {
                center_buf[sample_count++] = tof_height_mm[index];
            }
        }

        if (sample_count > 0U)
        {
            center_mm = HeightEst_MedianMm(center_buf, sample_count);

            for (index = 0U; index < VL53L1X_SENSOR_COUNT; index++)
            {
                if (0U != tof_valid[index])
                {
                    float deviation_mm = fabsf(tof_height_mm[index] - center_mm);
                    float robust_weight = 1.0f;

                    if (deviation_mm > HEIGHT_EST_HUBER_K_MM)
                    {
                        robust_weight = HEIGHT_EST_HUBER_K_MM / deviation_mm;
                    }

                    weighted_sum += robust_weight * tof_height_mm[index];
                    weight_sum += robust_weight;
                }
            }
        }
    }

    if (weight_sum > HEIGHT_EST_WEIGHT_EPS)
    {
        z_meas_mm = weighted_sum / weight_sum;
        meas_valid = 1U;
    }
    else
    {
        z_meas_mm = h_pred_mm;
    }

    if (0U == s_height_est_ready)
    {
        if (0U == meas_valid)
        {
            g_tof_fused_height_mm = (float)VL53L1X_VALID_RANGE_MAX;
            g_tof_fused_vz_mps = 0.0f;
            g_height_fused_vz_mps = 0.0f;
            return;
        }

        s_height_est_mm = HeightEst_ClampHeightMm(z_meas_mm);
        s_height_est_vz_mps = 0.0f;
        s_height_est_ready = 1U;
        s_predict_hold_cnt = 0U;
        g_tof_fused_valid = 1U;
    }
    else if (0U != meas_valid)
    {
        if (0U != hard_relock)
        {
            s_height_est_mm = HeightEst_ClampHeightMm(z_meas_mm);
            s_height_est_vz_mps = 0.0f;
        }
        else
        {
            float residual_mm = z_meas_mm - h_pred_mm;

            s_height_est_mm = HeightEst_ClampHeightMm(h_pred_mm + HEIGHT_EST_AB_ALPHA * residual_mm);
            s_height_est_vz_mps = v_pred_mps + (HEIGHT_EST_AB_BETA / HEIGHT_EST_DT_S) * (residual_mm * 0.001f);
        }

        s_predict_hold_cnt = 0U;
        g_tof_fused_valid = 1U;
    }
    else
    {
        s_height_est_mm = HeightEst_ClampHeightMm(h_pred_mm);
        s_height_est_vz_mps = v_pred_mps * HEIGHT_EST_VEL_DECAY;

        if (s_predict_hold_cnt < HEIGHT_EST_PREDICT_HOLD_CNT)
        {
            s_predict_hold_cnt++;
            g_tof_fused_valid = 1U;
        }
        else
        {
            g_tof_fused_valid = 0U;
        }
    }

    float acc_z_mps2 = AccelCalibration_GetAccelDownForOutputMps2();
    g_tof_fused_height_mm = s_height_est_mm;
    g_tof_fused_vz_mps = s_height_est_vz_mps;
    g_height_fused_vz_mps = s_height_est_vz_mps;
    log_tof1_height_mm = HeightEst_ClampHeightMm(g_tof1_height_mm);
    log_tof2_height_mm = HeightEst_ClampHeightMm(g_tof2_height_mm);
    log_tof3_height_mm = HeightEst_ClampHeightMm(g_tof3_height_mm);
    log_tof4_height_mm = HeightEst_ClampHeightMm(g_tof4_height_mm);
    /* 仅发送四路姿态解耦后的高度，1300 mm 视为无效，离线再重算融合参数 */
    FC_START_CRSF_state_e fc_state = FC_START_CRSF_Get_State();
    if (fc_state == FC_START_CRSF_STATE_FLYING)
    {
        wifi_justfloat(
            tick_1000us_cnt,
            log_tof1_height_mm, log_tof2_height_mm, log_tof3_height_mm, log_tof4_height_mm,
            g_euler.roll, g_euler.pitch, acc_z_mps2, g_tof_fused_vz_mps,g_tof_fused_height_mm
        );
    }
}

void Height_Est_update_100HZ(void)
{
    TOF_update_100HZ();
}
