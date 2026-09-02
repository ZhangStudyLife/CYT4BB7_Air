/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#ifndef HEIGHT_EST_H_
#define HEIGHT_EST_H_

#include "../../HW_Drivers/VL53L1X/VL53L1X.h"
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HEIGHT_DROPOUT_NONE                0U
#define HEIGHT_DROPOUT_NO_ASCENT           1U
#define HEIGHT_DROPOUT_CONTROLLED_DESCENT  2U

extern float g_tof_fused_height_mm;
extern float g_tof1_height_mm;
extern float g_tof2_height_mm;
extern float g_tof3_height_mm;
extern float g_tof4_height_mm;
extern uint8 g_tof_fused_valid;
extern float g_height_fused_vz_mps;
extern float g_height_meas_health;
extern float g_height_acc_up_mps2;
extern uint32 g_height_tof_sample_seq;
extern uint8 g_height_tof_fresh_mask;
extern uint8 g_height_tof_valid_mask;
extern uint8 g_height_meas_valid;
extern uint8 g_height_inlier_count;
extern float g_height_meas_mm;
extern float g_height_residual_m;
extern float g_height_measurement_dt_s;
extern float g_height_inst_v_mps;
extern float g_height_acc_lpf_mps2;
extern float g_height_observer_v_mps;
extern uint16 g_height_miss_count;
extern float g_height_measurement_age_ms;
extern float g_height_tof_alpha;
extern float g_height_safe_upper_mm;
extern float g_height_safe_lower_mm;
extern float g_height_safe_brake_distance_mm;
extern uint8 g_height_dropout_mode;
extern uint8 g_height_pollution_active;
extern uint8 g_height_safety_valid;

void TOF_Init(void);
void Height_Est_predict_1000HZ(void);
void Height_Est_update_100HZ(void);

#ifdef __cplusplus
}
#endif

#endif
