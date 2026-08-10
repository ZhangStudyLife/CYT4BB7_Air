#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Image/car_lamp_cross_check.h"

#define TEST_EPSILON (0.01f)

_Static_assert(sizeof(car_lamp_track_snapshot_t) == 32U,
               "authority snapshot wire layout changed");
_Static_assert(sizeof(car_lamp_cross_check_diag_t) == 96U,
               "shared diagnostic layout changed");

static void set_camera_frame(image_sync_set_t *set,
                             image_camera_e camera,
                             uint32 sequence,
                             uint32 time_ms,
                             float center_x,
                             float center_y,
                             uint8 measured)
{
    car_lamp_projection_point_t center;
    car_lamp_projection_point_t source;
    struct image_data *data = &set->camera[camera];
    image_frame_meta_t *meta = &set->meta[camera];

    image_data_clear(data);
    image_frame_meta_clear(meta, camera);
    meta->frame_sequence = sequence;
    meta->capture_time_ms = time_ms;
    meta->frame_valid = 1U;
    meta->timestamp_valid = 1U;
    if(measured == 0U)
    {
        return;
    }

    center.x = center_x;
    center.y = center_y;
    assert(CarLampProjection_FromCenter(camera, &center, &source) != 0U);
    data->car_lamp_data[0].valid = 1U;
    data->car_lamp_data[0].cx = source.x;
    data->car_lamp_data[0].cy = source.y;
    data->car_lamp_data[0].width = 4.0f;
    data->car_lamp_data[0].length = 12.0f;
    data->car_lamp_data[0].angle = 0.0f;
    data->car_lamp_measured_mask = 0x01U;
}

static void update_at(image_sync_set_t *set, uint32 now_ms,
                      float roll_deg, float pitch_deg)
{
    (void)CarLampCrossCheck_UpdateAt(
        set, now_ms, roll_deg, pitch_deg, 1000.0f, 1U, 1U);
}

static void acquire_center(image_sync_set_t *set)
{
    uint32 sequence;

    for(sequence = 1U; sequence <= 3U; sequence++)
    {
        uint32 time_ms = sequence * 20U;

        set_camera_frame(set, Center, sequence, time_ms,
                         (float)(sequence - 1U), 0.0f, 1U);
        update_at(set, time_ms, 0.0f, 0.0f);
    }
}

static void test_single_camera_acquire_and_prior_roi(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    set_camera_frame(&set, Center, 1U, 20U, 0.0f, 0.0f, 1U);
    update_at(&set, 20U, 0.0f, 0.0f);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_ACQUIRE);
    set_camera_frame(&set, Center, 2U, 40U, 1.0f, 0.0f, 1U);
    update_at(&set, 40U, 0.0f, 0.0f);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_ACQUIRE);
    set_camera_frame(&set, Center, 3U, 60U, 2.0f, 0.0f, 1U);
    update_at(&set, 60U, 0.0f, 0.0f);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_TRACKED);
    assert(CarLampCrossCheck_GetDiag()->roi_hit_mask == 0U);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_STRONG);
    assert(snapshot.roi_mode == 0U);
    assert(fabsf(snapshot.center_x - 2.0f) < TEST_EPSILON);

    set_camera_frame(&set, Center, 4U, 80U, 3.0f, 0.0f, 1U);
    update_at(&set, 80U, 0.0f, 0.0f);
    assert((CarLampCrossCheck_GetDiag()->roi_hit_mask &
            CAR_LAMP_CAMERA_BIT(Center)) != 0U);
    assert(g_car_lamp_roi_sample_count[Center] == 1U);
}

static void test_remote_source_does_not_wait_for_center(void)
{
    image_sync_set_t set;
    uint32 sequence;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    for(sequence = 1U; sequence <= 3U; sequence++)
    {
        uint32 time_ms = 100U + sequence * 20U;

        set_camera_frame(&set, Front, sequence, time_ms,
                         4.0f, -2.0f, 1U);
        update_at(&set, time_ms, 0.0f, 0.0f);
    }
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_TRACKED);
    assert(CarLampCrossCheck_GetDiag()->support_camera_mask ==
           CAR_LAMP_CAMERA_BIT(Front));
}

static void test_untimed_frames_are_not_evidence(void)
{
    image_sync_set_t set;
    uint32 sequence;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    for(sequence = 1U; sequence <= 3U; sequence++)
    {
        set_camera_frame(&set, Front, sequence, sequence * 20U,
                         0.0f, 0.0f, 1U);
        set.meta[Front].timestamp_valid = 0U;
        update_at(&set, sequence * 20U, 0.0f, 0.0f);
    }
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_SEARCH);
    assert(g_car_lamp_invalid_time_reject_count == 3U);
}

static void test_age_boundaries_and_duplicate_no_renew(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;
    car_lamp_roi_t roi;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    acquire_center(&set);

    update_at(&set, 90U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_STRONG);
    assert(snapshot.state == CAR_LAMP_TRACK_TRACKED);

    update_at(&set, 91U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_POSITIVE);
    assert(snapshot.state == CAR_LAMP_TRACK_COAST);

    update_at(&set, 110U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_POSITIVE);
    update_at(&set, 111U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_LOCATE);
    assert(snapshot.state == CAR_LAMP_TRACK_LOST);

    update_at(&set, 140U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_LOCATE);
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 140U, &roi) != 0U);
    update_at(&set, 141U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_NONE);
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 141U, &roi) == 0U);
    assert(g_car_lamp_duplicate_frame_reject_count >= 5U);
}

static void test_positive_frame_cannot_extend_track(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    acquire_center(&set);
    set_camera_frame(&set, Front, 1U, 80U, 2.0f, 0.0f, 1U);
    update_at(&set, 120U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.last_support_time_ms == 60U);
    assert(snapshot.state == CAR_LAMP_TRACK_LOST);
    assert(snapshot.quality == CAR_LAMP_EVIDENCE_LOCATE);
}

static void test_stale_and_out_of_order_frames_cannot_update(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    acquire_center(&set);

    set_camera_frame(&set, Center, 2U, 70U, 40.0f, 0.0f, 1U);
    update_at(&set, 70U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.reference_time_ms == 60U);
    assert(fabsf(snapshot.center_x - 2.0f) < TEST_EPSILON);

    set_camera_frame(&set, Center, 4U, 10U, 40.0f, 0.0f, 1U);
    update_at(&set, 100U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.last_support_time_ms == 60U);
    assert(g_car_lamp_stale_frame_reject_count == 1U);
}

static void test_source_sequence_rebases_after_restart_gap(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;
    uint32 sequence;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    acquire_center(&set);
    set_camera_frame(&set, Center, 100U, 80U, 3.0f, 0.0f, 1U);
    update_at(&set, 80U, 0.0f, 0.0f);

    for(sequence = 7U; sequence <= 9U; sequence++)
    {
        uint32 time_ms = 180U + (sequence - 7U) * 20U;

        set_camera_frame(&set, Center, sequence, time_ms,
                         4.0f, 0.0f, 1U);
        update_at(&set, time_ms, 0.0f, 0.0f);
    }
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.state == CAR_LAMP_TRACK_TRACKED);
    assert(snapshot.last_support_time_ms == 220U);
    assert(fabsf(snapshot.center_x - 4.0f) < TEST_EPSILON);
}

static void test_last_measurement_and_filtered_velocity(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;
    car_lamp_roi_t roi;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    acquire_center(&set);
    set_camera_frame(&set, Center, 4U, 80U, 3.0f, 0.0f, 1U);
    update_at(&set, 80U, 0.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.center_x - 3.0f) < TEST_EPSILON);
    assert(fabsf(snapshot.velocity_x - 15.0f) < TEST_EPSILON);
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 100U, &roi) != 0U);
    assert(fabsf(roi.expected_x - 3.3f) < TEST_EPSILON);
}

static void test_roi_age_and_partial_intersection(void)
{
    car_lamp_track_snapshot_t snapshot;
    car_lamp_roi_t roi;
    car_lamp_projection_point_t candidate;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.sequence = 1U;
    snapshot.reference_time_ms = 100U;
    snapshot.last_support_time_ms = 100U;
    snapshot.center_x = 100.0f;
    snapshot.center_y = 0.0f;
    snapshot.quality = CAR_LAMP_EVIDENCE_STRONG;

    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 130U, &roi) != 0U);
    assert(roi.evidence_level == CAR_LAMP_EVIDENCE_STRONG);
    assert(fabsf(roi.expected_x - 100.0f) < TEST_EPSILON);
    assert(fabsf(roi.expected_center_x - 100.0f) < TEST_EPSILON);
    assert(fabsf(roi.max_x - CAR_LAMP_IMAGE_HALF_WIDTH) < TEST_EPSILON);
    assert(roi.min_x < CAR_LAMP_IMAGE_HALF_WIDTH);
    candidate.x = CAR_LAMP_IMAGE_HALF_WIDTH;
    candidate.y = 0.0f;
    assert(CarLampCrossCheck_CandidateMatchesRoi(
               &roi, Center, &candidate, 8.0f) != 0U);
    snapshot.center_x = 125.0f;
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 100U, &roi) == 0U);
    assert(CarLampCrossCheck_GetRoiAtWithLampHalfLength(
               &snapshot, Center, 100U, 12.0f, &roi) != 0U);
    assert(fabsf(roi.half_size - 34.0f) < TEST_EPSILON);
    snapshot.center_x = 100.0f;
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 131U, &roi) != 0U);
    assert(roi.evidence_level == CAR_LAMP_EVIDENCE_POSITIVE);
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 151U, &roi) != 0U);
    assert(roi.evidence_level == CAR_LAMP_EVIDENCE_LOCATE);
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 181U, &roi) == 0U);

    snapshot.reference_time_ms = 0xFFFFFFF0UL;
    snapshot.last_support_time_ms = 0xFFFFFFF0UL;
    snapshot.center_x = 0.0f;
    assert(CarLampCrossCheck_GetRoiAt(
               &snapshot, Center, 0x00000004UL, &roi) != 0U);
    assert(roi.evidence_level == CAR_LAMP_EVIDENCE_STRONG);
}

static void test_front_back_soft_pair_is_diagnostic_only(void)
{
    image_sync_set_t set;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    set_camera_frame(&set, Front, 1U, 100U, 0.0f, 0.0f, 1U);
    set_camera_frame(&set, Back, 1U, 100U, 18.0f, 0.0f, 1U);
    update_at(&set, 100U, 0.0f, 0.0f);
    assert(g_car_lamp_soft_pair_hint_count == 1U);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_ACQUIRE);
    assert(CarLampCrossCheck_GetDiag()->support_camera_mask == 0U);
}

static void test_high_tilt_requires_pair_to_acquire(void)
{
    image_sync_set_t set;
    uint32 sequence;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    for(sequence = 1U; sequence <= 3U; sequence++)
    {
        set_camera_frame(&set, Center, sequence, sequence * 20U,
                         0.0f, 0.0f, 1U);
        update_at(&set, sequence * 20U, 25.0f, 0.0f);
    }
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_ACQUIRE);

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    for(sequence = 1U; sequence <= 2U; sequence++)
    {
        uint32 time_ms = 100U + sequence * 20U;

        set_camera_frame(&set, Center, sequence, time_ms,
                         0.0f, 0.0f, 1U);
        set_camera_frame(&set, Front, sequence, time_ms - 2U,
                         0.0f, 0.0f, 1U);
        update_at(&set, time_ms, 25.0f, 0.0f);
    }
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_TRACKED);
}

static void test_async_pair_requires_both_sources_to_advance(void)
{
    image_sync_set_t set;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    set_camera_frame(&set, Center, 1U, 100U, 0.0f, 0.0f, 1U);
    set_camera_frame(&set, Front, 1U, 98U, 0.0f, 0.0f, 1U);
    update_at(&set, 100U, 25.0f, 0.0f);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_ACQUIRE);

    set_camera_frame(&set, Center, 2U, 120U, 0.0f, 0.0f, 1U);
    update_at(&set, 120U, 25.0f, 0.0f);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_ACQUIRE);

    set_camera_frame(&set, Front, 2U, 118U, 0.0f, 0.0f, 1U);
    update_at(&set, 120U, 25.0f, 0.0f);
    assert(CarLampCrossCheck_GetDiag()->state == CAR_LAMP_TRACK_TRACKED);
    assert(CarLampCrossCheck_GetDiag()->support_camera_mask ==
           (CAR_LAMP_CAMERA_BIT(Front) | CAR_LAMP_CAMERA_BIT(Center)));
}

static void test_high_tilt_single_camera_continues_but_cannot_take_over(void)
{
    image_sync_set_t set;
    car_lamp_track_snapshot_t snapshot;

    memset(&set, 0, sizeof(set));
    CarLampCrossCheck_Init(Center);
    acquire_center(&set);

    set_camera_frame(&set, Center, 4U, 80U, 3.0f, 0.0f, 1U);
    update_at(&set, 80U, 25.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.center_x - 3.0f) < TEST_EPSILON);
    assert(snapshot.support_camera_mask == CAR_LAMP_CAMERA_BIT(Center));

    set_camera_frame(&set, Front, 1U, 100U, 40.0f, 0.0f, 1U);
    update_at(&set, 100U, 25.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.center_x - 3.0f) < TEST_EPSILON);
    assert(snapshot.reference_time_ms == 80U);
    assert((CarLampCrossCheck_GetDiag()->conflict_camera_mask &
            CAR_LAMP_CAMERA_BIT(Front)) != 0U);

    set_camera_frame(&set, Center, 5U, 120U, 4.0f, 0.0f, 1U);
    update_at(&set, 120U, 25.0f, 0.0f);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.center_x - 4.0f) < TEST_EPSILON);
    assert(snapshot.support_camera_mask == CAR_LAMP_CAMERA_BIT(Center));
}

static void test_roi_mode_is_diagnostic_only(void)
{
    car_lamp_track_snapshot_t snapshot;

    CarLampCrossCheck_Init(Center);
    CarLampCrossCheck_SetRoiMode(1U);
    CarLampCrossCheck_GetSnapshot(&snapshot);
    assert(snapshot.roi_mode == 1U);
    assert(CarLampCrossCheck_GetDiag()->roi_mode == 1U);
    CarLampCrossCheck_SetRoiMode(0U);
}

static void test_runtime_fallback_counts_only_new_frames(void)
{
    CarLampCrossCheck_Init(Center);
    CarLampCrossCheck_ApplyRuntimeDiag(
        0U,
        (uint8)(CAR_LAMP_CAMERA_BIT(Front) |
                CAR_LAMP_CAMERA_BIT(Center)),
        0U,
        CAR_LAMP_CAMERA_BIT(Front));
    assert(g_car_lamp_full_frame_fallback_count == 1U);

    CarLampCrossCheck_ApplyRuntimeDiag(
        0U,
        (uint8)(CAR_LAMP_CAMERA_BIT(Front) |
                CAR_LAMP_CAMERA_BIT(Center)),
        0U,
        0U);
    assert(g_car_lamp_full_frame_fallback_count == 1U);

    CarLampCrossCheck_ApplyRuntimeDiag(
        0U,
        CAR_LAMP_CAMERA_BIT(Center),
        0U,
        CAR_LAMP_CAMERA_BIT(Center));
    assert(g_car_lamp_full_frame_fallback_count == 2U);
    assert(CarLampCrossCheck_GetDiag()->full_frame_fallback_count == 2U);
}

int main(void)
{
    test_single_camera_acquire_and_prior_roi();
    test_remote_source_does_not_wait_for_center();
    test_untimed_frames_are_not_evidence();
    test_age_boundaries_and_duplicate_no_renew();
    test_positive_frame_cannot_extend_track();
    test_stale_and_out_of_order_frames_cannot_update();
    test_source_sequence_rebases_after_restart_gap();
    test_last_measurement_and_filtered_velocity();
    test_roi_age_and_partial_intersection();
    test_front_back_soft_pair_is_diagnostic_only();
    test_high_tilt_requires_pair_to_acquire();
    test_async_pair_requires_both_sources_to_advance();
    test_high_tilt_single_camera_continues_but_cannot_take_over();
    test_roi_mode_is_diagnostic_only();
    test_runtime_fallback_counts_only_new_frames();
    puts("car_lamp_authority_test: ok");
    return 0;
}
