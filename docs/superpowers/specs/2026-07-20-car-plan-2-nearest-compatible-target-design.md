# CarPlan 2 Near-Target Selection Design

## Goal

Prevent `car_plan_2` from continuing toward a farther beacon when a clearly nearer physical beacon is similarly aligned with the car's actual velocity, without reintroducing beacon-index jumps or unstable cross-camera switching.

## Assumptions

- At most two physical beacons are lit at the same time.
- All beacon candidates are mapped to Center coordinates before comparison.
- Beacon distance means Center-pixel distance from the fused car center at `g_car_lamp_fused.cx` and `g_car_lamp_fused.cy + 10 px`.
- Same-camera pixel distance is more reliable than cross-camera mapped distance.
- A target is easy to approach while moving when its planned direction and actual car velocity have cosine at least `0.8`.
- `area` represents detection quality only; it is not a reliable cross-camera distance measure.

## Existing Behavior Preserved

- Scan `beacon_data[0]` and `[1]` from Front, Center, and Back.
- Merge same-camera detections within `8 px` and cross-camera detections within `15 px`.
- Match the locked physical beacon within `25 px` of its predicted position.
- Preserve Front-to-Center handoff through the same physical cluster.
- Hold a temporarily missing target for about `100 ms`.
- Preserve the existing `100 ms` severe velocity-conflict switch.
- Continue generating speed from the mapped beacon and `g_car_lamp_fused` geometry.

## Selection Rules

### Initial Acquisition

1. Generate the planned direction, distance, and velocity cosine for every valid physical beacon cluster.
2. When car speed is below `0.3 m/s`, select the nearest cluster.
3. When car speed is at least `0.3 m/s`, select the nearest cluster whose velocity cosine is at least `0.8`.
4. If no cluster passes the direction gate, select the cluster with the greatest velocity cosine.
5. Use `max_area` only as the final tie-breaker when the distance difference is at most `1 px` and the velocity-cosine difference is at most `0.02`.

### Locked Target Update

1. Match and update the existing locked physical beacon exactly as before.
2. Preserve the existing severe velocity-conflict switch when the locked target cosine is below `0.2` and a challenger cosine is above `0.85` for about `100 ms`.
3. Independently evaluate a nearer-target switch only when both the locked target and challenger have velocity cosine at least `0.8`, or when car speed is below `0.3 m/s`.
4. For two clusters observed by at least one common camera, the challenger must be no farther than `75%` of the locked distance and at least `8 px` nearer for `5` consecutive 100 Hz updates.
5. For clusters without a common camera, the challenger must be no farther than `60%` of the locked distance and at least `15 px` nearer for `8` consecutive 100 Hz updates.
6. Reset the nearer-target counter immediately when any required condition fails.
7. A closer target that fails the direction gate cannot break the lock.

## Minimal State Change

- Add one camera bit mask to each temporary physical cluster so same-camera and cross-camera comparisons can be distinguished.
- Add one `uint8` nearer-target persistence counter to the lock state.
- Do not add a weighted score, metric-distance estimator, target history array, or new public parameter.

## Data Flow

`six raw candidates -> Center mapping -> physical clusters -> direction/distance evaluation -> locked-target match -> severe-conflict check -> nearer-target check -> speed output`

## Verification

- At log timestamp `70734 ms`, Front `[0]` at about `15.7 px` and cosine `0.95` must replace Front `[1]` at about `30.7 px` and cosine `0.999` after about `50 ms`.
- In the cross-camera episode near `78413 ms`, the roughly `7.4 px` compatible target must replace the roughly `36.3 px` target after about `80 ms`.
- A nearer target behind a car moving forward at about `1.3 m/s` must not break a direction-compatible lock.
- Swapping raw beacon indices `[0]` and `[1]` must not change the selected physical target.
- A Front-to-Center observation handoff of the same physical beacon must not reset the lock or create a one-frame zero output.
- Existing loss hold and severe velocity-conflict switching must retain their current timing.

## Scope

Only `car_plan_2.c` is an implementation target. No change is planned for `car_plan_2.h`, `car_plan`, communication parameters, Mode3 control, or car-side firmware.
