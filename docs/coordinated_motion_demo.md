# Coordinated Motion Demo

`coordinated_motion_demo`는 실행 당시의 자세를 기준으로 세 관절을 함께
움직인 뒤 시작 자세로 복귀하고 모터를 정지하는 1회성 데모다.

상완과 전완은 기구 구조를 반영해 항상 반대 방향으로 움직인다.

| Pose | Shoulder | UpperArm | ForeArm |
|---|---:|---:|---:|
| A | +5° | -4° | +6° |
| B | -5° | +4° | -6° |
| Home | 0° | 0° | 0° |

각 전환은 4초 동안 quintic smoothstep으로 보간하고, 자세마다 0.75초
유지한다. 목표점은 실행 시 읽은 시작 자세에 대한 상대각이다.

실행 전 다음 조건을 모두 검사한다.

- 세 관절의 limit가 `confirmed: true`인지
- 시작 자세와 모든 목표점이 soft limit 안인지
- 계획된 최대 속도가 각 관절의 `max_slew_rate_rad_s` 이하인지
- 세 관절의 fresh feedback이 존재하는지

## Build and run

```bash
cmake -S . -B build
cmake --build build --target coordinated_motion_demo -j"$(nproc)"

./build/coordinated_motion_demo \
  can0 \
  config/joint_limits.yaml \
  config/motor_config.yaml
```

`Space`을 누르면 현재 자세를 기준으로 계획을 생성하고 한 번 실행한다.
실행 중 `Space`, `Q`, 또는 `Ctrl+C`를 누르면 전체 모터를 정지하고
종료한다. CAN 오류, stale feedback, hard-limit 위반도 기존
`MotorDriver`의 latched fault를 통해 전체 정지를 발생시킨다.
