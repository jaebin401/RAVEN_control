# Coordinated Motion Demo

`coordinated_motion_demo`는 실행 당시의 자세를 기준으로 세 관절을 함께
움직인 뒤 시작 자세로 복귀하는 1회성 데모다. 복귀 후에는 모터를 자동으로
정지하지 않고 시작 자세를 유지하며 사용자의 비활성화 입력을 기다린다.

상완과 전완은 기구 구조를 반영해 항상 반대 방향으로 움직인다.

| Pose | Shoulder | UpperArm | ForeArm |
|---|---:|---:|---:|
| A | +10° | -8° | +12° |
| B | -10° | +8° | -12° |
| Home | 0° | 0° | 0° |

위 값은 라디안이 아니라 **도(degree)** 단위이며
`apps/coordinated_motion_demo.cpp`의 `POSE_A_OFFSET_DEG`와
`POSE_B_OFFSET_DEG`에서 수정할 수 있다.

전체 동작은 다음 순서로 진행한다.

```text
Start → Pose A → Pose B → Start
```

각 전환은 4초 동안 quintic smoothstep으로 보간하고, 자세마다 0.75초
유지한다. 목표점은 실행 시 읽은 시작 자세에 대한 상대각이다.

이동 구간에서는 quintic smoothstep의 해석적 미분으로 목표 속도를 함께
계산해 RS02 operation-control(MIT) 명령에 전송한다. 시작점과 끝점의 목표
속도는 0이며, 자세 유지 구간과 Home Hold에서도 0을 전송한다.

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
  config/motor_config.yaml \
  logs/my_demo.csv
```

네 번째 인자인 로그 경로는 생략할 수 있다. 생략하면
`logs/coordinated_motion_YYYYMMDD_HHMMSS.csv` 형식으로 자동 생성된다.
`logs/`는 Git 추적에서 제외된다.

`Space`을 누르면 현재 자세를 기준으로 계획을 생성하고 한 번 실행한다.
동작을 마치면 시작 자세를 계속 유지한다. 이 상태에서 `Space`을 누르면
전체 모터를 비활성화하고 종료한다.

실행 중 `Space`, `Q`, 또는 `Ctrl+C`를 누르면 긴급 중단한다. CAN 오류,
stale feedback, hard-limit 위반도 기존 `MotorDriver`의 latched fault를
통해 전체 정지를 발생시킨다.

## Motion diagnostics

제어 중에는 파일을 쓰지 않고 메모리에만 샘플을 저장한다. 사용자가 모터를
비활성화하거나 fault로 종료된 뒤 CSV를 한 번에 기록한다.

CSV에는 다음 값이 포함된다.

- 제어 주기 `dt`, 예약 시각 대비 lateness, deadline miss
- 관절별 command/encoder position과 trajectory/전송/encoder velocity
- position error와 P·D·feedforward·총 제어토크 추정값
- feedback timestamp 기준 age와 validity

`trajectory_velocity_rad_s`와 `sent_velocity_rad_s`에는 실제 MIT 패킷에
사용한 목표 속도가 기록된다. 현재 feedforward torque는 0이다.

현재 기본 `position_request_period_ms: 100`에서는 엔코더가 10Hz로만
갱신된다. 물리적 끊김과 피드백 샘플링을 구분하는 진단 실행에서는 다음
설정을 권장한다.

```yaml
runtime:
  control_period_ms: 20
  feedback_timeout_ms: 250
  position_request_period_ms: 20
```

로그 요약은 Python 표준 라이브러리만으로 실행할 수 있다.

```bash
python3 tools/analyze_motion_log.py logs/my_demo.csv
```

`matplotlib`이 설치되어 있다면 command/encoder와 loop timing 그래프를
PNG로 저장할 수 있다.

```bash
python3 tools/analyze_motion_log.py logs/my_demo.csv --plot
```
