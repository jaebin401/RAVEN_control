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

실행 중 `Space`, `Q`, 또는 `Ctrl+C`를 누르면 긴급 중단한다. 모터 fault,
hard-limit 위반, CAN 송신 오류는 기존 `MotorDriver`의 latched fault를 통해
전체 모터를 즉시 비활성화한다.

## Motion diagnostics

제어 중에는 파일을 쓰지 않고 메모리에만 샘플을 저장한다. 사용자가 모터를
비활성화하거나 fault로 종료된 뒤 CSV를 한 번에 기록한다.

CSV에는 다음 값이 포함된다.

- 제어 주기 `dt`, 예약 시각 대비 lateness, deadline miss
- 관절별 command/encoder position과 trajectory/전송/encoder velocity
- position error와 P·D·feedforward·총 제어토크 추정값
- Type 2 측정 토크, 모터 온도, fault flags와 mode state
- position/Type 2 feedback timestamp 기준 age와 validity

`trajectory_velocity_rad_s`와 `sent_velocity_rad_s`에는 실제 MIT 패킷에
사용한 목표 속도가 기록된다. 현재 feedforward torque는 0이다.

활성화 전에는 Type 17 `mechPos`를 사용해 초기 위치와 hard limit를
검사한다. 활성화 후에는 각 Type 1 명령에 대한 Type 2 응답을 주 피드백으로
사용하며 위치, 속도, 출력토크, 온도와 fault를 제어주기마다 갱신한다.
`position_request_period_ms`는 비활성 상태의 Type 17 요청 주기에만
적용된다. Type 2가 `feedback_timeout_ms` 동안 들어오지 않으면 새로운
궤적 명령을 무시하고, 각 관절의 마지막 안전 목표 위치와 Kp/Kd 및
feedforward torque를 유지하되 목표 속도를 0으로 보내는 Feedback Hold에
진입한다. 피드백이 복구돼도 자동으로 움직임을 재개하지 않으며 사용자가
`Space`를 눌러 Type 4 비활성화 명령을 보낼 때까지 홀드를 유지한다.

기본 실행 설정은 다음과 같다.

```yaml
runtime:
  control_period_ms: 20
  feedback_timeout_ms: 250
  position_request_period_ms: 100
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
