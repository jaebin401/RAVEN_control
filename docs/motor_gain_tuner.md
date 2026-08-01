# RS02 Motor Gain Tuner

`motor_gain_tuner`는 별도 그래픽 환경 없이 터미널 전체 화면에서 조인트별
Kp/Kd를 조정하는 디버그 도구다. 모든 MIT command는 기존
`MotorDriver`와 `JointLimiter`를 통과한다. 목표각을 slew-rate로 이동하는
동안에는 setpoint 변화량으로 계산한 목표 속도도 함께 전송한다.

## Build and run

Linux와 SocketCAN 환경에서 저장소 루트를 기준으로 실행한다.

```bash
cmake -S . -B build
cmake --build build --target motor_gain_tuner -j"$(nproc)"

./build/motor_gain_tuner \
  can0 \
  config/joint_limits.yaml \
  config/motor_config.yaml
```

## Keys

| 키 | 동작 |
|---|---|
| `←` / `→` | 조정할 조인트 선택 |
| `Tab` | Kp와 Kd 선택 전환 |
| `↑` / `↓` | 선택한 gain 증감 (`Kp 1.0`, `Kd 0.01`) |
| `W` / `S` | 선택한 조인트 목표각을 ±2° 변경 |
| `Space` | 전체 모터 enable/stop |
| `V` | 현재 gain을 `motor_config.yaml`에 저장 |
| `Q` | 전체 모터 stop 후 종료 |
| `H/J/K/L` | 방향키를 사용할 수 없는 터미널용 대체 키 |

`V` 저장은 모든 모터가 stop 상태일 때만 허용한다. 저장하면 YAML의 기존
주석과 수동 정렬은 유지되지 않지만 설정값은 모두 보존된다.

## Safety behavior

- enable 전 fresh feedback과 모든 joint limit의 `confirmed` 상태를 검사한다.
- Type 2 피드백 timeout은 마지막 안전 위치를 유지하는 Feedback Hold로
  이어지며, `Space`를 누르면 전체 모터를 비활성화한다.
- hard-limit 위반, 모터 fault, CAN 오류, 범위를 벗어난 gain은 전체 모터
  stop과 latched fault를 발생시킨다.
- 목표각은 joint soft limit에서 clamp된다.
- soft limit에서 목표각이 clamp되면 바깥쪽 목표 속도는 0으로 전송된다.
- gain 조정은 실행 중 즉시 적용되지만 `V`를 누르기 전에는 파일에 저장되지
  않는다.
- 저 gain에서도 중력으로 링크가 떨어질 수 있으므로 튜닝 중에는 기구를
  지지하고 작은 목표각부터 시험한다.
