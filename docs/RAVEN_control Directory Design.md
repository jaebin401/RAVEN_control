# RAVEN_control 디렉토리 설계

## 배경 · 왜 새 레포인가

`Robstrid-CAN_study`는 Seeed Studio 오픈소스 기반 액추에이터 프로토콜 스터디 레포(단일/다중 모터 터미널 제어, ImGui 게인 튜닝)로, 목적 자체가 탐색·실험이다. 반면 Phase A~D 로드맵(MATLAB 검증 → C++ 이식)을 따라가는 **실제 팔 구동 코드**는 성격이 다르므로 분리하기로 결정.

- `Robstrid-CAN_study` = 프로토콜/게인 실험실 (계속 유지, 검증된 조각만 가져다 씀)
- `RAVEN_control` = Phase B/C/D 데모가 실제로 도는 프로덕션 제어 스택

## 이름 결정: `RAVEN_control` (Controller 아님)

기존 레포 네이밍(`RAVEN_hardware`, `RAVEN_sim`)이 소문자+언더스코어, 명사형 패턴을 따름. `RAVEN_Controller`는 대문자가 섞여 컨벤션과 어긋나고, "Controller"는 피드백 루프 하나만 가리키는 좁은 의미. 이 레포는 CAN HAL + 기구학/역기구학 + 역동역학 + 궤적/로깅 + 세이프티까지 포함하는 **제어 시스템 전체**이므로 `sim`과 대칭되는 `control`로 확정.

> ⚠️ 미해결 리스크: `RAVEN_sim`의 `raven_description`(URDF)이 링크 길이 등 기구 파라미터의 단일 진실 소스인데, `RAVEN_control`의 닫힌형 IK(Phase D)는 이 값을 별도로 갖게 됨. `config/robot_params.yaml`에 값을 넣을 때 반드시 "출처 URDF 커밋 해시"를 주석으로 남길 것 — 두 레포가 갈라지면서 드리프트할 위험.

## 현재 구현 구조

```
RAVEN_control/
├── CMakeLists.txt
├── .gitignore
├── docs/
├── config/
│   └── joint_limits.yaml           # hard/soft limit 및 실측 확인 상태
├── include/raven_control/
│   ├── hal/
│   │   ├── can_interface.hpp
│   │   └── motor_driver.hpp
│   └── safety/
│       └── joint_limiter.hpp
├── src/
│   ├── hal/
│   │   ├── can_interface.cpp
│   │   └── motor_driver.cpp
│   └── safety/
│       └── joint_limiter.cpp
├── apps/
│   └── debug/
│       └── position_control_multi.cpp
├── tools/
│   ├── motor_id_config.cpp
│   └── show_xbox_control_data.cpp
└── tests/
    ├── joint_limiter_test.cpp
    └── motor_driver_safety_test.cpp
```

지금 필요하지 않은 `watchdog`, `signal_handler`, `startup_sequence` 등의
클래스는 미리 만들지 않는다. 기능이 커져 책임 분리가 실제로 필요해지는 시점에
추가한다. 이후 Phase B/C/D가 시작되면 `kinematics/`, `dynamics/`,
`control/`, `sequencing/`, `logging/`을 단계적으로 확장한다.

## 현재 파일별 역할

| 파일/폴더 | 역할 |
|---|---|
| `config/joint_limits.yaml` | 조인트별 제한값의 단일 설정 지점. `confirmed: false`이면 enable 금지 |
| `safety/joint_limiter` | 하드 리밋 판정, 목표값 clamp, soft-wall 토크 계산만 담당하는 순수 로직 |
| `hal/can_interface` | Linux SocketCAN의 열기·송신·수신만 담당 |
| `hal/motor_driver` | RS02 프레임 변환, 피드백 관리, enable/stop, 송신 직전 `JointLimiter` 적용 |
| `apps/debug/position_control_multi` | 키 입력과 화면 표시를 제공하는 검증용 앱. 실제 최종 main이 아님 |
| `tools/` | 모터 설정·입력 확인용 독립 유지보수 도구 |
| `tests/` | 하드웨어 없이 limiter와 fail-safe 동작을 검증 |

## 핵심 설계 결정

| 결정 | 내용 | 근거 |
|---|---|---|
| 레포 분리 | CAN_study(실험) vs RAVEN_control(프로덕션) | 실험 코드와 실제 구동 코드 혼재 방지 |
| 이름 | `RAVEN_control` (Controller 기각) | 기존 네이밍 컨벤션 일치 + 범위가 제어 시스템 전체 |
| HAL 분리 | `hal/`을 인터페이스로 격리 | 게이밍 랩탑 → 라즈베리파이 이식 시 다른 계층 안 건드림 |
| 제어 모드 | Strategy 패턴 (`control_mode.hpp` 공통 인터페이스) | Phase B/C/D를 갈아끼우는 구조로 apps/는 조립만 담당 |
| 시작 시퀀스 | "엔코더 홈잉"이 아니라 "안전 자세로의 점대점 이동" | RS02는 절대 엔코더 + 영점 이미 저장(Type 6/22) → 재캘리브레이션 불필요. Phase D 궤적 생성의 첫 실사용 사례로 재해석 |
| 조인트 리밋 | `JointLimiter`를 hal 송신 직전 관문으로 배치, 이중 인터페이스(clampTarget / softWallTorque) | Phase B(Kp=Kd=0, backdrivable)는 목표값이 없어 단순 clamp 무력 → 실측값 기준 반발 토크 별도 필요 |
| 리밋 이중화 검토 필요 | YAML 소프트 리밋 + RS02 자체 펌웨어 리밋 레지스터(§4.2.7 add_offset 관련 문구로 존재 시사) 병행 검토 | Phase B처럼 소프트웨어 체크가 무력화되기 쉬운 모드의 마지막 안전망 |
