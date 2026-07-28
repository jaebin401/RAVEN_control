# RAVEN_control 디렉토리 설계

## 배경 · 왜 새 레포인가

`Robstrid-CAN_study`는 Seeed Studio 오픈소스 기반 액추에이터 프로토콜 스터디 레포(단일/다중 모터 터미널 제어, ImGui 게인 튜닝)로, 목적 자체가 탐색·실험이다. 반면 Phase A~D 로드맵(MATLAB 검증 → C++ 이식)을 따라가는 **실제 팔 구동 코드**는 성격이 다르므로 분리하기로 결정.

- `Robstrid-CAN_study` = 프로토콜/게인 실험실 (계속 유지, 검증된 조각만 가져다 씀)
- `RAVEN_control` = Phase B/C/D 데모가 실제로 도는 프로덕션 제어 스택

## 이름 결정: `RAVEN_control` (Controller 아님)

기존 레포 네이밍(`RAVEN_hardware`, `RAVEN_sim`)이 소문자+언더스코어, 명사형 패턴을 따름. `RAVEN_Controller`는 대문자가 섞여 컨벤션과 어긋나고, "Controller"는 피드백 루프 하나만 가리키는 좁은 의미. 이 레포는 CAN HAL + 기구학/역기구학 + 역동역학 + 궤적/로깅 + 세이프티까지 포함하는 **제어 시스템 전체**이므로 `sim`과 대칭되는 `control`로 확정.

> ⚠️ 미해결 리스크: `RAVEN_sim`의 `raven_description`(URDF)이 링크 길이 등 기구 파라미터의 단일 진실 소스인데, `RAVEN_control`의 닫힌형 IK(Phase D)는 이 값을 별도로 갖게 됨. `config/robot_params.yaml`에 값을 넣을 때 반드시 "출처 URDF 커밋 해시"를 주석으로 남길 것 — 두 레포가 갈라지면서 드리프트할 위험.

## 확정 디렉토리 구조

```
RAVEN_control/
├── CMakeLists.txt
├── readme.md
├── .gitignore
├── docs/
│   └── decisions/                  # ADR
├── config/                         # 파라미터 전용 (코드에 하드코딩 금지)
│   ├── robot_params.yaml           # 링크 길이 등 — URDF 출처 커밋 명시
│   ├── motor_map.yaml              # CAN ID ↔ 조인트 매핑, gear ratio, zero_sta 값
│   ├── joint_limits.yaml           # 관절별 min/max 각도, soft margin
│   └── gains.yaml                  # Kp/Kd 프리셋
├── include/raven_control/
│   ├── hal/                        # 하드웨어 추상 계층 — RPi 이식 시 이 폴더만 교체
│   │   ├── can_interface.hpp
│   │   └── motor_driver.hpp        # RS02 프로토콜, 송신 직전 JointLimiter 관문 통과
│   ├── kinematics/                 # 하드웨어 무관 순수 수학
│   │   ├── forward_kinematics.hpp
│   │   └── inverse_kinematics.hpp  # Phase D: MATLAB 닫힌형 해 포팅
│   ├── dynamics/
│   │   └── gravity_compensation.hpp # Phase B: inverseDynamics 포팅
│   ├── control/                    # 전략 패턴 — Phase B/C/D
│   │   ├── control_mode.hpp
│   │   ├── gravity_comp_hold.hpp
│   │   ├── teach_and_replay.hpp
│   │   └── point_to_point.hpp
│   ├── sequencing/
│   │   └── startup_sequence.hpp    # DISABLED → HOMING → READY 상태 머신
│   ├── safety/
│   │   ├── joint_limiter.hpp       # clampTarget() / softWallTorque() / isHardViolation()
│   │   ├── signal_handler.hpp      # SIGINT/SIGTERM → anti-backdrive 정지
│   │   └── watchdog.hpp
│   └── logging/
│       └── trajectory_logger.hpp   # Phase A 로깅 포맷, Teach & Replay 재사용
├── src/                            # 위 헤더 구현
├── apps/                           # 데모 진입점 (조립만, 로직 없음)
│   ├── gravity_comp_hold_demo.cpp
│   ├── teach_replay_demo.cpp
│   └── point_to_point_demo.cpp
├── tools/                          # motor_id_config류 유틸 (CAN_study에서 이식)
└── tests/                          # kinematics/IK, joint_limiter 단위테스트
```

## 폴더별 역할

| 폴더 | 역할 |
|---|---|
| `config/` | 숫자·파라미터 전용. 코드는 읽기만 함 |
| `hal/` | 하드웨어에 닿는 유일한 계층. RPi 이식 시 여기만 교체 |
| `kinematics/` `dynamics/` | 하드웨어 무관 순수 수학, 가장 테스트하기 쉬운 영역 |
| `control/` | Phase B/C/D가 각각 하나의 전략(strategy). 공통 인터페이스로 교체 가능 |
| `sequencing/` | 상태 전이 순서만 담당, 제어 로직은 모름 |
| `safety/` | 어떤 control 모드가 실행되든 무조건 거치는 관문 |
| `apps/` | 위 조각들을 조립하는 얇은 실행 파일 |

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
