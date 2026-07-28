# Commit Convention

**Project:** RAVEN — Robotic Arm for Venturing into Engineering by uNdergraduate student
**Version:** 1.0
**Date:** 2026-07-28

---

## 1. 기본 구조

```
<type>(<scope>): <subject>
```

- `type` — 필수. 아래 목록에서 선택
- `scope` — 선택. 변경 대상 모듈 (`include/raven_control/` 하위 폴더명 기준)
- `subject` — 필수. 영문, 명령형 현재시제, 마침표 없음

**예시**

```
feat(safety): add JointLimiter clampTarget and softWallTorque
fix(hal): correct CAN extended frame ID byte order
refactor(control): extract common update loop into ControlMode base
docs: add commit convention
chore: initial directory structure
test(kinematics): add closed-form IK unit tests
```

---

## 2. Type 목록

| Type | 용도 |
|---|---|
| `feat` | 새 기능 추가 |
| `fix` | 버그 수정 |
| `refactor` | 동작 변화 없는 구조 개선 |
| `docs` | README, ADR, 주석 등 문서 |
| `chore` | 빌드 설정, 디렉토리 구조, `.gitignore` 등 |
| `test` | 테스트 추가·수정 |

> 필요해지는 시점에만 type을 추가한다. 쓰지 않을 type을 미리 정의하지 않는다.

---

## 3. Scope 목록

`RAVEN_control`의 `include/raven_control/` 하위 폴더명을 그대로 사용한다.

| Scope | 대상 |
|---|---|
| `hal` | CAN 인터페이스, RS02 모터 드라이버 |
| `kinematics` | FK / IK |
| `dynamics` | 중력보상, 역동역학 |
| `control` | 제어 모드 (Phase B/C/D) |
| `sequencing` | 기동 시퀀스, 상태 머신 |
| `safety` | JointLimiter, 시그널 핸들러, 워치독 |
| `logging` | 궤적 로깅 |
| `config` | YAML 파라미터 파일 |

- 변경이 여러 모듈에 걸치거나 특정 모듈에 속하지 않으면 scope를 생략한다
- `docs`, `chore`는 보통 scope 없이 사용한다

---

## 4. 표기 규칙

- 언어는 **영문** (포트폴리오 공개 목적, `RAVEN_hardware` README 영문화 기조와 일치)
- `type`, `scope`는 소문자
- `subject`는 명령형 현재시제 — `add`, `fix`, `remove` (`added`, `adds` 아님)
- `subject` 끝에 마침표를 찍지 않는다
- 제목 줄은 50자 내외로 유지

---

## 5. 본문 (선택)

제목만으로 부족할 때 빈 줄 하나를 두고 본문을 작성한다. **무엇을 바꿨는지보다 왜 바꿨는지**를 적는다 — 무엇은 diff에 이미 드러난다.

```
fix(hal): guard motor enable against stale CAN feedback

Enable was issued before the first feedback frame arrived, so the
initial target position was set from an uninitialized value. Wait for
one valid feedback frame before entering the control loop.
```

설계 근거가 길어지면 커밋 본문 대신 `docs/decisions/`에 ADR로 남기고, 커밋에서는 해당 ADR을 참조한다.

---

## 6. 커밋 단위

- 한 커밋은 하나의 논리적 변경만 담는다
- 기능 구현과 포맷팅 정리는 분리한다
- 동작하지 않는 중간 상태를 `main`에 커밋하지 않는다 — 실험 중이라면 브랜치를 사용한다

---

## 7. 브랜치

1인 개발이므로 기본은 `main`에 직접 커밋한다. 브랜치는 **되돌릴 가능성이 있는 실험**에만 사용한다.

```
feat/<short-name>     예: feat/gravity-comp
```

성공하면 `main`에 merge하고, 실패하면 브랜치째 삭제한다.

> PR 템플릿은 두지 않는다. 리뷰어가 없는 상태에서는 형식적 절차가 되므로, QUB 통합 등으로 공동 작업이 시작되는 시점에 실제 필요에 맞춰 도입한다.

---

## 8. 태그

마일스톤 완료 시점에 태그를 남긴다. `RAVEN_hardware`의 GitHub Releases 스냅샷 방식과 동일한 목적이다.

```
v<major>.<minor>.<patch>     예: v0.2.0 (Phase B 중력보상 홀드 모드 완성)
```
