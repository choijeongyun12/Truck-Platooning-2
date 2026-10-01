# Truck-Platooning-2

CARLA + ROS2 기반 전기 트럭(e-Truck) 군집주행(Platooning) 연구 코드입니다.
차선 인지 기반 자율 추종, 군집 내 동적 순서 재배치(Intra-Platooning Position Change), 그리고 Python 기반 BMS를 통한 에너지 효율 평가를 다룹니다.

---

## Overview

군집주행(Platooning)은 여러 대의 차량이 일정한 간격을 유지하며 주행해 연비와 교통 효율을 높이는 기술입니다. 하지만 선두 차량 교체, 에너지 관리, 고장 대응 등으로 군집 내 차량 위치를 바꿔야 하는 상황이 발생하며, 이를 안전하고 효율적으로 수행하는 전략이 필요합니다.

또한 전기 트럭(e-Truck) 군집에서는 선두 차량이 공기역학적 항력을 가장 크게 받아 배터리(SoC) 소모 속도가 후행 차량보다 훨씬 빠릅니다. 이 불균형을 해소하지 못하면 후행 차량의 배터리가 남아 있어도 선두 차량의 방전으로 군집 운행이 중단됩니다.

이 저장소는 위 두 문제를 다룬 연구의 구현 코드를 담고 있습니다.

1. **동적 순서 재배치(Dynamic Sequence Reordering)** — CARLA 시뮬레이터와 카메라 기반 차선 인지(BEV + Sliding Window), 2D 라이다 기반 차간 거리 측정을 이용해 군집 내 차량 순서를 안전하게 교체하는 두 가지 시나리오(순환 재배치 / 특정 차량 선두 승격)를 구현
2. **주행 효율성 평가(Range-Efficiency Assessment)** — ROS Bridge와 Python 기반 BMS 모듈을 연동해 순서 재배치 전략이 군집의 평균 에너지 효율, SoC 균일성, 주행 가능 거리에 미치는 영향을 정량적으로 평가하는 프레임워크

---

## Features

- CARLA(Town04_Opt 맵) 기반 3대 전기 트럭 군집 시뮬레이션
- 카메라 BEV 변환 + Sliding Window 다중 차선 인지
- 2D 라이다 Point Cloud 기반 차간 거리 측정 및 PID 종방향 제어
- Python 기반 BMS: SoC 추정, 위치별 공기역학적 보정, 실시간 에너지 모니터링(Live Energy Monitor)
- 군집 순서 재배치 모드
  - Mode 1 `Lane Keeping` — 순서 변경 없이 기본 주행
  - Mode 2 `Auto Reorder` — 선두 차량이 후미로, 후행 차량들이 한 단계씩 전진하는 전체 순서 재배치
  - Mode 3 `Auto Promote Tail` — SoC가 가장 높은 후미 차량을 선두로 승격

### 조작 키 (`platooning_manager`)

| 키 | 동작 |
| --- | --- |
| `t` | 전체 재배치 (좌측 차선 경유) |
| `y` | 전체 재배치 (우측 차선 경유) |
| `j` | 2번 위치 차량을 선두로 승격 (좌측 차선) |
| `k` | 3번 위치 차량을 선두로 승격 (좌측 차선) |
| `q` / `e` | Truck 0 좌/우 차선 변경 |
| `a` / `d` | Truck 1 좌/우 차선 변경 |
| `z` / `c` | Truck 2 좌/우 차선 변경 |
| `ESC` | 종료 |

---

## Repository Structure

```
.
├── truck_control/          # ROS2(Python) 패키지 — 차선 인지, PID 제어, 에너지/SoC 모니터링, 군집 관리
│   ├── truck_control/      # 노드 구현체
│   └── matlab/             # SoC/에너지 후처리 및 시각화 스크립트
├── tm_experiment_control/  # ROS2(Python) 패키지 — 결정론적 시나리오 러너 및 실험 자동화
├── truck_platooning/       # ROS2(C++) 패키지 — Python 노드의 C++ 포팅 (진행 중, 일부 미완성)
└── trailer_watcher.py      # CARLA 상에서 트레일러-트럭 간 오프셋을 보정하는 유틸리티 스크립트
```

**Tech Stack:** CARLA Simulator · ROS2 · ROS Bridge · Python · C++ · MATLAB

---

## Publications

이 저장소의 코드는 아래 두 논문의 실험 구현을 기반으로 합니다.

- Yoonjin Cho, Daeho Won, Jeongyun Choi, Jong-Chan Kim, **"Dynamic Sequence Reordering for Truck Platooning"** (트럭 군집주행을 위한 동적 순서 재배치), *The Korean Society of Automotive Engineers (KSAE) 추계학술대회*, 2025.
- Yoonjin Cho, Daeho Won, Jeongyun Choi, Jong-Chan Kim, **"Range-Efficiency Assessment Method for Dynamic e-Truck Platoon"** (전기 트럭 군집주행 대열 최적화를 위한 주행 효율성 평가 방법), *Korean Society of Automotive Engineers (KSAE)*, 2026.

---

## Contributors

- [Yoonjin Cho (조윤진)](https://github.com/gerrard088)
- [Daeho Won (원대호)](https://github.com/1daymore)
- [Jeongyun Choi (최정윤)](https://github.com/choijeongyun12)
