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

![프레임워크 전체 구조](docs/images/fig4-framework-architecture.png)

*CARLA Simulator ↔ ROS Bridge ↔ Python BMS Control의 실시간 폐루프(Closed-loop) 연동 구조*

---

## Features

- CARLA(Town04_Opt 맵) 기반 3대 전기 트럭 군집 시뮬레이션
- 카메라 BEV 변환 + Sliding Window 다중 차선 인지
- 2D 라이다 Point Cloud 기반 차간 거리 측정 및 PID 종방향 제어
- Python 기반 BMS: SoC 추정, 위치별 공기역학적 보정, 실시간 에너지 모니터링(Live Energy Monitor)

<p float="left">
  <img src="docs/images/fig1-town04-map.png" width="48%" alt="Town04_Opt Map" />
  <img src="docs/images/fig3-driving-point-setup.png" width="48%" alt="주행 지점 설정" />
</p>

*Town04_Opt 맵(좌)과 피치각 시험을 통해 선정한 평탄 구간 기준 주행 지점 설정(우)*

- 군집 순서 재배치 모드
  - Mode 1 `Lane Keeping` — 순서 변경 없이 기본 주행
  - Mode 2 `Auto Reorder` — 선두 차량이 후미로, 후행 차량들이 한 단계씩 전진하는 전체 순서 재배치

    ![Scenario 2: Full Platoon Reorder](docs/images/fig5-scenario2-full-reorder.png)

  - Mode 3 `Auto Promote Tail` — SoC가 가장 높은 후미 차량을 선두로 승격

    ![Scenario 3: Tail Vehicle Promotion](docs/images/fig6-scenario3-tail-promotion.png)

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

## Results

세 시나리오(S1: Lane Keeping, S2: Auto Reorder, S3: Auto Promote Tail)를 군집 내 한 대라도 SoC 50%에 도달하는 시점까지 비교했습니다.

<p float="left">
  <img src="docs/images/fig7-final-soc-by-scenario.png" width="48%" alt="시나리오별 최종 SoC" />
  <img src="docs/images/fig9-soc-uniformity.png" width="48%" alt="시나리오별 SoC 표준편차" />
</p>

- **SoC 균일성**: 고정 군집(S1, σ 6.738%) 대비 S2는 0.183%, S3는 0.261%로 각각 약 **37배, 26배** 개선
- **주행 가능 거리**: S1 88.02 km → S2 91.96 km(+4.5%), S3 92.96 km(**+5.6%**)
- **평균 에너지 효율(η_avg)**: S1 2,095.9 Wh/km, S2 2,438.6 Wh/km, S3 2,405.6 Wh/km — 재배치 기동에 소모되는 추가 에너지가 존재하지만, 배터리 밸런싱을 통해 군집 지속 가능 거리를 늘리는 효과가 더 큼

<p float="left">
  <img src="docs/images/fig8-scenario23-distance-comparison.png" width="48%" alt="S2, S3 세부 거리 비교" />
  <img src="docs/images/fig10-maneuver-duration.png" width="48%" alt="재배치 기동 시간" />
</p>

재배치 기동 시간(T_m)은 S2 17.8초, S3 15.3초로, 후미 차량을 직접 선두로 승격하는 S3가 더 빠르고 효율적인 기동을 보였습니다.

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
