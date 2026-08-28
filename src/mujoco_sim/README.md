# mujoco_sim — SO-101 MuJoCo 시뮬레이션 실습

실기 SO-101과 MuJoCo 시뮬레이션을 잇는 교육 모듈입니다.
SO-101 시뮬레이션 모델이 레포에 동봉돼 있어(`so101/`, 출처·라이선스는
그 안의 README 참조) **git clone과 pip 설치만으로 모든 실습이 됩니다.**

## 준비

```bash
pip install -r requirements.txt   # mujoco 포함
```

실습은 세 방향이고, 순서대로 진행합니다. 아래 명령은 모두 레포
루트에서 실행합니다. `--port`와 `--id`는 실기 캘리브레이션 때 쓴
값입니다 (포트 확인: `lerobot-find-port`).

## 1. 시뮬 → 시뮬 (로봇 불필요)

```bash
python -m mujoco.viewer --mjcf src/mujoco_sim/so101/scene.xml
```

뷰어 오른쪽 Control 슬라이더 6개로 관절을 움직여 봅니다.
순기구학(FK)을 눈으로 체감하는 단계입니다.

## 2. 실기 → 시뮬 (디지털 트윈)

```bash
python -m src.mujoco_sim.digital_twin --port /dev/ttyACM0 --id my_follower
```

실행하면 팔로워암의 토크가 풀리고, 손으로 팔을 움직이면 화면 속
로봇이 30Hz로 따라옵니다.

⚠ 토크가 풀리면 팔이 중력에 처집니다. 실행 전에 팔을 잡거나 눕혀
두세요.

## 3. 시뮬 → 실기 (실기 구동)

```bash
python -m src.mujoco_sim.sim_to_robot --port /dev/ttyACM0 --id my_follower
```

뷰어의 Control 슬라이더를 움직이면 실기 팔이 따라 움직입니다.
시작 시 시뮬 자세를 실기 현재 자세에 맞추므로 팔이 튀지 않습니다.

⚠ 실기가 움직이는 실습입니다.
- 팔 주변 공간을 확보하고 시작하세요.
- 관절 이동량은 전송 1회당 `--max_step_deg`(기본 3도)로 제한됩니다.
  첫 실행은 `--max_step_deg 1`로 낮춰 방향부터 확인하길 권합니다.
- 비상 시 뷰어를 닫거나 Ctrl+C — 전송이 즉시 멈춥니다.

## 파일 구성

| 파일 | 역할 |
|------|------|
| `so101/` | SO-101 MJCF 모델·메시 (SO-ARM100 저장소에서 동봉, Apache-2.0) |
| `sim_bridge.py` | 실기(deg·0~100) ↔ 시뮬(rad) 단위 변환, 관절 매핑 |
| `digital_twin.py` | 실기 → 시뮬 미러링 |
| `sim_to_robot.py` | 시뮬 → 실기 구동 (이동량 제한 포함) |
