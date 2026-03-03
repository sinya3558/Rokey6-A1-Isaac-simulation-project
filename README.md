# Rokey6-A1-Isaac-simulation-project

---
## 팀원 소개

<table><tr>
   <td align="center"><a href="https://github.com/sinya3558"><img src="imgs/haedal.png" width="100px;" alt=""/>
   <br /><sub><b>Seunga Kim</b><br></sub></a><br /></td>
   <td align="center"><a href="https://github.com/dozi-del"><img src="imgs/kani.png" width="100px;" alt=""/>
   <br /><sub><b>Doyoung Kim</b><br></sub></a><br /></td>
   <td align="center"><a href="https://github.com/G0RaNii"><img src="imgs/manju.png" width="100px;" alt=""/>
   <br /><sub><b>Gyuhyeok Moon</b><br></sub></a><br /></td>
   <td align="center"><a href="https://github.com/donghyun0313"><img src="imgs/momonga.png" width="100px;" alt=""/>
   <br /><sub><b>Donghyun Park</b><br></sub></a><br /></td>
</tr></table>

---

## 프로젝트 소개

Isaac Sim 5.0.0과 ROS2 기반의 **물류 자동화 Digital Twin 프로젝트**입니다.  

두 대의 로봇팔이 컨베이어 벨트 위 상자를 처리하고,  
YOLO 기반 객체 인식을 통해 **fragile(파손주의) 여부를 판별**한 뒤,  
판별 결과에 따라 서로 다른 팔레트로 분류 및 팔레타이징을 수행합니다.

또한 공정 중 발생하는 주요 이벤트 데이터를 ROS2 토픽으로 발행하고,  
DB Control Node가 이를 구독하여 데이터베이스에 저장하는 구조로 설계되었습니다.

본 프로젝트는 시뮬레이션 환경과 실제 공정 간 퍼포먼스 차이를 비교하고,  
공정 최적화를 위한 디지털 트윈 구현을 목표로 합니다.

---

# 시스템 설계

## 전체 시스템 아키텍처

```mermaid
flowchart LR

A[Conveyor Belt] --> B[Robot Arm #1 Pick]
B --> C[YOLO Detection Node]
C -->|Fragile| D[Robot Arm #2 -> Fragile Pallet]
C -->|Normal| E[Robot Arm #2 -> Normal Pallet]

C --> F[DB Publish Topic]
F --> G[DB Control Node]
G --> H[(Database)]
