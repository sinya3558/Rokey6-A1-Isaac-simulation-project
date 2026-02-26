1. **manipulator 1** : action server (deliver_box_server)
 - action : MoveBoxes
 - role : main controller 가 goal + request 보내면, 컨베이어 벨트에서 박스를 집어서 할당된 위치 포지션으로 올려놓기

2. **manipulator 2** : action server (classifier_server)
   - action : MoveBoxes
   - role :
     -  카메라 /rgb 센서 토픽으로 fragile or normal 분류
     - 박스 구성 : fragile 2 개, normal 2 개 팔레트에 적재 완료 시 success -> True. 그 때 팔레트 상태 'Ready' 를 내부 상태 또는 별도 토픽/서비스로 업데이트
  
3. **ARM** : action server ( deliver_pallet_server)
   - action : MovePallet
   - role : 
     - goal 에 들어온 팔레트 위치로 이동
     - 도착한 후, (option 1) fixed joint 생성 or (option 2) 팔레트 삭제 후 뿅 로직
     - final dest 까지 운반하면서 distance_remaining, ETA_secs 를 feedback 으로 보내기
     - 도착하면 Done 상태로 result 반환