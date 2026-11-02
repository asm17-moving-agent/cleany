# cleany_control_bridge 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## control_bridge

실행 파일: `control_bridge`. 원본: [node.py](../cleany_control_bridge/node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 서비스 호출 | `mission/offer` | `cleany_interfaces/srv/OfferMission` | Service 기본값 |
| 서비스 호출 | `mission/cancel` | `cleany_interfaces/srv/CancelMission` | Service 기본값 |
| 서비스 호출 | `mission/snapshot` | `cleany_interfaces/srv/GetRuntimeSnapshot` | Service 기본값 |

ROS 토픽이나 action을 직접 제공하지 않는다. Backend WebSocket은 `url`로 설정하고 상태 동기화 간격은 `heartbeat_interval` 기본 1초다. [Gateway 계약](../../cleany_interfaces/docs/mission.md)을 따른다.
