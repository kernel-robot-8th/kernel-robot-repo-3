# 목표 데이터 흐름

이 문서는 [시스템 명세](system-spec.md)에 정의한 목표의 흐름만 설명합니다. 현재 코드에서 가능한 요청과 응답은 [구현 현황](../implementation/current-state.md)을 확인하세요.

## 시스템 구성요소

아래 그림은 시스템의 책임 경계와 주요 연결을 보여줍니다. **명령은 오케스트레이터에서 어댑터를 거쳐 장치로 내려가고**, **상태·결과 이벤트는 어댑터에서 관제 서버로 올라옵니다**. 두 카메라는 로봇팔에 장착된 별도 입력이며, 영상 경로는 작업 명령·상태 경로와 분리됩니다.

```mermaid
flowchart LR
    OP[운영자]

    subgraph CONTROL[관제 시스템]
        subgraph CLIENT[사용자 화면]
            UI[관제 UI]
        end
        subgraph SERVER[관제 서버]
            API[HTTP API]
            ORCH[작업 오케스트레이터]
            EVENT[이벤트 처리기]
            STATE[(로봇 상태 저장소)]
            HISTORY[(작업·단계 이력)]
            CAM[카메라 관리자]
        end
    end

    subgraph ADAPTERS[장치 연결 계층]
        MAD[이동 로봇 어댑터]
        AAD[로봇팔 어댑터]
    end

    subgraph DEVICES[현장 장치]
        MOBILE[이동 로봇]
        subgraph ARM[로봇팔]
            ARMCTRL[로봇팔 제어기]
            HEAD[Head Camera]
            JOINT[Joint Camera]
        end
    end

    OP <--> UI
    UI <-->|작업·상태 API| API
    API -->|작업 실행 요청| ORCH
    ORCH -->|이동 명령| MAD
    ORCH -->|팔 작업 명령| AAD
    MAD -->|장치 명령| MOBILE
    AAD -->|장치 명령| ARMCTRL
    MOBILE -->|상태·결과| MAD
    ARMCTRL -->|상태·결과| AAD
    MAD -->|공통 이벤트| EVENT
    AAD -->|공통 이벤트| EVENT
    EVENT -->|현재 작업 결과| ORCH
    EVENT -->|로봇 snapshot| STATE
    EVENT -->|작업 이벤트| HISTORY
    ORCH -->|단계·최종 결과| HISTORY
    API -->|상태 조회| STATE
    API -->|작업 조회| HISTORY
    HEAD -->|프레임·연결 상태| CAM
    JOINT -->|프레임·연결 상태| CAM
    CAM -->|영상 스트림| UI
    CAM -->|카메라 상태 조회| API
```

화살표는 데이터가 전달되는 방향입니다. 이동 로봇과 로봇팔의 장치별 프로토콜은 각 어댑터 안에서 공통 이벤트로 변환합니다. 카메라 영상 자체는 상태 API로 보내지 않고 스트림 경로로 전달하며, API에는 카메라별 연결·프레임 상태만 제공합니다.

## 작업 실행 흐름

```mermaid
sequenceDiagram
    actor User as 운영자
    participant UI as 관제 UI
    participant API as 관제 API
    participant O as 오케스트레이터
    participant M as 이동 로봇
    participant A as 로봇팔
    participant H as 작업 이력

    User->>UI: 작업 실행
    UI->>API: 작업 요청
    API->>O: 검증·실행 요청
    API-->>UI: 접수 + 작업 ID
    O->>H: VALIDATING 및 단계 이력 기록
    O->>M: 목적지 이동 명령
    M-->>O: 수락 / 거절
    O->>H: 단계 상태 갱신
    M-->>O: 진행 상태 및 최종 이동 결과

    alt 이동 완료가 확인됨
        O->>A: 팔 작업 명령
        A-->>O: 수락 및 진행 상태
        A-->>O: 최종 팔 작업 결과
        O->>H: 작업·단계 최종 결과 기록
    else 실패·시간 초과·결과 불명
        O->>H: 실패 또는 NEEDS_INTERVENTION 기록
        Note over O,A: 팔 작업 명령을 보내지 않음
    end
    O-->>UI: 작업 상태 갱신
```

## 카메라 영상 흐름

카메라 상태와 로봇 작업 상태는 별도 흐름으로 수집합니다. 프레임은 카메라별 영상 경로로 UI에 전달하고 작업 상태 응답에는 포함하지 않습니다.

```mermaid
flowchart LR
    HEAD[Head Camera] --> HCAP[Head 수집기]
    JOINT[Joint Camera] --> JCAP[Joint 수집기]
    HCAP --> STREAM[영상 스트림 경로]
    JCAP --> STREAM
    HCAP --> STATUS[카메라별 상태·프레임 시각·FPS]
    JCAP --> STATUS
    STREAM --> UI[관제 UI]
    STATUS --> API[관제 API]
    API --> UI
```

## 로봇 이벤트 의미

로봇 어댑터는 작업 식별자를 포함해 명령 수락, 실행 시작, 진행, 성공, 실패, 시간 초과 또는 결과 불명을 보고해야 합니다. 오케스트레이터는 현재 작업과 일치하는 결과만 반영합니다. 프로세스가 살아 있다는 사실만으로 로봇 준비나 작업 완료를 추론하지 않습니다.
