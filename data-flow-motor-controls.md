# Motor Control Data Flow

## System Overview

High-level view of data flow through the MEBot/RAMMP motor control system.

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {
  'primaryColor': '#f5f5f5',
  'primaryTextColor': '#333',
  'lineColor': '#555',
  'fontSize': '13px'
}}}%%

flowchart LR
    %% Styles
    classDef input fill:#e3f2fd,stroke:#1565c0,stroke-width:2px,color:#0d47a1
    classDef state fill:#fff8e1,stroke:#f9a825,stroke-width:2px,color:#e65100
    classDef estop fill:#ffebee,stroke:#c62828,stroke-width:2px,color:#b71c1c
    classDef control fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,color:#1b5e20
    classDef output fill:#ede7f6,stroke:#5e35b1,stroke-width:2px,color:#311b92
    classDef hardware fill:#fce4ec,stroke:#ad1457,stroke-width:2px,color:#880e4f

    %% Input Layer
    Serial["Serial\n115200"]:::input
    Encoders["12x Encoders"]:::input

    %% Processing
    Parser["Command\nParser"]:::state
    Watchdog["Watchdog\n60s"]:::state
    
    State{"State\nMachine"}:::state
    ESTOP["ESTOP"]:::estop

    Motors["6x Motor\nInstances"]:::control
    PID["Cascaded\nPID"]:::control

    %% Output
    RoboClaw["3x RoboClaw\nControllers"]:::output
    Actuators["Physical\nMotors"]:::hardware
    Telemetry["Telemetry\n10Hz"]:::output

    %% Flow
    Serial --> Parser
    Parser --> State
    Parser -.->|"feed"| Watchdog
    Watchdog -->|"timeout"| ESTOP
    
    State -->|"TUNER_MODE"| Motors
    State -->|"z cmd"| ESTOP
    ESTOP -->|"c cmd"| State
    ESTOP ==>|"disable()"| Motors

    Encoders --> Motors
    Motors --> PID
    PID -->|"PWM"| RoboClaw
    RoboClaw --> Actuators

    Motors -.->|"pos, vel, pwm"| Telemetry
    State -.->|"state"| Telemetry
    Telemetry --> Serial
```

---

## Cascaded PID Control

Detail view of the control loop inside each Motor instance. Position control cascades into velocity control.

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {
  'primaryColor': '#f5f5f5',
  'primaryTextColor': '#333',
  'lineColor': '#555',
  'fontSize': '13px'
}}}%%

flowchart TB
    %% Styles
    classDef mode fill:#fff8e1,stroke:#f9a825,stroke-width:2px,color:#e65100
    classDef target fill:#e3f2fd,stroke:#1565c0,stroke-width:1px,color:#0d47a1
    classDef sensor fill:#e0f7fa,stroke:#00838f,stroke-width:1px,color:#006064
    classDef pid fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,color:#1b5e20
    classDef output fill:#ede7f6,stroke:#5e35b1,stroke-width:2px,color:#311b92
    classDef disabled fill:#ffebee,stroke:#c62828,stroke-width:2px,color:#b71c1c

    %% Mode Selection
    Mode{"ControlMode"}:::mode
    
    Mode -->|"DISABLED"| Zero["0"]:::disabled
    Mode -->|"OPEN_LOOP"| DirectPath:::output
    Mode -->|"VELOCITY"| VelPath:::pid
    Mode -->|"POSITION"| PosPath:::pid

    %% Targets
    TargetPos["target_pos"]:::target
    TargetVel["target_vel"]:::target
    TargetPWM["target_pwm"]:::target

    %% Sensors
    CurrPos["current_pos"]:::sensor
    CurrVel["current_vel"]:::sensor

    %% Position PID
    subgraph PositionLoop [" Position PID "]
        PosErr["error"]
        PosPID["Kp, Ki, Kd"]:::pid
    end

    %% Velocity PID  
    subgraph VelocityLoop [" Velocity PID "]
        VelErr["error"]
        VelPID["kp, ki, kd"]:::pid
    end

    %% PWM Output
    PWM["PWM Output\n±32767"]:::output

    %% Position Control Path
    PosPath --> TargetPos
    TargetPos --> PosErr
    CurrPos --> PosErr
    PosErr --> PosPID
    PosPID -->|"vel cmd"| VelErr

    %% Velocity Control Path
    VelPath --> TargetVel
    TargetVel --> VelErr
    CurrVel --> VelErr
    VelErr --> VelPID
    VelPID --> PWM

    %% Open Loop Path
    DirectPath --> TargetPWM
    TargetPWM --> PWM

    %% Disabled Path
    Zero --> PWM
```

---

## Hardware Mapping

### Motor → RoboClaw Wiring

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'fontSize': '12px'}}}%%

flowchart LR
    classDef motor fill:#e3f2fd,stroke:#1565c0,stroke-width:1px
    classDef roboclaw fill:#ede7f6,stroke:#5e35b1,stroke-width:2px
    classDef channel fill:#fce4ec,stroke:#ad1457,stroke-width:1px

    subgraph Motors [" Motors "]
        RC["RC\nRear Caster"]:::motor
        FC["FC\nFront Caster"]:::motor
        ML["ML\nMain Left"]:::motor
        MR["MR\nMain Right"]:::motor
        MLC["ML_C\nLeft Carriage"]:::motor
        MRC["MR_C\nRight Carriage"]:::motor
    end

    subgraph Controllers [" RoboClaw Controllers "]
        RC_Casters["roboclaw_casters\nSerial4"]:::roboclaw
        RC_Main["roboclaw_main\nSerial5"]:::roboclaw
        RC_Carriages["roboclaw_carriages\nSerial3"]:::roboclaw
    end

    RC -->|"M1"| RC_Casters
    FC -->|"M2"| RC_Casters
    ML -->|"M1"| RC_Main
    MR -->|"M2"| RC_Main
    MLC -->|"M1"| RC_Carriages
    MRC -->|"M2"| RC_Carriages
```

### Encoder → Motor Mapping

| Motor | ID | Encoder Index | Description |
|-------|:--:|:-------------:|-------------|
| RC | 0 | `encoderf[3]` | Rear Caster |
| FC | 1 | `encoderf[2]` | Front Caster |
| ML | 2 | `encoderf[7]` | Main Left Wheel |
| MR | 3 | `encoderf[5]` | Main Right Wheel |
| ML_C | 4 | `encoderf[11]` | Left Carriage |
| MR_C | 5 | `encoderf[12]` | Right Carriage |

> **Note**: Encoder indices may need verification against physical wiring.
