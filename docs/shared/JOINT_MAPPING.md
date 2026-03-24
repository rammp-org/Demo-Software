# Joint Mapping

The system relies on a unified 1-indexed Joint ID system to map physical components to GUI controls.

| Joint ID | Short Name | Full Name           | Firmware Motor Instance | Encoder Array Index (`encoderf`) | Notes                                                 |
| -------- | ---------- | ------------------- | ----------------------- | -------------------------------- | ----------------------------------------------------- |
| 1        | RC         | Rear Caster         | `rc`                    | 3                                | Legacy reference: "RC Bottom (0-850)"                 |
| 2        | FC         | Front Caster        | `fc`                    | 2                                |                                                       |
| 3        | ML         | Main Left           | `ml`                    | 7                                | Left Drive Wheel                                      |
| 4        | MR         | Main Right          | `mr`                    | 5                                | Right Drive Wheel                                     |
| 5        | ML_C       | Main Left Carriage  | `ml_carriage`           | 11                               | Has associated Limit Switches (`CARRIAGE_SW1`, `SW2`) |
| 6        | MR_C       | Main Right Carriage | `mr_carriage`           | 12                               | Has associated Limit Switches (`CARRIAGE_SW3`, `SW4`) |

## Usage

- **Firmware:** The `switch (cmd.actuator_id - 1)` mapping in `Base.ino:482` maps the incoming 1-indexed ID to the physical Motor instance.
- **Python App:** The `pid_tuner/data/joint_config.py` uses this exact mapping to render the UI components.
