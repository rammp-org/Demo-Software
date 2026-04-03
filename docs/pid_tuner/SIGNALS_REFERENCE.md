# Qt Signals Reference

The application uses PyQt's Signal/Slot mechanism extensively to pass data across thread boundaries and decouple components.

## `SerialHandler` Signals

| Signal                | Type            | Emitted When                     | Connected To                               |
| --------------------- | --------------- | -------------------------------- | ------------------------------------------ |
| `connection_changed`  | `(bool)`        | Serial port opens/closes         | `MainWindow._on_connection_changed`        |
| `data_received`       | `(EncoderData)` | Complete TELEMETRY packet parsed | `DataStore.process_encoder_data`           |
| `config_received`     | `(ConfigData)`  | Complete CONFIG packet parsed    | `DataStore.set_config`                     |
| `raw_lines_received`  | `(list)`        | Batched lines received           | `SerialConsole.append_lines`               |
| `error_occurred`      | `(str)`         | PySerial raises an exception     | `MainWindow._on_error`                     |
| `seq_ack_received`    | `(int)`         | Robot ACKs a keyframe upload     | `SequenceEditor._on_seq_ack`               |
| `seq_status_received` | `(int,int,int)` | Robot reports sequence progress  | `DataStore.seq_status_updated` (forwarded) |

## `DataStore` Signals

| Signal                 | Type            | Emitted When                    | Connected To                                                            |
| ---------------------- | --------------- | ------------------------------- | ----------------------------------------------------------------------- |
| `data_updated`         | `(int)`         | Telemetry processed (20Hz)      | `PlotWidget._on_data_updated`, `EncoderOverview._update_values`         |
| `config_updated`       | `(int)`         | Config for `joint_id` received  | `ControlPanel._on_config_updated`, `ConfigViewerWidget._on_config_updated` |
| `imu_updated`          | `()`            | IMU part of telemetry processed | `IMU3DWidget._on_imu_updated`, `IMUDisplay._update_display`             |
| `limits_updated`       | `()`            | Limit switch states change      | `EncoderOverview._update_limits`                                        |
| `state_changed`        | `(int)`         | System state changes            | `StateIndicator.set_state`                                              |
| `simulation_changed`   | `(bool)`        | Simulation mode toggled         | `MainWindow` (internal sync)                                            |
| `directions_updated`   | `()`            | Motor/Encoder directions change | `ControlPanel._update_direction_indicator`, `ConfigViewerWidget`        |
| `mode_changed`         | `(int)`         | Control mode confirmed          | `ControlPanel._on_mode_confirmed`                                       |
| `leveling_updated`     | `()`            | Leveling debug data received    | `IMU3DWidget._on_leveling_updated`                                      |
| `strain_gauge_updated` | `()`            | Strain gauge values updated     | `StrainGaugeDisplay._update_display`                                    |
| `seq_status_updated`   | `(int,int,int)` | Sequence status updated         | `SequenceEditor._on_seq_status`                                         |
| `seq_targets_changed`  | `()`            | Sequence targets changed        | `SequencePlotter._update_plot`                                          |

## `UI` Signals

| Signal                                | Type     | Emitted When                   | Connected To                                                 |
| ------------------------------------- | -------- | ------------------------------ | ------------------------------------------------------------ |
| `ControlPanel.mode_changed`           | `(int)`  | User changes control mode      | `EncoderOverview.set_mode_for_all`                           |
| `EncoderOverview.joint_selected`      | `(int)`  | User clicks a bar              | `MainWindow._on_encoder_bar_clicked`                         |
| `SerialConsole.command_sent`          | `(str)`  | User presses Enter in terminal | `SerialHandler.send_raw`                                     |
| `SequenceEditor.sequence_mode_requested` | `(bool)` | User requests sequence mode    | `SerialHandler.enter_sequence_mode`                          |
| `LuciClient.connected_changed`        | `(bool)` | LUCI connection state changes  | `DriveWheelDisplay._on_luci_connection_changed`              |

