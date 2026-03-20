# Qt Signals Reference

The application uses PyQt's Signal/Slot mechanism extensively to pass data across thread boundaries and decouple components.

## `SerialHandler` Signals
| Signal | Type | Emitted When | Connected To |
|---|---|---|---|
| `connection_changed` | `(bool)` | Serial port opens/closes | `MainWindow._on_connection_changed` |
| `data_received` | `(list)` | Complete TELEMETRY packet parsed | `DataStore.process_telemetry_data` |
| `config_received` | `(list)` | Complete CONFIG packet parsed | `DataStore.process_config_data` |
| `raw_line_received` | `(str)` | Any line received (for terminal) | `SerialConsole.append_line` |
| `error_occurred` | `(str)` | PySerial raises an exception | `MainWindow._show_error` |

## `DataStore` Signals
| Signal | Type | Emitted When | Connected To |
|---|---|---|---|
| `data_updated` | `()` | Telemetry processed (10Hz) | `PlotWidget._update_plot`, `EncoderOverview._update_values` |
| `config_updated` | `(int)` | Config for `joint_id` received | `ControlPanel._on_config_updated`, `EncoderOverview._on_config_updated` |
| `imu_updated` | `()` | IMU part of telemetry processed | `IMU3DWidget._on_imu_updated`, `IMUDisplay._update_display` |
| `limits_updated`| `()` | Limit switch states change | `ControlPanel._update_limit_switches`, `EncoderOverview._update_limits` |
| `state_updated` | `(int)` | System state changes | `StateIndicator.set_state` |

## `UI` Signals
| Signal | Type | Emitted When | Connected To |
|---|---|---|---|
| `ControlPanel.joint_changed` | `(int)` | User changes dropdown | `EncoderOverview.set_selected_joint`, `PlotWidget.set_joint` |
| `EncoderOverview.joint_selected`| `(int)` | User clicks a bar | `ControlPanel.set_joint`, `PlotWidget.set_joint` |
| `SerialConsole.command_sent` | `(str)` | User presses Enter in terminal| `SerialHandler.send_raw_command` |