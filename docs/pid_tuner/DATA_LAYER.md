# Data Layer Reference

The Python Application uses a centralized `DataStore` (a Singleton pattern, passed by dependency injection) to manage state.

## `DataStore` (`data/data_store.py`)

**Lines: 1-828**

### Core Responsibilities

- Caching the latest values from the `TELEMETRY` stream for all 8 joints.
- Storing time-series ring buffers (circular arrays) for `pyqtgraph` to plot.
- Caching the `CONFIG` values downloaded from the Teensy EEPROM.
- Emitting signals to alert the UI that it needs to redraw.

### Key Logical Sections

- **Data Classes (Lines 15-302):** Defines the `JointData`, `IMUData` dataclasses.
- **Initialization (Lines 338-416):** Sets up the underlying `numpy` arrays for high-performance circular buffers and initializes 8 `JointData` instances.
- **Signal Definitions (Lines 317-331):** Defines 12 signals: `data_updated`, `simulation_changed`, `state_changed`, `imu_updated`, `limits_updated`, `directions_updated`, `mode_changed`, `config_updated`, `leveling_updated`, `strain_gauge_updated`, `seq_status_updated`, and `seq_targets_changed`.
- **Parsing Ingestion (Lines 675-808):** `process_encoder_data()` takes the telemetry packet from the Serial thread, slices it, and updates the ring buffers for all 8 joints simultaneously.
- **IMU Handling (Lines 695-718):** Slices the IMU and Quaternion data from the telemetry packet.
- **Strain Gauge Handling (Lines 730-734):** Slices the strain gauge load cell data from the telemetry packet.
- **Config Ingestion (Lines 650-653):** `set_config()` stores the `ConfigData` and emits `config_updated`.
- **Sequence Targets (Lines 823-828):** Manages target positions for automated sequences.
