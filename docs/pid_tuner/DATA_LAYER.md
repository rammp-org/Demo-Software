# Data Layer Reference

The Python Application uses a centralized `DataStore` (a Singleton pattern, passed by dependency injection) to manage state.

## `DataStore` (`data/data_store.py`)
**Lines: 1-659**

### Core Responsibilities
- Caching the latest values from the `TELEMETRY` stream.
- Storing time-series ring buffers (circular arrays) for `pyqtgraph` to plot.
- Caching the `CONFIG` values downloaded from the Teensy EEPROM.
- Emitting signals to alert the UI that it needs to redraw.

### Key Logical Sections
- **Data Classes (Lines 18-93):** Defines the `JointData`, `JointConfig`, and `IMUData` dataclasses.
- **Initialization (Lines 95-171):** Sets up the underlying `numpy` arrays for high-performance circular buffers.
- **Signal Definitions (Lines 98-103):** Defines `data_updated`, `config_updated`, `imu_updated`, `limits_updated`.
- **Parsing Ingestion (Lines 296-418):** `process_telemetry_data()` takes the 44-value array from the Serial thread, slices it, and updates the ring buffers for all 6 joints simultaneously.
- **IMU Handling (Lines 420-461):** Slices the IMU and Quaternion data from the telemetry packet.
- **Config Ingestion (Lines 463-492):** `process_config_data()` parses the 13-value EEPROM array and stores it.