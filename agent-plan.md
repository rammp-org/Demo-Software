# Plan: 3D IMU Visualization

## Goal
Add a 3D visualization of the IMU orientation to the control panel to help debug self-leveling kinematics. Show the actual orientation vs the target orientation using 3D coordinate axes.

## 1. Firmware Updates (`Base.ino`)
- Update `sendTelemetry()` to send 4 additional floats: `IMU.current_quat.w()`, `x()`, `y()`, `z()`.
- Increases telemetry fields from 40 to 44.

## 2. Python Protocol Updates (`protocol.py`)
- Update `EncoderData` dataclass with `imu_qw`, `imu_qx`, `imu_qy`, `imu_qz`.
- Update `ProtocolParser` to support the 44-value format (while maintaining backwards compatibility for 40-value).

## 3. Python Data Store Updates (`data_store.py`)
- Store the new quaternion components in `IMUData`.
- Expose properties `imu_qw`, `imu_qx`, `imu_qy`, `imu_qz` on `DataStore`.

## 4. Dependencies
- Add `PyOpenGL>=3.1.0` to `requirements.txt`.

## 5. New 3D Widget (`imu_3d_widget.py`)
- Create `IMU3DWidget` using `pyqtgraph.opengl.GLViewWidget`.
- Add a floor grid (`GLGridItem`).
- Add an Actual Orientation axis (`GLAxisItem`).
- Add a Target Orientation axis (`GLAxisItem`, rendered semi-transparent or slightly thinner).
- On `data_store.imu_updated`, rotate the Actual axis using the telemetry quaternion.
- Rotate the Target axis using the target pitch and target roll inputs.

## 6. UI Integration (`control_panel.py`)
- Instantiate `IMU3DWidget`.
- Add it to the main layout below the existing numerical `IMUDisplay`.
- Set a minimum height of 250px so it's clearly visible in the scrollable pane.