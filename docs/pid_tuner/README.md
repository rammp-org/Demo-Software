# PID Tuner Application

The PID Tuner is a PyQt6 application used to interface with the RAMMP / MEBot Teensy firmware. It provides a real-time dashboard for adjusting PID control loops, visualizing live data through high-speed plots, and testing various maneuvers (like sine sweeps and steps).

## Quick Start

Make sure you have python 3 installed, then run:

```bash
cd pid_tuner
pip install -r requirements.txt
python run.py
```

## Documentation Modules

- [**App Architecture**](ARCHITECTURE.md): Discover how the data flows from the Serial port, through the DataStore, and into the UI elements.
- [**UI Layer**](UI_LAYER.md): Detailed breakdowns of each UI Widget (`ControlPanel`, `PlotWidget`, `EncoderOverview`, etc.).
- [**Data Layer**](DATA_LAYER.md): Details on the `DataStore` singleton and the schemas of data moving through the app.
- [**Serial Layer**](SERIAL_LAYER.md): Overview of the multithreaded serial reader and `ProtocolParser`.
- [**Signals Reference**](SIGNALS_REFERENCE.md): A comprehensive reference of every PyQt signal and slot connecting these components.
