# PID Tuner Architecture

The application is structured to strictly decouple the Serial I/O thread, the Data caching layer, and the User Interface thread. Communication across these boundaries is achieved safely using PyQt Signals and Slots.

## Component Flowchart

```mermaid
flowchart TD
    subgraph Serial Thread [Background Thread]
        SerialHandler(SerialHandler)
        ProtocolParser(ProtocolParser)
        ProtocolEncoder(ProtocolEncoder)
        
        SerialHandler -- read_line --> ProtocolParser
        ProtocolEncoder -- write_string --> SerialHandler
    end
    
    subgraph Data Layer [Main Thread]
        DataStore[(DataStore)]
    end
    
    subgraph UI Layer [Main Thread]
        MainWindow(MainWindow)
        ControlPanel(ControlPanel)
        PlotWidget(PlotWidget)
        EncoderOverview(EncoderOverview)
        SerialConsole(SerialConsole)
        IMU3DWidget(IMU3DWidget)
        StrainGaugeDisplay(StrainGaugeDisplay)
        ConfigViewer(ConfigViewer)
    end

    %% Data flowing IN (Telemetry)
    ProtocolParser -- "QtSignal(data_received)" --> DataStore
    DataStore -- "QtSignal(data_updated)" --> PlotWidget
    DataStore -- "QtSignal(data_updated)" --> EncoderOverview
    DataStore -- "QtSignal(imu_updated)" --> IMU3DWidget
    DataStore -- "QtSignal(data_updated)" --> StrainGaugeDisplay
    DataStore -- "QtSignal(config_updated)" --> ConfigViewer
    
    %% Raw Console feed
    ProtocolParser -- "QtSignal(raw_line_received)" --> SerialConsole
    
    %% Commands flowing OUT
    ControlPanel -- "method call" --> SerialHandler
    SerialConsole -- "QtSignal(command_sent)" --> SerialHandler
```

## Design Principles

1. **Thread Safety via Signals:** The `SerialHandler` lives in a `QThread`. It never touches UI elements directly. It emits `data_received(list)` which is queued to the Main Thread where `DataStore` ingests it.
1. **Centralized State:** `DataStore` acts as the single source of truth. UI components (`PlotWidget`, `ControlPanel`) read from `DataStore` rather than caching their own copies of the robot state.
1. **Decoupled UI:** The `MainWindow` instantiates the UI components, passing them references to `DataStore` and `SerialHandler`. Components hook up to the signals they care about.
