# PID Controller

The `PIDController` class (`src/PIDController/PIDController.cpp`) is a generic, self-contained closed-loop controller used as both the **Position** and **Velocity** loop inside each `Motor` instance. Two `PIDController` objects exist per joint — `pos_pid` and `vel_pid` — making 12 independent controllers total across the 6 joints.

## Class Overview

```
src/PIDController/
├── PIDController.h   (32 lines)  — Public interface and state members
└── PIDController.cpp (77 lines)  — Compute logic, anti-windup, LPF
```

### Key Members

| Member               | Type              | Description                                                         |
| -------------------- | ----------------- | ------------------------------------------------------------------- |
| `kp`, `ki`, `kd`     | `float`           | Proportional, Integral, Derivative gains                            |
| `kff`                | `float`           | Feed-Forward gain                                                   |
| `min_out`, `max_out` | `float`           | Output clamp limits                                                 |
| `max_ramp_rate`      | `float`           | Maximum rate of change of output per second (0.0 = unlimited)       |
| `scaling`            | `float`           | Gain divisor (see below)                                            |
| `integral`           | `float`           | Running integral accumulator                                        |
| `prev_error`         | `float`           | Error from the previous `compute()` call (for derivative)           |
| `lpf_alpha`          | `float`           | Output Low-Pass Filter coefficient (0–1, default `1.0` = no filter) |
| `_filtered_output`   | `float` (private) | LPF state variable                                                  |

______________________________________________________________________

## The `scaling` Divisor

The constructor signature is:

```cpp
PIDController(float kp, float ki, float kd, float kff,
              float min_out, float max_out, float scaling);
```

The `scaling` parameter is **divided into all gain terms** before they are applied:

```cpp
float p_out = kp / scaling * error;
float i_out = ki / scaling * integral;
float d_out = kd / scaling * derivative;
float ff_out = kff / scaling * setpoint;
```

**Why?** The two loops operate on very different numerical magnitudes. The Position PID works in encoder **ticks** (values in the thousands) and outputs a `target_vel` also in the tick-per-second range. The Velocity PID works on tick-per-second values and must output a normalized `[-1, 1]` PWM fraction. Rather than requiring the operator to mentally track these scale factors when entering gains in the UI, `scaling` absorbs the domain difference:

| Loop      | `scaling` | Effect                                                                                                      |
| --------- | --------- | ----------------------------------------------------------------------------------------------------------- |
| `pos_pid` | `1`       | Gains entered directly in ticks/ticks units                                                                 |
| `vel_pid` | `10000`   | Gains entered as if working with 10k-scale velocity values; output is automatically normalized to `[-1, 1]` |

This means `vel_pid` gains entered in the tuner GUI (`p1:0.5`, `v1:0.5`) do not need to be tiny fractions to produce valid PWM outputs — the `10000` divisor handles the normalization internally.

______________________________________________________________________

## Compute Algorithm (Lines 9–48)

The full compute sequence on each `loop()` call:

```
PIDController::compute(setpoint, measured, dt)
```

### Step 1 — Error

```cpp
float error = setpoint - measured;
```

### Step 2 — Feed-Forward

```cpp
float ff_out = kff / scaling * setpoint;
```

Feed-forward is applied directly to the **setpoint**, not the error. This allows a stationary target (`error = 0`) to still produce a holding output proportional to position/velocity — useful for gravity compensation or velocity bias.

### Step 3 — Proportional

```cpp
float p_out = kp / scaling * error;
```

### Step 4 — Integral with Conditional Anti-Windup

```cpp
integral += error * dt;
float i_out = ki / scaling * integral;
```

The integrator accumulates **before** the output is clamped. Anti-windup is applied **after** summing all terms:

```cpp
float output = ff_out + p_out + i_out + d_out;

if (output > max_out) {
    output = max_out;
    integral -= error * dt; // Undo the integration step
} else if (output < min_out) {
    output = min_out;
    integral -= error * dt; // Undo the integration step
}
```

This is a **back-calculation / conditional integration** strategy: if the output would saturate, the integration step is reversed, preventing the integral from accumulating during saturation. This avoids the classic "windup" problem where the integrator builds up a large internal value that then causes overshoot when the system comes back into range.

### Step 5 — Derivative

```cpp
float derivative = (error - prev_error) / dt;
float d_out = kd / scaling * derivative;
```

The derivative is computed on the **error** (not on `measured`). This means setpoint changes will cause a derivative kick. In practice, setpoints are changed relatively infrequently from the GUI, so this is acceptable.

### Step 6 — Trapezoidal Ramp Rate Limit

```cpp
if (max_ramp_rate > 0.0f) {
    float max_change = max_ramp_rate * dt;
    if (output - _filtered_output > max_change) output = _filtered_output + max_change;
    else if (_filtered_output - output > max_change) output = _filtered_output - max_change;
}
```

If configured via `setRampRate()`, the output is constrained to change by no more than `max_ramp_rate * dt` per cycle. This effectively creates a trapezoidal motion profile, softening aggressive commands and preventing hard jerks to the motors.

### Step 7 — Output Low-Pass Filter

```cpp
_filtered_output += lpf_alpha * (output - _filtered_output);
return _filtered_output;
```

The LPF is applied to the **final output**, not the raw computed value. This smooths the PWM/velocity signal sent downstream. With `lpf_alpha = 1.0` (the default), the filter is a pass-through. Smaller values (e.g. `0.3`) increasingly smooth the output at the cost of additional phase lag.

______________________________________________________________________

## Full Signal Flow Diagram

```mermaid
flowchart TD
    SP[Setpoint] --> FF["FF = kff/scaling × setpoint"]
    SP --> ErrCalc
    Meas[Measured] --> ErrCalc["error = setpoint − measured"]

    ErrCalc --> P["P = kp/scaling × error"]
    ErrCalc --> I["integral += error × dt\nI = ki/scaling × integral"]
    ErrCalc --> D["derivative = Δerror / dt\nD = kd/scaling × derivative"]

    FF --> Sum((Σ))
    P --> Sum
    I --> Sum
    D --> Sum

    Sum --> Clamp{"output > max_out\nor < min_out?"}
    Clamp -- Yes --> Undo["Undo integral step\noutput = clamp(output)"]
    Clamp -- No --> Ramp Limit
    Undo --> Ramp Limit["Ramp Rate Limit:\nClamp Δoutput ≤ max_change"]

    Ramp Limit --> LPF["LPF:\nfiltered += α × (output − filtered)"]

    LPF --> Out[Return filtered output]
```

______________________________________________________________________

## `reset()` (Lines 63–67)

Called when a mode transition occurs or when the user sends an `R<id>` command.

```cpp
void PIDController::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
    _filtered_output = 0.0f;
}
```

Resetting the LPF state (`_filtered_output`) as well as the integrator ensures there is no "memory" of the previous operating point when the controller is re-armed. Without this, the LPF state would bleed a stale PWM value into the first output after a mode switch.

______________________________________________________________________

## Instantiation in `Motor`

Both PID controllers are constructed in `Motor.cpp:6-7`:

```cpp
pos_pid(0.0f, 0.0f, 0.0f, 0.0f, -10000.0f, 10000.0f, 1),
vel_pid(0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 10000)
```

- `pos_pid` output range `[-10000, 10000]` maps to a target velocity in ticks/sec. `scaling = 1`.
- `vel_pid` output range `[-1, 1]` maps directly to a normalized PWM fraction fed to `Motor::update()`. `scaling = 10000`.

These defaults are overwritten on boot by the values loaded from EEPROM via `ConfigStorage` (see `Base.ino:346-356`).
