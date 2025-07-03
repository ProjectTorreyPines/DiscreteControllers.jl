[![CI](https://github.com/mgyoo86/DiscreteControllers.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/mgyoo86/DiscreteControllers.jl/actions/workflows/CI.yml)
[![codecov](https://codecov.io/github/mgyoo86/DiscreteControllers.jl/graph/badge.svg?token=UJJVCRCCL6)](https://codecov.io/github/mgyoo86/DiscreteControllers.jl)

# DiscreteControllers.jl

A lightweight Julia package for discrete controller timing management with autonomous timing control, data logging, and basic diagnostics.

Built on top of [DiscretePIDs.jl](https://github.com/DiscretePIDs/DiscretePIDs.jl) to provide:
- **Autonomous sampling time management** - Controllers handle their own timing automatically
- **Automatic data logging** - Built-in logging for analysis and visualization
- **Basic performance diagnostics** - Simple tools for monitoring controller performance
- **Simple visualization utilities** - Easy plotting and analysis capabilities

## Installation

```julia
using Pkg
Pkg.add("DiscreteControllers")
```

## Quick Start

```julia
using DiscreteControllers
using DiscretePIDs

# Create a PID controller
pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

# Create discrete controller with timing management
controller = DiscreteController(;
    pid = pid,
    target = 100.0,
    name = "temperature",
    measure_func = () -> read_temperature(),     # Your measurement function
    actuate_func = (signal) -> set_heater_power(signal),  # Your actuation function
    Ts = 0.01,  # 10ms sampling period
    enable_logging = true  # Enable automatic data logging
)

# In your simulation loop
for t in 0:0.001:10  # 1ms simulation steps, but controller only updates every 10ms
    # Controller automatically manages timing - only updates when Ts has elapsed
    was_updated = update_controller!(controller, t)

    if was_updated
        println("Control updated at time $t")
    end
end

# Export logged data for analysis
export_controller_log(controller, "controller_data.csv")

# Get performance diagnostics
diagnostics = diagnose_controller(controller)
print_controller_diagnostics(diagnostics)
```

## Key Features

### Autonomous Timing Management
- Each controller manages its own sampling time automatically
- No external scheduler needed - just call `update_controller!` with current time
- Controller only performs control calculations when sampling time has elapsed
- Precise timing with configurable tolerance

### Built-in Logging
- Automatic logging of controller state during operation
- Export to CSV for analysis with external tools
- Simple enable/disable and clear functionality

### Simple Diagnostics
- Basic performance metrics (update rate, missed deadlines, etc.)
- Control state monitoring (errors, outputs, measurements)
- Easy-to-read diagnostic reports

## API Overview

### Core Functions
- `DiscreteController(...)` - Create a new controller with timing management
- `update_controller!(ctrl, time)` - Update controller to new time (handles timing automatically)
- `set_target!(ctrl, target)` - Change setpoint
- `activate!(ctrl)` / `deactivate!(ctrl)` - Enable/disable control
- `reset!(ctrl, time)` - Reset controller state

### Logging Functions
- `enable_logging!(ctrl)` / `disable_logging!(ctrl)` - Control automatic logging
- `export_controller_log(ctrl, filename)` - Export logged data to CSV
- `clear_controller_log!(ctrl)` - Clear logged data

### Diagnostics Functions
- `diagnose_controller(ctrl)` - Get performance diagnostics
- `print_controller_diagnostics(diagnostics)` - Print readable diagnostic report

## Why This Package?

This package fills the gap between low-level PID controllers and complex control systems by providing:

1. **Time management** - Handles discrete-time sampling automatically
2. **Data collection** - Built-in logging for analysis and tuning
3. **Simplicity** - Lightweight and focused on essential features
4. **Flexibility** - Easy integration with any measurement/actuation system

Perfect for applications where you need discrete control with proper timing, data collection for analysis, and simple performance monitoring - without the complexity of full control system frameworks.

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## License

This project is licensed under the Apache License 2.0.
