[![CI](https://github.com/mgyoo86/DiscreteControllers.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/mgyoo86/DiscreteControllers.jl/actions/workflows/CI.yml)
[![codecov](https://codecov.io/github/mgyoo86/DiscreteControllers.jl/graph/badge.svg?token=UJJVCRCCL6)](https://codecov.io/github/mgyoo86/DiscreteControllers.jl)

# DiscreteControllers.jl

A lightweight Julia package for discrete-time controllers with autonomous timing management, built-in logging, and optional plotting capabilities.

Built on top of [DiscretePIDs.jl](https://github.com/DiscretePIDs/DiscretePIDs.jl) to provide:
- **Autonomous timing management** - Controllers handle their own sampling time automatically
- **Built-in logging** - Automatic data collection during control updates
- **Flexible construction** - Create controllers with PID parameters or existing DiscretePID objects
- **Optional plotting** - Visualization capabilities when Plots.jl is available

## Installation

```julia
using Pkg
Pkg.add("DiscreteControllers")
```

## Quick Start

```julia
using DiscreteControllers

# Create discrete controller with PID parameters
ctrl = DiscreteController(
    0.01;  # Sampling time: 10ms
    K=1.0, Ti=2.0, Td=0.1,  # PID parameters
    sp=100.0,  # Setpoint
    name="temperature_controller"
)

# With external interface for automatic measurement/actuation
ctrl = DiscreteController(
    0.01;
    K=1.0, Ti=2.0, Td=0.1,
    sp=100.0,
    external=ExternalInterface(
        measure_process_variable = () -> read_sensor(),
        apply_manipulated_variable = (mv) -> set_actuator(mv)
    )
)

# In simulation loop
for t in 0:0.001:10  # 1ms simulation steps
    # Controller automatically manages timing - only updates every 10ms
    was_updated = update_controller!(ctrl, t)

    if was_updated
        println("Controller updated at time $t")
    end
end

# Export logged data
export_log(ctrl, "controller_data.csv")

# Visualize logged data (optional)
using Plots
plot(ctrl)
```

## License

This project is licensed under the Apache License 2.0.
