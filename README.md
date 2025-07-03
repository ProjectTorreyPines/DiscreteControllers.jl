[![CI](https://github.com/mgyoo86/DiscreteControllers.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/mgyoo86/DiscreteControllers.jl/actions/workflows/CI.yml)
[![codecov](https://codecov.io/github/mgyoo86/DiscreteControllers.jl/graph/badge.svg?token=UJJVCRCCL6)](https://codecov.io/github/mgyoo86/DiscreteControllers.jl)

# DiscreteControllers.jl

A lightweight Julia package for discrete-time controllers with autonomous timing management, built-in logging, and optional plotting capabilities.

Built on top of [DiscretePIDs.jl](https://github.com/JuliaControl/DiscretePIDs.jl) to provide:
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

### Example 1: Basic Usage

```julia
using DiscreteControllers

# Your plant initialization
plant = construct_plant_system()

# Define constant target temperature (setpoint)
target_temperature = 30.0

# Create discrete controller with PID parameters
ctrl = DiscreteController(
    0.01;  # Sampling time: 10ms
    K=1.0, Ti=2.0, Td=0.1,  # PID parameters
    sp = target_temperature,  # Setpoint
    name = "temperature_controller"
)

# Manual simulation loop - you handle measurements and actuation
dt = 1e-3 # 1ms simulation step
for t in 0:dt:10
    # Your plant simulation continues independently
    simulate_plant_step!(plant, dt)

    # Update controller with temperature measurement
    current_temperature = get_temperature(plant)
    set_pv!(ctrl, current_temperature)

    # Controller automatically manages timing - only updates every 10ms
    was_updated = update_controller!(ctrl, t)

    if was_updated
        println("Controller updated at time $t")
        println("Control output: $(ctrl.mv)")

        # You manually apply the control signal
        heater_power = ctrl.mv
        set_heater_power!(plant, heater_power)  # Your function to control heater
    end
end
```

### Example 2: Advanced usage with SystemInterface

```julia
using DiscreteControllers

# Your plant initialization
plant = construct_plant_system()

# Suppose you have time-varying setpoint
scheduled_temperature(time) = time < 5.0 ? 10.0 : 25.0  # Step change at t=5s

# Create controller with automatic system interface
ctrl = DiscreteController(
    0.01;  # 10ms sampling time
    K=1.0, Ti=2.0, Td=0.1,
    system_interface=SystemInterface(
        # Read setpoint from a schedule (time-varying setpoint)
        read_setpoint = (time) -> scheduled_temperature(time),
        # Read current measurement from sensor/plant
        read_process_var = () -> get_temperature(plant),
        # Send control signal to actuator/plant
        apply_control_signal = (power) -> set_heater_power!(plant, power)
    )
)

# Fully automatic simulation loop
dt = 1e-3 # 1ms simulation step
for t in 0:dt:10
    # Your plant simulation continues independently
    simulate_plant_step!(plant, dt)

    # Controller handles everything automatically when updated:
    # - Reads setpoint from schedule
    # - Reads temperature from sensor
    # - Computes PID control
    # - Sends signal to heater
    was_updated = update_controller!(ctrl, t)

    if was_updated
        println("Controller updated at t=$t: SP=$(ctrl.sp), PV=$(ctrl.pv), MV=$(ctrl.mv)")
    end
end
```

### Data Export and Visualization

```julia
# Show current status of controller
show(ctrl)

# Export logged data for both examples
export_log(ctrl, "controller_data.csv")

# Visualize logged data (optional)
using Plots
plot(ctrl)
```

## License

This project is licensed under the Apache License 2.0.
