#!/usr/bin/env julia

"""
Simple test script for DiscreteController logging functionality.

This script simulates a simple temperature control system with:
- A virtual plant (first-order system)
- Noise and disturbances
- Automatic data logging
- CSV export for analysis
"""

using Pkg
Pkg.activate(".")

# Import required packages
using DiscreteControllers

# Simulation parameters
const SIM_TIME = 10.0      # Total simulation time [s]
const SIM_DT = 0.001       # Simulation time step [s]
const CTRL_TS = 0.01       # Controller sampling time [s]
const TARGET_TEMP = 75.0   # Target temperature [°C]

# Virtual plant state
mutable struct SimplePlant
    temperature::Float64    # Current temperature [°C]
    thermal_mass::Float64   # Thermal time constant [s]
    heater_gain::Float64    # Heater effectiveness [°C/signal]
    ambient_temp::Float64   # Ambient temperature [°C]
    noise_level::Float64    # Measurement noise level [°C]
end

# Plant constructor
function SimplePlant(;
    initial_temp::Float64 = 20.0,
    thermal_mass::Float64 = 2.0,
    heater_gain::Float64 = 0.5,
    ambient_temp::Float64 = 20.0,
    noise_level::Float64 = 0.1
)
    SimplePlant(initial_temp, thermal_mass, heater_gain, ambient_temp, noise_level)
end

# Plant dynamics (simple first-order system)
function update_plant!(plant::SimplePlant, heater_signal::Float64, dt::Float64)
    # First-order thermal dynamics: τ*dT/dt = heater_input - (T - T_ambient)
    temp_error = plant.temperature - plant.ambient_temp
    heating_effect = plant.heater_gain * heater_signal

    # Euler integration
    dT_dt = (heating_effect - temp_error) / plant.thermal_mass
    plant.temperature += dT_dt * dt
end

# Measurement with noise
function read_temperature(plant::SimplePlant)
    return plant.temperature + plant.noise_level * randn()
end

function main()
    println("=" ^ 60)
    println("DiscreteController Logging Test")
    println("=" ^ 60)

    # Create virtual plant
    plant = SimplePlant(
        initial_temp = 20.0,
        thermal_mass = 3.0,    # Slower system
        heater_gain = 1.2,     # Strong heater
        ambient_temp = 20.0,
        noise_level = 0.2      # Some measurement noise
    )

    # Create PID controller
    pid = DiscretePID(K=2.0, Ti=1.0, Td=0.1, Ts=CTRL_TS)

    # Storage for heater signal
    current_heater_signal = Ref(0.0)

    # Create controller with logging enabled
    controller = DiscreteController(;
        pid = pid,
        target = TARGET_TEMP,
        name = "temperature_controller",
        measure_func = () -> read_temperature(plant),
        actuate_func = (signal) -> current_heater_signal[] = signal,
        Ts = CTRL_TS,
        initial_time = 0.0,
        enable_logging = true  # Enable automatic logging!
    )

    println("Controller created with logging enabled")
    println("Target temperature: $(TARGET_TEMP)°C")
    println("Controller sampling time: $(CTRL_TS)s")
    println("Simulation time: $(SIM_TIME)s")
    println()

    # Simulation loop
    println("Running simulation...")

    # Add some setpoint changes to make it interesting
    setpoint_changes = [
        (2.0, 80.0),    # At t=2s, change to 80°C
        (5.0, 70.0),    # At t=5s, change to 70°C
        (8.0, 85.0),    # At t=8s, change to 85°C
    ]
    change_idx = 1

    for t in 0:SIM_DT:SIM_TIME
        # Check for setpoint changes
        if change_idx <= length(setpoint_changes) && t >= setpoint_changes[change_idx][1]
            new_target = setpoint_changes[change_idx][2]
            set_target!(controller, new_target)
            println("Time $(t)s: Target changed to $(new_target)°C")
            change_idx += 1
        end

        # Update controller (automatically logs when it updates)
        update_controller!(controller, t)

        # Update plant with current heater signal
        update_plant!(plant, current_heater_signal[], SIM_DT)

        # Print progress occasionally
        if t % 1.0 ≈ 0.0  # Every second
            temp = plant.temperature
            target = controller.target
            error = target - temp
            println("t=$(round(t,digits=1))s: T=$(round(temp,digits=1))°C, Target=$(round(target,digits=1))°C, Error=$(round(error,digits=1))°C")
        end
    end

    println("\nSimulation completed!")

    # Check logging results
    if controller.logger !== nothing
        num_samples = length(controller.logger.timestamps)
        println("Logged $(num_samples) data points")

        # Export to CSV
        filename = "temperature_control_test.csv"
        export_controller_log(controller, filename)
        println("Data exported to: $(filename)")

        # Show some statistics
        if num_samples > 0
            final_temp = controller.state.last_measurement
            final_error = abs(controller.target - final_temp)
            avg_error = sum(abs.(controller.logger.errors)) / length(controller.logger.errors)

            println("\nResults Summary:")
            println("  Final temperature: $(round(final_temp, digits=2))°C")
            println("  Final error: $(round(final_error, digits=2))°C")
            println("  Average absolute error: $(round(avg_error, digits=2))°C")
            println("  Total controller updates: $(controller.monitor.update_count)")
        end
    else
        @warn "No logger found - logging may not be working properly"
    end

    # Performance diagnostics
    println("\nController Diagnostics:")
    print_controller_diagnostics(controller, detailed=true)

    println("=" ^ 60)
    println("Test completed! Check '$(pwd())/temperature_control_test.csv' for detailed data.")
    println("You can plot this data in Python, R, or Excel to visualize controller performance.")
    println("=" ^ 60)
end

# Run the test
if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
