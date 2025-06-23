"""
Type definitions for DiscreteControllers.jl

This module defines all the core types used in the discrete control system:
- TimingManager: handles sampling time and scheduling
- PerformanceMonitor: tracks performance metrics
- Logger: collects time-series data
- DiscreteController: main controller type
- ExternalInterface: groups I/O callbacks for external system integration
"""

"""
    TimingManager{FT<:AbstractFloat}

Internal structure for managing controller timing and scheduling.
"""
mutable struct TimingManager{FT<:AbstractFloat}
    current_time::FT          # Current simulation time [s]
    last_update_time::FT      # Last control update time [s]
    next_scheduled_time::FT   # Next scheduled update time [s]
    const tolerance::FT       # Timing tolerance for sampling
end

"""
    PerformanceMonitor

Internal structure for tracking controller performance metrics.
"""
mutable struct PerformanceMonitor
    update_count::Int         # Total updates performed
    missed_deadlines::Int     # Missed sampling deadlines
end

"""
    Logger{FT<:AbstractFloat}

Simple internal logger for collecting controller data over time.
"""
mutable struct Logger{FT<:AbstractFloat}
    timestamps::Vector{FT}
    setpoints::Vector{FT}
    process_variables::Vector{FT}
    manipulated_variables::Vector{FT}
    errors::Vector{FT}
    update_counts::Vector{Int}

    Logger{FT}() where {FT<:AbstractFloat} = new{FT}(FT[], FT[], FT[], FT[], FT[], Int[])
end

"""
    ExternalInterface

Interface functions for connecting the controller to external systems.
Groups all I/O functions together for better organization.
"""
mutable struct ExternalInterface
    # Input functions (called before control calculation)
    measure_process_variable::Union{Function, Nothing}  # () -> Real - read sensor/measurement
    set_setpoint::Union{Function, Nothing}              # () -> Real - get reference/setpoint

    # Output functions (called after control calculation)
    apply_manipulated_variable::Union{Function, Nothing} # (mv::Real) -> nothing - send to actuator

    ExternalInterface(;
        measure_process_variable::Union{Function, Nothing} = nothing,
        set_setpoint::Union{Function, Nothing} = nothing,
        apply_manipulated_variable::Union{Function, Nothing} = nothing
    ) = new(measure_process_variable, set_setpoint, apply_manipulated_variable)
end

"""
    DiscreteController{FT<:AbstractFloat}

A self-contained discrete controller that manages its own sampling timing.

# Fields
- `name::String`: Name/identifier for this controller
- `Ts::FT`: Sample time (immutable, must be set at construction) [s]
- `pid::DiscretePID{FT}`: The underlying discrete PID controller
- `sp::FT`: Current setpoint value
- `pv::FT`: Current process variable (measured value)
- `error::FT`: Current control error (sp - pv)
- `mv::FT`: Current manipulated variable (control output)
- `measure_process_variable::Union{Function, Nothing}`: Optional function to read PV `() -> Real`
- `set_setpoint::Union{Function, Nothing}`: Optional function to get setpoint `() -> Real`
- `apply_manipulated_variable::Union{Function, Nothing}`: Optional function to apply MV `(mv::Real) -> nothing`
- `timing::TimingManager{FT}`: Timing and scheduling management
- `monitor::PerformanceMonitor`: Performance monitoring metrics
- `enable_logging::Bool`: Whether to log data during control updates (every Ts)
- `logger::Logger{FT}`: Logger for control updates (every Ts)

# Example
```julia
using DiscretePIDs, DiscreteControllers

# Create underlying PID
pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

# Create discrete controller with value-based interface
ctrl = DiscreteController(;
    pid = pid,
    sp = 100.0,
    name = "temperature",
    Ts = 0.01,  # 10ms sampling
    initial_time = 0.0
)

# Or with callback-based interface
ctrl = DiscreteController(;
    pid = pid,
    sp = 100.0,
    name = "temperature",
    Ts = 0.01,
    external = ExternalInterface(
        measure_process_variable = () -> read_temperature(),
        apply_manipulated_variable = (mv) -> set_heater_power(mv)
    ),
    initial_time = 0.0
)

# In simulation loop
for t in 0:0.001:10  # 1ms simulation steps
    was_updated = update_controller!(ctrl, t)  # Only updates when Ts elapsed
end
```
"""
mutable struct DiscreteController{FT<:AbstractFloat}
    # Core identification and control
    name::String
    const Ts::FT              # Sample time (immutable)
    is_active::Bool           # Controller enable/disable state
    pid::DiscretePID{FT}

    # Process variables (always stored as values for consistency)
    sp::FT                    # Setpoint
    pv::FT                    # Process variable (measurement)
    error::FT                 # Control error (sp - pv)
    mv::FT                    # Manipulated variable (control output)

    # Optional functions for external system integration
    # These are called at specific points in the control loop if provided
    external::ExternalInterface

    # Internal management structures
    timing::TimingManager{FT}
    monitor::PerformanceMonitor

    # Logging (always available, flag controls usage)
    enable_logging::Bool
    logger::Logger{FT}

    function DiscreteController{FT}(;
        pid::DiscretePID{FT},
        sp::Real,
        name::String,
        Ts::Real,
        pv::Real = 0.0,
        initial_time::Real = 0.0,
        is_active::Bool = true,
        timing_tolerance::Real = 1e-12,
        external::ExternalInterface = ExternalInterface(),
        enable_logging::Bool = false
    ) where {FT<:AbstractFloat}
        @assert Ts > 0 "Sample time Ts must be positive"
        @assert Ts == pid.Ts "Controller's sample time (Ts=$(Ts)) must match PID sample time=$(pid.Ts)"

        # Convert inputs to appropriate types
        sp_val = FT(sp)
        pv_val = FT(pv)
        Ts_val = FT(Ts)

        new{FT}(
            name, Ts_val, is_active, pid,
            sp_val, pv_val, sp_val - pv_val, FT(0.0),
            external,
            TimingManager{FT}(
                FT(initial_time), FT(initial_time),
                FT(initial_time + Ts_val), FT(timing_tolerance)
            ),
            PerformanceMonitor(0, 0),
            enable_logging,
            Logger{FT}()
        )
    end
end

# Convenience constructor for Float64
DiscreteController(args...; kwargs...) = DiscreteController{Float64}(args...; kwargs...)
