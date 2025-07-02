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
@kwdef mutable struct TimingManager{FT<:AbstractFloat}
    current_time::FT = zero(FT)          # Current simulation time [s]
    last_update_time::FT = zero(FT)     # Last control update time [s]
    next_scheduled_time::FT = zero(FT)  # Next scheduled update time [s]
    const tolerance::FT = FT(1e-6)      # Timing tolerance for sampling
end

"""
    PerformanceMonitor

Internal structure for tracking controller performance metrics.
"""
@kwdef mutable struct PerformanceMonitor
    update_count::Int = 0        # Total updates performed
    missed_deadlines::Int = 0    # Missed sampling deadlines
end

"""
    Logger{FT<:AbstractFloat}

Simple internal logger for collecting controller data over time.
"""
@kwdef mutable struct Logger{FT<:AbstractFloat}
    timestamps::Vector{FT} = FT[]
    setpoints::Vector{FT} = FT[]
    process_variables::Vector{FT} = FT[]
    manipulated_variables::Vector{FT} = FT[]
    errors::Vector{FT} = FT[]
    update_counts::Vector{Int} = Int[]
end

"""
    ExternalInterface

Interface functions for connecting the controller to external systems.
Groups all I/O functions together for better organization.
"""
@kwdef mutable struct ExternalInterface
    # Input functions (called before control calculation)
    measure_process_variable::Union{Function, Nothing} = nothing # () -> Real - read sensor/measurement
    set_setpoint::Union{Function, Nothing} = nothing              # () -> Real - get reference/setpoint

    # Output functions (called after control calculation)
    apply_manipulated_variable::Union{Function, Nothing} = nothing # (mv::Real) -> nothing - send to actuator
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
@kwdef mutable struct DiscreteController{FT<:AbstractFloat}
    # Core identification and control
    name::String = ""
    is_active::Bool = true          # Controller enable/disable state

    const Ts::FT              # Sample time (immutable)
    pid::DiscretePID{FT}

    # Process variables (always stored as values for consistency)
    sp::FT = zero(FT)                   # Setpoint
    pv::FT = zero(FT)                   # Process variable (measurement)
    error::FT = sp - pv                 # Control error (sp - pv)
    mv::FT = zero(FT)                   # Manipulated variable (control output)

    # Optional functions for external system integration
    # These are called at specific points in the control loop if provided
    external::ExternalInterface = ExternalInterface()

    # Internal management structures
    timing::TimingManager{FT} = TimingManager{FT}()
    monitor::PerformanceMonitor = PerformanceMonitor()

    # Logging (always available, flag controls usage)
    enable_logging::Bool = true
    logger::Logger{FT} = Logger{FT}()
end


# Constructor with a given DiscretePID
function DiscreteController(pid::DiscretePID{FT};
    initial_time::Real = 0.0,
    kwargs...
) where {FT<:AbstractFloat}
    @assert pid.Ts > 0 "Sample time Ts must be positive"
    @assert !haskey(kwargs, :Ts) "Cannot specify Ts when providing a DiscretePID (PID already has Ts=$(pid.Ts))"

    # Convert inputs to appropriate types
    Ts = pid.Ts
    return DiscreteController{FT}(;
        Ts, pid,
        timing = TimingManager{FT}(
            current_time = FT(initial_time),
            last_update_time = FT(initial_time),
            next_scheduled_time = FT(initial_time + Ts)
        ),
        kwargs...
    )
end

# Constructor with sampling time Ts (creates PID internally)
function DiscreteController(Ts::FT;
    initial_time::Real = 0.0,
    kwargs...
) where {FT<:AbstractFloat}
    @assert Ts > 0 "Sample time Ts must be positive"
    @assert !haskey(kwargs, :pid) "Cannot specify pid when providing Ts (use DiscreteController(pid; ...) instead)"

    # Filter kwargs based on field names
    pid_fields = Set(fieldnames(DiscretePID{FT}))
    controller_fields = Set(fieldnames(DiscreteController{FT}))

    # Separate kwargs using filter and set membership
    pid_kwargs = filter(p -> first(p) ∈ pid_fields, kwargs)
    controller_kwargs = filter(p -> first(p) ∈ controller_fields, kwargs)

    # Convert pid_kwargs values to FT type for numeric fields
    pid_kwargs_converted = Dict{Symbol, Any}(
        k => (v isa Real ? FT(v) : v) for (k, v) in pid_kwargs
    )

    # Create DiscretePID with PID-specific parameters and kwargs
    pid = DiscretePID(; Ts, pid_kwargs_converted...)

    # Create DiscreteController with controller-specific kwargs
    return DiscreteController{FT}(;
        Ts, pid,
        timing = TimingManager{FT}(
            current_time = FT(initial_time),
            last_update_time = FT(initial_time),
            next_scheduled_time = FT(initial_time + Ts)
        ),
        controller_kwargs...
    )
end

# Convenience constructors for common use cases
function DiscreteController(Ts::Int; kwargs...)
    return DiscreteController(Float64(Ts); kwargs...)
end
