"""
Type definitions for DiscreteControllers.jl

This module defines all the core types used in the discrete control system:
- TimingManager: handles sampling time and scheduling
- PerformanceMonitor: tracks performance metrics
- Logger: collects time-series data
- DiscreteController: main controller type
- SystemInterface: groups I/O callbacks for system interaction
"""

"""
    TimingManager{FT<:AbstractFloat}

Internal structure for managing controller timing and scheduling.
"""
@kwdef mutable struct TimingManager{FT<:AbstractFloat}
    initial_time::FT = zero(FT)         # Initial time for the controller [s]
    current_time::FT = initial_time     # Current simulation time [s]
    next_scheduled_time::FT             # Next scheduled update time [s] (no default, must be set)
    last_update_time::FT = typemin(FT)  # Last control update time [s] (default to -Inf, meaning not yet updated)
    tolerance::FT = FT(1e-6)            # Timing relative tolerance for sampling
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
    SystemInterface

Interface functions for connecting the controller to system_interface systems.
Groups all I/O functions together for better organization.
"""
@kwdef mutable struct SystemInterface
    read_setpoint::Union{Function, Nothing} = nothing       # (time) -> Real - get reference/setpoint
    read_process_var::Union{Function, Nothing} = nothing    # () -> Real - read sensor/measurement
    apply_control_signal::Union{Function, Nothing} = nothing # (signal) -> nothing - send to actuator
end

"""
    DiscreteController{FT<:AbstractFloat}

Discrete controller with autonomous timing management and built-in logging.

# Fields
- `name::String`: Controller identifier
- `Ts::FT`: Sample time [s] (immutable)
- `pid::DiscretePID{FT}`: Underlying PID controller
- `sp::FT`: Current setpoint
- `pv::FT`: Current process variable
- `error::FT`: Current control error (sp - pv)
- `mv::FT`: Current manipulated variable
- `system_interface::SystemInterface`: Optional I/O callbacks
- `timing::TimingManager{FT}`: Timing management
- `monitor::PerformanceMonitor`: Performance metrics
- `enable_logging::Bool`: Logging enable flag
- `logger::Logger{FT}`: Data logger

# Examples
```julia
# With PID parameters
ctrl = DiscreteController(0.01; K=1.0, Ti=2.0, Td=0.1, sp=100.0)

# With system_interface interface
ctrl = DiscreteController(0.01; K=1.0, Ti=2.0, sp=100.0,
    system_interface=SystemInterface(
        read_process_var = () -> read_sensor(),
        apply_control_signal = (signal) -> set_actuator(signal)
    ))
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
    mv::FT = NaN                        # Manipulated variable (control output)

    # Optional functions for system_interface system integration
    # These are called at specific points in the control loop if provided
    system_interface::SystemInterface = SystemInterface()

    # Internal management structures
    timing::TimingManager{FT}  # No default - must be set in constructor
    monitor::PerformanceMonitor = PerformanceMonitor()

    # Logging (always available, flag controls usage)
    enable_logging::Bool = true
    logger::Logger{FT} = Logger{FT}()
end


"""
    DiscreteController(pid::DiscretePID; kwargs...)

Create a discrete controller with an existing DiscretePID.

# Arguments
- `pid::DiscretePID`: Pre-configured PID controller
- `initial_time::Real=0.0`: Initial simulation time
- `kwargs...`: Additional controller parameters (sp, name, system_interface, etc.)
"""
function DiscreteController(pid::DiscretePID{FT};
    initial_time::Real = 0.0,
    kwargs...
) where {FT<:AbstractFloat}
    @assert pid.Ts > 0 "Sample time Ts must be positive"
    @assert !haskey(kwargs, :Ts) "Cannot specify Ts when providing a DiscretePID (PID already has Ts=$(pid.Ts))"
    @assert !haskey(kwargs, :timing) "Timing is set automatically based on Ts and initial_time, do not provide timing in kwargs"

    # Convert kwargs to mutable dict and validate/set initial state with system_interface interface
    kwargs = Dict{Symbol, Any}(kwargs)
    validate_and_set_initial_state_with_external!(kwargs, initial_time)

    # Convert inputs to appropriate types
    Ts = pid.Ts

    return DiscreteController{FT}(;
        Ts, pid,
        timing = TimingManager{FT}(
            initial_time = FT(initial_time),
            current_time = FT(initial_time),
            next_scheduled_time = FT(initial_time + Ts),  # First update at initial_time + Ts
            last_update_time = typemin(FT)
        ),
        kwargs...
    )
end

"""
    DiscreteController(Ts::Real; kwargs...)

Create a discrete controller with sampling time and PID parameters.

# Arguments
- `Ts::Real`: Sampling time [s]
- `initial_time::Real=0.0`: Initial simulation time
- `K, Ti, Td, ...`: PID parameters passed to DiscretePID constructor
- `sp, name, system_interface, ...`: Controller-specific parameters

# Examples
```julia
# Basic PID controller
ctrl = DiscreteController(0.01; K=1.0, Ti=2.0, sp=100.0)

# With system_interface interface
ctrl = DiscreteController(0.01; K=1.0, Ti=2.0, sp=100.0,
    system_interface=SystemInterface(
        read_process_var = () -> read_sensor()
    ))
```
"""
function DiscreteController(Ts::FT;
    initial_time::Real = 0.0,
    kwargs...
) where {FT<:AbstractFloat}
    @assert Ts > 0 "Sample time Ts must be positive"
    @assert !haskey(kwargs, :pid) "Cannot specify pid when providing Ts (use DiscreteController(pid; ...) instead)"
    @assert !haskey(kwargs, :timing) "Timing is set automatically based on Ts and initial_time, do not provide timing in kwargs"

    # Convert kwargs to mutable dict and validate/set initial state with system_interface interface
    kwargs = Dict{Symbol, Any}(kwargs)
    validate_and_set_initial_state_with_external!(kwargs, initial_time)

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
            initial_time = FT(initial_time),
            current_time = FT(initial_time),
            next_scheduled_time = FT(initial_time + Ts),  # First update at initial_time + Ts
            last_update_time = typemin(FT)
        ),
        controller_kwargs...
    )
end

# Convenience constructors for common use cases
function DiscreteController(Ts::Int; kwargs...)
    return DiscreteController(Float64(Ts); kwargs...)
end

"""
    validate_and_set_initial_state_with_external!(kwargs_dict, initial_time)

Helper function to validate and set initial state (sp, pv) when system_interface interface is provided.
Modifies kwargs_dict in-place to add sp and/or pv if they are not provided but system_interface functions exist.
"""
function validate_and_set_initial_state_with_external!(kwargs_dict::Dict{Symbol, Any}, initial_time::Real)
    if haskey(kwargs_dict, :system_interface)
        system_interface = kwargs_dict[:system_interface]

        @assert system_interface isa SystemInterface "system_interface must be an instance of SystemInterface"

        # Handle setpoint
        if !isnothing(system_interface.read_setpoint)
            if haskey(kwargs_dict, :sp)
                external_sp = system_interface.read_setpoint(initial_time)
                @assert kwargs_dict[:sp] == external_sp "Given initial setpoint sp ($(kwargs_dict[:sp])) must match system_interface setpoint function output ($(external_sp))"
            else
                kwargs_dict[:sp] = system_interface.read_setpoint(initial_time)
            end
        end

        # Handle process variable
        if !isnothing(system_interface.read_process_var)
            if haskey(kwargs_dict, :pv)
                external_pv = system_interface.read_process_var()
                @assert kwargs_dict[:pv] == external_pv "Given initial process variable pv ($(kwargs_dict[:pv])) must match system_interface measurement ($(external_pv))"
            else
                kwargs_dict[:pv] = system_interface.read_process_var()
            end
        end
    end
end