"""
Core controller functionality for DiscreteControllers.jl

This module implements the main control logic and state management:
- Controller update and timing management
- State management (activate, deactivate, reset)
- Setpoint and process variable handling
- Control loop execution with hybrid value/callback support
"""

"""
    update_controller!(ctrl::DiscreteController{FT}, new_time::FT) where {FT<:AbstractFloat}

Update a discrete controller to a new simulation time.

The controller will only perform a control update if sufficient time has passed
since the last update (≥ Ts - tolerance). This ensures proper discrete-time
behavior with accurate sampling.

# Arguments
- `ctrl::DiscreteController{FT}`: The controller to update
- `new_time::FT`: Current simulation time

# Returns
- `Bool`: `true` if control update was performed, `false` if just time advanced

# Example
```julia
# In simulation loop
for t in 0:dt:t_final
    # Only updates control when sampling time is reached
    was_updated = update_controller!(ctrl, t)

    if was_updated
        println("Control updated at time \$t")
    end
end
```
"""
function update_controller!(
    ctrl::DiscreteController{FT},
    new_time::FT
) where {FT<:AbstractFloat}

    # Validate time progression
    if new_time < ctrl.timing.current_time
        @warn "Time regression detected in $(ctrl.name): $(new_time) < $(ctrl.timing.current_time)"
        return false
    end

    # Update current time
    ctrl.timing.current_time = new_time

    # Check if controller is active
    if !ctrl.is_active
        return false
    end

    # Check if sampling time has arrived
    # Use next_scheduled_time for more precise timing control
    should_update = new_time >= (ctrl.timing.next_scheduled_time - ctrl.timing.tolerance * ctrl.Ts)

    if should_update
        try
            # Update setpoint if callback provided
            if ctrl.external.set_setpoint !== nothing
                ctrl.sp = FT(ctrl.external.set_setpoint(new_time))
            end

            # Update process variable if callback provided
            if ctrl.external.measure_process_variable !== nothing
                ctrl.pv = FT(ctrl.external.measure_process_variable())
            end

            # Update error
            ctrl.error = ctrl.sp - ctrl.pv

            # Compute control signal using PID
            ctrl.mv = ctrl.pid(
                ctrl.sp,      # setpoint
                ctrl.pv,      # process variable
                FT(0.0)       # feedforward (could be extended)
            )

            # Apply control signal if callback provided
            if ctrl.external.apply_manipulated_variable !== nothing
                ctrl.external.apply_manipulated_variable(ctrl.mv)
            end

            # Update internal timing state
            ctrl.timing.last_update_time = new_time
            ctrl.timing.next_scheduled_time = new_time + ctrl.Ts
            ctrl.monitor.update_count += 1

            # Auto-log data if enabled (only during control updates)
            if ctrl.enable_logging
                log_to_logger!(ctrl.logger, ctrl, new_time)
            end

        catch e
            @error "Controller update failed" controller=ctrl.name exception=e
            ctrl.monitor.missed_deadlines += 1
            return false
        end
    end

    return should_update  # Return whether control update was performed
end

"""
    set_setpoint!(ctrl::DiscreteController, new_sp::Real)

Change the setpoint value of the controller.

# Arguments
- `ctrl::DiscreteController`: The controller to modify
- `new_sp::Real`: New setpoint value
"""
function set_setpoint!(ctrl::DiscreteController{FT}, new_sp::Real) where {FT<:AbstractFloat}
    ctrl.sp = FT(new_sp)
    ctrl.error = ctrl.sp - ctrl.pv  # Update error immediately
end

"""
    set_pv!(ctrl::DiscreteController, new_pv::Real)

Set the process variable value manually (when not using measure_pv callback).

# Arguments
- `ctrl::DiscreteController`: The controller to modify
- `new_pv::Real`: New process variable value
"""
function set_pv!(ctrl::DiscreteController{FT}, new_pv::Real) where {FT<:AbstractFloat}
    ctrl.pv = FT(new_pv)
    ctrl.error = ctrl.sp - ctrl.pv  # Update error immediately
end

"""
    get_setpoint(ctrl::DiscreteController)

Get the current setpoint value.

# Returns
- Current setpoint value
"""
function get_setpoint(ctrl::DiscreteController)
    return ctrl.sp
end

"""
    get_pv(ctrl::DiscreteController)

Get the current process variable value.

# Returns
- Current process variable value
"""
function get_pv(ctrl::DiscreteController)
    return ctrl.pv
end

"""
    get_mv(ctrl::DiscreteController)

Get the current manipulated variable (control output).

# Returns
- Current manipulated variable value
"""
function get_mv(ctrl::DiscreteController)
    return ctrl.mv
end

"""
    get_error(ctrl::DiscreteController)

Get the current control error (sp - pv).

# Returns
- Current error value
"""
function get_error(ctrl::DiscreteController)
    return ctrl.error
end

"""
    activate!(ctrl::DiscreteController)

Activate the controller (enable control updates).
"""
function activate!(ctrl::DiscreteController)
    ctrl.is_active = true
end

"""
    deactivate!(ctrl::DiscreteController)

Deactivate the controller (disable control updates).
"""
function deactivate!(ctrl::DiscreteController)
    ctrl.is_active = false
end

"""
    reset!(ctrl::DiscreteController, time::Real)

Reset the controller's internal state and timing.

# Arguments
- `ctrl::DiscreteController`: The controller to reset
- `time::Real`: Current time to reset to
"""
function reset!(ctrl::DiscreteController{FT}, time::Real) where {FT<:AbstractFloat}
    ctrl.timing.current_time = FT(time)
    ctrl.timing.last_update_time = FT(time)
    ctrl.timing.next_scheduled_time = FT(time + ctrl.Ts)
    ctrl.mv = FT(0.0)
    ctrl.pv = FT(0.0)
    ctrl.error = ctrl.sp - ctrl.pv
    ctrl.monitor.update_count = 0
    ctrl.monitor.missed_deadlines = 0

    # Reset PID internal state
    reset_state!(ctrl.pid)
end

"""
    get_sampling_time(ctrl::DiscreteController)

Get the controller's sampling time.

# Returns
- Sampling time Ts
"""
function get_sampling_time(ctrl::DiscreteController)
    return ctrl.Ts
end

"""
    is_active(ctrl::DiscreteController)

Check if the controller is currently active.

# Returns
- `true` if controller is active, `false` otherwise
"""
function is_active(ctrl::DiscreteController)
    return ctrl.is_active
end

"""
    get_update_count(ctrl::DiscreteController)

Get the total number of control updates performed.

# Returns
- Number of control updates
"""
function get_update_count(ctrl::DiscreteController)
    return ctrl.monitor.update_count
end

"""
    get_missed_deadlines(ctrl::DiscreteController)

Get the number of missed control deadlines.

# Returns
- Number of missed deadlines
"""
function get_missed_deadlines(ctrl::DiscreteController)
    return ctrl.monitor.missed_deadlines
end

"""
    time_until_next_update(ctrl::DiscreteController)

Get the time remaining until the next scheduled control update.

# Returns
- Time until next update (can be negative if overdue)
"""
function time_until_next_update(ctrl::DiscreteController)
    return ctrl.timing.next_scheduled_time - ctrl.timing.current_time
end

"""
    set_timing_tolerance!(ctrl::DiscreteController, tolerance::Real)

Set the timing tolerance for sampling decisions.

# Arguments
- `ctrl::DiscreteController`: The controller to modify
- `tolerance::Real`: New timing tolerance value
"""
function set_timing_tolerance!(ctrl::DiscreteController{FT}, tolerance::Real) where {FT<:AbstractFloat}
    ctrl.timing.tolerance = FT(tolerance)
end
