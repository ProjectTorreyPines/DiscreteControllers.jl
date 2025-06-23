"""
Logging functionality for DiscreteControllers.jl

This module provides logging capabilities for discrete controllers:
- Control update logging (only during control updates every Ts)
- Data export to CSV
- Log management functions
"""

"""
    enable_logging!(ctrl::DiscreteController)

Enable logging during control updates.
"""
function enable_logging!(ctrl::DiscreteController)
    ctrl.enable_logging = true
end

"""
    disable_logging!(ctrl::DiscreteController)

Disable logging.
"""
function disable_logging!(ctrl::DiscreteController)
    ctrl.enable_logging = false
end

"""
    clear_log!(ctrl::DiscreteController)

Clear the controller's log data.
"""
function clear_log!(ctrl::DiscreteController)
    clear_logger!(ctrl.logger)
end

"""
    export_log(ctrl::DiscreteController, filename::String)

Export the controller's log data to CSV.
"""
function export_log(ctrl::DiscreteController, filename::String)
    export_logger_csv(filename, ctrl.logger)
end

"""
    clear_logger!(logger::Logger)

Clear all logged data from the logger.
"""
function clear_logger!(logger::Logger)
    empty!(logger.timestamps)
    empty!(logger.setpoints)
    empty!(logger.process_variables)
    empty!(logger.manipulated_variables)
    empty!(logger.errors)
    empty!(logger.update_counts)
end

"""
    export_logger_csv(filename::String, logger::Logger)

Export logged controller data to CSV for plotting and analysis.
"""
function export_logger_csv(filename::String, logger::Logger)
    if isempty(logger.timestamps)
        @warn "Logger is empty, no data to export"
        return
    end

    open(filename, "w") do io
        # Header
        println(io, "time,setpoint,process_variable,manipulated_variable,error,update_count")

        # Data rows
        for i in 1:length(logger.timestamps)
            println(io, "$(logger.timestamps[i]),$(logger.setpoints[i]),$(logger.process_variables[i]),$(logger.manipulated_variables[i]),$(logger.errors[i]),$(logger.update_counts[i])")
        end
    end

    println("Controller time-series data exported to: $filename")
end

"""
    log_to_logger!(logger::Logger{FT}, ctrl::DiscreteController{FT}, time::FT) where {FT<:AbstractFloat}

Internal helper function to log controller state to a specific logger.
"""
function log_to_logger!(logger::Logger{FT}, ctrl::DiscreteController{FT}, time::FT) where {FT<:AbstractFloat}
    push!(logger.timestamps, time)
    push!(logger.setpoints, ctrl.sp)
    push!(logger.process_variables, ctrl.pv)
    push!(logger.manipulated_variables, ctrl.mv)
    push!(logger.errors, ctrl.error)
    push!(logger.update_counts, ctrl.monitor.update_count)
end

"""
    show_controller_status(ctrl::DiscreteController)

Display a brief status summary of the controller.
"""
function show_controller_status(ctrl::DiscreteController)
    println("Controller: $(ctrl.name)")
    println("  Status: $(ctrl.state.is_active ? "ACTIVE" : "INACTIVE")")
    println("  Setpoint: $(round(ctrl.sp, digits=3))")
    println("  Process Variable: $(round(ctrl.pv, digits=3))")
    println("  Error: $(round(ctrl.error, digits=3))")
    println("  Manipulated Variable: $(round(ctrl.mv, digits=3))")
    println("  Sample Time: $(ctrl.Ts) s")
    println("  Updates: $(ctrl.monitor.update_count)")

    # Logging status
    println("  Logging:")
    if ctrl.enable_logging
        println("    Enabled: $(length(ctrl.logger.timestamps)) data points")
    else
        println("    Disabled")
    end
end
