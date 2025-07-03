"""
Logging functionality for DiscreteControllers.jl

This module provides logging capabilities for discrete controllers:
- Control update logging (only during control updates every Ts)
- Data export to CSV
- Log management functions
"""

using Printf

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

function Base.show(io::IO, ::MIME"text/plain", ctrl::DiscreteController)
    show_controller_status(io, ctrl)
end

function Base.show(io::IO, ctrl::DiscreteController)
    show_controller_status(io, ctrl)
end

"""
    show_controller_status(io::IO, ctrl::DiscreteController)

Display a brief status summary of the controller to the specified IO stream.
"""
function show_controller_status(io::IO, ctrl::DiscreteController)
    # Header with controller name
    printstyled(io, "Controller: "; bold=true)
    printstyled(io, ctrl.name; color=:cyan, bold=true)
    println(io)

    # Status
    print(io, "├─ Status: ")
    if ctrl.is_active
        printstyled(io, "ACTIVE"; color=:green, bold=true)
    else
        printstyled(io, "INACTIVE"; color=:red, bold=true)
    end
    println(io)

    # Sample time
    print(io, "├─ Sample Time: ")
    print(io, "$(ctrl.Ts) s")
    println(io)

    # Setpoint
    print(io, "├─ Setpoint: ")
    printstyled(io, @sprintf("%.3f", ctrl.sp); color=:blue, bold=true)
    println(io)

    # Process variable
    print(io, "├─ Process Variable: ")
    printstyled(io, @sprintf("%.3f", ctrl.pv); color=:blue, bold=true)
    println(io)

    # Error with color coding based on magnitude
    print(io, "├─ Error: ")
    error_str = @sprintf("%.3f", ctrl.error)

    # Calculate relative error (sp-pv)/sp, handle sp=0 case
    if abs(ctrl.sp) > 1e-10  # Avoid division by zero
        rel_error = (ctrl.sp - ctrl.pv) / ctrl.sp * 100  # Convert to percentage
        rel_error_str = @sprintf("%.1f%%", rel_error)

        print(io, error_str)

        if abs(rel_error) > 20.0  # > 20% error
            error_color = :red
        elseif abs(rel_error) > 5.0  # > 5% error
            error_color = :yellow
        else  # <= 5% error
            error_color = :green
        end
        print(io, " (")
        printstyled(io, rel_error_str; color=error_color, bold=true)
        print(io, ")")
    else
        # When setpoint is zero, just show absolute error
        if abs(ctrl.error) > 10.0
            printstyled(io, error_str; color=:red, bold=true)
        elseif abs(ctrl.error) > 1.0
            printstyled(io, error_str; color=:yellow, bold=true)
        else
            printstyled(io, error_str; color=:green, bold=true)
        end
    end
    println(io)

    # Control output
    print(io, "├─ Control Output: ")
    if isnan(ctrl.mv)
        printstyled(io, "N/A")
    else
        printstyled(io, @sprintf("%.3f", ctrl.mv))
    end
    println(io)

    # Updates count and missed deadlines
    print(io, "├─ Updates: $(ctrl.monitor.update_count)")
    # Show missed deadlines prominently if any
    if ctrl.monitor.missed_deadlines > 0
        print(io, " | ")
        printstyled(io, "⚠️  MISSED DEADLINES: $(ctrl.monitor.missed_deadlines)"; color=:red, bold=true)
    end
    println(io)

    # Logging status
    print(io, "╰─ Logging: ")
    if ctrl.enable_logging
        printstyled(io, "Enabled"; color=:green)
        print(io, " ($(length(ctrl.logger.timestamps)) data points)")
    else
        printstyled(io, "Disabled"; color=:red)
    end
    println(io)

    # Add empty line for better separation when called multiple times
    println(io)
end

"""
    show_controller_status(ctrl::DiscreteController)

Display a brief status summary of the controller to stdout.
"""
function show_controller_status(ctrl::DiscreteController)
    show_controller_status(stdout, ctrl)
end
