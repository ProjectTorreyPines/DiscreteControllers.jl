"""
Plots.jl extension for DiscreteControllers.jl

This extension provides plot recipes for visualizing controller data when Plots.jl is loaded.
"""
module DiscreteControllersPlotsExt

using DiscreteControllers
using Plots
using Printf

"""
    @recipe function plot(ctrl::DiscreteController)

Plot recipe for DiscreteController objects. Creates a multi-panel plot showing all analysis views.
"""
@recipe function plot(ctrl::DiscreteController)
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    layout := (2, 2)
    size := (1200, 900)
    plot_title --> "Controller Performance: $(ctrl.name)"

    margin --> 5Plots.mm

    # Panel 1: Setpoint tracking
    @series begin
        subplot := 1
        ctrl, Val(:track)
    end

    # Panel 2: Error analysis
    @series begin
        subplot := 2
        ctrl, Val(:error)
    end

    # Panel 3: Control output
    @series begin
        subplot := 3
        ctrl, Val(:control)
    end

    # Panel 4: Phase portrait
    @series begin
        subplot := 4
        ctrl, Val(:phase)
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, mode::Symbol)

Convenience wrapper that converts Symbol to Val{Symbol} for type dispatch.
"""
@recipe function plot(ctrl::DiscreteController, mode::Symbol)
    ctrl, Val(mode)
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:track})

Recipe for setpoint tracking visualization.
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:track})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    title := "Setpoint Tracking Performance"
    xlabel := "Time [s]"
    ylabel := "Value"

    @series begin
        label := "Setpoint"
        color := RGB(0.2, 0.2, 0.2)
        linewidth := 3
        linestyle := :dash
        ctrl.logger.timestamps, ctrl.logger.setpoints
    end

    @series begin
        label := "Process Variable"
        color := :red
        linewidth := 2
        alpha := 0.8
        if length(ctrl.logger.timestamps) < 30
            marker := :circle
        end
        ctrl.logger.timestamps, ctrl.logger.process_variables
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:error})

Recipe for error analysis visualization.
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:error})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    title --> "Control Error Over Time"
    xlabel --> "Time [s]"
    ylabel --> "Error"

    @series begin
        label := "Error"
        color := :orange
        linewidth := 2
        ctrl.logger.timestamps, ctrl.logger.errors
    end

    @series begin
        label := ""
        color := :black
        linestyle := :dash
        alpha := 0.5
        ctrl.logger.timestamps, zeros(length(ctrl.logger.timestamps))
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:control})

Recipe for manipulated variable visualization with discrete step-like behavior.
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:control})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    title --> "Manipulated Variable (Discrete)"
    xlabel --> "Time [s]"
    ylabel --> "MV"

    # Calculate step-like data for discrete control output
    if length(ctrl.logger.timestamps) > 1
        step_times = Float64[]
        step_values = Float64[]
        for i in 1:length(ctrl.logger.timestamps)-1
            push!(step_times, ctrl.logger.timestamps[i])
            push!(step_values, ctrl.logger.manipulated_variables[i])
            push!(step_times, ctrl.logger.timestamps[i+1])
            push!(step_values, ctrl.logger.manipulated_variables[i])
        end
        push!(step_times, ctrl.logger.timestamps[end])
        push!(step_values, ctrl.logger.manipulated_variables[end])
    else
        step_times = ctrl.logger.timestamps
        step_values = ctrl.logger.manipulated_variables
    end

    @series begin
        label := "Control Output"
        color := :green
        linewidth := 2
        step_times, step_values
    end

    # Calculate MV range for intelligent y-axis scaling
    mv_min, mv_max = extrema(ctrl.logger.manipulated_variables)
    mv_range = mv_max - mv_min
    mv_margin = max(mv_range * 0.1, 0.1)  # 10% margin or minimum 0.1

    # Set y-axis limits based on actual MV data
    ylims := (mv_min - mv_margin, mv_max + mv_margin)

    # Show limit lines with values in legend
    if isfinite(ctrl.pid.umax)
        @series begin
            label := @sprintf("Max MV (%.2g)", ctrl.pid.umax)
            color := RGB(0.2, 0.2, 0.2)
            linestyle := :dash
            linewidth := 2.0
            seriestype := :hline
            y := [ctrl.pid.umax]
        end
    end

    if isfinite(ctrl.pid.umin)
        @series begin
            label := @sprintf("Min MV (%.2g)", ctrl.pid.umin)
            color := RGB(0.2, 0.2, 0.2)
            linestyle := :dash
            linewidth := 2.0
            seriestype := :hline
            y := [ctrl.pid.umin]
        end
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:phase})

Recipe for phase portrait visualization (Error vs Error Rate).
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:phase})
    if isempty(ctrl.logger.timestamps) || length(ctrl.logger.timestamps) < 2
        @warn "Controller logger needs at least 2 points for phase plot"
        return
    end

    title --> "Phase Portrait (Error vs d(Error)/dt)"
    xlabel --> "Error"
    ylabel --> "Error Rate [1/s]"

    # Calculate error rate
    error_rate = [0; diff(ctrl.logger.errors) ./ diff(ctrl.logger.timestamps)]

    @series begin
        label := "Trajectory"
        color := :red
        linewidth := 2
        marker := :circle
        markersize := 3
        alpha := 0.5
        ctrl.logger.errors, error_rate
    end

    @series begin
        label := "Origin"
        color := :black
        marker := :cross
        markersize := 20
        seriestype := :scatter
        [0], [0]
    end
end

end # module
