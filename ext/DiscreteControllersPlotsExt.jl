"""
Plots.jl extension for DiscreteControllers.jl

This extension provides plot recipes for visualizing controller data when Plots.jl is loaded.
"""
module DiscreteControllersPlotsExt

using DiscreteControllers
using Plots
using Printf

"""
    get_time_scale_info(timescale::Symbol)

Get the scaling factor and unit label for the given timescale.
Supported timescales: :s, :ms, :us (or :μs)
"""
function get_time_scale_info(timescale::Symbol)
    if timescale == :s
        return 1.0, "s"
    elseif timescale == :ms
        return 1e3, "ms"
    elseif timescale == :us || timescale == :μs
        return 1e6, "μs"
    else
        @warn "Unknown timescale $(timescale), using seconds"
        return 1.0, "s"
    end
end

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
    @recipe function plot(ctrl::DiscreteController, ::Val{:setpoint})

Recipe for setpoint visualization only.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:setpoint})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)
    scaled_timestamps = ctrl.logger.timestamps .* scale_factor

    title --> "Setpoint"
    xlabel --> "Time [$(time_unit)]"
    ylabel --> "Value"

    @series begin
        label --> "Setpoint"
        color --> RGB(0.2, 0.2, 0.2)
        linewidth --> 3
        linestyle --> :dash
        scaled_timestamps, ctrl.logger.setpoints
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:pv})

Recipe for process variable visualization only.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:pv})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)
    scaled_timestamps = ctrl.logger.timestamps .* scale_factor

    title --> "Process Variable"
    xlabel --> "Time [$(time_unit)]"
    ylabel --> "Value"

    @series begin
        label --> "Process Variable"
        color --> :red
        linewidth --> 2
        alpha --> 0.8
        if length(scaled_timestamps) < 30
            marker --> :circle
        end
        scaled_timestamps, ctrl.logger.process_variables
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:track})

Recipe for setpoint tracking visualization (combines setpoint and PV).
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:track})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    title := "Setpoint Tracking Performance"
    ylabel --> "Value"

    # Plot setpoint
    @series begin
        ctrl, Val(:setpoint)
    end

    # Plot process variable
    @series begin
        ctrl, Val(:pv)
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:error})

Recipe for error analysis visualization.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:error})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)
    scaled_timestamps = ctrl.logger.timestamps .* scale_factor

    title --> "Control Error Over Time"
    xlabel --> "Time [$(time_unit)]"
    ylabel --> "Error"

    @series begin
        label := "Error"
        color := :orange
        linewidth := 2
        scaled_timestamps, ctrl.logger.errors
    end

    @series begin
        label := ""
        color := :black
        linestyle := :dash
        alpha := 0.5
        scaled_timestamps, zeros(length(scaled_timestamps))
    end
end

"""
    @recipe function plot(ctrl::DiscreteController, ::Val{:control})

Recipe for manipulated variable visualization with discrete step-like behavior.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:control})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)

    title --> "Manipulated Variable (Discrete)"
    xlabel --> "Time [$(time_unit)]"
    ylabel --> "MV"

    # Calculate step-like data for discrete control output
    if length(ctrl.logger.timestamps) > 1
        step_times = Float64[]
        step_values = Float64[]
        for i in 1:length(ctrl.logger.timestamps)-1
            push!(step_times, ctrl.logger.timestamps[i] * scale_factor)
            push!(step_values, ctrl.logger.manipulated_variables[i])
            push!(step_times, ctrl.logger.timestamps[i+1] * scale_factor)
            push!(step_values, ctrl.logger.manipulated_variables[i])
        end
        push!(step_times, ctrl.logger.timestamps[end] * scale_factor)
        push!(step_values, ctrl.logger.manipulated_variables[end])
    else
        step_times = ctrl.logger.timestamps .* scale_factor
        step_values = ctrl.logger.manipulated_variables
    end

    @series begin
        label := "Control Output"
        color := :blue
        linewidth := 2
        step_times, step_values
    end

    # Calculate MV range for intelligent y-axis scaling
    mv_min, mv_max = extrema(ctrl.logger.manipulated_variables)
    mv_range = mv_max - mv_min
    mv_margin = max(mv_range * 0.15, 0.1)  # ±15% margin or minimum 0.1

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
Note: Error rate units are affected by timescale choice.
"""
@recipe function plot(ctrl::DiscreteController, ::Val{:phase})
    if isempty(ctrl.logger.timestamps) || length(ctrl.logger.timestamps) < 2
        @warn "Controller logger needs at least 2 points for phase plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)

    title --> "Phase Portrait"
    xlabel --> "Error"
    ylabel --> "Error Rate [1/$(time_unit)]"

    # Calculate error rate (accounting for timescale)
    scaled_timestamps = ctrl.logger.timestamps .* scale_factor
    error_rate = [0; diff(ctrl.logger.errors) ./ diff(scaled_timestamps)]

    # Calculate plot ranges with margins for better visualization
    error_min, error_max = extrema(ctrl.logger.errors)
    error_rate_min, error_rate_max = extrema(error_rate)

    error_range = error_max - error_min
    error_rate_range = error_rate_max - error_rate_min

    # Add 10% margin or minimum 0.1, whichever is larger
    error_margin = max(error_range * 0.1, 0.1)
    error_rate_margin = max(error_rate_range * 0.1, 0.1)

    xlims := (error_min - error_margin, error_max + error_margin)
    ylims := (error_rate_min - error_rate_margin, error_rate_max + error_rate_margin)

    Ntt = length(scaled_timestamps)

    @series begin
        label := "Trajectory"
        color := :red
        linewidth := 2
        marker := :circle
        markersize --> _compute_marker_size(Ntt)
        alpha --> _compute_alpha(Ntt)
        ctrl.logger.errors, error_rate
    end

    # Add origin crosshairs for better visibility
    @series begin
        label := ""
        color := RGB(0.4, 0.4, 0.4)
        linestyle := :dash
        linewidth := 1
        alpha := 0.8
        seriestype := :hline
        y := [0]
    end

    @series begin
        label := ""
        color := RGB(0.4, 0.4, 0.4)
        linestyle := :dash
        linewidth := 1
        alpha := 0.8
        seriestype := :vline
        x := [0]
    end

    @series begin
        label := "Origin"
        color := :black
        marker := :cross
        markersize := 8
        seriestype := :scatter
        [0], [0]
    end
end

function _compute_marker_size(nsample::Integer; max_size::Float64=10.0, min_size::Float64=4.0, max_Nsample::Integer=50)
    s = max_size - (max_size - min_size) / max_Nsample * nsample
    return clamp(s, min_size, max_size)
end

function _compute_alpha(nsample::Integer; max_α::Float64=0.9, min_α::Float64=0.4, max_Nsample::Integer=50)
    a = max_α - (max_α - min_α) / max_Nsample * nsample
    return clamp(a, min_α, max_α)
end

end # module
