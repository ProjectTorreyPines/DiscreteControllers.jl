"""
Plots.jl extension for DiscreteControllers.jl

This extension provides plot recipes for visualizing controller data when Plots.jl is loaded.
"""
module DiscreteControllersPlotsExt

using DiscreteControllers
using Plots
using Printf
using HelpPlots

"""
    get_time_scale_info(timescale::Symbol)

Get the scaling factor and unit label for the given timescale.
Supported timescales: :s, :ms, :us (or :μs)
"""
function get_time_scale_info(timescale::Symbol)
    id = HelpPlots.recipe_dispatch(timescale)
    HelpPlots.assert_type_and_record_argument(id, Symbol, "scaling of timestamps (:s, :ms, :us, or :μs)"; timescale)
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
        ctrl, Val(:track) # plot_track
    end

    # Panel 2: Error analysis
    @series begin
        subplot := 2
        ctrl, Val(:error) # plot_error
    end

    # Panel 3: Control output
    @series begin
        subplot := 3
        ctrl, Val(:control) # plot_control
    end

    # Panel 4: Phase portrait
    @series begin
        subplot := 4
        ctrl, Val(:phase) # plot_phase
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
    @recipe function plot_setpoint(ctrl::DiscreteController, ::Val{:setpoint})

Recipe for setpoint visualization only.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot_setpoint(ctrl::DiscreteController, ::Val{:setpoint})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)
    scaled_timestamps = ctrl.logger.timestamps .* scale_factor

    pv_name = isempty(ctrl.pv_name) ? "Process Variable" : ctrl.pv_name
    pv_name_with_unit = isempty(ctrl.pv_unit) ? pv_name : "$(pv_name) [$(ctrl.pv_unit)]"

    title --> pv_name_with_unit
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
    @recipe function plot_pv(ctrl::DiscreteController, ::Val{:pv})

Recipe for process variable visualization only.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot_pv(ctrl::DiscreteController, ::Val{:pv})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)
    scaled_timestamps = ctrl.logger.timestamps .* scale_factor

    pv_name = isempty(ctrl.pv_name) ? "Process Variable" : ctrl.pv_name
    pv_name_with_unit = isempty(ctrl.pv_unit) ? pv_name : "$(pv_name) [$(ctrl.pv_unit)]"

    title --> pv_name_with_unit

    xlabel --> "Time [$(time_unit)]"
    ylabel --> "Value"

    @series begin
        label --> pv_name
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
    @recipe function plot_track(ctrl::DiscreteController, ::Val{:track})

Recipe for setpoint tracking visualization (combines setpoint and PV).
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot_track(ctrl::DiscreteController, ::Val{:track})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    pv_name = isempty(ctrl.pv_name) ? "Process Variable" : ctrl.pv_name
    pv_name_with_unit = isempty(ctrl.pv_unit) ? pv_name : "$(pv_name) [$(ctrl.pv_unit)]"

    title := pv_name_with_unit
    ylabel --> "Value"

    # Plot setpoint - calls plot_setpoint recipe
    @series begin
        ctrl, Val(:setpoint) # plot_setpoint
    end

    # Plot process variable - calls plot_pv recipe
    @series begin
        ctrl, Val(:pv) # plot_pv
    end
end

"""
    @recipe function plot_error(ctrl::DiscreteController, ::Val{:error})

Recipe for error analysis visualization.
Supports timescale keyword: :s, :ms, :us (or :μs)
"""
@recipe function plot_error(ctrl::DiscreteController, ::Val{:error})
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
@recipe function plot_control(ctrl::DiscreteController, ::Val{:control})
    if isempty(ctrl.logger.timestamps)
        @warn "Controller logger is empty, nothing to plot"
        return
    end

    # Get timescale from plotattributes or use default
    timescale = get(plotattributes, :timescale, :s)
    scale_factor, time_unit = get_time_scale_info(timescale)

    mv_name = isempty(ctrl.mv_name) ? "Control Output" : ctrl.mv_name
    mv_name_with_unit = isempty(ctrl.mv_unit) ? mv_name : "$(mv_name) [$(ctrl.mv_unit)]"

    title --> mv_name_with_unit
    xlabel --> "Time [$(time_unit)]"
    ylabel --> "Value"

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
        label := mv_name
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
    @recipe function plot_phase(ctrl::DiscreteController, ::Val{:phase})

Recipe for phase portrait visualization (Error vs Error Rate).
Note: Error rate units are affected by timescale choice.
"""
@recipe function plot_phase(ctrl::DiscreteController, ::Val{:phase})
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
