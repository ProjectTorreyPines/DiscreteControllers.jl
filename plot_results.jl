#!/usr/bin/env julia

"""
Simple plotting script for DiscreteController logging test results using Plots.jl.

Usage:
1. Run the Julia test script: julia test_logging.jl
2. Run this script: julia plot_results.jl

Requires: Plots.jl, CSV.jl, DataFrames.jl
Install with: julia -e 'using Pkg; Pkg.add(["Plots", "CSV", "DataFrames"])'
"""

using Pkg
Pkg.activate(".")

using Plots
using CSV
using DataFrames
using Statistics

function plot_controller_results(csv_file::String = "temperature_control_test.csv")
    """Plot the controller test results from CSV file."""

    if !isfile(csv_file)
        println("Error: $csv_file not found!")
        println("Please run 'julia test_logging.jl' first to generate the data.")
        return
    end

    # Read the data
    println("Reading data from $csv_file...")
    data = CSV.read(csv_file, DataFrame)

    # Create the plots
    println("Creating plots...")

    # Set plot theme
    theme(:default)

    # Plot 1: Temperature tracking
    p1 = plot(data.time, data.target,
             label="Target",
             linestyle=:dash,
             linewidth=2,
             color=:red,
             title="Temperature Tracking")
    plot!(p1, data.time, data.measurement,
          label="Measured",
          linewidth=1.5,
          color=:blue)
    ylabel!(p1, "Temperature [°C]")

    # Plot 2: Control signal
    p2 = plot(data.time, data.output,
             label="Control Signal",
             linewidth=1.5,
             color=:green,
             title="Controller Output")
    ylabel!(p2, "Control Output")

    # Plot 3: Error
    p3 = plot(data.time, data.error,
             label="Control Error",
             linewidth=1.5,
             color=:red,
             title="Control Error (Target - Measurement)")
    hline!(p3, [0],
           color=:black,
           linestyle=:solid,
           alpha=0.3,
           label="")
    ylabel!(p3, "Error [°C]")
    xlabel!(p3, "Time [s]")

    # Combine all plots
    final_plot = plot(p1, p2, p3,
                     layout=(3, 1),
                     size=(800, 600),
                     plot_title="Temperature Controller Performance Test",
                     plot_titlevspan=0.05)

    # Save plot
    plot_file = replace(csv_file, ".csv" => "_plot.png")
    savefig(final_plot, plot_file)
    println("Plot saved as: $plot_file")

    # Display plot
    display(final_plot)

    # Show statistics
    println("\nPerformance Statistics:")
    println("  Simulation time: $(round(data.time[end], digits=1)) seconds")
    println("  Data points: $(nrow(data))")
    println("  Final temperature: $(round(data.measurement[end], digits=2))°C")
    println("  Final target: $(round(data.target[end], digits=2))°C")
    println("  Final error: $(round(data.error[end], digits=2))°C")
    println("  Mean absolute error: $(round(mean(abs.(data.error)), digits=2))°C")
    println("  RMS error: $(round(sqrt(mean(data.error.^2)), digits=2))°C")
    println("  Max control output: $(round(maximum(data.output), digits=2))")
    println("  Min control output: $(round(minimum(data.output), digits=2))")

    return final_plot
end

function plot_controller_results_interactive(csv_file::String = "temperature_control_test.csv")
    """Create an interactive plot using PlotlyJS backend."""

    if !isfile(csv_file)
        println("Error: $csv_file not found!")
        return
    end

    # Switch to PlotlyJS for interactivity
    try
        using PlotlyJS
        plotlyjs()
        println("Using PlotlyJS backend for interactive plots")
    catch
        println("PlotlyJS not available, using default backend")
    end

    # Read data
    data = CSV.read(csv_file, DataFrame)

    # Create subplots with shared x-axis
    p1 = plot(data.time, [data.target data.measurement],
             label=["Target" "Measured"],
             linestyle=[:dash :solid],
             linewidth=[2 1.5],
             color=[:red :blue],
             title="Temperature Tracking",
             ylabel="Temperature [°C]")

    p2 = plot(data.time, data.output,
             label="Control Signal",
             linewidth=1.5,
             color=:green,
             title="Controller Output",
             ylabel="Control Output")

    p3 = plot(data.time, data.error,
             label="Control Error",
             linewidth=1.5,
             color=:red,
             title="Control Error",
             ylabel="Error [°C]",
             xlabel="Time [s]")
    hline!(p3, [0], color=:black, alpha=0.3, label="")

    # Combine plots
    final_plot = plot(p1, p2, p3,
                     layout=(3, 1),
                     size=(900, 700),
                     plot_title="Interactive Controller Performance Analysis")

    display(final_plot)
    return final_plot
end

function main()
    println("=" ^ 60)
    println("DiscreteController Results Visualization")
    println("=" ^ 60)

    csv_file = "temperature_control_test.csv"

    # Check if custom file is provided
    if length(ARGS) > 0
        csv_file = ARGS[1]
    end

    # Create standard plot
    plot_result = plot_controller_results(csv_file)

    # Ask if user wants interactive plot
    println("\nWould you like an interactive plot? (y/n)")
    response = readline()
    if lowercase(strip(response)) in ["y", "yes"]
        plot_controller_results_interactive(csv_file)
    end

    println("\n" * "=" ^ 60)
    println("Visualization completed!")
    println("Check '$(pwd())/$(replace(csv_file, ".csv" => "_plot.png"))' for saved plot.")
    println("=" ^ 60)

    return plot_result
end

# Run the visualization if this file is executed directly
if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
