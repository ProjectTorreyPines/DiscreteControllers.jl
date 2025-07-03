"""
DiscreteControllers.jl

A lightweight Julia package for discrete-time controllers with autonomous timing
management, built-in logging, and optional plotting capabilities.

Built on DiscretePIDs.jl to provide:
- Autonomous timing management
- Built-in data logging
- Flexible construction
- Optional visualization
"""
module DiscreteControllers

using DiscretePIDs
using Printf

# Core components
include("types.jl")
include("controller.jl")  # Core controller functionality
include("logging.jl")     # Logging and data export functionality

# Exports
export DiscretePID # Re-export from DiscretePIDs
export DiscreteController, SystemInterface

# Core controller functions
export update_controller!, activate!, deactivate!, reset!
export set_setpoint!, set_pv!, get_setpoint, get_pv, get_mv, get_error
export get_sampling_time, is_active, get_update_count, get_missed_deadlines
export time_until_next_update, set_timing_tolerance!

# Logging functions
export enable_logging!, disable_logging!
export clear_log!, export_log
export show_controller_status

end # module DiscreteControllers
