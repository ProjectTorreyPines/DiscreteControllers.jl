"""
DiscreteControllers.jl

A lightweight Julia package for discrete controller timing management with
autonomous timing control, data logging, and basic diagnostics.

Built on top of DiscretePIDs.jl to provide:
- Autonomous sampling time management
- Automatic data logging for analysis
- Basic performance diagnostics
- Simple visualization utilities
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
export DiscreteController, ExternalInterface

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
