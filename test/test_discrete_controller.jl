using Test
using DiscreteControllers

@testset "DiscreteController Tests" begin
    # Test data
    measurement_data = Ref(0.0)
    control_output_data = Ref(0.0)

    # Mock functions
    read_setpoint_func = (time) -> (100.0 + time)
    measure_func = () -> measurement_data[]
    actuate_func = (signal) -> control_output_data[] = signal

    @testset "Constructor with pid arg" begin
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

        ctrl = DiscreteController(
            pid
            ;
            sp = 100.0,
            name = "test_controller",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.0
        )

        @test ctrl.sp == 100.0
        @test ctrl.name == "test_controller"
        @test ctrl.Ts == 0.01
        @test ctrl.is_active == true
        @test ctrl.monitor.update_count == 0
    end

    @testset "Constructor with Ts arg" begin
        Ts = 0.01

        ctrl = DiscreteController(
            Ts;
            sp = 100.0,
            name = "ctrl with Ts",
            Ts = 0.01,
            initial_time = 10.0
        )

        @test ctrl.sp == 100.0
        @test ctrl.error == 100.0
        @test ctrl.name == "ctrl with Ts"
        @test ctrl.Ts == 0.01
        @test ctrl.is_active == true
        @test ctrl.monitor.update_count == 0
    end

    @testset "Constructor with types" begin
        Ts = Float32(0.01)

        ctrl = DiscreteController(
            Ts;
            K=2.5,
            sp = 100.0
        )

        @test ctrl.pid.K === Float32(2.5)
        @test ctrl.pid.umax === typemax(Float32)
        @test ctrl.sp === Float32(100.0)

        # When Ts is an Int, it should convert to Float64
        Ts = 10
        ctrl = DiscreteController(10)
        @test ctrl.Ts === Float64(10.0)
        @test ctrl.pid.Ts === Float64(10.0)
    end

    @testset "Constructor validation" begin
        # zero or negative Ts should throw an error
        @test_throws AssertionError ctrl = DiscreteController(0.0)
        @test_throws AssertionError ctrl = DiscreteController(-0.1)
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=-0.01)
        @test_throws AssertionError DiscreteController(pid)

        # pid and Ts cannot be used together
        Ts = 0.1
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.1)
        @test_throws AssertionError DiscreteController(pid; Ts)
        @test_throws AssertionError DiscreteController(Ts; pid)
    end

    @testset "Constructor with various keywords" begin
        Ts = 0.1
        ctrl = DiscreteController(
            Ts;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = 100.0,
            name = "keyword_test",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.5
        )

        @test ctrl.sp == 100.0
        @test ctrl.name == "keyword_test"
        @test ctrl.Ts == Ts
        @test ctrl.is_active == true
        @test ctrl.pid.K == 1.0
        @test ctrl.pid.umax == Inf
        @test ctrl.pid.Ti == 2.0

        @test ctrl.timing.current_time == 0.5
        @test ctrl.timing.last_update_time == -Inf # not yet updated
        @test ctrl.timing.next_scheduled_time == 0.6
    end

    @testset "Timing Management" begin
        Ts = 0.01
        ctrl = DiscreteController(
            Ts;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = 100.0,
            name = "timing_test",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.0
        )

        # Should not update before sampling time
        measurement_data[] = 50.0
        result = update_controller!(ctrl, 0.005)  # 5ms, less than Ts=10ms
        @test result == false
        @test ctrl.monitor.update_count == 0

        # Should update at sampling time
        result = update_controller!(ctrl, 0.01)  # 10ms, equal to Ts
        @test result == true
        @test ctrl.monitor.update_count == 1
        @test ctrl.pv == 50.0

        # Should update again after another sampling period
        measurement_data[] = 75.0
        result = update_controller!(ctrl, 0.02)
        @test result == true
        @test ctrl.monitor.update_count == 2
        @test ctrl.pv == 75.0

        # Time regression should not update
        result = @test_logs (:warn, r"Time regression detected in") match_mode=:any begin
            update_controller!(ctrl, 0.0)
        end
        @test result == false
        @test ctrl.monitor.update_count == 2  # Should not increment
    end

    @testset "Consistency with setpoint func" begin
        Ts = 0.01
        initial_time = 0.5
        ini_sp = read_setpoint_func(initial_time)

        system_interface = SystemInterface(
            read_setpoint = read_setpoint_func,
            read_process_var = measure_func,
            apply_control_signal = actuate_func
        )

        ctrl = DiscreteController(
            Ts;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = ini_sp,
            name = "setpoint_func_test",
            system_interface,
            initial_time
        )

        # Initial setpoint should be 100.0
        @test ctrl.sp == ini_sp

        # Update controller to trigger setpoint function
        for k = 1:3
            time = initial_time + k*Ts
            result = update_controller!(ctrl, time)
            @test result == true
            @test ctrl.monitor.update_count == k

            # Check if setpoint was correctly applied
            @test ctrl.sp == read_setpoint_func(time)
        end


        # Test error cases where initial state (sp, pv) is not consistent with external_interface's functions
        @test_throws AssertionError DiscreteController( Ts; sp = -100.0, system_interface, initial_time)
        @test_throws AssertionError DiscreteController( Ts; pv = -100.0, system_interface, initial_time)
    end


    @testset "Control Logic" begin
        Ts = 0.01
        ctrl = DiscreteController(
            Ts;
            K = 2.0, Ti = 1.0, Td = 0.0,
            sp = 100.0,
            name = "control_test",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.0
        )

        # Test proportional response
        measurement_data[] = 80.0  # Error = 20.0
        update_controller!(ctrl, 0.01)

        # With K=2.0, expect control output ≈ 2.0 * 20.0 = 40.0
        @test abs(control_output_data[] - 40.0) < 1.0  # Allow some tolerance for PID internals
        @test ctrl.mv == control_output_data[]
    end

    @testset "Activation/Deactivation" begin
        Ts = 0.01
        ctrl = DiscreteController(
            Ts;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = 100.0,
            name = "activation_test",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.0
        )

        # Deactivate controller
        deactivate!(ctrl)
        @test ctrl.is_active == false

        # Should not update when inactive
        measurement_data[] = 50.0
        result = update_controller!(ctrl, 0.01)
        @test result == false
        @test ctrl.monitor.update_count == 0

        # Reactivate
        activate!(ctrl)
        @test ctrl.is_active == true

        # Should update when active
        result = update_controller!(ctrl, 0.02)
        @test result == true
        @test ctrl.monitor.update_count == 1
    end

    @testset "Setpoint Changes" begin
        Ts = 0.01
        ctrl = DiscreteController(
            Ts;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = 100.0,
            name = "setpoint_test",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.0
        )

        @test ctrl.sp == 100.0

        set_setpoint!(ctrl, 150.0)
        @test ctrl.sp == 150.0
    end

    @testset "Reset Functionality" begin
        Ts = 0.01
        ctrl = DiscreteController(
            Ts;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = 100.0,
            name = "reset_test",
            system_interface = SystemInterface(
                read_process_var = measure_func,
                apply_control_signal = actuate_func
            ),
            initial_time = 0.0
        )

        # Run some updates
        measurement_data[] = 50.0
        update_controller!(ctrl, 0.01)
        update_controller!(ctrl, 0.02)

        @test ctrl.monitor.update_count == 2
        @test ctrl.timing.current_time == 0.02

        # Reset
        reset!(ctrl, 1.0)

        @test ctrl.monitor.update_count == 0
        @test ctrl.timing.current_time == 1.0
        @test ctrl.timing.last_update_time == 1.0
        @test ctrl.timing.next_scheduled_time == 1.01
    end

    @testset "Setter and Getter Functions" begin
        # Create a basic controller for testing
        ctrl = DiscreteController(
            0.01;
            K = 1.0, Ti = 2.0, Td = 0.1,
            sp = 100.0,
            name = "setter_getter_test",
            initial_time = 0.0
        )

        # Test initial values
        @test get_setpoint(ctrl) === 100.0
        @test get_pv(ctrl) === 0.0
        @test get_mv(ctrl) === NaN
        @test get_error(ctrl) === 100.0  # sp - pv = 100 - 0
        @test get_update_count(ctrl) === 0
        @test get_missed_deadlines(ctrl) === 0
        @test get_sampling_time(ctrl) === 0.01
        @test is_active(ctrl) === true

        # Test setpoint setter and getter
        set_setpoint!(ctrl, 150.0)
        @test get_setpoint(ctrl) === 150.0
        @test ctrl.sp === 150.0
        @test get_error(ctrl) === 150.0  # Error should update automatically

        # Test PV setter and getter
        set_pv!(ctrl, 50.0)
        @test get_pv(ctrl) === 50.0
        @test ctrl.pv === 50.0
        @test get_error(ctrl) === 100.0  # 150 - 50

        # Test with different numeric types
        set_setpoint!(ctrl, 200)  # Int
        @test get_setpoint(ctrl) === 200.0

        set_pv!(ctrl, 75.5f0)  # Float32
        @test get_pv(ctrl) === 75.5
        @test get_error(ctrl) === 124.5  # 200 - 75.5

        # Test MV and update count after a control update
        measurement_data = Ref(75.5)
        ctrl.system_interface.read_process_var = () -> measurement_data[]

        # Run one control update
        result = update_controller!(ctrl, 0.01)
        @test result === true
        @test get_update_count(ctrl) === 1
        @test get_mv(ctrl) !== 0.0  # Should have some control output

        # Test timing functions
        @test time_until_next_update(ctrl) ≈ 0.01  # Should be approximately Ts

        # Test timing tolerance
        original_tolerance = ctrl.timing.tolerance
        set_timing_tolerance!(ctrl, 1e-9)
        @test ctrl.timing.tolerance === 1e-9
        set_timing_tolerance!(ctrl, original_tolerance)  # Reset

        # Test activation/deactivation
        @test is_active(ctrl) === true
        deactivate!(ctrl)
        @test is_active(ctrl) === false
        activate!(ctrl)
        @test is_active(ctrl) === true

        # Test missed deadlines (simulate error condition)
        # Temporarily break the measurement function to cause an error
        ctrl.system_interface.read_process_var = () -> error("Simulated sensor failure")

        # Use @test_logs to capture the expected error message and suppress it from output
        result = @test_logs (:error, r"Controller update failed") match_mode=:any begin
            update_controller!(ctrl, 0.02)
        end

        @test result === false
        @test get_missed_deadlines(ctrl) === 1

        # Fix the measurement function
        ctrl.system_interface.read_process_var = () -> measurement_data[]

        # Test reset functionality
        reset!(ctrl, 5.0)
        @test get_update_count(ctrl) === 0
        @test get_missed_deadlines(ctrl) === 0
        @test ctrl.timing.current_time === 5.0
        @test get_mv(ctrl) === 0.0
        @test get_pv(ctrl) === 0.0
        @test get_error(ctrl) === get_setpoint(ctrl)  # sp - 0
    end

    @testset "Coverage tests on Edge cases" begin
        ctrl = DiscreteController(
            0.1;  # 100ms sampling
        )

        @test_logs (:warn, r"Logger is empty, no data to export") match_mode=:any begin
            export_log(ctrl, "logger_output.csv")
        end

        ctrl.enable_logging = false
        show_controller_status(ctrl)
    end
end
