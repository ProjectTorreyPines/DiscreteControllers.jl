using Test
using DiscreteControllers

@testset "DiscreteController Tests" begin
    # Test data
    measurement_data = Ref(0.0)
    control_output_data = Ref(0.0)

    # Mock functions
    measure_func = () -> measurement_data[]
    actuate_func = (signal) -> control_output_data[] = signal

    @testset "Constructor" begin
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

        ctrl = DiscreteController(
            pid
            ;
            sp = 100.0,
            name = "test_controller",
            external = ExternalInterface(
                measure_process_variable = measure_func,
                apply_manipulated_variable = actuate_func
            ),
            Ts = 0.01,
            initial_time = 0.0
        )

        @test ctrl.sp == 100.0
        @test ctrl.name == "test_controller"
        @test ctrl.Ts == 0.01
        @test ctrl.is_active == true
        @test ctrl.monitor.update_count == 0
    end

    @testset "Timing Management" begin
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

        ctrl = DiscreteController(
            pid;
            sp = 100.0,
            name = "timing_test",
            external = ExternalInterface(
                measure_process_variable = measure_func,
                apply_manipulated_variable = actuate_func
            ),
            Ts = 0.01,
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
    end

    @testset "Control Logic" begin
        pid = DiscretePID(K=2.0, Ti=1.0, Td=0.0, Ts=0.01)  # Simple P controller

        ctrl = DiscreteController(
            pid;
            sp = 100.0,
            name = "control_test",
            external = ExternalInterface(
                measure_process_variable = measure_func,
                apply_manipulated_variable = actuate_func
            ),
            Ts = 0.01,
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
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

        ctrl = DiscreteController(
            pid;
            sp = 100.0,
            name = "activation_test",
            external = ExternalInterface(
                measure_process_variable = measure_func,
                apply_manipulated_variable = actuate_func
            ),
            Ts = 0.01,
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
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

        ctrl = DiscreteController(
            pid;
            sp = 100.0,
            name = "setpoint_test",
            external = ExternalInterface(
                measure_process_variable = measure_func,
                apply_manipulated_variable = actuate_func
            ),
            Ts = 0.01,
            initial_time = 0.0
        )

        @test ctrl.sp == 100.0

        set_setpoint!(ctrl, 150.0)
        @test ctrl.sp == 150.0
    end

    @testset "Reset Functionality" begin
        pid = DiscretePID(K=1.0, Ti=2.0, Td=0.1, Ts=0.01)

        ctrl = DiscreteController(
            pid;
            sp = 100.0,
            name = "reset_test",
            external = ExternalInterface(
                measure_process_variable = measure_func,
                apply_manipulated_variable = actuate_func
            ),
            Ts = 0.01,
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
end
