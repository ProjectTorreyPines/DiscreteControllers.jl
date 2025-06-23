using Test
using DiscreteControllers

if !isempty(ARGS) && !(length(ARGS) == 1 && ARGS[1] == "")
    println(ARGS)
    for testfile in ARGS
        @info "Running test file: $testfile"
        include(testfile)
    end
else
    include("test_discrete_controller.jl")
    include("test_realistic_examples.jl")
end