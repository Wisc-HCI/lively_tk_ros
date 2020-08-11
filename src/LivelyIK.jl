module LivelyIK
using Lib

include("relaxed_ik/relaxed_ik.jl")

export RelaxedIK, get_standard, solve, test, my_test

my_test() = test()

end # module
