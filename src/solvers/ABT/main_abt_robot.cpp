#include "oppt/problemEnvironment/ProblemEnvironment.hpp"
#include "solverABT.hpp"
#include "ABTOptions.hpp"

int main(int argc, char const* argv[])
{    
    oppt::ProblemEnvironment problemEnvironment;
    int ret = problemEnvironment.setup<solvers::ABT, oppt::ABTExtendedOptions>(argc, argv);
    if (ret != 0)
        return ret;
    return problemEnvironment.runEnvironment(argc, argv);
}


