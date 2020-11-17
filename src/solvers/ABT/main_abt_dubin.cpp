#include "oppt/problemEnvironment/ProblemEnvironment.hpp"
#include "algorithmABT.hpp"
#include "ABTOptions.hpp"
#include "oppt/robotHeaders/Dubin/DubinRobot.hpp"

int main(int argc, char const* argv[])
{   
    oppt::ProblemEnvironment problemEnvironment;
    int ret = problemEnvironment.setup<oppt::DubinRobot, algorithm::ABT, oppt::ABTExtendedOptions>(argc, argv);
    if (ret != 0)
        return ret;
    return problemEnvironment.runEnvironment(argc, argv);
}
