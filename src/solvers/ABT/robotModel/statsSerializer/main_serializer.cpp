#include "StatsSerializer.hpp"
#include <tclap/CmdLine.h>
#include <unistd.h>

using namespace oppt;

/** The main method for the "solve" executable for RockSample. */
int main(int argc, char const* argv[])
{
    try {
        cout << argv[1] << endl;
        TCLAP::CmdLine cmd("Command description message", ' ', "0.9");
        TCLAP::ValueArg<std::string> pathArg("p",
                                             "path",
                                             "Path where the log files are stored in",
                                             true,
                                             "homer",
                                             "string");
        cmd.add(pathArg);
        cmd.parse(argc, argv);
        std::string logFilePath(pathArg.getValue());
        oppt::StatsSerializer serializer;        

        // Try path relative to the current directory        
        if (!boost::filesystem::is_directory(logFilePath)) {
            ERROR("Given path doesn't exist: " + logFilePath);            
        }
        
        serializer.setPath(logFilePath);
	serializer.readLogfiles();
	LOGGING("Done");
	return 0;
        
        

    } catch (TCLAP::ArgException& e) { // catch any exceptions
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }
}
