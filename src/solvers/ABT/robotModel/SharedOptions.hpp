/** @file SharedOptions.hpp
 *
 * Defines the SharedOptions class, which comes with additional configuration options that apply
 * to many problems.
 */
#ifndef SHAREDOPTIONS_HPP_
#define SHAREDOPTIONS_HPP_

#include "oppt/options/tclap/CmdLine.h"
#include "oppt/options/option_parser.hpp"
#include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"
#include "oppt/opptCore/typedefs.hpp"

namespace shared
{
/** An expanded Options class, which comes with some additional settings that are shared by many
 * of the individual example problems.
 *
 * These extra configuration options allow for extra configuration at runtime instead of
 * compile-time.
 */
struct SharedOptions: public oppt::ProblemEnvironmentOptions {
    SharedOptions() = default;
    virtual ~SharedOptions() = default;

    /* ---------------------- Generic settings  ------------------ */
    /** The base config path - the path of the configuration files will be relative to this path. */
    std::string baseConfigPath = "";

    /** Makes a parser which can parse options from config files, or from the command line,
     * into a SharedOptions instance.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            ProblemEnvironmentOptions::makeParser(simulating);        
        return std::move(parser);
    }       
};
} /* namespace shared */

#endif /* SHAREDOPTIONS_HPP_ */

