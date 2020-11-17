/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */
//    Copyright (c) 2014 Dimitri Klimenko
//
//    Permission is hereby granted, free of charge, to any person obtaining a copy
//    of this software and associated documentation files (the "Software"), to deal
//    in the Software without restriction, including without limitation the rights
//    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//    copies of the Software, and to permit persons to whom the Software is
//    furnished to do so, subject to the following conditions:
//
//    The above copyright notice and this permission notice shall be included in
//    all copies or substantial portions of the Software.
//
//    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//    THE SOFTWARE.


/** @file option_parser.hpp
 *
 * A basic option parser that is able to parse configuration settings from both the command line,
 * and from INI-style configuration files.
 *
 * This parser uses the following libraries to do the underlying parsing:
 * inih [http://code.google.com/p/inih/] for parsing INI files
 * TCLAP [http://tclap.sourceforge.net/] for parsing command lines
 *
 * and combines them together in a convenient, object-oriented manner
 *
 * Options are represented via structs, which should inherit from the BaseOptions struct. This
 * inheritance approach makes it easy to build one set of options as an extension upon another
 * set of options.
 *
 * The core class is the OptionParser class, which provides an interface for registering
 * options and associated command-line aliases for those options.
 */
#ifndef OPTION_PARSER_HPP_
#define OPTION_PARSER_HPP_

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <regex>

#include <oppt/options/tclap/CmdLine.h>
#include <oppt/options/inih/ini.h>

#include "oppt/defs.hpp"

using std::cout;
using std::endl;

namespace options
{
/** A simple exception class, thrown when there is something wrong with the input. */
class OptionParsingException : public std::exception
{
public:
    /** Constructs a new OptionParsingException with the given message string. */
    OptionParsingException(std::string const& message);
    /** what() is the standard way for an exception to communicate an error message. */
    virtual char const* what() const throw() {
        return message_.c_str();
    }
private:
    std::string message_;
};

/** The base option struct. */
struct BaseOptions {
    BaseOptions() = default;
    virtual ~BaseOptions() = default;
};

/** The base argument class; serves as a wrapper for a TCLAP::Arg. */
class BaseArg
{
public:
    BaseArg() = default;
    virtual ~BaseArg() = default;
    _NO_COPY_OR_MOVE(BaseArg);

    /** Returns the TCLAP::Arg wrapped by this BaseArg. */
    virtual TCLAP::Arg& getArg() = 0;
};

/** The base class to represent a single option.
 *
 * An option is identified via two strings, allowing ease of use with INI files:
 * - the section, which corresponds to the INI section heading, e.g. [general]
 * - the name, which corresponds to the name from the name=value pair on a line in the INI file.
 *
 * The option is also responsible for storing its command line aliases.
 */
class BaseOption
{
public:
    /** Constructs an option from its section heading and name; initially this option will have
     * no command-line aliases.
     */
    BaseOption(std::string section, std::string name) :
        section_(section),
        name_(name),
        aliases_() {
    }
    virtual ~BaseOption() = default;

    /** Returns the section name. */
    std::string getSection() {
        return section_;
    }

    /** Returns the option name. */
    std::string getName() {
        return name_;
    }

    /** Adds a command line alias for this option. */
    void addAlias(std::unique_ptr<BaseArg> alias) {
        aliases_.push_back(std::move(alias));
    }

    /** Prints all of the command line aliases for this option. */
    void printAliases(std::ostream& os) {
        if (!aliases_.empty()) {
            os << "Aliases for " << getSection() << "." << getName() << ":" << std::endl;
            for (auto const & alias : aliases_) {
                std::string shortOpt = alias->getArg().getFlag();
                if (!shortOpt.empty()) {
                    os << "-" << shortOpt << std::endl;
                }
                std::string longOpt = alias->getArg().getName();
                if (!longOpt.empty()) {
                    os << "--" << longOpt << std::endl;
                }
            }
        }
    }

    /* ---------------- Virtual methods ------------------ */
    /** Resets the state of this option. */
    virtual void reset() = 0;
    /** Parses the value of this option from a string. */
    virtual void parse(BaseOptions* options, std::string s) = 0;
    /** Resets the value of this option to its default value. */
    virtual void setDefault(BaseOptions* options) = 0;
    /** Returns true iff this option has been set. */
    virtual bool isSet() = 0;
    /** Returns true if this option is currently defaulted (and hence can be overridden). */
    virtual bool isDefaulted() = 0;
private:
    /** The section name. */
    std::string section_;
    /** The option name. */
    std::string name_;
    /** A vector to store the command line aliases for this option. */
    std::vector<std::unique_ptr<BaseArg>> aliases_;
};

/** The core class for parsing options.
 *
 * This class internally stores a TCLAP::CmdLine object to handle parsing of command lines, but
 * also stores an unordered_map-based data structure to map INI section headings and option names
 * to the associated options.
 *
 * Note that each option is associated with a pointer-to-class-member object, which allows
 * the option parser to be constructed once and used on different configuration files to load
 * different options of the same kind.
 */
class OptionParser
{
public:
    /** Constructs a new option parser with the given message (this is the message used by
     * TCLAP::CmdLine as the "summary" description of the whole program).
     */
    OptionParser(std::string const& message);
    ~OptionParser() = default;
    _NO_COPY_OR_MOVE(OptionParser);

    /** Adds a single mandatory option via its section, name, and a pointer-to-class-member. */
    template<typename ValueType, typename OptionsType>
    void addOption(std::string section, std::string name, ValueType OptionsType::*optionPtr);

    /** Adds a single option, with a default value, via its section, name,
     *  and a pointer-to-class-member. */
    template<typename ValueType, typename OptionsType>
    void addOptionWithDefault(std::string section, std::string name,
                              ValueType OptionsType::*optionPtr, ValueType defaultValue);

    /** Adds a new command-line alias for the given option; this is made by constructing
     * the given ArgType with all of the remaining arguments. */
    template<typename ArgType, typename ValueType, typename OptionsType, typename ... Args>
    void addCmdAlias(std::string section, std::string name, ValueType OptionsType::*optionPtr,
                     Args && ... args);

    /** Adds a new value-base command line argument, using TCLAP::ValueArg. */
    template<typename ValueType, typename OptionsType> void addValueArg(std::string section,
            std::string name, ValueType OptionsType::*optionPtr, std::string shortOpt,
            std::string longOpt, std::string description, std::string typeDescription);

    /** Adds a new switch-base command line argument, using TCLAP::SwitchArg. */
    template<typename OptionsType> void addSwitchArg(std::string section, std::string name,
            bool OptionsType::*optionPtr, std::string shortOpt, std::string longOpt,
            std::string description, bool switchValue);

    /** Initializes the parser so that it is ready to read into the given option struct.
     * Any options with default values will initially be set to their defaults.
     */
    void setOptions(BaseOptions* options);

    /** Returns the option struct currently in use by this parser. */
    BaseOptions* getOptions();

    /** Parses the given command line; command line options always override previous values. */
    void parseCmdLine(int argc, char const* argv[]);

    /** Parses the given configuration file. Config file options will override defaulted options,
     * but will never override any options that have been already been set (e.g. via command line).
     */
    void parseCfgFile(std::string path);

    /** Checks to make sure that all options have been set - if any option has not been,
     * the program will exit. */
    void finalize();

private:
    /** A handler method, used by ini_parse in ini.h */
    static int iniHandler(void* user, char const* section, char const* name, char const* value);

    /** The set of options currently being parsed by this parser. */
    BaseOptions* options_;
    /** A command-line parser to do the key work for parsing the command line. */
    TCLAP::CmdLine cmdLine_;
    /** A map of maps to store the individual options, by section and option name. */
    std::unordered_map<std::string, std::unordered_map<std::string, std::unique_ptr<BaseOption>>> optionsMap_;
};

/** Generic parser. */
template<typename ValueType> struct Parser {
    /** Parses a string into the templated value type. */
    static ValueType parse(std::string s) {
        ValueType value;
        std::istringstream(s) >> value;
        return value;
    }
};

/** Specialised parser for std::string so that spaces will be correctly handled. */
template<> struct Parser<std::string> {
    /** Parses a string into a string (we just copy it). */
    static std::string parse(std::string s) {
        return s;
    }
};

/** Specialised parser for bool (we need to properly handle "true" and "false". */
template<> struct Parser<bool> {
    /** Parses a string into a boolean. */
    static bool parse(std::string s) {
        bool value;
        std::istringstream(s) >> std::boolalpha >> value;
        return value;
    }
};

/** Specialised parser for std::vector<FloatType> */
template<> struct Parser<std::vector<FloatType>> {
    static std::vector<FloatType> parse(std::string s) {
        std::vector<FloatType> parsed_vals;
        s.erase(std::remove(s.begin(), s.end(), '['), s.end());
        s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
        s.erase(std::remove(s.begin(), s.end(), ','), s.end());
        std::istringstream str(s);
        FloatType val;
        while (str >> val) {
            parsed_vals.push_back(val);
        }

        return parsed_vals;
    }
};

/** Specialised parser for std::vector<std::string> */
template<> struct Parser<std::vector<std::string>> {
    static std::vector<std::string> parse(std::string s) {
        std::vector<std::string> parsed_vals;
        s.erase(std::remove(s.begin(), s.end(), '['), s.end());
        s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, ',')) {
            item = std::regex_replace(item, std::regex("\\s+"), "");
            elems.push_back(item);
        }

        return elems;
    }
};

/** Specialised parser for std::vector<std::vector<std::string>> */
template<> struct Parser<std::vector<std::vector<std::string>>> {
    static std::vector<std::vector<std::string>> parse(std::string s) {
        std::vector<std::vector<std::string>> parsed_vals;

        // Remove outer brackets
        unsigned firstBracket = s.find("[");
        unsigned lastBracket = s.find_last_of("]");
        s.erase(firstBracket, 1);
        s.erase(s.begin() + lastBracket - 1, s.end());

        // Process inner brackets
        while (s.find("[") != std::string::npos) {
            std::string sCopy = s;
            firstBracket = sCopy.find("[");
            lastBracket = sCopy.find("]");
            sCopy.erase(firstBracket, 1);
            sCopy.erase(sCopy.begin() + lastBracket - 1, sCopy.end());
            // Process subsstring
            std::vector<std::string> elems;
            std::stringstream ss(sCopy);
            std::string item;
            while (getline(ss, item, ',')) {
                item = std::regex_replace(item, std::regex("\\s+"), "");
                elems.push_back(item);
            }

            parsed_vals.push_back(elems);
            s.erase(s.begin(), s.begin() + lastBracket + 1);
            if (s.find(",") == std::string::npos)
                break;
            unsigned commaPos = s.find(",");
            s.erase(s.begin(), s.begin() + commaPos + 1);
            unsigned whitespacePos = s.find(" ");
            s.erase(s.begin(), s.begin() + whitespacePos + 1);
        }

        return parsed_vals;
    }
};

/** Specialised parser for std::vector<std::vector<FloatType>> */
template<> struct Parser<std::vector<std::vector<FloatType>>> {
    static std::vector<std::vector<FloatType>> parse(std::string s) {
        std::vector<std::vector<FloatType>> parsed_vals;

        // Remove outer brackets
        unsigned firstBracket = s.find("[");
        unsigned lastBracket = s.find_last_of("]");
        s.erase(firstBracket, 1);
        s.erase(s.begin() + lastBracket - 1, s.end());

        // Process inner brackets
        while (s.find("[") != std::string::npos) {
            std::string sCopy = s;
            firstBracket = sCopy.find("[");
            lastBracket = sCopy.find("]");
            sCopy.erase(firstBracket, 1);
            sCopy.erase(sCopy.begin() + lastBracket - 1, sCopy.end());
            // Process subsstring
            std::vector<FloatType> elems;
            std::stringstream ss(sCopy);
            std::string item;
            while (getline(ss, item, ',')) {
                item = std::regex_replace(item, std::regex("\\s+"), "");
                elems.push_back(atof(item.c_str()));
            }

            parsed_vals.push_back(elems);
            s.erase(s.begin(), s.begin() + lastBracket + 1);
            if (s.find(",") == std::string::npos)
                break;
            unsigned commaPos = s.find(",");
            s.erase(s.begin(), s.begin() + commaPos + 1);
            unsigned whitespacePos = s.find(" ");
            s.erase(s.begin(), s.begin() + whitespacePos + 1);
        }

        return parsed_vals;
    }
};

/** Specialised parser for std::vector<int> */
template<> struct Parser<std::vector<int>> {
    static std::vector<int> parse(std::string s) {
        std::vector<int> parsed_vals;
        s.erase(std::remove(s.begin(), s.end(), '['), s.end());
        s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
        s.erase(std::remove(s.begin(), s.end(), ','), s.end());
        std::istringstream str(s);
        int val;
        while (str >> val) {
            parsed_vals.push_back(val);
        }

        return parsed_vals;
    }
};

/** Specialised parser for std::vector<unsigned int> */
template<> struct Parser<std::vector<unsigned int>> {
    static std::vector<unsigned int> parse(std::string s) {
        std::vector<unsigned int> parsed_vals;
        s.erase(std::remove(s.begin(), s.end(), '['), s.end());
        s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
        s.erase(std::remove(s.begin(), s.end(), ','), s.end());
        std::istringstream str(s);
        unsigned int val;
        while (str >> val) {
            parsed_vals.push_back(val);
        }

        return parsed_vals;
    }
};

/** A templated class representing a single option.
 *
 * As with the base option, the option is identified by its section and name. This class also
 * stores a pointer-to-class member so that it knows its associated option, and
 * can optionally store a default value.
 */
template<typename ValueType, typename OptionsType>
class Option: public BaseOption
{
public:
    /** Creates a new option with the given sectin, name, and pointer-to-class-member.
     * A default value can also be provided (nullptr => no default).
     */
    Option(std::string section, std::string name,
           ValueType OptionsType::*optionPtr, std::unique_ptr<ValueType> defaultValue = nullptr) :
        BaseOption(section, name),
        optionPtr_(optionPtr),
        isSet_(false),
        isDefaulted_(false),
        defaultValue_(std::move(defaultValue)) {
    }
    ~Option() = default;
    _NO_COPY_OR_MOVE(Option);

    /* ---------------- Virtual methods ------------------ */
    virtual bool isSet() override {
        return isSet_;
    }

    virtual bool isDefaulted() override {
        return isDefaulted_;
    }

    virtual void reset() override {
        isSet_ = false;
        isDefaulted_ = false;
    }

    virtual void parse(BaseOptions* options, std::string s) override {
        setValue(options, Parser<ValueType>::parse(s));
    }

    virtual void setDefault(BaseOptions* options) override {
        if (defaultValue_ != nullptr) {
            setValue(options, *defaultValue_);
            isDefaulted_ = true;
        }
    }

    /** Sets the value associated with this specific option in the given options struct
     * to the given value.
     */
    void setValue(BaseOptions* options, ValueType value) {
        static_cast<OptionsType&>(*options).*optionPtr_ = value;
        isSet_ = true;
        isDefaulted_ = false;
    }

private:
    /** The pointer-to-class-data-member for this specific option. */
    ValueType OptionsType::* optionPtr_;
    /** True iff this option has been set. */
    bool isSet_;
    /** True iff this option is defaulted. */
    bool isDefaulted_;
    /** The default value for this option, or nullptr if no such value exists. */
    std::unique_ptr<ValueType> defaultValue_;
};

/** A templated class representing a single command line alias.
 *
 * This is basically just a wrapper for a TCLAP::Arg which uses the TCLAP::Visitor feature
 * to offer an event-based way of handling how the arguments are read.
 *
 * As such, ArgType must be a subclass of TCLAP::Arg in order to work.
 */
template<typename ArgType, typename ValueType, typename OptionsType>
class VisitingArg: public BaseArg, public TCLAP::Visitor
{
public:
    _NO_COPY_OR_MOVE(VisitingArg);

    /** Constructs this arg; the parameters are passed onto the constructor for ArgType which
     * should be a subclass of TCLAP::Arg.
     */
    template<typename ... Args>
    VisitingArg(OptionParser* parser, Option<ValueType, OptionsType>* option, Args && ... args) :
        parser_(parser),
        option_(option),
        arg_(std::forward<Args>(args)..., this) {
    }
    ~VisitingArg() = default;

    /** Returns a reference to the TCLAP::Arg. */
    virtual TCLAP::Arg& getArg() override {
        return arg_;
    }

    /** The visit method for the Visitor pattern. */
    virtual void visit() override {
        // Sets the value of the option.
        option_->setValue(parser_->getOptions(), arg_.getValue());
    }

private:
    /** The option parser this alias is associated with. */
    OptionParser* parser_;
    /** The specific option. */
    Option<ValueType, OptionsType>* option_;
    /** The actual TCLAP::Arg. */
    ArgType arg_;
};

template<typename ValueType, typename OptionsType>
void OptionParser::addOption(std::string section, std::string name,
                             ValueType OptionsType::*optionPtr)
{
    optionsMap_[section][name] = std::make_unique<Option<ValueType, OptionsType>>(section, name,
                                 optionPtr);
}

template<typename ValueType, typename OptionsType>
void OptionParser::addOptionWithDefault(std::string section, std::string name,
                                        ValueType OptionsType::* optionPtr, ValueType defaultValue)
{
    optionsMap_[section][name] = std::make_unique<Option<ValueType, OptionsType>>(section, name,
                                 optionPtr, std::make_unique<ValueType>(defaultValue));
}

template<typename ArgType, typename ValueType, typename OptionsType, typename ... Args>
void OptionParser::addCmdAlias(std::string section, std::string name,
                               ValueType OptionsType::*/*optionPtr*/, Args && ... args)
{
    Option<ValueType, OptionsType>* option =
        static_cast<Option<ValueType, OptionsType> *>(optionsMap_[section][name].get());
    std::unique_ptr<BaseArg> visitor = (
                                           std::make_unique<VisitingArg<ArgType, ValueType, OptionsType>>(this, option,
                                                   std::forward<Args>(args)...));
    cmdLine_.add(visitor->getArg());
    option->addAlias(std::move(visitor));
}

template<typename ValueType, typename OptionsType> void OptionParser::addValueArg(
    std::string section, std::string name, ValueType OptionsType::*optionPtr,
    std::string shortOpt, std::string longOpt,
    std::string description, std::string typeDescription)
{
    addCmdAlias<TCLAP::ValueArg<ValueType>, ValueType, OptionsType>(section, name, optionPtr,
            shortOpt, longOpt, description, false, ValueType(), typeDescription);
}

template<typename OptionsType> void OptionParser::addSwitchArg(
    std::string section, std::string name, bool OptionsType::*optionPtr,
    std::string shortOpt, std::string longOpt,
    std::string description, bool switchValue)
{
    addCmdAlias<TCLAP::SwitchArg, bool, OptionsType>(section, name, optionPtr,
            shortOpt, longOpt, description, !switchValue);
}
} /* namespace options */

#endif /* OPTION_PARSER_HPP_ */
