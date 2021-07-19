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
#ifndef _COMMAND_TYPES_HPP_
#define _COMMAND_TYPES_HPP_

namespace oppt
{

enum CommandType {
    ADD_DISPLAY,
    REMOVE_DISPLAY,
    SET_FIXED_FRAME,
    SET_FRAME_RATE,
    LOOK_AT,
    TAKE_SCREENSHOT,
    CHANGE_BACKGROUND_COLOR
};

struct DisplayCommand {
    std::string typeString = "";

    virtual CommandType getType() const = 0;

};

struct AddDisplayCommand: public DisplayCommand {
    AddDisplayCommand():
        DisplayCommand() {
        typeString = "addDisplay";

    }

    CommandType getType() const override {
        return CommandType::ADD_DISPLAY;
    }

    std::string name = "";

    std::string displayType = "";

    bool enabled = true;

    std::string topic = "";

    std::string topicType = "";

    std::string frame = "";

};

struct RemoveDisplayCommand: public DisplayCommand {
    RemoveDisplayCommand():
        DisplayCommand() {
        typeString = "removeDisplay";

    }

    CommandType getType() const override {
        return CommandType::REMOVE_DISPLAY;
    }

    std::string name = "";
};

struct SetFixedFrameCommand: public DisplayCommand {
    SetFixedFrameCommand():
        DisplayCommand() {
        typeString = "setFixedFrame";

    }

    CommandType getType() const override {
        return CommandType::SET_FIXED_FRAME;
    }

    std::string fixedFrame = "";
};

struct SetFrameRateCommand: public DisplayCommand {
    SetFrameRateCommand():
        DisplayCommand() {
        typeString = "setFrameRate";
    }

    CommandType getType() const override {
        return CommandType::SET_FRAME_RATE;
    }

    unsigned int frameRate = 0;

};

struct LookAtCommand: public DisplayCommand {
    LookAtCommand():
        DisplayCommand() {
        typeString = "lookAtCommand";

    }

    CommandType getType() const override {
        return CommandType::LOOK_AT;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ChangeBackgroundColorCommand: public DisplayCommand {
    ChangeBackgroundColorCommand():
        DisplayCommand() {
        typeString = "changeBackgroundColor";

    }

    CommandType getType() const override {
        return CommandType::CHANGE_BACKGROUND_COLOR;
    }

    unsigned int r = 0;
    unsigned int g = 0;
    unsigned int b = 0;
};

struct TakeScreenshotCommand: public DisplayCommand {
    TakeScreenshotCommand(): DisplayCommand() {
        typeString = "takeScreenshot";
    }

    CommandType getType() const override {
        return CommandType::TAKE_SCREENSHOT;
    }

    std::string filename = "";

};

struct AddDisplayProperties {
    std::string command = "";

    std::string name = "";

    std::string displayType = "";

    bool enabled = true;

    std::string topic = "";

    std::string topicType = "";

    bool setFixedFrame = false;

    std::string fixedFrame = "";

};

}

#endif
