/*
 * OperatorConsole.cpp
 *
 *  Created on: Nov 25, 2026
 *      Author: w_wh
 */

//Amir: added includes for input parsing/validation
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "OperatorConsole.h"
#include "Msg_structs.h"

#define COMMS_CHANNEL_NAME "mammad_comms"

//Amir: helpers for reading and validating operator input
namespace {

void printUsage() {
    std::cout
        << "\nOperator Console commands:\n"
        << "  REQUEST_CHANGE_OF_HEADING <planeID> <velocityX> <velocityY> <velocityZ> [altitude]\n"
        << "  REQUEST_CHANGE_POSITION   <planeID> <x> <y> <z>\n"
        << "  REQUEST_CHANGE_ALTITUDE   <planeID> <altitude>\n"
        << "  REQUEST_AUGMENTED_INFO    <planeID>\n"
        << "  CHANGE_TIME_CONSTRAINT_COLLISIONS <seconds>\n"
        << "  HELP\n"
        << "  EXIT\n\n";
}

std::string trim(const std::string& text) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = text.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = text.find_last_not_of(whitespace);
    return text.substr(first, last - first + 1);
}

std::string toUpperCase(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

bool parseInteger(const std::string& text, int& value) {
    try {
        std::size_t consumed = 0;
        const long parsed = std::stol(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        value = static_cast<int>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseNumber(const std::string& text, double& value) {
    try {
        std::size_t consumed = 0;
        value = std::stod(text, &consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

void sendToComms(const Message_inter_process& msg) {
    int coid = name_open(COMMS_CHANNEL_NAME, 0);
    if (coid == -1) {
        std::cerr << "Operator Console: communications system is not running ("
                  << COMMS_CHANNEL_NAME << ")\n";
        return;
    }

    if (msg.type == MessageType::REQUEST_AUGMENTED_INFO) {
        msg_plane_info info;
        std::memset(&info, 0, sizeof(info));

        const int rc = MsgSend(coid, &msg, sizeof(msg), &info, sizeof(info));
        name_close(coid);

        if (rc == -1) {
            std::perror("Operator Console: MsgSend failed");
        } else if (info.id < 0) {
            std::cerr << "Operator Console: aircraft did not answer\n";
        } else {
            std::cout << "Operator Console: aircraft " << info.id << " pos ("
                      << info.PositionX << ", " << info.PositionY << ", "
                      << info.PositionZ << ") vel (" << info.VelocityX << ", "
                      << info.VelocityY << ", " << info.VelocityZ << ")\n";
        }
        return;
    }

    int status = -1;
    const int rc = MsgSend(coid, &msg, sizeof(msg), &status, sizeof(status));
    name_close(coid);

    if (rc == -1) {
        std::perror("Operator Console: MsgSend failed");
    } else if (status != 0) {
        std::cerr << "Operator Console: request rejected\n";
    } else {
        std::cout << "Operator Console: delivered\n";
    }
}

} // namespace

OperatorConsole::OperatorConsole() {
	//constructor
	//EXAMPLE: pthread_create(&thread_id, NULL, updatePositionThread, (void*)this);
    Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
	//destructor
    //Amir: atomic exit so the input thread observes it safely
    exit.store(true);
    if (Operator_Console.joinable()) {
        Operator_Console.join();
    }
}

//Amir: rewritten to read, parse and validate operator commands
void OperatorConsole::HandleConsoleInputs() {
    printUsage();

    bool promptShown = false;
    std::string line;

    while (!exit.load()) {
        if (!promptShown) {
            std::cout << "> " << std::flush;
            promptShown = true;
        }

        // Poll stdin so ~OperatorConsole() doesn't hang in join().
        struct pollfd pfd;
        pfd.fd = STDIN_FILENO;
        pfd.events = POLLIN;
        pfd.revents = 0;

        const int ready = poll(&pfd, 1, 200);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (ready == 0) {
            continue;  // timeout; re-check exit
        }
        if (!(pfd.revents & (POLLIN | POLLHUP))) {
            break;
        }

        if (!std::getline(std::cin, line)) {
            // stdin was closed; there is nothing left to read.
            break;
        }
        promptShown = false;

        line = trim(line);
        if (line.empty()) {
            continue;
        }

        //Amir: split the line into command + arguments
        std::istringstream input(line);
        std::vector<std::string> tokens;
        for (std::string token; input >> token; ) {
            tokens.push_back(token);
        }

        const std::string command = toUpperCase(tokens[0]);

        //Amir: allow the input thread to terminate cleanly
        if (command == "EXIT" || command == "QUIT") {
            Message_inter_process exitMsg;
            std::memset(&exitMsg, 0, sizeof(exitMsg));
            exitMsg.header = true;
            exitMsg.type = MessageType::EXIT;
            exitMsg.planeID = -1;
            exitMsg.dataSize = 0;
            sendToComms(exitMsg);

            std::cout << "Operator Console: exiting.\n";
            exit.store(true);
            break;
        }

        if (command == "HELP") {
            printUsage();
            continue;
        }

        //Amir: message that will carry the parsed command
        Message_inter_process msg{};
        msg.header = true;

        //Amir: parse and serialize each supported operator command
        if (command == "REQUEST_CHANGE_OF_HEADING") {
            int planeID = 0;
            msg_change_heading payload{};
            if (tokens.size() < 5 || !parseInteger(tokens[1], planeID) ||
                !parseNumber(tokens[2], payload.VelocityX) ||
                !parseNumber(tokens[3], payload.VelocityY) ||
                !parseNumber(tokens[4], payload.VelocityZ)) {
                std::cerr << "Operator Console: usage: REQUEST_CHANGE_OF_HEADING "
                             "<planeID> <velocityX> <velocityY> <velocityZ> [altitude]\n";
                continue;
            }
            if (tokens.size() >= 6 && !parseNumber(tokens[5], payload.altitude)) {
                std::cerr << "Operator Console: altitude must be a number\n";
                continue;
            }
            if (tokens.size() > 6) {
                std::cerr << "Operator Console: too many arguments for "
                             "REQUEST_CHANGE_OF_HEADING\n";
                continue;
            }

            payload.ID = planeID;
            msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;
            msg.planeID = planeID;
            msg.dataSize = sizeof(payload);
            std::memcpy(msg.data.data(), &payload, sizeof(payload));
        } else if (command == "REQUEST_CHANGE_POSITION") {
            int planeID = 0;
            msg_change_position payload{};
            if (tokens.size() != 5 || !parseInteger(tokens[1], planeID) ||
                !parseNumber(tokens[2], payload.x) ||
                !parseNumber(tokens[3], payload.y) ||
                !parseNumber(tokens[4], payload.z)) {
                std::cerr << "Operator Console: usage: REQUEST_CHANGE_POSITION "
                             "<planeID> <x> <y> <z>\n";
                continue;
            }

            msg.type = MessageType::REQUEST_CHANGE_POSITION;
            msg.planeID = planeID;
            msg.dataSize = sizeof(payload);
            std::memcpy(msg.data.data(), &payload, sizeof(payload));
        } else if (command == "REQUEST_CHANGE_ALTITUDE") {
            int planeID = 0;
            double altitude = 0.0;
            if (tokens.size() != 3 || !parseInteger(tokens[1], planeID) ||
                !parseNumber(tokens[2], altitude)) {
                std::cerr << "Operator Console: usage: REQUEST_CHANGE_ALTITUDE "
                             "<planeID> <altitude>\n";
                continue;
            }

            msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;
            msg.planeID = planeID;
            // Payload is a single double (the new altitude).
            msg.dataSize = sizeof(altitude);
            std::memcpy(msg.data.data(), &altitude, sizeof(altitude));
        } else if (command == "REQUEST_AUGMENTED_INFO") {
            int planeID = 0;
            if (tokens.size() != 2 || !parseInteger(tokens[1], planeID)) {
                std::cerr << "Operator Console: usage: REQUEST_AUGMENTED_INFO "
                             "<planeID>\n";
                continue;
            }

            msg.type = MessageType::REQUEST_AUGMENTED_INFO;
            msg.planeID = planeID;
            msg.dataSize = 0;
        } else if (command == "CHANGE_TIME_CONSTRAINT_COLLISIONS") {
            int seconds = 0;
            if (tokens.size() != 2 || !parseInteger(tokens[1], seconds) ||
                seconds < 0) {
                std::cerr << "Operator Console: usage: "
                             "CHANGE_TIME_CONSTRAINT_COLLISIONS <seconds>\n";
                continue;
            }

            msg.type = MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS;
            msg.planeID = -1;
            // Payload is a single int (the new time constraint in seconds).
            msg.dataSize = sizeof(seconds);
            std::memcpy(msg.data.data(), &seconds, sizeof(seconds));
        } else {
            std::cerr << "Operator Console: unknown command '" << tokens[0]
                      << "' (type HELP)\n";
            continue;
        }

        logCommand(msg.planeID >= 0
                       ? command + " for plane " + std::to_string(msg.planeID)
                       : command);
        sendToComms(msg);
    }

    std::cout << "Operator Console: input thread stopped.\n";
}

void OperatorConsole::logCommand(const std::string& command) {
    // TODO
}
