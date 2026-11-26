#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

class OperatorConsole {
public:
	OperatorConsole();
    ~OperatorConsole();

private:
    void HandleConsoleInputs();

    // Command senders via IPC
    void sendHeadingCommand(int aircraftID, double vx, double vy, double vz);
    void sendVelocityCommand(int aircraftID, double vx, double vy, double vz);
    void sendPositionCommand(int aircraftID, double x, double y, double z);
    void sendAltitudeCommand(int aircraftID, double altitude);

    void logCommand(const std::string& command);

    std::thread Operator_Console;
    bool exit = false;
};



#endif /* OPERATORCONSOLE_H_ */
