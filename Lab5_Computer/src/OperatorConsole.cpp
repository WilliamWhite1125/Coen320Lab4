
OperatorConsole::OperatorConsole() : exit(false) {
    Operator_Console = std::thread(&OperatorConsole::HandleConsoleInputs, this);
}

OperatorConsole::~OperatorConsole() {
    exit = true;
    if (Operator_Console.joinable()) {
        Operator_Console.join();
    }
}

void OperatorConsole::HandleConsoleInputs() {
    std::string command;
    std::cout << "\n========== OPERATOR CONSOLE ==========" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  heading <aircraftID> <vx> <vy> <vz>  - Change aircraft heading (velocity vector)" << std::endl;
    std::cout << "  velocity <aircraftID> <vx> <vy> <vz> - Directly set aircraft velocity components" << std::endl;  // ← NEW
    std::cout << "  position <aircraftID> <x> <y> <z>    - Change aircraft position" << std::endl;
    std::cout << "  altitude <aircraftID> <alt>          - Change aircraft altitude" << std::endl;
    std::cout << "  exit                                 - Exit console" << std::endl;
    std::cout << "======================================\n" << std::endl;

    while (!exit) {
        std::cout << "OperatorConsole> ";
        std::getline(std::cin, command);

        if (command.empty()) continue;

        std::istringstream iss(command);
        std::string action;
        iss >> action;
//        // ← NEW: Velocity command handler
        if (action == "velocity") {
            int aircraftID;
            double vx, vy, vz;
            if (iss >> aircraftID >> vx >> vy >> vz) {
                sendVelocityCommand(aircraftID, vx, vy, vz);
                logCommand("velocity " + std::to_string(aircraftID) + " " +
                          std::to_string(vx) + " " + std::to_string(vy) + " " + std::to_string(vz));
            } else {
                std::cerr << "Invalid format. Use: velocity <aircraftID> <vx> <vy> <vz>" << std::endl;
            }
        }
        else if (action == "position") {
            int aircraftID;
            double x, y, z;
            if (iss >> aircraftID >> x >> y >> z) {
                sendPositionCommand(aircraftID, x, y, z);
                logCommand("position " + std::to_string(aircraftID) + " " +
                          std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z));
            } else {
                std::cerr << "Invalid format. Use: position <aircraftID> <x> <y> <z>" << std::endl;
            }
        }
        else if (action == "altitude") {
            int aircraftID;
            double alt;
            if (iss >> aircraftID >> alt) {
                sendAltitudeCommand(aircraftID, alt);
                logCommand("altitude " + std::to_string(aircraftID) + " " + std::to_string(alt));
            } else {
                std::cerr << "Invalid format. Use: altitude <aircraftID> <alt>" << std::endl;
            }
        }
        else if (action == "exit") {
            std::cout << "Exiting operator console..." << std::endl;
            exit = true;
            break;
        }
        else {
            std::cerr << "Unknown command: " << action << std::endl;
        }
    }
}

void OperatorConsole::sendVelocityCommand(int aircraftID, double vx, double vy, double vz){ 
  const char* ID = "TODO: Correct Aircraft channel";
  int aircraft_channel = name_open(ID, 0);

  if (aircraft_channel == -1) {
        std::cerr << "ERROR: Could not connect to Aircraft " << aircraftID << std::endl;
        return;
    }

    Message_inter_process msg;
    msg.header = 1;
    msg.planeID = aircraftID;
    // Use REQUEST_CHANGE_OF_HEADING since it carries velocity data
    // If your system has REQUEST_CHANGE_VELOCITY, use that instead:
    msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;  // or REQUEST_CHANGE_VELOCITY if defined

    msg_change_heading velocityCmd;
    velocityCmd.ID = aircraftID;
    velocityCmd.VelocityX = vx;
    velocityCmd.VelocityY = vy;
    velocityCmd.VelocityZ = vz;
    velocityCmd.altitude = 0;  // Not used for pure velocity change

    msg.dataSize = sizeof(msg_change_heading);
    std::memcpy(msg.data.data(), &velocityCmd, sizeof(msg_change_heading));

    int reply = 0;
    int status = MsgSend(aircraft_channel, &msg, sizeof(msg), &reply, sizeof(reply));

    if (status == -1) {
        std::cerr << "ERROR: Failed to send velocity command to Aircraft " << aircraftID << std::endl;
    } else if (reply != 0) {
        std::cerr << "WARNING: Aircraft " << aircraftID << " returned error code " << reply << std::endl;
    } else {
        std::cout << "✓ Velocity command sent to Aircraft " << aircraftID
                  << " [vx=" << vx << ", vy=" << vy << ", vz=" << vz << "]" << std::endl;
    }

    name_close(aircraft_channel);
}

void OperatorConsole::sendPositionCommand(int aircraftID, double x, double y, double z) {
  const char* ID = "TODO: Correct Aircraft channel";
  int aircraft_channel = name_open(ID, 0);

  if (aircraft_channel == -1) {
        std::cerr << "ERROR: Could not connect to Aircraft " << aircraftID << std::endl;
        return;
  }

  Message_inter_process msg;
  msg.header = 1;
  msg.planeID = aircraftID;
  msg.type = MessageType::REQUEST_CHANGE_POSITION;

  msg_change_position posCmd;
  posCmd.x = x;
  posCmd.y = y;
  posCmd.z = z;

  msg.dataSize = sizeof(msg_change_position);
  std::memcpy(msg.data.data(), &posCmd, sizeof(msg_change_position));

  int reply = 0;
  int status = MsgSend(aircraft_channel, &msg, sizeof(msg), &reply, sizeof(reply));
    
  if (status == -1) {
      std::cerr << "ERROR: Failed to send position command to Aircraft " << aircraftID << std::endl;
  } else if (reply != 0) {
      std::cerr << "WARNING: Aircraft " << aircraftID << " returned error code " << reply << std::endl;
  } else {
      std::cout << "✓ Position command sent to Aircraft " << aircraftID << std::endl;
  }

  name_close(aircraft_channel);
}


void OperatorConsole::sendAltitudeCommand(int aircraftID, double altitude) {
  const char* ID = "TODO: Correct Aircraft channel";
  int aircraft_channel = name_open(ID, 0);

  if (aircraft_channel == -1) {
        std::cerr << "ERROR: Could not connect to Aircraft " << aircraftID << std::endl;
        return;
  }

  Message_inter_process msg;
  msg.header = 1;
  msg.planeID = aircraftID;
  msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;

  msg.dataSize = sizeof(double);
  std::memcpy(msg.data.data(), &altitude, sizeof(double));

  int reply = 0;
  int status = MsgSend(aircraft_channel, &msg, sizeof(msg), &reply, sizeof(reply));
    
  if (status == -1) {
      std::cerr << "ERROR: Failed to send altitude command to Aircraft " << aircraftID << std::endl;
  } else if (reply != 0) {
      std::cerr << "WARNING: Aircraft " << aircraftID << " returned error code " << reply << std::endl;
  } else {
      std::cout << "✓ Altitude command sent to Aircraft " << aircraftID << std::endl;
  }
  name_close(aircraft_channel);
}

void OperatorConsole::logCommand(const std::string& command) {
    std::cout << "[LOG] Command executed: " << command << std::endl;
}
