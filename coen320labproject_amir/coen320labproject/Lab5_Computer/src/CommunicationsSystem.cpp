/*
 * CommunicationsSystem.cpp
 *
 * Bridge between the operator console and the aircraft. It receives
 * Message_inter_process commands on "mammad_comms" and forwards them to the
 * aircraft channel ("mammad_radar<id>").
 *
 * The class header is fixed, so the channel handle and the shutdown flag are
 * file-scope state (the same style the Display and Radar modules use).
 */

#include "CommunicationsSystem.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "Msg_structs.h"

#define COMMS_CHANNEL_NAME "mammad_comms"
#define AIRCRAFT_CHANNEL_PREFIX "mammad_radar"
#define COMPUTER_CHANNEL_NAME "mammad_computer"
#define DISPLAY_CHANNEL_NAME "mammad_display"
#define STOP_PULSE_CODE (_PULSE_CODE_MINAVAIL + 1)

namespace {

name_attach_t* g_channel = NULL;
std::atomic<bool> g_stop(false);

// Filled by messageAircraft() so HandleCommunications() can build a reply.
bool g_lastAircraftOk = false;
msg_plane_info g_lastAircraftInfo;

int sendToAircraft(const Message_inter_process& msg, msg_plane_info* infoOut) {
    if (msg.planeID < 0) {
        return -1;
    }

    const std::string channelName =
        std::string(AIRCRAFT_CHANNEL_PREFIX) + std::to_string(msg.planeID);
    int coid = name_open(channelName.c_str(), 0);
    if (coid == -1) {
        return -1;
    }

    int status = -1;
    if (msg.type == MessageType::REQUEST_AUGMENTED_INFO) {
        msg_plane_info info;
        std::memset(&info, 0, sizeof(info));
        if (MsgSend(coid, &msg, sizeof(msg), &info, sizeof(info)) != -1) {
            status = 0;
            if (infoOut != NULL) {
                *infoOut = info;
            }
        }
    } else {
        if (MsgSend(coid, &msg, sizeof(msg), &status, sizeof(status)) == -1) {
            status = -1;
        }
    }

    name_close(coid);
    return status;
}

int sendToComputer(const Message_inter_process& msg) {
    int coid = name_open(COMPUTER_CHANNEL_NAME, 0);
    if (coid == -1) {
        return -1;
    }

    int status = -1;
    if (MsgSend(coid, &msg, sizeof(msg), &status, sizeof(status)) == -1) {
        status = -1;
    }
    name_close(coid);
    return status;
}

void sendExit(const char* channelName) {
    Message_inter_process msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.header = true;
    msg.type = MessageType::EXIT;
    msg.planeID = -1;
    msg.dataSize = 0;

    int coid = name_open(channelName, 0);
    if (coid == -1) {
        return;  // target is not running
    }

    int status = 0;
    MsgSend(coid, &msg, sizeof(msg), &status, sizeof(status));
    name_close(coid);
}

} // namespace

CommunicationsSystem::CommunicationsSystem() {
    g_stop.store(false);

    g_channel = name_attach(NULL, COMMS_CHANNEL_NAME, 0);
    if (g_channel == NULL) {
        perror("CommunicationsSystem: name_attach failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "CommunicationsSystem: listening on " << COMMS_CHANNEL_NAME
              << std::endl;

    Communications_System =
        std::thread(&CommunicationsSystem::HandleCommunications, this);
}

CommunicationsSystem::~CommunicationsSystem() {
    g_stop.store(true);

    // Wake the worker if it is blocked in MsgReceive().
    if (g_channel != NULL) {
        int coid = ConnectAttach(0, 0, g_channel->chid, 0, 0);
        if (coid != -1) {
            MsgSendPulse(coid, -1, STOP_PULSE_CODE, 0);
            ConnectDetach(coid);
        }
    }

    if (Communications_System.joinable()) {
        Communications_System.join();
    }

    if (g_channel != NULL) {
        name_detach(g_channel, 0);
        g_channel = NULL;
    }
}

void CommunicationsSystem::HandleCommunications() {
    while (!g_stop.load()) {
        Message_inter_process msg;
        std::memset(&msg, 0, sizeof(msg));

        const int rcvid = MsgReceive(g_channel->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (g_stop.load()) {
                break;
            }
            continue;
        }
        if (rcvid == 0) {
            continue;  // pulse used to wake us during shutdown
        }

        switch (msg.type) {
        case MessageType::REQUEST_CHANGE_OF_HEADING:
        case MessageType::REQUEST_CHANGE_POSITION:
        case MessageType::REQUEST_CHANGE_ALTITUDE:
        case MessageType::REQUEST_AUGMENTED_INFO: {
            // Adapt the received inter-process message to the Message type
            // expected by messageAircraft(). data points into msg, which stays
            // valid for the duration of this call.
            Message inProcess;
            std::memset(&inProcess, 0, sizeof(inProcess));
            inProcess.header = false;
            inProcess.type = msg.type;
            inProcess.planeID = msg.planeID;
            inProcess.data = msg.data.data();
            inProcess.dataSize = msg.dataSize;

            messageAircraft(inProcess);

            if (msg.type == MessageType::REQUEST_AUGMENTED_INFO) {
                if (!g_lastAircraftOk) {
                    g_lastAircraftInfo.id = -1;
                }
                MsgReply(rcvid, 0, &g_lastAircraftInfo,
                         sizeof(g_lastAircraftInfo));
            } else {
                const int status = g_lastAircraftOk ? 0 : -1;
                MsgReply(rcvid, 0, &status, sizeof(status));
            }
            break;
        }

        case MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS: {
            const int status = sendToComputer(msg);
            MsgReply(rcvid, 0, &status, sizeof(status));
            break;
        }

        case MessageType::EXIT: {
            const int status = 0;
            MsgReply(rcvid, 0, &status, sizeof(status));
            sendExit(COMPUTER_CHANNEL_NAME);
            sendExit(DISPLAY_CHANNEL_NAME);
            g_stop.store(true);
            break;
        }

        default: {
            const int status = -1;  // unsupported message type
            MsgReply(rcvid, 0, &status, sizeof(status));
            break;
        }
        }
    }
}

void CommunicationsSystem::messageAircraft(const Message& msg) {
    Message_inter_process ipc;
    std::memset(&ipc, 0, sizeof(ipc));
    ipc.header = true;
    ipc.type = msg.type;
    ipc.planeID = msg.planeID;

    const size_t size = std::min(msg.dataSize, ipc.data.size());
    if (msg.data != NULL && size > 0) {
        std::memcpy(ipc.data.data(), msg.data, size);
    }
    ipc.dataSize = size;

    g_lastAircraftOk = false;
    std::memset(&g_lastAircraftInfo, 0, sizeof(g_lastAircraftInfo));

    if (sendToAircraft(ipc, &g_lastAircraftInfo) == 0) {
        g_lastAircraftOk = true;
    }
}