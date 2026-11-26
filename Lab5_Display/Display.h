#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <vector>
#include <cstring>
#include <iomanip>
#include <utility>

#include "Msg_structs.h"  // Include for SharedMemory and msg_plane_info

#define display_channel_name "mammad_display"

class Display {
public:
    Display();
    ~Display();

    bool initializeSharedMemory();
    bool startServer();
    void joinThreads();
    void shutdownServer();

private:
    void monitorAirspace();    // Thread to read shared memory and display aircraft
    void listenForCollisions(); // Thread to receive collision alerts via IPC

    void displayAircraft(const std::vector<msg_plane_info>& planes);
    void displayCollisionWarning(const std::vector<std::pair<int, int>>& collisionPairs);

    int shm_fd;
    SharedMemory* shared_mem;
    name_attach_t* display_channel;

    std::thread displayThread;
    std::thread serverThread;

    std::atomic<bool> running;
    std::atomic<bool> collisionDetected;
    std::vector<std::pair<int,int>> lastCollisionPairs;
};

#endif // DISPLAY_H
