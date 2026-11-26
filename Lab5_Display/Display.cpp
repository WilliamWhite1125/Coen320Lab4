#include "Display.h"
#include <chrono>
#include <sstream>
#include <sys/neutrino.h>

#include "Msg_structs.h"

#define SHM_NAME "/mammad_buffer"
#define SHM_SIZE sizeof(SharedMemory)
#define SEM_NAME "/mammad_sem"

Display::Display() : shm_fd(-1), shared_mem(nullptr), display_channel(nullptr), running(false), collisionDetected(false) {}

Display::~Display() {
    shutdownServer();
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
    }
    if (shm_fd != -1) {
        close(shm_fd);
    }
}

bool Display::initializeSharedMemory() {
    // Try to open the shared memory object created by Radar

    // Retry until shared memory is available
    while (true) {
        shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);

        if (shm_fd == -1) {
            std::cerr << "Display: waiting for radar shared memory (" SHM_NAME
                        << ")...\n";
            sleep(1);
            continue;
        }

        // Map shared memory into process address space
        shared_mem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);

        if (shared_mem == MAP_FAILED) {
            perror("Display: mmap failed");
            close(shm_fd);
            shm_fd = -1;
            sleep(1);
            continue;
        }

        std::cout << "Display: Shared memory initialized successfully." << std::endl;
        return true;
    }
}

bool Display::startServer() {
    if (!initializeSharedMemory()) {
        std::cerr << "Display: Failed to initialize shared memory." << std::endl;
        return false;
    }

    running.store(true);

    // Start the display thread to monitor airspace
    displayThread = std::thread(&Display::monitorAirspace, this);

    // Start the server thread to listen for collision messages
    serverThread = std::thread(&Display::listenForCollisions, this);

    std::cout << "Display: Server started successfully." << std::endl;
    return true;
}

void Display::joinThreads() {
    if (displayThread.joinable()) {
        displayThread.join();
    }
    if (serverThread.joinable()) {
        serverThread.join();
    }
}

void Display::shutdownServer() {
    running.store(false);

    // Detach channel if it was created
    if (display_channel) {
        name_detach(display_channel, 0);
        display_channel = nullptr;
    }

    // Wait for threads to finish
    joinThreads();
}

void Display::monitorAirspace() {
    std::cout << "Display: Monitoring airspace..." << std::endl;

    while (running.load()) {
        if (shared_mem) {
            // Always display aircraft status, even if empty
            std::vector<msg_plane_info> planes;
            
            // Use sequence number to ensure consistent read (no partial updates)
            int seq_before = shared_mem->sequence.load();
            
            if (shared_mem->count > 0 && !shared_mem->is_empty.load()) {
                std::cout << "Display: Reading " << shared_mem->count << " planes from shared memory." << std::endl;
                for (int i = 0; i < shared_mem->count && i < 100; ++i) {  // Boundary check
                    planes.push_back(shared_mem->plane_data[i]);
                }
            } else if (shared_mem->is_empty.load()) {
                std::cout << "Display: Shared memory is empty." << std::endl;
            } else {
                std::cout << "Display: Shared memory count is " << shared_mem->count << " (empty flag: " 
                         << shared_mem->is_empty.load() << ")." << std::endl;
            }
            
            // Verify sequence number didn't change during read (indicating consistent data)
            int seq_after = shared_mem->sequence.load();
            if (seq_before == seq_after) {
                displayAircraft(planes);
            } else {
                std::cout << "Display: WARNING - Sequence mismatch during read (before=" << seq_before 
                         << " after=" << seq_after << "), data inconsistent, skipping display." << std::endl;
            }
        }

        // Sleep before next update
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Display: Monitoring stopped." << std::endl;
}

void Display::listenForCollisions() {
    std::cout << "Display: Starting IPC server for collision notifications..." << std::endl;

    // Create a channel for the display server
    display_channel = name_attach(NULL, display_channel_name, 0);

    if (display_channel == NULL) {
        std::cerr << "Display: Failed to create server channel." << std::endl;
        return;
    }

    // Drain any stale messages left from previous run
    {
        struct _msg_info info;
        Message_inter_process stale;
        int stale_rcvid;

        // Set a short timeout (100ms) for receive so we don't block
        uint64_t timeout = 100000000ULL; // 100ms in nanoseconds
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &timeout, NULL);

        while ((stale_rcvid = MsgReceive(display_channel->chid, &stale, sizeof(stale), &info)) != -1) {
            // Discard stale message
            MsgReply(stale_rcvid, 0, nullptr, 0);
            // Reset timeout for next iteration
            timeout = 100000000ULL;
            TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &timeout, NULL);
        }
    }

    std::cout << "Display: IPC server ready on channel: " << DISPLAY_IPC_CHANNEL << std::endl;

    Message_inter_process msg;
    int rcvid;

    while (running.load()) {
        // Receive messages from Computer System
        rcvid = MsgReceive(display_channel->chid, &msg, sizeof(msg), nullptr);

        if (rcvid == -1) {
            // Error or timeout, continue listening
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Check if it's a collision detection message
        if (msg.type == MessageType::COLLISION_DETECTED) {
            // Reject stale messages older than 5 seconds
            uint64_t now = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (msg.timestamp > 0 && (now > msg.timestamp) && ((now - msg.timestamp) > 5)) {
                int reply = 0;
                MsgReply(rcvid, 0, &reply, sizeof(reply));
                continue; // skip stale message
            }
            // Deserialize collision pairs from message data
            // Use safe memcpy to avoid struct packing/alignment issues
            size_t numPairs = msg.dataSize / (2 * sizeof(int));  // 2 ints per pair
            std::vector<std::pair<int, int>> collisionPairs;

            for (size_t i = 0; i < numPairs; ++i) {
                int id1 = 0, id2 = 0;
                // Safely copy each aircraft ID
                std::memcpy(&id1, msg.data.data() + i * 2 * sizeof(int), sizeof(int));
                std::memcpy(&id2, msg.data.data() + i * 2 * sizeof(int) + sizeof(int), sizeof(int));
                
                // Validate aircraft IDs (sanity check)
                if (id1 >= 0 && id2 >= 0) {
                    collisionPairs.emplace_back(id1, id2);
                } else {
                    std::cerr << "WARNING: Invalid aircraft ID in collision pair: (" << id1 << ", " << id2 << ")" << std::endl;
                }
            }

            // Display collision warning, but only if different from last shown
            if (!collisionPairs.empty()) {
                if (collisionPairs != lastCollisionPairs) {
                    displayCollisionWarning(collisionPairs);
                    lastCollisionPairs = collisionPairs;
                    collisionDetected.store(true);
                }
            }
        }

        // Reply to sender (placeholder response)
        int reply = 0;
        MsgReply(rcvid, 0, &reply, sizeof(reply));
    }

    std::cout << "Display: IPC server stopped." << std::endl;
}

void Display::displayAircraft(const std::vector<msg_plane_info>& planes) {
    // Clear screen (using ANSI escape code for simplicity)
    std::cout << "\033[2J\033[H"; // Clear screen and move to top

    std::cout << "\n===========================================\n";
    std::cout << "           AIRCRAFT STATUS DISPLAY\n";
    std::cout << "===========================================\n";

    if (planes.empty()) {
        std::cout << "No aircraft in airspace.\n";
    } else {
        std::cout << std::setw(6) << "ID"
                  << std::setw(15) << "Position X"
                  << std::setw(15) << "Position Y"
                  << std::setw(15) << "Position Z"
                  << std::setw(12) << "Vel X"
                  << std::setw(12) << "Vel Y"
                  << std::setw(12) << "Vel Z\n";
        std::cout << "-------------------------------------------\n";

        for (const auto& plane : planes) {
            std::cout << std::setw(6) << plane.id
                      << std::setw(15) << std::fixed << std::setprecision(2) << plane.PositionX
                      << std::setw(15) << plane.PositionY
                      << std::setw(15) << plane.PositionZ
                      << std::setw(12) << plane.VelocityX
                      << std::setw(12) << plane.VelocityY
                      << std::setw(12) << plane.VelocityZ << "\n";
        }
    }

    std::cout << "===========================================\n";
}

void Display::displayCollisionWarning(const std::vector<std::pair<int, int>>& collisionPairs) {
    // Clear screen to prevent collision warnings from stacking
    std::cout << "\033[2J\033[H"; // Clear screen and move to top
    
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║      *** COLLISION WARNING ***          ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";

    for (const auto& pair : collisionPairs) {
        std::cout << ">>> COLLISION DETECTED: Aircraft " << pair.first
                  << " and Aircraft " << pair.second << " <<<\n";
    }

    std::cout << "════════════════════════════════════════\n\n";
}
