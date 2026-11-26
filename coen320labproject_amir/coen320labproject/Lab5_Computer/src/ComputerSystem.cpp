#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <cstdio>
#include <iomanip>      // For std::put_time
#include <cmath>
#include <mutex>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <cstring> // For memcpy
#include <semaphore.h>


// COEN320 Task 3.1, set the display channel name (it should be the same as the Coen320_Lab (Task0) Display channel name)
#define display_channel_name "mammad_display"

#define SHM_NAME "/mammad_buffer"
#define SHM_SIZE sizeof(SharedMemory)
#define SEM_NAME "/mammad_sem"

#define COMPUTER_CHANNEL_NAME "mammad_computer"
#define STOP_PULSE_CODE (_PULSE_CODE_MINAVAIL + 1)

namespace {
name_attach_t* g_computerChannel = NULL;
std::atomic<bool> g_operatorStop(false);
std::mutex g_constraintMutex;
}


ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();

    // Wake and stop the operator listener.
    g_operatorStop.store(true);
    if (g_computerChannel != NULL) {
        int coid = ConnectAttach(0, 0, g_computerChannel->chid, 0, 0);
        if (coid != -1) {
            MsgSendPulse(coid, -1, STOP_PULSE_CODE, 0);
            ConnectDetach(coid);
        }
    }

    if (monitorOperatorInput.joinable()) {
        monitorOperatorInput.join();
    }

    if (g_computerChannel != NULL) {
        name_detach(g_computerChannel, 0);
        g_computerChannel = NULL;
    }

    cleanupSharedMemory();
}

// TODO: Amir
bool ComputerSystem::initializeSharedMemory() {

   	ATCTimer timer(1,0);

	// Open the shared memory object
	while (true) {
        // COEN320 Task 3.2
		// Attempt to open the shared memory object (You need to use the same name as Task 2 in Radar)
        // In case of error, retry until successful
        // e.g. shm_open("/radar_shared_mem", O_RDONLY, 0666);
        // COEN320 Task 3.3
		// Map the shared memory object into the process's address space
        // The shared memory should be mapped to "shared_mem" (check for errors)

        shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
        if (shm_fd == -1) {
            perror("shm_open (shared memory may not exist yet)");
            timer.waitTimer();
            continue;
        }

        shared_mem = (SharedMemory *)mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE,
        MAP_SHARED, shm_fd, 0);
        if (shared_mem == MAP_FAILED) {
            perror("mmap");
            shared_mem = nullptr;
            close(shm_fd);
            shm_fd = -1;
            timer.waitTimer();
            continue;
        }

        std::cout << "Shared memory initialized successfully." << std::endl;
        return true;
	}
}

void ComputerSystem::cleanupSharedMemory() {
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
    }
    if (shm_fd != -1) {
        close(shm_fd);
    }
}

bool ComputerSystem::startMonitoring() {
    if (!initializeSharedMemory()) {
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }

    g_operatorStop.store(false);
    g_computerChannel = name_attach(NULL, COMPUTER_CHANNEL_NAME, 0);
    if (g_computerChannel == NULL) {
        perror("ComputerSystem: name_attach failed");
    }

    running = true;
    std::cout << "Starting monitoring thread." << std::endl;

    monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);
    if (g_computerChannel != NULL) {
        monitorOperatorInput = std::thread(&ComputerSystem::processMessage, this);
    }

    return true;
}

void ComputerSystem::joinThread() {
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

void ComputerSystem::monitorAirspace() {
	//std::cout << "Initial is_empty value: " << shared_mem->is_empty.load() << std::endl;
	ATCTimer timer(1,0);
	// Vector to store plane data
	std::vector<msg_plane_info> plane_data_vector;
	uint64_t timestamp;

	sem_t *sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
	if (sem == SEM_FAILED) {
		perror("sem_open");
		return;
	}

    // Keep monitoring indefinitely until `stopMonitoring` is called
	sem_wait(sem);
	while (shared_mem->is_empty.load()) {
		sem_post(sem);
		std::cout << "Waiting for planes in airspace...\n";
		timer.waitTimer();
		sem_wait(sem);
	}
	sem_post(sem);

	while (running) {
		sem_wait(sem);
		if (shared_mem->is_empty.load()) {
			std::cout << "No planes in airspace. Stopping monitoring.\n";
			running = false;
			sem_post(sem);
	        break;
        } else {
        	plane_data_vector.clear();
        	// Print separator and timestamp
            // std::cout << "\n================= Shared Memory Update =================\n";

            // Display the timestamp in a readable format
            timestamp = shared_mem->timestamp;
            //std::cout << "Last Update Timestamp: " << timestamp << "\n";
            //std::cout << "Number of planes in shared memory: " << shared_mem->count << "\n";

            for (int i = 0; i < shared_mem->count; ++i) {
            	const msg_plane_info& plane = shared_mem->plane_data[i];
            	// Store the plane info in the vector
            	plane_data_vector.push_back(plane);
            }
        }
        sem_post(sem);
		//**************Call Collision Detector*********************
		if (plane_data_vector.size()>1)
            checkCollision(timestamp, plane_data_vector);
		else
            std::cout << "No collision possible with single plane\n";
        // Sleep for a short interval before the next poll
       timer.waitTimer();
    }
	std::cout << "Exiting monitoring loop." << std::endl;
	sem_close(sem);
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    //std::cout << "Checking for collisions at time: " << currentTime << std::endl;

    for (int i = 0; i < planes.size(); i++){
        for(int j = i+1; j< planes.size();j++){

            if (checkAxes(planes[i], planes[j])){

                // Prepare the message
                Message_inter_process msg_to_send;
                std::vector<std::pair<int, int>> collisionPairs;
                // Store the collision pair plane ID 0 and 1
                collisionPairs.emplace_back(planes[i].id, planes[j].id);
                // Serialize collisionPairs
                size_t numPairs = collisionPairs.size();
                size_t dataSize = numPairs * sizeof(std::pair<int, int>);

                msg_to_send.header = true;
                msg_to_send.planeID = -1;
                msg_to_send.type = MessageType::COLLISION_DETECTED;
                msg_to_send.dataSize = dataSize;
                std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);
                sendCollisionToDisplay(msg_to_send);
            }
        }
    }

    // COEN320 Task 3.4
    // detect collisions between planes in the airspace within the time constraint
    // You need to Iterate through each pair of planes and in case of collision,
    // store the pair of plane IDs that are predicted to collide
    // You can use the function checkAxes provided below to check if two planes will collide
    // COEN320 Task 3.5
    // in case of collision, send message to Display system
    /*
    HINT:
    In case of collision a Message (type Message_inter_process) should be sent to the Display system
    The data field of the message should contain the list of pairs of plane IDs that are predicted to collide
    Make sure to fill dataSize field of the message appropriately
    e.g. (here a std::pair<int,int> is used to represent a pair of colliding planes)
    // Prepare the message
    Message_inter_process msg_to_send;
    std::vector<std::pair<int, int>> collisionPairs;
    // Store the collision pair plane ID 0 and 1
    collisionPairs.emplace_back(<plane 0>, <plane 1>);
    // Serialize collisionPairs
    size_t numPairs = collisionPairs.size();
    size_t dataSize = numPairs * sizeof(std::pair<int, int>);

    msg_to_send.planeID = -1;
    msg_to_send.type = MessageType::COLLISION_DETECTED;
    msg_to_send.dataSize = dataSize;
    std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);
    sendCollisionToDisplay(msg_to_send);

    */

}

bool ComputerSystem::checkAxes(msg_plane_info plane1, msg_plane_info plane2) {
    // COEN320 Task 3.4
    // Two planes collide if, at any second within the time constraint, their
    // separation on every axis is within the defined constraints.

    // Relative position and velocity of plane2 with respect to plane1
    double dx  = plane2.PositionX - plane1.PositionX;
    double dy  = plane2.PositionY - plane1.PositionY;
    double dz  = plane2.PositionZ - plane1.PositionZ;
    double dvx = plane2.VelocityX - plane1.VelocityX;
    double dvy = plane2.VelocityY - plane1.VelocityY;
    double dvz = plane2.VelocityZ - plane1.VelocityZ;

    // Advance the relative position one second at a time and look for a conflict
    for (int t = 0; t <= timeConstraintCollisionFreq; ++t) {
        double futureX = dx + dvx * t;
        double futureY = dy + dvy * t;
        double futureZ = dz + dvz * t;

        if (std::fabs(futureX) <= CONSTRAINT_X &&
            std::fabs(futureY) <= CONSTRAINT_Y &&
            std::fabs(futureZ) <= CONSTRAINT_Z) {
            return true;
        }
    }

    return false;
}


void ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg){
	int display_channel = name_open(display_channel_name, 0);
	if (display_channel == -1) {
	    name_close(display_channel);
	    return;
		// Amir: I removed it so it won't crash
		// throw std::runtime_error("Computer system: Error occurred while attaching to display");
	}
	int reply;

	int status = MsgSend(display_channel, &msg, sizeof(msg), &reply, sizeof(reply));
	if (status == -1) {
		perror("Computer system: Error occurred while sending message to display channel");
	}
}
