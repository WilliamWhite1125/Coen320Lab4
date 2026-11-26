#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include <cstring> // For memcpy

// COEN320 Task 3.1, set the display channel name (it should be the same as the Coen320_Lab (Task0) Display channel name)
#define display_channel_name "mammad_display"


ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
	// Open the shared memory object
	while (true) {
        // COEN320 Task 3.2
		// Attempt to open the shared memory object (You need to use the same name as Task 2 in Radar)
        // In case of error, retry until successful
        const char* shm_name = SHARED_MEMORY_NAME;
        shm_fd = shm_open(shm_name, O_RDONLY, 0666);

        if (shm_fd == -1) {
            std::cerr << "Waiting for shared memory to be created by Radar..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        // COEN320 Task 3.3
		// Map the shared memory object into the process's address space
        // The shared memory should be mapped to "shared_mem" (check for errors)

        shared_mem = (SharedMemory*)mmap(NULL, sizeof(SharedMemory), PROT_READ, MAP_SHARED, shm_fd, 0);

        if (shared_mem == MAP_FAILED) {
            perror("mmap failed");
            close(shm_fd);
            shm_fd = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
    if (initializeSharedMemory()) {
        running = true; // will be used in monitorAirspace
        std::cout << "Starting monitoring thread." << std::endl;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);
        return true;
    } else {
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }
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
    // Keep monitoring indefinitely until `stopMonitoring` is called
	while (shared_mem->is_empty.load()) {
		std::cout << "Waiting for planes in airspace...\n";
		timer.waitTimer();
	}

	while (running) {
		if (shared_mem->is_empty.load()) {
			std::cout << "No planes in airspace. Stopping monitoring.\n";
			running = false;
	        break;
        } else {
        	plane_data_vector.clear();
        	// Print separator and timestamp
            //std::cout << "\n================= Shared Memory Update =================\n";

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
		//**************Call Collision Detector*********************
		if (plane_data_vector.size()>1)
            checkCollision(timestamp, plane_data_vector);
		else
            std::cout << "No collision possible with single plane\n";
        // Sleep for a short interval before the next poll
       timer.waitTimer();
    }
	std::cout << "Exiting monitoring loop." << std::endl;
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
    // COEN320 Task 3.4 - Implementation
    // A collision is defined as two planes entering the defined airspace constraints within the time constraint
    // Check if plane1 and plane2 will collide within the time constraint using their velocities

    // Calculate the distance between the two planes
    double dx = plane2.PositionX - plane1.PositionX;
    double dy = plane2.PositionY - plane1.PositionY;
    double dz = plane2.PositionZ - plane1.PositionZ;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Define a collision threshold (minimum safe separation)
    const double COLLISION_THRESHOLD = 5000.0; // 5 km minimum separation

    // Check if current distance is within collision threshold
    if (distance < COLLISION_THRESHOLD) {
        return true; // Collision detected
    }

    // Predict future positions based on velocities
    // Calculate relative velocity
    double relative_vx = plane2.VelocityX - plane1.VelocityX;
    double relative_vy = plane2.VelocityY - plane1.VelocityY;
    double relative_vz = plane2.VelocityZ - plane1.VelocityZ;

    // Check if planes are converging (moving towards each other)
    // Using dot product of relative position and relative velocity
    double dot_product = dx * relative_vx + dy * relative_vy + dz * relative_vz;

    // If dot product is negative, planes are approaching
    if (dot_product < 0) {
        // Check predicted collision by calculating closest approach distance
        // For simplicity, check if they will be within constraints in near future
        double future_x1 = plane1.PositionX + plane1.VelocityX;
        double future_y1 = plane1.PositionY + plane1.VelocityY;
        double future_z1 = plane1.PositionZ + plane1.VelocityZ;

        double future_x2 = plane2.PositionX + plane2.VelocityX;
        double future_y2 = plane2.PositionY + plane2.VelocityY;
        double future_z2 = plane2.PositionZ + plane2.VelocityZ;

        double future_dx = future_x2 - future_x1;
        double future_dy = future_y2 - future_y1;
        double future_dz = future_z2 - future_z1;

        double future_distance = std::sqrt(future_dx * future_dx + future_dy * future_dy + future_dz * future_dz);

        if (future_distance < COLLISION_THRESHOLD) {
            return true; // Collision predicted
        }
    }

    return false; // No collision detected
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

    // Close the channel
    name_close(display_channel);
}
