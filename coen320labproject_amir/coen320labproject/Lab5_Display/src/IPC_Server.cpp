#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <sys/dispatch.h>
#include <semaphore.h>
#include <sys/mman.h>

#include "ATCTimer.h"
#include "Msg_structs.h"

#define SHM_NAME "/mammad_buffer"
#define SHM_SIZE sizeof(SharedMemory)
#define SEM_NAME "/mammad_sem"
#define MAX_MSG_SIZE 256


int shm_fd;
SharedMemory* shared_mem;
sem_t *sem;


struct CollisionPair {
  int planeA;
  int planeB;
};

typedef struct {
	char response[MAX_MSG_SIZE];
	int result;
} ServerReply;

typedef struct {
	Message_inter_process msg;
	struct _pulse pulse;
}	ServerRecvBuffer;



bool initializeSharedMemory() {

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


        sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
        if (sem == SEM_FAILED) {
        	perror("sem_open");
        	continue;
        }

        std::cout << "Shared memory initialized successfully." << std::endl;

        return true;
	}
}


void displayTopDownView(std::vector<msg_plane_info>& planes, double t,
                        int rows = 25, int cols = 50) {
    const double MIN_COORD = 0.0;
    const double MAX_COORD = 100000.0;
    const double SPAN = MAX_COORD - MIN_COORD;

    // Grid initialized with '.' for empty airspace
    std::vector<std::string> grid(rows, std::string(cols, '.'));

    for (const auto& p : planes) {

        // Use (t - p.arrivaTime) if motion begins upon arrival, or t if global clock
        double dt = t;
        double x = p.PositionX + p.VelocityX * dt;
        double y = p.PositionY + p.VelocityY * dt;

        // Map [0, 100000] to [0, cols - 1] and [0, rows - 1]
        int col = static_cast<int>(((x - MIN_COORD) / SPAN) * (cols - 1));
        int row = static_cast<int>(((y - MIN_COORD) / SPAN) * (rows - 1));

        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            // Use '*' if multiple planes land in the same console cell
            char marker = static_cast<char>('0' + (p.id % 10));
            if (grid[row][col] != '.' && grid[row][col] != marker) {
                grid[row][col] = '*';
            } else {
                grid[row][col] = marker;
            }
        }
    }

    std::cout << "\n=== Top-Down Airspace View (X-Y) at t = " << t << " ===\n";
    std::cout << "Y: 0 -> " << static_cast<int>(MAX_COORD)
              << " (top to bottom) | X: 0 -> " << static_cast<int>(MAX_COORD)
              << " (left to right)\n\n";

    std::cout << "+" << std::string(cols, '-') << "+\n";
    for (int r = 0; r < rows; ++r) {
        std::cout << "|" << grid[r] << "|\n";
    }
    std::cout << "+" << std::string(cols, '-') << "+\n";

    std::cout << "Legend: [1-9] Plane ID | [*] Overlap in cell | [.] Empty\n";
    for (const auto& p : planes) {
    	double x = p.PositionX + p.VelocityX * t;
        double y = p.PositionY + p.VelocityY * t;
        double z = p.PositionZ + p.VelocityZ * t;
        std::cout << "  Plane " << p.id << ": (" << std::fixed
                      << std::setprecision(0) << x << ", " << y << ", " << z
                      << ")\n";

    }
}


void waitForPlanes() {
	ATCTimer timer(1,0);
	sem_wait(sem);
	while (shared_mem->is_empty.load()) {
		sem_post(sem);
		std::cout << "Waiting for planes in airspace...\n";
		timer.waitTimer();
		sem_wait(sem);
	}
	sem_post(sem);
}


void printMemory() {

	sem_wait(sem);

	std::vector<msg_plane_info> plane_data_vector;
	uint64_t timestamp;

	if (shared_mem->is_empty.load()) {
		sem_post(sem);
		 return;
	} else {
		// Print separator and timestamp

	    // Display the timestamp in a readable format
		timestamp = shared_mem->timestamp;

		for (int i = 0; i < shared_mem->count; ++i) {
			const msg_plane_info& plane = shared_mem->plane_data[i];
				// Store the plane info in the vector
				plane_data_vector.push_back(plane);
		}
	}
	sem_post(sem);
	displayTopDownView(plane_data_vector, timestamp);
}



int main(void) {

	printf("Display Server: Starting...\n");

	name_attach_t *attach = name_attach(NULL, "mammad_display", 0);
	if (attach == NULL) {
		perror("Display Server: name_attach failed");
		return EXIT_FAILURE;
	}

	printf("Display Server: Channel created with ID %d. Waiting for clients...\n",
         attach->chid);


	ServerRecvBuffer buf;
	std::memset(&buf, 0, sizeof(buf));

	initializeSharedMemory();
	waitForPlanes();


  while (1) {

	  printMemory();
	  int rcvid = MsgReceive(attach->chid, &buf, sizeof(buf), NULL);

    Message_inter_process &msg = buf.msg;
    ServerReply reply;
    std::memset(&reply, 0, sizeof(reply));

    printf("Display Server: Received message from rcvid %d. Type: %d\n",
           rcvid, static_cast<int>(msg.type));

    switch (msg.type) {
      case MessageType::COLLISION_DETECTED: {
        if (msg.dataSize >= sizeof(CollisionPair)) {
          CollisionPair pair;
          std::memcpy(&pair, msg.data.data(), sizeof(pair));
          printf("Display Server: *** COLLISION WARNING: Aircraft %d and %d ***\n",
                 pair.planeA, pair.planeB);
        } else {
          printf("Display Server: *** COLLISION WARNING (incomplete payload) ***\n");
        }
        std::snprintf(reply.response, sizeof(reply.response),
                      "Collision alert received");
        break;
      }

      case MessageType::EXIT: {
        printf("Display Server: Client requested quit. Exiting.\n");
        std::snprintf(reply.response, sizeof(reply.response),
                      "Server shutting down.");
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        goto end_server_loop;
      }

      default: {
        printf("Display Server: Unknown message type %d\n",
               static_cast<int>(msg.type));
        std::snprintf(reply.response, sizeof(reply.response),
                      "Unknown message type.");
        break;
      }
    }

    MsgReply(rcvid, EOK, &reply, sizeof(reply));
  }

end_server_loop:
  printf("Display Server: Shutting down.\n");
  name_detach(attach, 0);
  return EXIT_SUCCESS;
}


