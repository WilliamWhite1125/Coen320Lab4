#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <semaphore.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>
#include <time.h>
#include <unistd.h>

#include "Msg_structs.h"

#define SHM_NAME "/mammad_buffer"
#define SHM_SIZE sizeof(SharedMemory)
#define SEM_NAME "/mammad_sem"

// Display refresh rate and how long a collision warning stays on screen.
#define REFRESH_SECONDS 1
#define WARNING_TTL_SECONDS 5

// Code used by the periodic redraw pulse (delivered on the display channel).
#define TIMER_PULSE_CODE (_PULSE_CODE_MINAVAIL + 1)

int shm_fd = -1;
SharedMemory* shared_mem = nullptr;
sem_t* sem = SEM_FAILED;

// Must stay layout-compatible with the Computer's std::pair<int, int> payload.
struct CollisionPair {
  int planeA;
  int planeB;
};

static_assert(sizeof(CollisionPair) == 2 * sizeof(int),
              "CollisionPair must be two packed ints to match the sender");

struct ActiveWarning {
  int planeA;
  int planeB;
  std::chrono::steady_clock::time_point expires;
};

static std::vector<ActiveWarning> activeWarnings;

union ReceiveBuffer {
  Message_inter_process msg;
  struct _pulse pulse;
};

bool initializeSharedMemory() {
  bool reported = false;

  while (true) {
    // The Radar owns (creates) the shared memory and semaphore, so the display
    // only ever reads. Open read-only.
    shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
      if (!reported) {
        std::cerr << "Display: waiting for radar shared memory (" SHM_NAME
                  << ")...\n";
        reported = true;
      }
      sleep(1);
      continue;
    }

    shared_mem = static_cast<SharedMemory*>(
        mmap(NULL, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0));
    if (shared_mem == MAP_FAILED) {
      perror("Display: mmap");
      shared_mem = nullptr;
      close(shm_fd);
      shm_fd = -1;
      sleep(1);
      continue;
    }

    sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED) {
      perror("Display: sem_open");
      munmap(shared_mem, SHM_SIZE);
      shared_mem = nullptr;
      close(shm_fd);
      shm_fd = -1;
      sleep(1);
      continue;
    }

    std::cout << "Display: attached to radar shared memory.\n";
    return true;
  }
}

void recordCollision(int planeA, int planeB) {
  const auto expiry = std::chrono::steady_clock::now() +
                      std::chrono::seconds(WARNING_TTL_SECONDS);

  // Refresh an existing warning instead of duplicating it.
  for (auto& warning : activeWarnings) {
    if ((warning.planeA == planeA && warning.planeB == planeB) ||
        (warning.planeA == planeB && warning.planeB == planeA)) {
      warning.expires = expiry;
      return;
    }
  }

  activeWarnings.push_back({planeA, planeB, expiry});
}

void pruneWarnings() {
  const auto now = std::chrono::steady_clock::now();
  activeWarnings.erase(
      std::remove_if(activeWarnings.begin(), activeWarnings.end(),
                     [now](const ActiveWarning& warning) {
                       return warning.expires <= now;
                     }),
      activeWarnings.end());
}

void displayTopDownView(const std::vector<msg_plane_info>& planes, uint64_t t,
                        std::ostream& out, int rows = 30, int cols = 70) {
  const double MIN_COORD = 0.0;
  const double MAX_COORD = 100000.0;
  const double SPAN = MAX_COORD - MIN_COORD;

  std::vector<std::string> grid(rows, std::string(cols, '.'));

  for (const auto& plane : planes) {
    // Radar already reports the current position, so use it directly.
    // Do NOT add velocity * t again.
    const int col =
        static_cast<int>(((plane.PositionX - MIN_COORD) / SPAN) * (cols - 1));
    // Row 0 is printed at the top, so invert Y to draw the usual orientation.
    const int row =
        (rows - 1) -
        static_cast<int>(((plane.PositionY - MIN_COORD) / SPAN) * (rows - 1));

    if (row < 0 || row >= rows || col < 0 || col >= cols) {
      continue;  // outside the drawn airspace window
    }

    const int symbol = ((plane.id % 36) + 36) % 36;
    const char marker = (symbol < 10)
                            ? static_cast<char>('0' + symbol)
                            : static_cast<char>('A' + (symbol - 10));

    if (grid[row][col] != '.' && grid[row][col] != marker) {
      grid[row][col] = '*';  // multiple aircraft share this cell
    } else {
      grid[row][col] = marker;
    }
  }

  out << "\n=== ATC Display - Top-Down Airspace (X-Y) | t = " << t << " s | "
      << planes.size() << " aircraft ===\n";

  // Collision warnings: prominent, red, and they persist for a few seconds.
  if (activeWarnings.empty()) {
    out << "No active collision warnings.\n";
  } else {
    out << "\033[1;31m";
    for (const auto& warning : activeWarnings) {
      out << "*** COLLISION WARNING: AIRCRAFT " << warning.planeA << " AND "
          << warning.planeB << " ***\n";
    }
    out << "\033[0m";
  }

  out << "Y: 0 -> " << static_cast<int>(MAX_COORD) << " (bottom -> top)"
      << " | X: 0 -> " << static_cast<int>(MAX_COORD) << " (left -> right)\n\n";

  out << "+" << std::string(cols, '-') << "+\n";
  for (int r = 0; r < rows; ++r) {
    out << "|" << grid[r] << "|\n";
  }
  out << "+" << std::string(cols, '-') << "+\n";

  out << "Legend: [0-9A-Z] aircraft ID | [*] overlap in cell | [.] empty\n";

  if (planes.empty()) {
    out << "  (airspace is empty)\n";
    return;
  }

  out << std::fixed << std::setprecision(0);
  for (const auto& plane : planes) {
    out << "  Aircraft " << std::setw(2) << plane.id << ": pos ("
        << std::setw(7) << plane.PositionX << ", " << std::setw(7)
        << plane.PositionY << ", " << std::setw(6) << plane.PositionZ
        << ")  vel (" << plane.VelocityX << ", " << plane.VelocityY << ", "
        << plane.VelocityZ << ")\n";
  }
}

void printMemory() {
  if (shared_mem == nullptr || sem == SEM_FAILED) {
    return;
  }

  std::vector<msg_plane_info> plane_data_vector;
  uint64_t timestamp = 0;

  // Copy the snapshot while holding the semaphore, then render unlocked.
  sem_wait(sem);
  timestamp = shared_mem->timestamp;
  if (!shared_mem->is_empty.load()) {
    const int count =
        std::max(0, std::min(shared_mem->count,
                             static_cast<int>(sizeof(shared_mem->plane_data) /
                                              sizeof(shared_mem->plane_data[0]))));
    plane_data_vector.assign(shared_mem->plane_data,
                             shared_mem->plane_data + count);
  }
  sem_post(sem);

  pruneWarnings();

  std::ostringstream frame;
  displayTopDownView(plane_data_vector, timestamp, frame);

  // Redraw in place instead of scrolling raw prints.
  std::cout << "\033[2J\033[H" << frame.str() << std::flush;
}

int main(void) {
  printf("Display Server: Starting...\n");

  name_attach_t* attach = name_attach(NULL, "mammad_display", 0);
  if (attach == NULL) {
    perror("Display Server: name_attach failed");
    return EXIT_FAILURE;
  }

  printf("Display Server: Channel created with ID %d. Waiting for clients...\n",
         attach->chid);

  initializeSharedMemory();

  // Periodic redraw pulse delivered to our own channel.
  struct sigevent event;
  SIGEV_PULSE_INIT(&event, attach->coid, SIGEV_PULSE_PRIO_INHERIT,
                   TIMER_PULSE_CODE, 0);

  timer_t refreshTimer;
  if (timer_create(CLOCK_MONOTONIC, &event, &refreshTimer) == -1) {
    perror("Display Server: timer_create failed");
    name_detach(attach, 0);
    return EXIT_FAILURE;
  }

  struct itimerspec refreshSpec;
  std::memset(&refreshSpec, 0, sizeof(refreshSpec));
  refreshSpec.it_value.tv_sec = REFRESH_SECONDS;
  refreshSpec.it_interval.tv_sec = REFRESH_SECONDS;

  if (timer_settime(refreshTimer, 0, &refreshSpec, NULL) == -1) {
    perror("Display Server: timer_settime failed");
  }

  // Draw one frame immediately so the operator sees the airspace right away.
  printMemory();

  bool running = true;
  ReceiveBuffer buf;

  while (running) {
    std::memset(&buf, 0, sizeof(buf));

    const int rcvid = MsgReceive(attach->chid, &buf, sizeof(buf), NULL);
    if (rcvid == -1) {
      perror("Display Server: MsgReceive failed");
      continue;
    }

    // rcvid == 0 means a pulse was received (not a client message).
    if (rcvid == 0) {
      if (buf.pulse.code == TIMER_PULSE_CODE) {
        printMemory();
      }
      continue;
    }

    bool redraw = false;
    int reply = EOK;

    switch (buf.msg.type) {
      case MessageType::COLLISION_DETECTED: {
        // The payload is a list of plane pairs; read every one of them.
        const size_t payloadSize = std::min(buf.msg.dataSize, buf.msg.data.size());
        const size_t pairCount = payloadSize / sizeof(CollisionPair);

        if (pairCount == 0) {
          printf("Display Server: *** COLLISION WARNING (empty payload) ***\n");
        } else {
          for (size_t i = 0; i < pairCount; ++i) {
            CollisionPair pair;
            std::memcpy(&pair,
                        buf.msg.data.data() + i * sizeof(CollisionPair),
                        sizeof(pair));
            recordCollision(pair.planeA, pair.planeB);
          }
          redraw = true;
        }
        break;
      }

      case MessageType::EXIT: {
        printf("Display Server: Client requested quit. Exiting.\n");
        running = false;
        break;
      }

      default: {
        // Ignore unrelated message types without spamming the display.
        break;
      }
    }

    // Reply with an int: this matches the 4-byte reply buffer used by
    // ComputerSystem::sendCollisionToDisplay().
    MsgReply(rcvid, EOK, &reply, sizeof(reply));

    if (redraw) {
      printMemory();
    }
  }

  timer_delete(refreshTimer);
  name_detach(attach, 0);

  if (shared_mem != nullptr) {
    munmap(shared_mem, SHM_SIZE);
    shared_mem = nullptr;
  }
  if (shm_fd != -1) {
    close(shm_fd);
    shm_fd = -1;
  }
  if (sem != SEM_FAILED) {
    sem_close(sem);
    sem = SEM_FAILED;
  }

  printf("Display Server: Shutting down.\n");
  return EXIT_SUCCESS;
}