include "Display.h"
#include <iostream>

int main() {
    std::cout << "Display System: Initializing..." << std::endl;

    Display display;

    if (display.startServer()) {
        std::cout << "Display System: Ready and waiting for data..." << std::endl;

        // Keep the server running
        display.joinThreads();

        std::cout << "Display System: Shutting down..." << std::endl;
    } else {
        std::cerr << "Display System: Failed to start server." << std::endl;
        return 1;
    }

    return 0;
}
