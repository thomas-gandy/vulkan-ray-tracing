#include <iostream>
#include "vulkanApplication.h"


int main() {
    try {
        VulkanApplication app{"Ray Tracer"};
        app.start();
    } catch (std::exception& e) {
        std::cout << "Thrown Exception: " << e.what() << std::endl;
    }

    return 0;
}
