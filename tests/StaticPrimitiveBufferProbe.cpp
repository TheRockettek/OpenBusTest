#include "CoreRenderer.h"

#include <iostream>

int main() {
    openbus::rendering::StaticPrimitiveBuffer cache;
    if (!cache.requiresUpload(18, 7)) {
        std::cerr << "An uninitialized primitive buffer must upload its first contents\n";
        return 1;
    }

    cache.buffer = 1;
    cache.vertexCount = 18;
    cache.contentRevision = 7;
    if (cache.requiresUpload(18, 7)) {
        std::cerr << "An unchanged primitive buffer should remain cached\n";
        return 1;
    }
    if (!cache.requiresUpload(18, 8)) {
        std::cerr << "Same-sized geometry with a new revision must be uploaded\n";
        return 1;
    }
    if (!cache.requiresUpload(21, 7)) {
        std::cerr << "A changed vertex count must be uploaded\n";
        return 1;
    }
    return 0;
}