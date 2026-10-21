#include "app/vehicle/VehicleDrawSorting.h"

#include <iostream>
#include <vector>

namespace {

struct TestVertex {
    float x;
    float y;
    float z;
    int triangle;
};

struct TestBatch {
    double depth;
    int renderType;
    int id;
};

} // namespace

int main() {
    std::vector<TestBatch> batches = {
        {2.0, 3, 1}, {4.0, 2, 2}, {4.0, 1, 3}, {4.0, 1, 4}, {1.0, 0, 5}};
    openbus::app::vehicle::sortTransparentBatches(batches);
    const std::vector<int> expectedBatchOrder = {3, 4, 2, 1, 5};
    for (std::size_t index = 0; index < batches.size(); ++index) {
        if (batches[index].id != expectedBatchOrder[index]) {
            std::cerr << "transparent batches were not sorted back-to-front with stable "
                         "render-type ties\n";
            return 1;
        }
    }

    std::vector<TestVertex> triangles = {
        {0, 0, -1, 1}, {1, 0, -1, 1}, {0, 1, -1, 1},
        {0, 0, -3, 2}, {1, 0, -3, 2}, {0, 1, -3, 2},
        {0, 0, -2, 3}, {1, 0, -2, 3}, {0, 1, -2, 3},
        {0, 0, -3, 4}, {1, 0, -3, 4}, {0, 1, -3, 4}};
    const std::vector<std::size_t> triangleOrder =
        openbus::app::vehicle::transparentTriangleOrder(
            triangles, openbus::rendering::identityMatrix());
    const std::vector<std::size_t> expectedTriangleOrder = {3, 9, 6, 0};
    if (triangleOrder != expectedTriangleOrder) {
        std::cerr << "transparent triangles were not sorted far-to-near with stable ties\n";
        return 1;
    }
    return 0;
}
