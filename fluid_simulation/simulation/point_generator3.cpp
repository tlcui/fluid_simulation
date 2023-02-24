#include "point_generator3.h"

PointGenerator3::PointGenerator3() {
}

PointGenerator3::~PointGenerator3() {
}

void PointGenerator3::generate(
    const BoundingBox3D& boundingBox,
    double spacing,
    std::vector<Vector3D>* points) const {
    forEachPoint(
        boundingBox,
        spacing,
        [&points](const Vector3D& point) {
            points->emplace_back(point);
            return true;
        });
}