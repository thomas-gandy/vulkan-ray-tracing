
#include "plane.h"


Plane::Plane() {
    this->vertices.push_back({{-1, 0, -1}, {1, 1, 1}});
    this->vertices.push_back({{-1, 0, 1}, {1, 1, 1}});
    this->vertices.push_back({{1, 0, 1}, {1, 1, 1}});

    this->vertices.push_back({{-1, 0, -1}, {1, 1, 1}});
    this->vertices.push_back({{1, 0, 1}, {1, 1, 1}});
    this->vertices.push_back({{1, 0, -1}, {1, 1, 1}});

    this->triangles.push_back({this->vertices[0], this->vertices[1], this->vertices[2]});
    this->triangles.push_back({this->vertices[3], this->vertices[4], this->vertices[5]});
}
