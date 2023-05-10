
#include "triangle.h"

TriangleModel::TriangleModel() {
    Vertex vertex{{}, {}};
    vertex.position = {-1, -1, 0};
    vertex.color = {1, 0, 0};
    this->vertices.push_back(vertex);

    vertex.position = {0, 1, 0};
    vertex.color = {1, 1, 1};
    this->vertices.push_back(vertex);

    vertex.position = {1, -1, 0};
    vertex.color = {0, 0, 1};
    this->vertices.push_back(vertex);

    this->triangles.push_back({this->vertices[0], this->vertices[1], this->vertices[2]});
}
