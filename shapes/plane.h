
#ifndef PROCEDURALGENERATION_PLANE_H
#define PROCEDURALGENERATION_PLANE_H

#include <vector>
#include "../geometry.h"

struct Plane: public VertexModel {
    std::vector<Vertex> vertices;
    Plane();
};

#endif //PROCEDURALGENERATION_PLANE_H
