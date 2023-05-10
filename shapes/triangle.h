//
// Created by Thomas Gandy on 14/03/2023.
//

#ifndef PROCEDURALGENERATION_TRIANGLE_H
#define PROCEDURALGENERATION_TRIANGLE_H

#include "../geometry.h"

struct TriangleModel: public VertexModel {
    std::vector<Vertex> vertices;
    TriangleModel();
};

#endif //PROCEDURALGENERATION_TRIANGLE_H
