//
// Created by Thomas Gandy on 22/02/2023.
//

#ifndef PROCEDURALGENERATION_PLYFILEHANDLER_H
#define PROCEDURALGENERATION_PLYFILEHANDLER_H

#include <string>
#include "plyHelper.h"
#include "../../../geometry.h"

struct PlyVertex {
    float x, y, z;
};

struct PlyFace {
    unsigned char numberOfVertices;
    int* vertices;
};

class PlyFileHandler {
    std::vector<Vertex> vertices{};
    PlyProperty vertexProperties[3] = { /* list of property information for a vertex */
            {(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex, x), 0, 0, 0, 0},
            {(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex, y), 0, 0, 0, 0},
            {(char*)"z", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex, z), 0, 0, 0, 0},
    };

    PlyProperty faceProperties[1] = { /* list of property information for a vertex */
            {(char*)"vertex_indices", PLY_INT, PLY_INT, offsetof(PlyFace, vertices), 1, PLY_UCHAR, PLY_UCHAR, offsetof(PlyFace, numberOfVertices)},
    };

public:
    std::vector<Triangle> triangles{};

    explicit PlyFileHandler(const std::string& filepath);
    [[nodiscard]] VertexModel getVertexModel() const;
};

#endif //PROCEDURALGENERATION_PLYFILEHANDLER_H
