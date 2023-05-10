
#include <iostream>

#include "terrain.h"
#include "../math/noise/fractalNoise.h"
#include "../math/noise/utility.h"

Terrain::Terrain() {
    FractalNoise fractalNoise;
    PerlinNoise perlinNoise;

    // Generate vertices
    auto verticesAlongX = static_cast<int>(this->width / this->distanceBetweenPointsX);
    auto verticesAlongZ = static_cast<int>(this->depth / this->distanceBetweenPointsZ);

    // Coordinate start values
    auto startX = -halfWidth;
    auto startZ = -halfDepth;

    auto currentX = startX;
    for (int x = 0; x < verticesAlongX; x++) {
        auto currentZ = startZ;
        for (int z = 0; z < verticesAlongZ; z++) {
            const auto y = fractalNoise.evaluate(currentX, currentZ);

//            const auto color = glm::vec3{1, 1, 1};
            const auto ySmoothed = smoothstep(glm::clamp(y, 0.0f, 1.0f));
            const auto color = glm::vec3{1, 1, 1};
//            const auto color = lerp(ySmoothed, this->lowColor, this->highColor);
            const auto newVertex = Vertex{{currentX, y, currentZ}, color};
            this->vertices.push_back(newVertex);

            currentZ += this->distanceBetweenPointsZ;
        }
        currentX += this->distanceBetweenPointsX;
    }

    // Create indices
    for (int w = 0; w < verticesAlongX - 1; w++) {
        for (int d = 0; d < verticesAlongZ - 1; d++) {
            auto v0 = w * verticesAlongZ + d;
            auto v1 = v0 + verticesAlongZ + 1;
            auto v2 = v1 - 1;
            this->triangles.push_back({this->vertices[v0], this->vertices[v1], this->vertices[v2]});
//            this->indices.push_back({v0, v1, v2});

            v2 = v1;
            v1 = v0 + 1;
            this->triangles.push_back({this->vertices[v0], this->vertices[v1], this->vertices[v2]});
//            this->indices.push_back({v0, v1, v2});
        }
    }
}

float Terrain::colourIntensityMap(float y) const {
    return y*y;
}
