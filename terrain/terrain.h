
#ifndef PROCEDURALGENERATION_TERRAIN_H
#define PROCEDURALGENERATION_TERRAIN_H

#include <vector>
#include <glm/glm.hpp>

#include "../geometry.h"

class Terrain: public VertexModel {
    std::vector<Vertex> vertices;


    const float width = 20.f;
    const float halfWidth = width * .5f;
    const float depth = 20.f;
    const float halfDepth = depth * .5f;

    const float distanceBetweenPointsX = 0.1f;
    const float distanceBetweenPointsZ = 0.1f;

    const glm::vec3 lowColor = {0, 1, 0};
    const glm::vec3 highColor = {0, 1, 0};

    [[nodiscard]] inline float colourIntensityMap(float y) const;

public:
    Terrain();
};

#endif //PROCEDURALGENERATION_TERRAIN_H
