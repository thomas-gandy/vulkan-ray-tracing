//
// Created by Thomas Gandy on 08/04/2023.
//

#ifndef PROCEDURALGENERATION_PERLINNOISE_H
#define PROCEDURALGENERATION_PERLINNOISE_H

#include <vector>
#include <random>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>


class PerlinNoise {
    static constexpr int TABLE_SIZE = 2 << 8;
    static constexpr int TABLE_MASK = TABLE_SIZE - 1; // Used as alternative to modulo for faster operation

    std::vector<glm::vec2> randomGradients;
    std::vector<int> permutations;

    [[nodiscard]] inline int getPermutationOffset(int latticeX, int latticeY) const;

public:
    explicit PerlinNoise(int seed = 2023);

    [[nodiscard]] float evaluate(float x, float y) const;
};


#endif //PROCEDURALGENERATION_PERLINNOISE_H
