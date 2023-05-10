//
// Created by Thomas Gandy on 08/04/2023.
//

#include <iostream>
#define GLM_FORCE_RADIANS
#include <glm/ext/scalar_constants.hpp>

#include "perlinNoise.h"
#include "utility.h"


PerlinNoise::PerlinNoise(int seed) {
    auto randomEngine = std::mt19937(seed);

    // Create random unit vectors and initialise ordered permutation table
    auto realGenerator = std::uniform_real_distribution<float>();
    for (int i = 0; i < TABLE_SIZE; i++) {
        const auto theta = realGenerator(randomEngine) * glm::pi<float>() * 2;
        const auto x = glm::cos(theta);
        const auto y = glm::sin(theta);
        this->randomGradients.push_back(glm::normalize(glm::vec2(x, y)));
        this->permutations.push_back(i);
    }

    // Randomise permutation table
    auto intGenerator = std::uniform_int_distribution<int> (0, TABLE_SIZE - 1);
    for (int i = 0; i < TABLE_SIZE; i++) {
        const auto swapIndex = intGenerator(randomEngine);
        std::swap(this->permutations[i], this->permutations[swapIndex]);
    }
}

int PerlinNoise::getPermutationOffset(int latticeX, int latticeY) const {
    return this->permutations[(this->permutations[latticeX & TABLE_MASK] + latticeY) & TABLE_MASK];
}

float PerlinNoise::evaluate(float x, float y) const {
    const auto p = glm::vec2(x, y);
    const auto xt = smoothstep(x - std::floor(x));
    const auto yt = smoothstep(y - std::floor(y));

    // Lattice Points
    const auto blp = glm::floor(p);
    const auto brp = blp + glm::vec2(1, 0);
    const auto tlp = blp + glm::vec2(0, 1);
    const auto trp = blp + glm::vec2(1, 1);

    // Lattice Point Gradients
    const auto tlg = this->randomGradients[this->getPermutationOffset((int)tlp.x, (int)tlp.y)];
    const auto trg = this->randomGradients[this->getPermutationOffset((int)trp.x, (int)trp.y)];
    const auto blg = this->randomGradients[this->getPermutationOffset((int)blp.x, (int)blp.y)];
    const auto brg = this->randomGradients[this->getPermutationOffset((int)brp.x, (int)brp.y)];

    // Vectors From Lattice Point to Point
    const auto tlpToP = p - tlp;
    const auto trpToP = p - trp;
    const auto blpToP = p - blp;
    const auto brpToP = p - brp;

    // Dot Product Between Lattice Point Gradients and Vector From Lattice Point to Point
    const auto tld = (glm::dot(tlpToP, tlg) + 1) * .5f; // Map from range [-1, 1] to [0, 1]
    const auto trd = (glm::dot(trpToP, trg) + 1) * .5f;
    const auto bld = (glm::dot(blpToP, blg) + 1) * .5f;
    const auto brd = (glm::dot(brpToP, brg) + 1) * .5f;

    // Two horizontal linear interpolations
    const auto thl = lerp(xt, tld, trd);
    const auto bhl = lerp(xt, bld, brd);

    // Return vertical interpolation between two horizontal interpolations
    return lerp(yt, bhl, thl);
}
