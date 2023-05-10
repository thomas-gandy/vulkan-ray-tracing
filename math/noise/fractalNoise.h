//
// Created by Thomas Gandy on 08/04/2023.
//

#ifndef PROCEDURALGENERATION_FRACTALNOISE_H
#define PROCEDURALGENERATION_FRACTALNOISE_H

#include <vector>
#include "perlinNoise.h"


class FractalNoise {
    float initialFrequency = 1;
    float initialAmplitude = 2.5;
    float lacunarity = 1.2f;
    float persistence = 0.5f;
    int octaveCount = 5;
    std::mt19937 octaveSeedGenerator;
    std::vector<PerlinNoise> octaves{};

    void addOctavesIfNecessary();

public:
    explicit FractalNoise(int seed = 2023);
    float evaluate(float x, float y);
};


#endif //PROCEDURALGENERATION_FRACTALNOISE_H
