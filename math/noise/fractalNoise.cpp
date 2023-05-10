//
// Created by Thomas Gandy on 08/04/2023.
//

#include "fractalNoise.h"


void FractalNoise::addOctavesIfNecessary() {
    auto seedDistributor = std::uniform_int_distribution<int>(0, octaveCount);
    for (auto i = octaves.size(); i < octaveCount; i++) {
        this->octaves.emplace_back(seedDistributor(this->octaveSeedGenerator));
    }
}

FractalNoise::FractalNoise(int seed) {
    octaveSeedGenerator = std::mt19937(seed);
    this->addOctavesIfNecessary();
}

float FractalNoise::evaluate(float x, float y) {
    auto totalAmplitude = 0.f;
    auto currentAmplitude = this->initialAmplitude;
    auto currentFrequency = this->initialFrequency;

    auto sum = 0.f;
    for (const auto& octaveNoise : octaves) {
        totalAmplitude += currentAmplitude;

        sum += octaveNoise.evaluate(x * currentFrequency, y * currentFrequency) * currentAmplitude;

        currentAmplitude *= this->persistence;
        currentFrequency *= this->lacunarity;
    }

    return sum - totalAmplitude * .5f;
}
