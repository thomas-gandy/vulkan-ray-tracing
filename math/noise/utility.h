//
// Created by Thomas Gandy on 29/01/2023.
//

#ifndef PROCEDURALGENERATION_UTILITY_H
#define PROCEDURALGENERATION_UTILITY_H

template <typename T>
[[maybe_unused]] inline T lerp(float t, const T& a, const T& b) {
    return a * (1 - t) + b * t;
}

[[maybe_unused]] inline float smoothstep(float t) {
    return t * t * (3 - 2 * t);
}

#endif //PROCEDURALGENERATION_UTILITY_H
