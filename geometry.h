//
// Created by Thomas Gandy on 07/05/2023.
//

#ifndef VULKANRAYTRACING_GEOMETRY_H
#define VULKANRAYTRACING_GEOMETRY_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/detail/type_quat.hpp>


struct Vertex {
    glm::vec3 position;
    alignas(16) glm::vec3 color;
};

struct Triangle {
    Vertex v0;
    Vertex v1;
    Vertex v2;
};
bool operator==(const Triangle& lhs, const Triangle& rhs);

struct LocationBuffer {
    alignas(16) glm::mat4x4 cameraToWorld;
    alignas(16) glm::vec3 location;
};

struct PointLight {
    alignas(16) glm::vec3 position;
    alignas(16) glm::vec3 color;
    alignas (4) float intensity;
};

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

struct ArcballSphere {
    glm::vec3 center;
    float radius;
};

struct Transform {
    glm::vec3 position{};
    glm::vec3 eulerRotation{0, 0, 0};
    glm::quat quaternionRotation{1.f, {0, 0, 0}};
    float scale = 1.f;

    [[nodiscard]] glm::mat4x4 getLocalToWorldTransform() const;
};

struct VertexModel {
    std::vector<Triangle> precomputedWorldTriangles;
    std::vector<Triangle> triangles;
    Transform transform;

    void precomputeTriangles();
    void yaw(float degrees);
};


#endif //VULKANRAYTRACING_GEOMETRY_H
