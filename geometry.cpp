//
// Created by Thomas Gandy on 08/05/2023.
//

#include <glm/gtc/quaternion.hpp>
#include "geometry.h"


bool operator==(const Triangle& lhs, const Triangle& rhs) {
    return lhs.v0.position == rhs.v0.position && lhs.v1.position == rhs.v1.position && lhs.v2.position == rhs.v2.position;
}

glm::mat4x4 Transform::getLocalToWorldTransform() const {
    glm::mat4x4 result;
    result = glm::translate(glm::mat4x4(1.f), this->position);
    result *= glm::mat4_cast(this->quaternionRotation);
    result = glm::scale(result, glm::vec3(this->scale));

    return result;
}

void VertexModel::precomputeTriangles() {
    this->precomputedWorldTriangles = {};
    const auto localToWorldTransform = this->transform.getLocalToWorldTransform();
    for (auto triangle : this->triangles) {
        triangle.v0.position = localToWorldTransform * glm::vec4(triangle.v0.position, 1);
        triangle.v1.position = localToWorldTransform * glm::vec4(triangle.v1.position, 1);
        triangle.v2.position = localToWorldTransform * glm::vec4(triangle.v2.position, 1);

        this->precomputedWorldTriangles.push_back(triangle);
    }
}

void VertexModel::yaw(float degrees) {
    this->transform.quaternionRotation = glm::angleAxis(glm::radians(degrees), glm::vec3(0, 1, 0)) * this->transform.quaternionRotation;
}

