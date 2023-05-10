//
// Created by Thomas Gandy on 08/05/2023.
//

#include <iostream>
#include <algorithm>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#include "camera.h"

void Camera::calculateWorldTransform() {
    if (this->lookingAtEngaged) {
        this->localToWorldTransform = glm::inverse(glm::lookAt(this->position, this->lookingAt, this->lookingAtUp));
        return;
    }

    auto transformationMatrix = glm::identity<glm::mat4x4>();
    transformationMatrix = glm::translate(transformationMatrix, this->position);
    transformationMatrix *= glm::mat4_cast(this->quaternionRotation);
//    transformationMatrix = glm::rotate(transformationMatrix, this->rotations.y, {0, 1, 0});
//    transformationMatrix = glm::rotate(transformationMatrix, this->rotations.x, {1, 0, 0});
//    transformationMatrix = glm::rotate(transformationMatrix, this->rotations.z, {0, 0, 1});


    this->localToWorldTransform = transformationMatrix;
}

void Camera::moveForward() {
    this->position += this->getForward() * this->keyMovementSensitivity;
}

void Camera::moveBack() {
    this->position -= this->getForward() * this->keyMovementSensitivity;
}

void Camera::moveLeft() {
    this->position -= this->getRight() * this->keyMovementSensitivity;
}

void Camera::moveRight() {
    this->position += this->getRight() * this->keyMovementSensitivity;
}

void Camera::moveUp() {
    this->position += this->getUp() * this->keyMovementSensitivity;
}

void Camera::moveDown() {
    this->position -= this->getUp() * this->keyMovementSensitivity;
}

void Camera::rotateLeft() {
    this->quaternionRotation *= glm::angleAxis(this->keyRotationIncrement, glm::vec3{0, 0, 1});
    this->rotations.z += this->keyRotationIncrement;
}

void Camera::rotateRight() {
    this->quaternionRotation *= glm::angleAxis(-this->keyRotationIncrement, glm::vec3{0, 0, 1});
    this->rotations.z -= this->keyRotationIncrement;
}

void Camera::toggleLookAt() {
    this->lookingAtEngaged = !this->lookingAtEngaged;
}

void Camera::update() {
    this->calculateWorldTransform();

    // w component 0 as translation not wanted
    this->currentForward = this->localToWorldTransform * glm::vec4(this->DEFAULT_FORWARD, 0);
    this->currentUp = this->localToWorldTransform * glm::vec4(this->DEFAULT_UP, 0);
    this->currentRight = glm::cross(this->currentUp, this->currentForward);
}

void Camera::lookAt(const glm::vec3 &positionToLookAt) {
    this->lookingAt = positionToLookAt;
}

void Camera::yaw(float degrees) {
    this->quaternionRotation *= glm::angleAxis(glm::radians(degrees), glm::vec3{0, 1, 0});
}

void Camera::pitch(float degrees) {
    this->quaternionRotation *= glm::angleAxis(glm::radians(degrees), glm::vec3{1, 0, 0});
}

void Camera::roll(float degrees) {
    this->quaternionRotation *= glm::angleAxis(glm::radians(degrees), glm::vec3{0, 0, 1});
}

const glm::mat4x4& Camera::getWorldTransform() const {
    return this->localToWorldTransform;
}

const glm::vec3 &Camera::getUp() const {
    return this->currentUp;
}

const glm::vec3 &Camera::getForward() const {
    return this->currentForward;
}

const glm::vec3 &Camera::getRight() const {
    return this->currentRight;
}

float Camera::getFOV() const {
    return this->fov;
}

float Camera::getFOVScale() const {
    return static_cast<float>(glm::tan(this->fov * 0.5));
}

void Camera::setFOV(float degrees) {
    degrees = std::max(std::min(179.f, degrees), 1.f);
    this->fov = glm::radians(degrees);
}
