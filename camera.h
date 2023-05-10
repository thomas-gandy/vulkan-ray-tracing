//
// Created by Thomas Gandy on 08/05/2023.
//

#ifndef VULKANRAYTRACING_CAMERA_H
#define VULKANRAYTRACING_CAMERA_H

#include <glm/glm.hpp>
#include <glm/trigonometric.hpp>
#include <glm/detail/type_quat.hpp>

class Camera {
    float fov = glm::radians<float>(90);

    const glm::vec3 lookingAtUp = {0, 1, 0}; // Up used for looking at
    glm::vec3 lookingAt{};

    const glm::vec3 DEFAULT_FORWARD{0, 0, 1};
    const glm::vec3 DEFAULT_UP = {0, 1, 0};
    glm::vec3 currentForward{}, currentUp{}, currentRight{};
    glm::mat4x4 localToWorldTransform{};

    void calculateWorldTransform();

public:
    void moveForward();
    void moveBack();
    void moveLeft();
    void moveRight();
    void moveUp();
    void moveDown();
    void rotateLeft();
    void rotateRight();
    void toggleLookAt();

    Camera() = default;

    bool lookingAtEngaged = false; // true if should focus on object and not look around with mouse
    const float sensitivity = 0.003f;
    const float keyRotationIncrement = 0.05f;
    float keyMovementSensitivity = 0.1f;

    glm::vec3 position{};
    glm::vec3 rotations{0, 0, 0}; // Axes rotations (actual use of order may be different)
    glm::quat quaternionRotation{1.f, {0, 0, 0}};

    void update();
    void lookAt(const glm::vec3& positionToLookAt);

    void yaw(float degrees);
    void pitch(float degrees);
    void roll(float degrees);

    [[nodiscard]] const glm::mat4x4& getWorldTransform() const;
    [[nodiscard]] const glm::vec3& getUp() const;
    [[nodiscard]] const glm::vec3& getForward() const;
    [[nodiscard]] const glm::vec3& getRight() const;
    [[nodiscard]] float getFOV() const;
    [[nodiscard]] float getFOVScale() const;
    void setFOV(float degrees);
};

#endif //VULKANRAYTRACING_CAMERA_H
