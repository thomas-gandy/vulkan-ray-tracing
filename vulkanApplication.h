//
// Created by Thomas Gandy on 29/04/2023.
//

#ifndef VULKANRAYTRACING_VULKANAPPLICATION_H
#define VULKANRAYTRACING_VULKANAPPLICATION_H

#include <string>
#include <vulkan/vulkan.hpp>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "geometry.h"
#include "camera.h"
#include "acceleration/kdTree.h"


class VulkanApplication {
    std::string applicationName;

    const uint32_t windowWidth = 1000;
    const uint32_t windowHeight = 750;
    int framebufferWidth;
    int framebufferHeight;
    bool applicationRunning = false;
    GLFWwindow* window = nullptr;

    uint32_t framesInFlight = 3;
    uint32_t maxSetsToAllocateForSingleFrame{};
    // Taking into account frame duplication
    uint32_t maxSetsToAllocateAcrossFrames{};
    int imageSetIndex = 0;
    int matrixSetIndex = 1;
    int triangleSetIndex = 2;
    int pointLightSetIndex = 3;
    vk::SurfaceKHR surface;
    vk::Instance instance;
    VkDebugUtilsMessengerEXT debugMessenger{};
    vk::PhysicalDevice physicalDevice;
    vk::Device logicalDevice;
    uint32_t computePresentationQueueFamilyIndex{};
    vk::Queue computePresentationQueue;

    vk::DeviceSize locationBufferSize{};
    std::vector<vk::Buffer> locationBuffers;
    std::vector<vk::DeviceMemory> locationBuffersMemory;

    vk::DeviceSize triangleBufferSize{};
    std::vector<vk::Buffer> triangleBuffers;
    std::vector<vk::DeviceMemory> triangleBuffersMemory;

    vk::DeviceSize pointLightBufferSize{};
    std::vector<vk::Buffer> pointLightBuffers;
    std::vector<vk::DeviceMemory> pointLightBuffersMemory;

    std::vector<vk::Image> frameImages;
    std::vector<vk::ImageView> frameImageViews;
    std::vector<vk::DeviceMemory> frameImageMemory;

    vk::SwapchainKHR swapchain;
    vk::Extent2D swapchainExtent;
    vk::SurfaceFormatKHR swapchainSurfaceFormat;
    std::vector<vk::Image> swapchainImages;
    vk::CommandPool commandPool;
    std::vector<vk::CommandBuffer> computeCommandBuffers;
    std::vector<vk::CommandBuffer> presentationCommandBuffers;
    std::vector<vk::DescriptorSetLayout> descriptorSetLayouts;
    // Taking into account frame duplication
    std::vector<vk::DescriptorSetLayout> framesDescriptorSetLayouts;
    vk::PipelineLayout computePipelineLayout;
    vk::Pipeline computePipeline;
    vk::DescriptorPool descriptorPool;
    std::vector<vk::DescriptorSet> descriptorSets;
    std::vector<vk::Semaphore> frameSemaphores;
    std::vector<vk::Fence> frameFences;
    std::vector<vk::Semaphore> swapchainSemaphores;
    std::vector<vk::Fence> swapchainFences;

    Camera camera;
    double lastMouseX = 0, lastMouseY = 0;
    bool mouseDragging = false;
    glm::quat startArcballPosition{};
    glm::quat currentArcballPosition{};
    bool presentationMode = false;
    bool presentationModeReleased = true;

    int objectIndexOfFocus = 1;
    glm::quat initialRotationOfObject{};


    KDTree kdTree;
    std::vector<Triangle> triangles;
    std::vector<PointLight> pointLights;
    std::vector<VertexModel> vertexModels;
    float t = 0;


    template <typename T>
    static void output(const T& container, std::ostream& os = std::cout);
    template <typename T>
    static bool isSupersetOf(const std::vector<T>& superset, const std::vector<T>& subset);
    static std::vector<uint32_t> getShaderCode(const std::string& filepath);
    static std::vector<const char*> convertStringVectorToCStr(const std::vector<std::string>& strings);
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
            VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
            VkDebugUtilsMessageTypeFlagsEXT messageType,
            const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
            void* pUserData
    );

    uint32_t getBestMemoryTypeIndexForAllocation(uint32_t supportedTypeIndexesBitField, vk::MemoryPropertyFlags additionalMemoryPropertyFlags);

    void createWindow();
    void createInstance();
    static void ensureLayersAreSupported(const std::vector<std::string>& requiredLayerNames);
    static void ensureInstanceExtensionsAreSupported(const std::vector<std::string>& requiredExtensionNames);
    void createDebugMessenger();
    void createWindowSurface();
    void choosePhysicalDevice();
    void ensureDeviceExtensionsAreSupported(const std::vector<std::string>& requiredExtensionNames);
    void createLogicalDevice();
    void createSwapchain();
    void transitionSwapchainImagesToPresentSources();
    void transitionRenderImagesToGeneral();
    static vk::PresentModeKHR selectBestPresentationMode(const std::vector<vk::PresentModeKHR>& presentationModes);
    vk::Extent2D selectBestImageExtent(const vk::SurfaceCapabilitiesKHR& surfaceCapabilities);
    vk::SurfaceFormatKHR selectBestSurfaceFormat(const std::vector<vk::SurfaceFormatKHR>& surfaceFormats);
    void createBuffers();
    void createLocationBuffers();
    void createTriangleBuffers();
    void createPointLightBuffers();
    vk::Buffer createBuffer(VkDeviceSize bufferSize, const vk::BufferUsageFlags& usageFlags);
    vk::DeviceMemory allocateBufferMemory(const vk::MemoryRequirements& memoryRequirements);
    vk::DeviceMemory allocateDeviceLocalBufferMemory(const vk::MemoryRequirements& memoryRequirements);
    void createImageAndViews();
    vk::ImageView createImageView(const vk::Image& image);
    vk::DeviceMemory allocateImageMemory(const vk::MemoryRequirements& memoryRequirements);
    void createCommandPool();
    void allocateCommandBuffers();
    void createComputePipeline();
    void createDescriptorPool();
    void allocateDescriptorSets();
    void createSynchronisationObjects();

    [[nodiscard]] int indexOfRenderSet(int frameIndex) const;
    [[nodiscard]] int indexOfMatrixSet(int frameIndex) const;
    [[nodiscard]] int indexOfTriangleSet(int frameIndex) const;
    [[nodiscard]] int indexOfPointLightSet(int frameIndex) const;
    vk::CommandBuffer getOneTimeCommandBuffer();
    void executeOneTimeCommandBuffer(const vk::CommandBuffer& commandBuffer);
    static vk::AccessFlags getAccessFlagsForLayout(vk::ImageLayout layout);
    vk::ImageMemoryBarrier getTransitionImageMemoryBarrier(const vk::Image& image, vk::ImageLayout srcLayout, vk::ImageLayout dstLayout);
    void updateRenderImageDescriptorSets();
    void updateRenderImageDescriptorSet(const vk::DescriptorSet& descriptorSet, const vk::ImageView& renderImageView);

    void updateLocationBuffer(const vk::DeviceMemory& memory);
    void updateLocationDescriptorSet(const vk::DescriptorSet& descriptorSet, const vk::Buffer& matrixBuffer);

    void updateTriangleBuffers();
    void updateTriangleBuffer(const vk::DeviceMemory& memory);
    void updateTriangleDescriptorSets();
    void updateTriangleDescriptorSet(const vk::DescriptorSet& descriptorSet, const vk::Buffer& triangleBuffer);

    void updatePointLightBuffers();
    void updatePointLightBuffer(const vk::DeviceMemory& memory);
    void updatePointLightDescriptorSets();
    void updatePointLightDescriptorSet(const vk::DescriptorSet& descriptorSet, const vk::Buffer& pointLightBuffer);

    void clearImage(const vk::CommandBuffer& commandBuffer, const vk::Image& image, const glm::vec3& clearColor);

    void generateStuff(const glm::vec3& center);
    void handleMouseAction();
    bool rayIntersectsSphere(Ray ray, ArcballSphere sphere, glm::vec3& surfaceCoordinateResult);

    void handleKeyboardAction();
    void mainloop();
    void renderFrame(int currentFrameIndex);

public:
    explicit VulkanApplication(std::string);
    VulkanApplication(const VulkanApplication&) = delete;
    VulkanApplication(VulkanApplication&&) = delete;
    ~VulkanApplication();

    void start();
};

#endif //VULKANRAYTRACING_VULKANAPPLICATION_H
