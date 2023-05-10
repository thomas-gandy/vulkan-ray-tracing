//
// Created by Thomas Gandy on 29/04/2023.
//

#include <iostream>
#include <fstream>
#include <unordered_set>
#include <glm/glm.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>

#include "vulkanApplication.h"
#include "geometry.h"
#include "terrain/terrain.h"
#include "shapes/plane.h"
#include "utility/files/ply/plyFileHandler.h"
#include "shapes/triangle.h"


VulkanApplication::VulkanApplication(std::string applicationName): applicationName(std::move(applicationName)) {
    Plane plane;
    plane.transform.scale = 2000;
    this->vertexModels.push_back(plane);

    Triangle triangle{
            {{-1, -1, 5}, {1.f, 0.f, 1.f}},
            {{0, 1, 5}, {1.f, 0.f, 1.f}},
            {{1, -1, 5}, {1.f, 0.f, 1.f}}
    };
    TriangleModel triangleModel;
    triangleModel.transform.position = {-100, 1.f, -2};
    for (int i = 0; i < 100; i++) {
//        this->vertexModels.push_back(triangleModel);
        triangleModel.transform.position += glm::vec3{2.f, 0, 0};
    }

    auto center = glm::vec3{0, 1, 0};
    for (int i = 0; i < 3; i++) {
        this->generateStuff(center);
        center += glm::vec3{0, 0, 20};
    }

//    Terrain terrain;
//    this->vertexModels.push_back(terrain);
//    this->triangles = terrain.triangles;

//    this->kdTree = KDTree(this->vertexModels);

    std::cout << "Offset of position: " << offsetof(Vertex, position) << std::endl;
    std::cout << "Offset of color: " << offsetof(Vertex, color) << std::endl;
    std::cout << "Offset of cameraToWorld: " << offsetof(LocationBuffer, cameraToWorld) << std::endl;
    std::cout << "Offset of location: " << offsetof(LocationBuffer, location) << std::endl;

    PlyFileHandler plyFile("/Users/tom/Programming/Projects/ProceduralGeneration/assets/models/bunny/bunny-high-resolution.ply");
//    PlyFileHandler plyFile("/Users/tom/Programming/Projects/ProceduralGeneration/assets/models/dragon/dragon-medium-resolution.ply");

    auto plyVertexModel = plyFile.getVertexModel();
    plyVertexModel.transform.scale = 10;
//    plyVertexModel.transform.quaternionRotation = glm::angleAxis(glm::radians(180.f), glm::vec3{0, 1, 0});
//    this->vertexModels.push_back(plyVertexModel);

    for (auto& vertexModel : this->vertexModels) {
        vertexModel.precomputeTriangles();
        this->triangles.insert(this->triangles.end(), vertexModel.precomputedWorldTriangles.begin(), vertexModel.precomputedWorldTriangles.end());
    }

    PointLight pointLight;
    pointLight.position = {0, 5, -2};
    pointLight.color = {1, 1, 1};
    pointLight.intensity = 250;
//    this->pointLights.push_back(pointLight);
    pointLight.position = {0, 5, 10};
//    this->pointLights.push_back(pointLight);
    pointLight.position = {0, 5, 30};
//    this->pointLights.push_back(pointLight);

    this->camera.position = {0, 1.2, -2};

    this->createWindow();
    this->createInstance();
//    this->createDebugMessenger();
    this->createWindowSurface();
    this->choosePhysicalDevice();
    this->createLogicalDevice();
    this->createSwapchain();
    this->createBuffers();
    this->createImageAndViews();
    this->createCommandPool();
    this->allocateCommandBuffers();
    this->transitionSwapchainImagesToPresentSources();
    this->transitionRenderImagesToGeneral();
    this->createComputePipeline();
    this->createDescriptorPool();
    this->allocateDescriptorSets();
    this->updateRenderImageDescriptorSets();
//    this->updateTriangleBuffers();
    this->updatePointLightBuffers();
    this->createSynchronisationObjects();
}

void VulkanApplication::generateStuff(const glm::vec3 &center) {
    const auto trianglesToSpawn = 10;
    const auto angleIncrement = (glm::pi<float>() * 2.f) / trianglesToSpawn;
    const auto spawnRadius = 5;
    auto theta = 0.f;

    TriangleModel triangleModel;
    for (int i = 0; i < trianglesToSpawn; i++) {
        const auto x = glm::cos(theta) * spawnRadius;
        const auto z = glm::sin(theta) * spawnRadius;
        const auto position = glm::vec3{x, 0, z};

        triangleModel.transform.position = position + center;
        triangleModel.yaw(glm::degrees(-theta));
        this->vertexModels.push_back(triangleModel);

        theta += angleIncrement;
    }

    this->pointLights.push_back({glm::vec3{0, 4, 0} + center, {1, 1, 1}, 250});
}

VulkanApplication::~VulkanApplication() {
    this->logicalDevice.waitIdle();

    for (const auto& semaphore : this->swapchainSemaphores) this->logicalDevice.destroySemaphore(semaphore);
    for (const auto& semaphore : this->frameSemaphores) this->logicalDevice.destroySemaphore(semaphore);
    for (const auto& fence : this->frameFences) this->logicalDevice.destroyFence(fence);
    this->logicalDevice.destroyDescriptorPool(this->descriptorPool);
    for (const auto& descriptorSetLayout : this->descriptorSetLayouts) this->logicalDevice.destroyDescriptorSetLayout(descriptorSetLayout);
    for (const auto& buffer : this->locationBuffers) this->logicalDevice.destroyBuffer(buffer);
    for (const auto& bufferAllocation : this->locationBuffersMemory) this->logicalDevice.freeMemory(bufferAllocation);
    for (const auto& buffer : this->triangleBuffers) this->logicalDevice.destroyBuffer(buffer);
    for (const auto& bufferAllocation : this->triangleBuffersMemory) this->logicalDevice.freeMemory(bufferAllocation);
    for (const auto& buffer : this->pointLightBuffers) this->logicalDevice.destroyBuffer(buffer);
    for (const auto& bufferAllocation : this->pointLightBuffersMemory) this->logicalDevice.freeMemory(bufferAllocation);
    for (const auto& imageView : this->frameImageViews) this->logicalDevice.destroyImageView(imageView);
    for (const auto& imageAllocation : this->frameImageMemory) this->logicalDevice.freeMemory(imageAllocation);
    for (const auto& image : this->frameImages) this->logicalDevice.destroyImage(image);
    this->logicalDevice.destroySwapchainKHR(this->swapchain);
    this->logicalDevice.destroyPipelineLayout(this->computePipelineLayout);
    this->logicalDevice.destroyPipeline(this->computePipeline);
    this->logicalDevice.freeCommandBuffers(this->commandPool, this->computeCommandBuffers);
    this->logicalDevice.destroyCommandPool(this->commandPool);
    this->logicalDevice.destroy();
    this->instance.destroySurfaceKHR(this->surface);
    this->instance.destroy();
    glfwDestroyWindow(this->window);
    glfwTerminate();
}

void VulkanApplication::createWindow() {
    if (glfwInit() == GLFW_FALSE) throw std::runtime_error("Failed to initialise GLFW");
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    this->window = glfwCreateWindow((int)this->windowWidth, (int)this->windowHeight, "Vulkan Ray Tracer", nullptr, nullptr);
    if (this->window == nullptr) throw std::runtime_error("Failed to create GLFW window");
    glfwSetInputMode(this->window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    glfwGetFramebufferSize(this->window, &this->framebufferWidth, &this->framebufferHeight);
    std::cout << "Framebuffer Size: " << this->framebufferWidth << "x" << this->framebufferHeight << std::endl;
}

void VulkanApplication::createInstance() {
    std::vector<std::string> requiredInstanceLayerNames = {"VK_LAYER_KHRONOS_validation"};
    std::vector<std::string> requiredInstanceExtensionNames = {VK_EXT_DEBUG_UTILS_EXTENSION_NAME};

    uint32_t glfwExtensionCount;
    const auto glfwExtensionsRaw = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    for (int i = 0; i < glfwExtensionCount; i++) requiredInstanceExtensionNames.emplace_back(glfwExtensionsRaw[i]);

    VulkanApplication::ensureLayersAreSupported(requiredInstanceLayerNames);
    VulkanApplication::ensureInstanceExtensionsAreSupported(requiredInstanceExtensionNames);

    std::vector<const char*> layerNamesToEnable = VulkanApplication::convertStringVectorToCStr(requiredInstanceLayerNames);
    std::vector<const char*> extensionNamesToEnable = VulkanApplication::convertStringVectorToCStr(requiredInstanceExtensionNames);

    vk::ApplicationInfo applicationInfo(this->applicationName.c_str(), 1, "My Engine", 1, VK_API_VERSION_1_1);
    vk::InstanceCreateInfo instanceCreateInfo{{}, &applicationInfo, layerNamesToEnable, extensionNamesToEnable};

    this->instance = vk::createInstance(instanceCreateInfo);
}

void VulkanApplication::ensureLayersAreSupported(const std::vector<std::string>& requiredLayerNames) {
    const auto availableLayers = vk::enumerateInstanceLayerProperties();
    std::vector<std::string> availableLayerNames{};
    for (const auto& availableLayer : availableLayers) {
        availableLayerNames.push_back(availableLayer.layerName);
    }

    if (!VulkanApplication::isSupersetOf(availableLayerNames, requiredLayerNames)) {
        std::cerr << "----- Available Layers -----" << std::endl;
        VulkanApplication::output(availableLayerNames, std::cerr);
        std::cerr << "----- Required Layers -----" << std::endl;
        VulkanApplication::output(requiredLayerNames, std::cerr);

        throw std::runtime_error("Failed to find all required instance layers");
    }
}

void VulkanApplication::ensureInstanceExtensionsAreSupported(const std::vector<std::string>& requiredExtensionNames) {
    const auto availableExtensions = vk::enumerateInstanceExtensionProperties();
    std::vector<std::string> availableExtensionNames{};
    for (const auto& availableExtension : availableExtensions) {
        availableExtensionNames.push_back(availableExtension.extensionName);
    }

    if (!VulkanApplication::isSupersetOf(availableExtensionNames, requiredExtensionNames)) {
        std::cerr << "----- Available Instance Extensions -----" << std::endl;
        VulkanApplication::output(availableExtensionNames, std::cerr);
        std::cerr << "----- Required Instance Extensions -----" << std::endl;
        VulkanApplication::output(requiredExtensionNames, std::cerr);

        throw std::runtime_error("Failed to find all required instance extensions");
    }
}

void VulkanApplication::createDebugMessenger() {
    vk::DebugUtilsMessengerCreateInfoEXT createInfo{
        vk::DebugUtilsMessengerCreateFlagsEXT(),
        vk::DebugUtilsMessageSeverityFlagsEXT(VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT),
        vk::DebugUtilsMessageTypeFlagsEXT(VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT),
        VulkanApplication::debugCallback
    };

//    this->debugMessenger = this->instance.createDebugUtilsMessengerEXT(createInfo);
}

void VulkanApplication::createWindowSurface() {
    VkSurfaceKHR surfaceResult;
    if (glfwCreateWindowSurface(this->instance, this->window, nullptr, &surfaceResult) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create window surface");
    }
    this->surface = vk::SurfaceKHR(surfaceResult);
}

void VulkanApplication::choosePhysicalDevice() {
    this->physicalDevice = this->instance.enumeratePhysicalDevices().front();
    const auto properties = this->physicalDevice.getProperties();
    std::cout << "Chosen Physical Device: " << properties.deviceName << std::endl;
}

void VulkanApplication::ensureDeviceExtensionsAreSupported(const std::vector<std::string> &requiredExtensionNames) {
    const auto availableExtensions = this->physicalDevice.enumerateDeviceExtensionProperties();
    std::vector<std::string> availableExtensionNames{};
    for (const auto& availableExtension : availableExtensions) {
        availableExtensionNames.push_back(availableExtension.extensionName);
    }

    if (!VulkanApplication::isSupersetOf(availableExtensionNames, requiredExtensionNames)) {
        std::cerr << "----- Available Device Extensions -----" << std::endl;
        VulkanApplication::output(availableExtensionNames, std::cerr);
        std::cerr << "----- Required Device Extensions -----" << std::endl;
        VulkanApplication::output(requiredExtensionNames, std::cerr);

        throw std::runtime_error("Failed to find all required device extensions");
    }
}

void VulkanApplication::createLogicalDevice() {
    std::vector<std::string> requiredLogicalExtensionNames = {"VK_KHR_portability_subset", "VK_KHR_swapchain"};
    this->ensureDeviceExtensionsAreSupported(requiredLogicalExtensionNames);
    const auto extensionNamesToEnable = VulkanApplication::convertStringVectorToCStr(requiredLogicalExtensionNames);

    const auto propertiesOfQueueFamilies = this->physicalDevice.getQueueFamilyProperties();
    uint32_t computePresentationQueueIndex = 0;
    for (const auto& queueFamilyProperties : propertiesOfQueueFamilies) {
        if (queueFamilyProperties.queueFlags & vk::QueueFlagBits::eCompute) {
            const auto presentationSupport = this->physicalDevice.getSurfaceSupportKHR(computePresentationQueueIndex, this->surface);
            if (presentationSupport) break;
        }
        computePresentationQueueIndex++;
    }

    if (computePresentationQueueIndex == propertiesOfQueueFamilies.size()) {
        throw std::out_of_range("Failed to find queue family supporting graphics");
    }
    this->computePresentationQueueFamilyIndex = computePresentationQueueIndex;

    float queuePriority = 1.f;
    vk::DeviceQueueCreateInfo deviceComputeQueueCreateInfo({}, this->computePresentationQueueFamilyIndex, 1, &queuePriority);
    vk::DeviceCreateInfo deviceCreateInfo({}, deviceComputeQueueCreateInfo, {}, extensionNamesToEnable);

    this->logicalDevice = this->physicalDevice.createDevice(deviceCreateInfo);
    this->computePresentationQueue = this->logicalDevice.getQueue(this->computePresentationQueueFamilyIndex, 0);
}

void VulkanApplication::createSwapchain() {
    const auto surfaceCapabilities = this->physicalDevice.getSurfaceCapabilitiesKHR(this->surface);
    const auto surfaceFormats = this->physicalDevice.getSurfaceFormatsKHR(this->surface);
    const auto presentationModes = this->physicalDevice.getSurfacePresentModesKHR(this->surface);

    this->framesInFlight = std::clamp(this->framesInFlight, surfaceCapabilities.minImageCount, surfaceCapabilities.maxImageCount);
    std::cout << "FRAMES IN FLIGHT: " << this->framesInFlight << std::endl;
    this->swapchainExtent = this->selectBestImageExtent(surfaceCapabilities);
    this->swapchainSurfaceFormat = VulkanApplication::selectBestSurfaceFormat(surfaceFormats);
    const auto presentationMode = VulkanApplication::selectBestPresentationMode(presentationModes);

    std::cout << "Chosen Extent: " << this->swapchainExtent.width << "x" << this->swapchainExtent.height << std::endl;

    vk::SwapchainCreateInfoKHR swapchainCreateInfoKhr{
        vk::SwapchainCreateFlagsKHR(),
        this->surface,
        this->framesInFlight,
        this->swapchainSurfaceFormat.format,
        this->swapchainSurfaceFormat.colorSpace,
        this->swapchainExtent,
        1,
        vk::ImageUsageFlags(vk::ImageUsageFlagBits::eTransferDst),
        vk::SharingMode::eExclusive,
        {},
        surfaceCapabilities.currentTransform,
        vk::CompositeAlphaFlagBitsKHR::eOpaque,
        presentationMode,
        VK_TRUE
    };

    this->swapchain = this->logicalDevice.createSwapchainKHR(swapchainCreateInfoKhr);
    this->swapchainImages = this->logicalDevice.getSwapchainImagesKHR(this->swapchain);
}

void VulkanApplication::transitionSwapchainImagesToPresentSources() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& swapchainImage = this->swapchainImages[i];
        const auto& commandBuffer = this->getOneTimeCommandBuffer();

        commandBuffer.pipelineBarrier(
                vk::PipelineStageFlags(vk::PipelineStageFlagBits::eTopOfPipe),
                vk::PipelineStageFlags(vk::PipelineStageFlagBits::eBottomOfPipe),
                {},
                {},
                {},
                this->getTransitionImageMemoryBarrier(swapchainImage, vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR)
        );

        this->executeOneTimeCommandBuffer(commandBuffer);
    }

//    std::cout << "SWAPCHAIN IMAGES FROM UNDEFINED TO TRANSFER_DST" << std::endl;
}

void VulkanApplication::transitionRenderImagesToGeneral() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& renderImage = this->frameImages[i];
        const auto& commandBuffer = this->getOneTimeCommandBuffer();

        commandBuffer.pipelineBarrier(
                vk::PipelineStageFlags(vk::PipelineStageFlagBits::eTopOfPipe),
                vk::PipelineStageFlags(vk::PipelineStageFlagBits::eComputeShader),
                {},
                {},
                {},
                this->getTransitionImageMemoryBarrier(renderImage, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral)
        );

        this->executeOneTimeCommandBuffer(commandBuffer);
    }

//    std::cout << "RENDER IMAGES FROM UNDEFINED TO GENERAL" << std::endl;
}

vk::PresentModeKHR VulkanApplication::selectBestPresentationMode(const std::vector<vk::PresentModeKHR> &presentationModes) {
    for (const auto& presentationMode : presentationModes) {
        if (presentationMode == vk::PresentModeKHR::eFifo) return presentationMode;
    }

    throw std::runtime_error("Failed to find supported presentation mode for swapchain");
}

vk::Extent2D VulkanApplication::selectBestImageExtent(const vk::SurfaceCapabilitiesKHR &surfaceCapabilities) {
    const auto min = surfaceCapabilities.minImageExtent;
    const auto max = surfaceCapabilities.maxImageExtent;
    const auto current = surfaceCapabilities.currentExtent;

    if (current.width == 0xFFFFFFFF && current.height == 0xFFFFFFFF) return current;
    const auto width = std::clamp((uint32_t)this->framebufferWidth, min.width, max.width);
    const auto height = std::clamp((uint32_t)this->framebufferHeight, min.height, max.height);

    return {width, height};
}

vk::SurfaceFormatKHR VulkanApplication::selectBestSurfaceFormat(const std::vector<vk::SurfaceFormatKHR> &surfaceFormats) {
//    for (const auto& surfaceFormat : surfaceFormats) {
//        std::cout << "Surface Format: " << vk::to_string(surfaceFormat.format) << std::endl;
//        std::cout << "Color Space: " << vk::to_string(surfaceFormat.colorSpace) << std::endl;
//    }

    for (const auto& surfaceFormat : surfaceFormats) {
        const auto supportedFormat = surfaceFormat.format == vk::Format::eR16G16B16A16Sfloat;
        const auto supportedColorSpace = surfaceFormat.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear;
        if (supportedFormat && supportedColorSpace) return surfaceFormat;
    }

    throw std::runtime_error("Failed to find surface format for swapchain");
}

void VulkanApplication::createBuffers() {
    /*
     * To allocate device memory and bind the buffer to it, you need to specify
     * vk::MemoryPropertyFlagBits and the offset into the device memory (often 0)
     *
     * If it is HOST_VISIBLE then you can map memory from CPU to GPU, otherwise a staging buffer with commands is needed
     */
    this->createLocationBuffers();
    this->createTriangleBuffers();
    this->createPointLightBuffers();
}

void VulkanApplication::createLocationBuffers() {
    const auto offsetInDeviceMemoryToMapTo = 0;
    this->locationBufferSize = sizeof(LocationBuffer);
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto buffer = this->createBuffer(this->locationBufferSize, vk::BufferUsageFlagBits::eUniformBuffer);
        const auto memoryRequirements = this->logicalDevice.getBufferMemoryRequirements(buffer);
        const auto allocatedMemory = this->allocateBufferMemory(memoryRequirements);
        this->logicalDevice.bindBufferMemory(buffer, allocatedMemory, offsetInDeviceMemoryToMapTo);

        this->locationBuffers.push_back(buffer);
        this->locationBuffersMemory.push_back(allocatedMemory);
    }
}

void VulkanApplication::createTriangleBuffers() {
    const auto offsetInDeviceMemoryToMapTo = 0;
    this->triangleBufferSize = sizeof(Triangle) * this->triangles.size();
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto buffer = this->createBuffer(this->triangleBufferSize, vk::BufferUsageFlagBits::eStorageBuffer);
        const auto memoryRequirements = this->logicalDevice.getBufferMemoryRequirements(buffer);
        const auto allocatedMemory = this->allocateBufferMemory(memoryRequirements);
        this->logicalDevice.bindBufferMemory(buffer, allocatedMemory, offsetInDeviceMemoryToMapTo);

        this->triangleBuffers.push_back(buffer);
        this->triangleBuffersMemory.push_back(allocatedMemory);
    }
}

void VulkanApplication::createPointLightBuffers() {
    const auto offsetInDeviceMemoryToMapTo = 0;
    this->pointLightBufferSize = sizeof(PointLight) * this->pointLights.size();
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto buffer = this->createBuffer(this->pointLightBufferSize, vk::BufferUsageFlagBits::eStorageBuffer);
        const auto memoryRequirements = this->logicalDevice.getBufferMemoryRequirements(buffer);
        const auto allocatedMemory = this->allocateBufferMemory(memoryRequirements);
        this->logicalDevice.bindBufferMemory(buffer, allocatedMemory, offsetInDeviceMemoryToMapTo);

        this->pointLightBuffers.push_back(buffer);
        this->pointLightBuffersMemory.push_back(allocatedMemory);
    }
}

vk::Buffer VulkanApplication::createBuffer(VkDeviceSize bufferSize, const vk::BufferUsageFlags &usageFlags) {
    vk::BufferCreateInfo bufferCreateInfo{vk::BufferCreateFlags{}, bufferSize, usageFlags};
    return this->logicalDevice.createBuffer(bufferCreateInfo);
}

vk::DeviceMemory VulkanApplication::allocateBufferMemory(const vk::MemoryRequirements &memoryRequirements) {
    const auto supportedMemoryTypeIndexesByBuffer = memoryRequirements.memoryTypeBits;
    const auto additionalMemoryPropertyFlags = vk::MemoryPropertyFlags(vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    const auto bestMemoryTypeIndex = this->getBestMemoryTypeIndexForAllocation(supportedMemoryTypeIndexesByBuffer, additionalMemoryPropertyFlags);

    vk::MemoryAllocateInfo allocateInfo{memoryRequirements.size, bestMemoryTypeIndex};
    const auto allocatedMemory = this->logicalDevice.allocateMemory(allocateInfo);

    return allocatedMemory;
}

vk::DeviceMemory VulkanApplication::allocateDeviceLocalBufferMemory(const vk::MemoryRequirements &memoryRequirements) {
    const auto supportedMemoryTypeIndexesByBuffer = memoryRequirements.memoryTypeBits;
    const auto additionalMemoryPropertyFlags = vk::MemoryPropertyFlags(vk::MemoryPropertyFlagBits::eDeviceLocal);
    const auto bestMemoryTypeIndex = this->getBestMemoryTypeIndexForAllocation(supportedMemoryTypeIndexesByBuffer, additionalMemoryPropertyFlags);

    vk::MemoryAllocateInfo allocateInfo{memoryRequirements.size, bestMemoryTypeIndex};
    const auto allocatedMemory = this->logicalDevice.allocateMemory(allocateInfo);

    return allocatedMemory;
}

void VulkanApplication::createImageAndViews() {
    vk::ImageCreateInfo imageCreateInfo{
        vk::ImageCreateFlags(),
        vk::ImageType::e2D,
        this->swapchainSurfaceFormat.format,
        {this->swapchainExtent.width, this->swapchainExtent.height, 1},
        1,
        1,
        vk::SampleCountFlagBits::e1,
        vk::ImageTiling::eOptimal,
        vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eStorage,
        vk::SharingMode::eExclusive,
        {},
        vk::ImageLayout::eUndefined
    };

    const auto offsetInDeviceMemoryToMapTo = 0;
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto image = this->logicalDevice.createImage(imageCreateInfo);
        const auto memoryRequirements = this->logicalDevice.getImageMemoryRequirements(image);
        const auto allocatedMemory = this->allocateImageMemory(memoryRequirements);
        this->logicalDevice.bindImageMemory(image, allocatedMemory, offsetInDeviceMemoryToMapTo);

        const auto imageView = this->createImageView(image);

        this->frameImages.push_back(image);
        this->frameImageMemory.push_back(allocatedMemory);
        this->frameImageViews.push_back(imageView);
    }
}

vk::ImageView VulkanApplication::createImageView(const vk::Image &image) {
    vk::ImageViewCreateInfo imageViewCreateInfo{
            vk::ImageViewCreateFlags(),
            image,
            vk::ImageViewType::e2D,
            this->swapchainSurfaceFormat.format,
            {},
            {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
    };

    return this->logicalDevice.createImageView(imageViewCreateInfo);
}

vk::DeviceMemory VulkanApplication::allocateImageMemory(const vk::MemoryRequirements& memoryRequirements) {
    const auto supportedMemoryTypeIndices = memoryRequirements.memoryTypeBits;
    const auto additionalMemoryPropertyFlags = vk::MemoryPropertyFlags(vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent);
    const auto bestMemoryTypeIndex = this->getBestMemoryTypeIndexForAllocation(supportedMemoryTypeIndices, additionalMemoryPropertyFlags);

    vk::MemoryAllocateInfo allocateInfo{memoryRequirements.size, bestMemoryTypeIndex};
    const auto allocatedMemory = this->logicalDevice.allocateMemory(allocateInfo);

    return allocatedMemory;
}

void VulkanApplication::createCommandPool() {
    vk::CommandPoolCreateInfo computeCommandPoolCreateInfo{vk::CommandPoolCreateFlags(VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT), this->computePresentationQueueFamilyIndex};
    this->commandPool = this->logicalDevice.createCommandPool(computeCommandPoolCreateInfo);
}

void VulkanApplication::allocateCommandBuffers() {
    vk::CommandBufferAllocateInfo computeCommandBufferAllocateInfo{this->commandPool, vk::CommandBufferLevel::ePrimary, this->framesInFlight};
    this->computeCommandBuffers = this->logicalDevice.allocateCommandBuffers(computeCommandBufferAllocateInfo);
    vk::CommandBufferAllocateInfo presentationCommandBufferAllocateInfo{this->commandPool, vk::CommandBufferLevel::ePrimary, this->framesInFlight};
    this->presentationCommandBuffers = this->logicalDevice.allocateCommandBuffers(presentationCommandBufferAllocateInfo);
}

void VulkanApplication::createComputePipeline() {
    const auto shaderCode = VulkanApplication::getShaderCode("/Users/tom/Programming/Projects/VulkanRayTracing/shaders/main.comp.spirv");
    vk::ShaderModuleCreateInfo shaderModuleCreateInfo{{}, shaderCode};
    const auto shaderModule = this->logicalDevice.createShaderModule(shaderModuleCreateInfo);

    vk::PipelineShaderStageCreateInfo shaderStageCreateInfo{{}, vk::ShaderStageFlagBits::eCompute, shaderModule, "main"};

    // Variables to create set layouts with
    std::vector<vk::DescriptorSetLayoutBinding> layoutBindings;
    vk::DescriptorSetLayoutCreateInfo setLayoutCreateInfo;

    // TODO: Could there be an issue reusing the same variable if vk:: takes by reference?
    // Descriptor set containing image (set = 0)
    layoutBindings = {{0, vk::DescriptorType::eStorageImage, 1, vk::ShaderStageFlagBits::eCompute}};
    setLayoutCreateInfo = {{}, layoutBindings};
    this->descriptorSetLayouts.push_back(this->logicalDevice.createDescriptorSetLayout(setLayoutCreateInfo));

    // Descriptor set containing matrix (set = 1)
    layoutBindings = {{0, vk::DescriptorType::eUniformBuffer, 1, vk::ShaderStageFlagBits::eCompute}};
    setLayoutCreateInfo = {{}, layoutBindings};
    this->descriptorSetLayouts.push_back(this->logicalDevice.createDescriptorSetLayout(setLayoutCreateInfo));

    // Descriptor set containing triangles (set = 2)
    layoutBindings = {{0, vk::DescriptorType::eStorageBuffer, 1, vk::ShaderStageFlagBits::eCompute}};
    setLayoutCreateInfo = {{}, layoutBindings};
    this->descriptorSetLayouts.push_back(this->logicalDevice.createDescriptorSetLayout(setLayoutCreateInfo));

    // Descriptor set containing point lights (set = 3)
    layoutBindings = {{0, vk::DescriptorType::eStorageBuffer, 1, vk::ShaderStageFlagBits::eCompute}};
    setLayoutCreateInfo = {{}, layoutBindings};
    this->descriptorSetLayouts.push_back(this->logicalDevice.createDescriptorSetLayout(setLayoutCreateInfo));

    const auto timesToDuplicate = this->framesInFlight;
    for (int i = 0; i < timesToDuplicate; i++) {
        for (const auto& descriptorSetLayout : this->descriptorSetLayouts) {
            this->framesDescriptorSetLayouts.push_back(descriptorSetLayout);
        }
    }
    this->maxSetsToAllocateForSingleFrame = this->descriptorSetLayouts.size();
    this->maxSetsToAllocateAcrossFrames = this->framesDescriptorSetLayouts.size();

    vk::PipelineLayoutCreateInfo pipelineLayoutCreateInfo{{}, this->descriptorSetLayouts};
    this->computePipelineLayout = this->logicalDevice.createPipelineLayout(pipelineLayoutCreateInfo);

    vk::ComputePipelineCreateInfo computePipelineCreateInfo{{}, shaderStageCreateInfo, this->computePipelineLayout};
    const auto creationResult = this->logicalDevice.createComputePipeline({}, computePipelineCreateInfo);
    if (creationResult.result != vk::Result::eSuccess) {
        throw std::runtime_error("Failed to create compute pipeline");
    }
    this->computePipeline = creationResult.value;

    this->logicalDevice.destroyShaderModule(shaderModule);
}

void VulkanApplication::createDescriptorPool() {
    vk::DescriptorPoolSize locationBufferPoolSize{vk::DescriptorType::eUniformBuffer, this->framesInFlight};
    vk::DescriptorPoolSize trianglesBufferPoolSize{vk::DescriptorType::eStorageBuffer, this->framesInFlight};
    vk::DescriptorPoolSize pointLightsBufferPoolSize{vk::DescriptorType::eStorageBuffer, this->framesInFlight};
    vk::DescriptorPoolSize storageImagePoolSize{vk::DescriptorType::eStorageImage, this->framesInFlight};
    std::vector<vk::DescriptorPoolSize> poolSizes{locationBufferPoolSize, trianglesBufferPoolSize, pointLightsBufferPoolSize, storageImagePoolSize};

    vk::DescriptorPoolCreateInfo descriptorPoolCreateInfo{{}, this->maxSetsToAllocateAcrossFrames, poolSizes};
    this->descriptorPool = this->logicalDevice.createDescriptorPool(descriptorPoolCreateInfo);
}

void VulkanApplication::allocateDescriptorSets() {
    vk::DescriptorSetAllocateInfo descriptorSetAllocateInfo{this->descriptorPool, this->maxSetsToAllocateAcrossFrames, this->framesDescriptorSetLayouts.data()};
    this->descriptorSets = this->logicalDevice.allocateDescriptorSets(descriptorSetAllocateInfo);
}

void VulkanApplication::updateRenderImageDescriptorSets() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& descriptorSet = this->descriptorSets[this->indexOfRenderSet(i)];
        const auto& renderImageView = this->frameImageViews[i];
        this->updateRenderImageDescriptorSet(descriptorSet, renderImageView);
    }
}

void VulkanApplication::updateTriangleDescriptorSets() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& descriptorSet = this->descriptorSets[this->indexOfTriangleSet(i)];
        const auto& triangleBuffer = this->triangleBuffers[i];
        this->updateTriangleDescriptorSet(descriptorSet, triangleBuffer);
    }
}

void VulkanApplication::createSynchronisationObjects() {
    vk::SemaphoreCreateInfo semaphoreCreateInfo{};
    vk::FenceCreateInfo fenceCreateInfo{vk::FenceCreateFlagBits::eSignaled};
    // Create fences in signaled state so frames can continue rendering until they loop back to themselves

    for (int i = 0; i < this->framesInFlight; i++) {
        const auto swapchainSemaphore = this->logicalDevice.createSemaphore(semaphoreCreateInfo);
        const auto renderOperationFinishedSemaphore = this->logicalDevice.createSemaphore(semaphoreCreateInfo);
        this->swapchainSemaphores.push_back(swapchainSemaphore);
        this->frameSemaphores.push_back(renderOperationFinishedSemaphore);

        const auto createdFence = this->logicalDevice.createFence(fenceCreateInfo);
        this->frameFences.push_back(createdFence);
    }
}

int VulkanApplication::indexOfMatrixSet(int frameIndex) const {
    return (int)(frameIndex * this->maxSetsToAllocateForSingleFrame + this->matrixSetIndex);
}

int VulkanApplication::indexOfRenderSet(int frameIndex) const {
    return (int)(frameIndex * this->maxSetsToAllocateForSingleFrame) + this->imageSetIndex;
}

int VulkanApplication::indexOfTriangleSet(int frameIndex) const {
    return (int)(frameIndex * this->maxSetsToAllocateForSingleFrame) + this->triangleSetIndex;
}

int VulkanApplication::indexOfPointLightSet(int frameIndex) const {
    return (int)(frameIndex * this->maxSetsToAllocateForSingleFrame) + this->pointLightSetIndex;
}

vk::CommandBuffer VulkanApplication::getOneTimeCommandBuffer() {
    vk::CommandBufferAllocateInfo allocateInfo{this->commandPool, vk::CommandBufferLevel::ePrimary, 1};
    const auto commands = this->logicalDevice.allocateCommandBuffers(allocateInfo);
    const auto command = commands[0];
    command.begin(vk::CommandBufferBeginInfo{vk::CommandBufferUsageFlags()});

    return command;
}

void VulkanApplication::executeOneTimeCommandBuffer(const vk::CommandBuffer &commandBuffer) {
    commandBuffer.end();
    vk::SubmitInfo submitInfo{{}, {}, commandBuffer};

    this->computePresentationQueue.submit(submitInfo);
    this->computePresentationQueue.waitIdle();
    this->logicalDevice.freeCommandBuffers(this->commandPool, commandBuffer);
}

vk::AccessFlags VulkanApplication::getAccessFlagsForLayout(vk::ImageLayout layout) {
    if (layout == vk::ImageLayout::eUndefined) return {vk::AccessFlagBits::eNone};
    if (layout == vk::ImageLayout::eGeneral) return {vk::AccessFlagBits::eShaderWrite};
    if (layout == vk::ImageLayout::ePresentSrcKHR) return {vk::AccessFlagBits::eMemoryRead};
    if (layout == vk::ImageLayout::eTransferSrcOptimal) return {vk::AccessFlagBits::eTransferRead};
    if (layout == vk::ImageLayout::eTransferDstOptimal) return {vk::AccessFlagBits::eTransferWrite};

    throw std::runtime_error("Unsupported image layout for transition");
}

vk::ImageMemoryBarrier VulkanApplication::getTransitionImageMemoryBarrier(const vk::Image& image, vk::ImageLayout srcLayout, vk::ImageLayout dstLayout) {
    return {
            this->getAccessFlagsForLayout(srcLayout),
            this->getAccessFlagsForLayout(dstLayout),
            srcLayout,
            dstLayout,
            VK_QUEUE_FAMILY_IGNORED,
            VK_QUEUE_FAMILY_IGNORED,
            image,
            {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
    };
}

void VulkanApplication::updateLocationBuffer(const vk::DeviceMemory& memory) {
    const auto cameraToWorld = this->camera.getWorldTransform();
    const auto cameraPosition = this->camera.position;
    LocationBuffer locationBuffer{cameraToWorld, cameraPosition};
    auto mappedMemory = this->logicalDevice.mapMemory(memory, 0, this->locationBufferSize, vk::MemoryMapFlags());
    std::memcpy(mappedMemory, &locationBuffer, this->locationBufferSize);
    this->logicalDevice.unmapMemory(memory);
}

void VulkanApplication::updateLocationDescriptorSet(const vk::DescriptorSet& descriptorSet, const vk::Buffer& matrixBuffer) {
    vk::DescriptorBufferInfo bufferInfo{matrixBuffer, 0, VK_WHOLE_SIZE};
    vk::WriteDescriptorSet writeDescriptorSet{descriptorSet, 0, 0, 1, vk::DescriptorType::eUniformBuffer, {}, &bufferInfo};
    this->logicalDevice.updateDescriptorSets(writeDescriptorSet, {});
}

void VulkanApplication::updateRenderImageDescriptorSet(const vk::DescriptorSet& descriptorSet, const vk::ImageView& renderImageView) {
    vk::DescriptorImageInfo imageInfo{{}, renderImageView, vk::ImageLayout::eGeneral};
    vk::WriteDescriptorSet writeDescriptorSet{descriptorSet, 0, 0, 1, vk::DescriptorType::eStorageImage, &imageInfo};
    this->logicalDevice.updateDescriptorSets(writeDescriptorSet, {});
}

void VulkanApplication::updateTriangleBuffers() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& memory = this->triangleBuffersMemory[i];
        this->updateTriangleBuffer(memory);
    }

    this->updateTriangleDescriptorSets();
}

void VulkanApplication::updateTriangleBuffer(const vk::DeviceMemory& memory) {
//    const auto stagingBuffer = this->createBuffer(this->triangleBufferSize, vk::BufferUsageFlagBits::eTransferSrc);
//    const auto memoryRequirements = this->logicalDevice.getBufferMemoryRequirements(stagingBuffer);
//    const auto stagingBufferMemory = this->allocateBufferMemory(memoryRequirements);
//    this->logicalDevice.bindBufferMemory(stagingBuffer, stagingBufferMemory, 0);
//
//    auto mappedMemory = this->logicalDevice.mapMemory(stagingBufferMemory, 0, this->triangleBufferSize, vk::MemoryMapFlags());
//    std::memcpy(mappedMemory, this->triangles.data(), this->triangleBufferSize);
//    this->logicalDevice.unmapMemory(stagingBufferMemory);
//
//    const auto copyRegion = vk::BufferCopy{0, 0, this->triangleBufferSize};
//    const auto commandBuffer = this->getOneTimeCommandBuffer();
//    commandBuffer.copyBuffer(stagingBuffer, buffer, copyRegion);
//    this->executeOneTimeCommandBuffer(commandBuffer);
//
//    this->logicalDevice.destroyBuffer(stagingBuffer);
//    this->logicalDevice.freeMemory(stagingBufferMemory);

    this->triangles.clear();
    for (auto& vertexModel : this->vertexModels) {
        vertexModel.precomputeTriangles();
        this->triangles.insert(this->triangles.end(), vertexModel.precomputedWorldTriangles.begin(), vertexModel.precomputedWorldTriangles.end());
    }
    auto mappedMemory = this->logicalDevice.mapMemory(memory, 0, this->triangleBufferSize, vk::MemoryMapFlags());
    std::memcpy(mappedMemory, this->triangles.data(), this->triangleBufferSize);
    this->logicalDevice.unmapMemory(memory);
}

void VulkanApplication::updateTriangleDescriptorSet(const vk::DescriptorSet &descriptorSet, const vk::Buffer &triangleBuffer) {
    vk::DescriptorBufferInfo bufferInfo{triangleBuffer, 0, VK_WHOLE_SIZE};
    vk::WriteDescriptorSet writeDescriptorSet{descriptorSet, 0, 0, 1, vk::DescriptorType::eStorageBuffer, {}, &bufferInfo};
    this->logicalDevice.updateDescriptorSets(writeDescriptorSet, {});
}

void VulkanApplication::updatePointLightBuffers() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& memory = this->pointLightBuffersMemory[i];
        this->updatePointLightBuffer(memory);
    }

    this->updatePointLightDescriptorSets();
}

void VulkanApplication::updatePointLightBuffer(const vk::DeviceMemory &memory) {
    auto mappedMemory = this->logicalDevice.mapMemory(memory, 0, this->pointLightBufferSize, vk::MemoryMapFlags());
    std::memcpy(mappedMemory, this->pointLights.data(), this->pointLightBufferSize);
    this->logicalDevice.unmapMemory(memory);
}

void VulkanApplication::updatePointLightDescriptorSets() {
    for (int i = 0; i < this->framesInFlight; i++) {
        const auto& descriptorSet = this->descriptorSets[this->indexOfPointLightSet(i)];
        const auto& pointLightBuffer = this->pointLightBuffers[i];
        this->updatePointLightDescriptorSet(descriptorSet, pointLightBuffer);
    }
}

void VulkanApplication::updatePointLightDescriptorSet(const vk::DescriptorSet &descriptorSet, const vk::Buffer &pointLightBuffer) {
    vk::DescriptorBufferInfo bufferInfo{pointLightBuffer, 0, VK_WHOLE_SIZE};
    vk::WriteDescriptorSet writeDescriptorSet{descriptorSet, 0, 0, 1, vk::DescriptorType::eStorageBuffer, {}, &bufferInfo};
    this->logicalDevice.updateDescriptorSets(writeDescriptorSet, {});
}

void VulkanApplication::clearImage(const vk::CommandBuffer &commandBuffer, const vk::Image &image, const glm::vec3 &clearColor) {
    vk::ClearColorValue clearColorValue;
    clearColorValue.float32[0] = clearColor[0]; clearColorValue.float32[1] = clearColor[1];
    clearColorValue.float32[2] = clearColor[2]; clearColorValue.float32[3] = 1.f;
    vk::ImageSubresourceRange subresourceRange = {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
    commandBuffer.clearColorImage(image, vk::ImageLayout::eTransferDstOptimal, clearColorValue, subresourceRange);
}

template<typename T>
void VulkanApplication::output(const T &container, std::ostream& os) {
    for (const auto& element : container) os << element << std::endl;
}

template<typename T>
bool VulkanApplication::isSupersetOf(const std::vector<T> &superset, const std::vector<T> &subset) {
    std::unordered_set<T> supersetSet(superset.begin(), superset.end());

    for (const auto subsetElement : subset) {
        if (supersetSet.find(subsetElement) == supersetSet.end()) return false;
    }

    return true;
}

std::vector<uint32_t> VulkanApplication::getShaderCode(const std::string &filepath) {
    std::ifstream inputFile(filepath, std::ios::binary);
    if (!inputFile.is_open()) {
        throw std::ios_base::failure("File " + filepath + " not found");
    }

    inputFile.seekg(0, std::ios::seekdir::end);
    const auto fileSize = inputFile.tellg() / sizeof(uint32_t);
    inputFile.seekg(0, std::ios::seekdir::beg);

    uint32_t value;
    std::vector<uint32_t> shaderCode(fileSize);
    for (auto i = 0; i < fileSize; i++) {
        inputFile.read(reinterpret_cast<char*>(&value), sizeof(uint32_t));
        shaderCode[i] = value;
    }

    return shaderCode;
}

std::vector<const char *> VulkanApplication::convertStringVectorToCStr(const std::vector<std::string> &strings) {
    std::vector<const char*> cstrs;
    cstrs.reserve(strings.size());
    for (const auto& string : strings) cstrs.push_back(string.c_str());

    return cstrs;
}

VkBool32 VulkanApplication::debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageType,
        const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData,
        void *pUserData
    ) {
    return VK_FALSE;
}

uint32_t VulkanApplication::getBestMemoryTypeIndexForAllocation(
        uint32_t supportedTypeIndexesBitField,
        vk::MemoryPropertyFlags additionalMemoryPropertyFlags
) {
    const auto physicalDeviceMemoryProperties = this->physicalDevice.getMemoryProperties();
    const auto numberOfMemoryTypes = physicalDeviceMemoryProperties.memoryTypeCount;

    for (uint32_t memoryTypeIndex = 0; memoryTypeIndex < numberOfMemoryTypes; memoryTypeIndex++) {
        const auto currentMemoryTypeIndexAsBitField = (1 << memoryTypeIndex);
        const auto currentMemoryTypeIsSupported = currentMemoryTypeIndexAsBitField & supportedTypeIndexesBitField;

        const auto currentMemoryTypeProperties = physicalDeviceMemoryProperties.memoryTypes[memoryTypeIndex].propertyFlags;
        const auto additionalPropertiesSupported = currentMemoryTypeProperties & additionalMemoryPropertyFlags;

        if (currentMemoryTypeIsSupported && additionalPropertiesSupported) {
            return memoryTypeIndex;
        }
    }

    throw std::runtime_error("Failed to find a valid memory type index.");
}

void VulkanApplication::start() {
    this->applicationRunning = true;
    this->mainloop();
}

void VulkanApplication::handleMouseAction() {
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        auto ndcX = -((float)2 * (float)mouseX / (this->windowWidth - 1) - 1.f);
        auto ndcY = ((float)2 * (float)mouseY / (this->windowHeight - 1) - 1.f);

        // Get distance between mouse and screen center to see if it is larger than radius (outside arcball)
        const auto norm = std::sqrt(ndcX*ndcX + ndcY*ndcY);
        float z = 0;
        if (norm > 1) { // Arcball assumed to have a radius of 1
            // If outside radius, map onto arcball edge
            const auto normalised = glm::normalize(glm::vec2(ndcX, ndcY));
            ndcX = normalised.x;
            ndcY = normalised.y;
        } else z = std::sqrt(1 - ndcX*ndcX - ndcY*ndcY);

        auto& objectOfFocus = this->vertexModels[this->objectIndexOfFocus];

        // Set arcball start of drag variables
        if (!this->mouseDragging) {
            this->mouseDragging = true;
            this->startArcballPosition = {0, {ndcX, ndcY, z}};
            this->initialRotationOfObject = this->vertexModels[this->objectIndexOfFocus].transform.quaternionRotation;
            return;
        }
        this->currentArcballPosition = {0, {ndcX, ndcY, z}};

        const auto drag = this->currentArcballPosition * glm::conjugate(this->startArcballPosition);
        objectOfFocus.transform.quaternionRotation = drag * this->initialRotationOfObject;
        objectOfFocus.precomputeTriangles();
    } else {
        this->mouseDragging = false;
        glfwSetInputMode(this->window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

    const auto relativeScreenX = mouseX - this->lastMouseX;
    const auto relativeScreenY = mouseY - this->lastMouseY;
    this->lastMouseX = mouseX;
    this->lastMouseY = mouseY;

    if (!this->mouseDragging) {
        this->camera.quaternionRotation *= glm::angleAxis(static_cast<float>(relativeScreenX) * this->camera.sensitivity, glm::vec3(0, 1, 0));
        this->camera.quaternionRotation *= glm::angleAxis(static_cast<float>(relativeScreenY) * this->camera.sensitivity, glm::vec3(1, 0, 0));
    }
}

bool VulkanApplication::rayIntersectsSphere(Ray ray, ArcballSphere sphere, glm::vec3 &surfaceCoordinateResult) {
    const auto rayOrigin = ray.origin;
    const auto rayDirection = ray.direction;
    const auto sphereCenter = sphere.center;
    const auto sphereRadius = sphere.radius;

    const auto a = glm::dot(rayDirection, rayDirection);
    const auto b = 2 * glm::dot(rayDirection, rayOrigin - sphereCenter);
    const auto c = glm::dot(rayOrigin - sphereCenter, rayOrigin - sphereCenter) - sphereRadius * sphereRadius;

    const auto discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;

    const auto t1 = (-b + std::sqrt(discriminant)) / (2 * a);
    const auto t2 = (-b - std::sqrt(discriminant)) / (2 * a);

    if (t1 < 0 && t2 < 0) return false;

    const auto t_hit = std::min(t1, t2);
    surfaceCoordinateResult = rayOrigin + t_hit * rayDirection;

    return true;
}

void VulkanApplication::handleKeyboardAction() {
    if (glfwGetKey(this->window, GLFW_KEY_A) == GLFW_PRESS) this->camera.moveLeft();
    if (glfwGetKey(this->window, GLFW_KEY_D) == GLFW_PRESS) this->camera.moveRight();
    if (glfwGetKey(this->window, GLFW_KEY_W) == GLFW_PRESS) this->camera.moveForward();
    if (glfwGetKey(this->window, GLFW_KEY_S) == GLFW_PRESS) this->camera.moveBack();
    if (glfwGetKey(this->window, GLFW_KEY_SPACE) == GLFW_PRESS) this->camera.moveUp();
    if (glfwGetKey(this->window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) this->camera.moveDown();
    if (glfwGetKey(this->window, GLFW_KEY_Q) == GLFW_PRESS) this->camera.rotateLeft();
    if (glfwGetKey(this->window, GLFW_KEY_E) == GLFW_PRESS) this->camera.rotateRight();
    if (glfwGetKey(this->window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(this->window, true);
    if (glfwGetKey(this->window, GLFW_KEY_P) == GLFW_PRESS && this->presentationModeReleased) {
        this->presentationMode = !this->presentationMode;
        this->presentationModeReleased = false;
    } else this->presentationModeReleased = true;
}

void VulkanApplication::mainloop() {
    int currentFrameIndex = 0;
    this->camera.update();
    while (this->applicationRunning && !glfwWindowShouldClose(this->window)) {
//        std::cout << "Rendering Frame: " << currentFrameIndex << std::endl;
//        const auto startTime = std::chrono::high_resolution_clock::now();

        glfwPollEvents();
        this->handleMouseAction();
        this->handleKeyboardAction();
        this->camera.update();
        if (this->presentationMode) {
            const auto speed = 1.f;
            const auto range = 35.f;
            const auto min = 5.f;
            this->camera.position.z = (glm::sin(t * speed) + 0) * range + min;
        }
        this->renderFrame(currentFrameIndex);

        currentFrameIndex = (currentFrameIndex + 1) % static_cast<int>(this->framesInFlight);
        t += 0.005;
//        const auto endTime = std::chrono::high_resolution_clock::now();
//        const auto elapsedTime = endTime - startTime;
//        std::cout << "CPU Frame Dispatch Time: " << elapsedTime.count() << std::endl;
    }

    this->logicalDevice.waitIdle();
}

void VulkanApplication::renderFrame(int currentFrameIndex) {
    auto& frameFence = this->frameFences[currentFrameIndex];
    const auto waitResult = this->logicalDevice.waitForFences(frameFence, VK_TRUE, UINT64_MAX);
    if (waitResult != vk::Result::eSuccess) throw std::runtime_error("Failed to wait for fence");
    this->logicalDevice.resetFences(frameFence);

    const auto& imageReadySemaphore = this->swapchainSemaphores[currentFrameIndex];
    const auto& renderFinishedSemaphore = this->frameSemaphores[currentFrameIndex];
    const auto swapchainImageIndex = this->logicalDevice.acquireNextImageKHR(this->swapchain, UINT64_MAX, imageReadySemaphore);
    const auto& swapchainImage = this->swapchainImages[swapchainImageIndex.value];

    const auto& computeCommandBuffer = this->computeCommandBuffers[currentFrameIndex];

    const auto& renderImage = this->frameImages[currentFrameIndex];
    const auto& renderImageSet = this->descriptorSets[this->indexOfRenderSet(currentFrameIndex)];

    const auto& locationBuffer = this->locationBuffers[currentFrameIndex];
    const auto& locationMemory = this->locationBuffersMemory[currentFrameIndex];
    const auto& locationSet = this->descriptorSets[this->indexOfMatrixSet(currentFrameIndex)];
    this->updateLocationBuffer(locationMemory);
    this->updateLocationDescriptorSet(locationSet, locationBuffer);

    const auto& triangleBuffer = this->triangleBuffers[currentFrameIndex];
    const auto& triangleMemory = this->triangleBuffersMemory[currentFrameIndex];
    const auto& triangleSet = this->descriptorSets[this->indexOfTriangleSet(currentFrameIndex)];
    this->updateTriangleBuffer(triangleMemory);
    this->updateTriangleDescriptorSet(triangleSet, triangleBuffer);

    const auto& pointLightBuffer = this->pointLightBuffers[currentFrameIndex];
    const auto& pointLightSet = this->descriptorSets[this->indexOfPointLightSet(currentFrameIndex)];
    const auto& pointLightMemory = this->pointLightBuffersMemory[currentFrameIndex];
    this->updatePointLightBuffer(pointLightMemory);
    this->updatePointLightDescriptorSet(pointLightSet, pointLightBuffer);

    std::vector<vk::DescriptorSet> descriptorSetsToBind{renderImageSet, locationSet, triangleSet, pointLightSet};

    computeCommandBuffer.begin(vk::CommandBufferBeginInfo{vk::CommandBufferUsageFlags()});
    computeCommandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, this->computePipeline);
    computeCommandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, this->computePipelineLayout, 0, descriptorSetsToBind, {});
    computeCommandBuffer.dispatch(64, 64, 1);

    computeCommandBuffer.pipelineBarrier(
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eComputeShader),
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eTransfer),
            {},
            {},
            {},
            this->getTransitionImageMemoryBarrier(renderImage, vk::ImageLayout::eGeneral, vk::ImageLayout::eTransferSrcOptimal)
    ); // render image general to src
    computeCommandBuffer.pipelineBarrier(
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eBottomOfPipe),
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eTransfer),
            {},
            {},
            {},
            this->getTransitionImageMemoryBarrier(swapchainImage, vk::ImageLayout::ePresentSrcKHR, vk::ImageLayout::eTransferDstOptimal)
    ); // swapchain present to image dst

    vk::ImageCopy imageCopy{
            {vk::ImageAspectFlagBits::eColor, 0, 0, 1},
            {0, 0, 0},
            {vk::ImageAspectFlagBits::eColor, 0, 0, 1},
            {0, 0, 0},
            {this->swapchainExtent.width, this->swapchainExtent.height, 1}
    };
    computeCommandBuffer.copyImage(
            renderImage,
            vk::ImageLayout::eTransferSrcOptimal,
            swapchainImage,
            vk::ImageLayout::eTransferDstOptimal,
            imageCopy
    );
//    this->clearImage(computeCommandBuffer, swapchainImage, {1, 0, 1});

    computeCommandBuffer.pipelineBarrier(
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eTransfer),
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eComputeShader),
            {},
            {},
            {},
            this->getTransitionImageMemoryBarrier(renderImage, vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eGeneral)
    ); // render image src to general
    computeCommandBuffer.pipelineBarrier(
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eTransfer),
            vk::PipelineStageFlags(vk::PipelineStageFlagBits::eBottomOfPipe),
            {},
            {},
            {},
            this->getTransitionImageMemoryBarrier(swapchainImage, vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::ePresentSrcKHR)
    ); // swapchain image dst to present
    computeCommandBuffer.end();

    const auto semaphoreWaitStage = vk::PipelineStageFlags(vk::PipelineStageFlagBits::eComputeShader);
    vk::SubmitInfo computeQueueSubmitInfo{imageReadySemaphore, semaphoreWaitStage, computeCommandBuffer, renderFinishedSemaphore};
    this->computePresentationQueue.submit(computeQueueSubmitInfo, frameFence);

    vk::PresentInfoKHR presentInfoKhr{renderFinishedSemaphore, this->swapchain, swapchainImageIndex.value};
    const auto result = this->computePresentationQueue.presentKHR(presentInfoKhr);
    if (result != vk::Result::eSuccess) throw std::runtime_error("Failed to present");
}
