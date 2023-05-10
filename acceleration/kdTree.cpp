//
// Created by Thomas Gandy on 31/01/2023.
//

#include "kdTree.h"

#include <utility>
#include <iostream>


KDTree::KDTree(std::vector<VertexModel> &models) {
    std::cout << "Constructing KDTree..." << std::endl;

    std::vector<Triangle> trianglesInWorldSpace{};
    for (auto& model : models) {
        model.precomputeTriangles();
        for (const auto& triangle: model.precomputedWorldTriangles) {
            trianglesInWorldSpace.push_back(triangle);
        }
    }

    // Maximum depth allowed for KD-Tree
    this->maxDepth = glm::max(1, static_cast<int>(glm::log2(static_cast<float>(trianglesInWorldSpace.size())))) + 1;

    const auto rootBoundingBox = KDTree::generateBoundingBox(trianglesInWorldSpace);
    this->root = this->generate(trianglesInWorldSpace, rootBoundingBox, 0);

    std::cout << "Furthest depth generated: " << this->furthestDepthGenerated << std::endl;
    std::cout << "KDTree constructed" << std::endl;
}

const std::unique_ptr<KDNode> &KDTree::getRoot() const {
    return this->root;
}

int KDTree::getBestSplittingAxis_largestAxisRegionInBB(const BoundingBox &boundingBox) {
    auto axisWithLargestDifference = 0;
    auto largestDifference = 0.f;

    for (auto axis = 0; axis < 3; axis++) {
        const auto differenceInAxis = boundingBox.max[axis] - boundingBox.min[axis];
        if (differenceInAxis > largestDifference) {
            largestDifference = differenceInAxis;
            axisWithLargestDifference = axis;
        }
    }

    return axisWithLargestDifference;
}

int KDTree::getBestSplittingAxis_largestVariance(const std::vector<glm::vec3>& vertices) {
    auto centroid = glm::vec3{0, 0, 0};
    auto vertexCount = 0;
    for (const auto& vertex : vertices) {
        centroid += vertex;
        vertexCount++;
    }
    const auto meanCentroid = centroid /= vertexCount;

    auto variance = glm::vec3{0, 0, 0};
    for (const auto& vertex : vertices) {
        const auto difference = vertex - meanCentroid;
        variance += difference * difference;
    }

    auto axisWithLargestVariance = 0;
    auto largestVariance = 0.f;
    for (auto axis = 0; axis < 3; axis++) {
        if (variance[axis] > largestVariance) {
            largestVariance = variance[axis];
            axisWithLargestVariance = axis;
        }
    }

    return axisWithLargestVariance;
}

BoundingBox KDTree::generateBoundingBox(std::vector<Triangle> &triangles) {
    BoundingBox boundingBox = {triangles[0].v0.position, triangles[0].v0.position};

    for (const auto& triangle : triangles) {
        for (const auto& vertex : {triangle.v0, triangle.v1, triangle.v2}) {
            boundingBox.min = glm::min(boundingBox.min, vertex.position);
            boundingBox.max = glm::max(boundingBox.max, vertex.position);
        }
    }

    return boundingBox;
}

std::unique_ptr<KDNode> KDTree::generate(std::vector<Triangle> &triangles, const BoundingBox& boundingBox, int depth) {
    if (triangles.empty()) return nullptr;

    if (depth > this->furthestDepthGenerated) this->furthestDepthGenerated = depth;
    if (triangles.size() <= 5 || depth >= this->maxDepth) { // Then return leaf node with current triangles
        return std::make_unique<KDNode>(KDNode{nullptr, nullptr, boundingBox, true, triangles});
    }

    std::vector<glm::vec3> vertices{};
    for (const auto& triangle : triangles) {
        for (const auto& vertex : {triangle.v0, triangle.v1, triangle.v2}) {
            vertices.push_back(vertex.position);
        }
    }
//    const auto axis = this->getBestSplittingAxis_largestVariance(vertices);
    const auto axis = depth % 3;
//    const auto axis = this->getBestSplittingAxis_largestAxisRegionInBB(boundingBox);

    std::sort(vertices.begin(), vertices.end(), [axis](const auto& a, const auto& b) {return a[axis] < b[axis];});
    const auto medianAxisSplitValue = vertices[vertices.size() / 2][axis];
    vertices = {}; // Free up some heap memory as individual vertices no longer needed

    // Assign triangles to either left or right child node, or both
    std::vector<Triangle> leftSplitTriangles{};
    std::vector<Triangle> rightSplitTriangles{};
    for (const auto& triangle : triangles) {
        bool addTriangleToLeftSplit = false;
        bool addTriangleToRightSplit = false;
        for (const auto& vertex : {triangle.v0, triangle.v1, triangle.v2}) {
            if (vertex.position[axis] <= medianAxisSplitValue) addTriangleToLeftSplit = true;
            if (vertex.position[axis] > medianAxisSplitValue) addTriangleToRightSplit = true;
        }
        if (addTriangleToLeftSplit) leftSplitTriangles.push_back(triangle);
        if (addTriangleToRightSplit) rightSplitTriangles.push_back(triangle);
    }

    // If both triangles were assigned all the same triangles, unlikely any more splitting can be done so just return
    if (leftSplitTriangles == rightSplitTriangles) {
        return std::make_unique<KDNode>(KDNode{nullptr, nullptr, boundingBox, true, triangles});
    }

    // Calculate bounding boxes of children
    auto leftBoundingBox = boundingBox;
    auto rightBoundingBox = boundingBox;
    leftBoundingBox.max[axis] = medianAxisSplitValue;
    rightBoundingBox.min[axis] = medianAxisSplitValue;

    auto leftNode = this->generate(leftSplitTriangles, leftBoundingBox, depth + 1);
    auto rightNode = this->generate(rightSplitTriangles, rightBoundingBox, depth + 1);

    return std::make_unique<KDNode>(KDNode{std::move(leftNode), std::move(rightNode), boundingBox, false, {}});
}
