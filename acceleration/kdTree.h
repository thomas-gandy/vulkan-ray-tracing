//
// Created by Thomas Gandy on 31/01/2023.
//

#ifndef PROCEDURALGENERATION_KDTREE_H
#define PROCEDURALGENERATION_KDTREE_H

#include <vector>
#include <memory>
#include <glm/glm.hpp>

#include "../geometry.h"


// Axis Aligned
struct BoundingBox {
    glm::vec3 min{};
    glm::vec3 max{};
};

struct KDNode {
    std::unique_ptr<KDNode> left;
    std::unique_ptr<KDNode> right;
    BoundingBox boundingBox;

    bool isLeaf;
    std::vector<Triangle> trianglesInWorldSpace;
};

class KDTree {
    std::unique_ptr<KDNode> root{};
    int maxDepth{};
    int furthestDepthGenerated = 0;

    static int getBestSplittingAxis_largestAxisRegionInBB(const BoundingBox& boundingBox);
    static int getBestSplittingAxis_largestVariance(const std::vector<glm::vec3>& vertices);

    /**
     * Create tight bounding box around passed triangles
     * @param triangles
     * @return
     */
    static BoundingBox generateBoundingBox(std::vector<Triangle> &triangles); // called only once

    /**
     * Create KD-Tree structure from passed triangles
     * @param triangles
     * @param boundingBox
     * @param depth
     * @return root of created KD-Tree
     */
    std::unique_ptr<KDNode> generate(std::vector<Triangle>& triangles, const BoundingBox& boundingBox, int depth);

public:
    KDTree() = default;
    explicit KDTree(std::vector<VertexModel>& models);

    [[nodiscard]] const std::unique_ptr<KDNode>& getRoot() const;
};


#endif //PROCEDURALGENERATION_KDTREE_H
