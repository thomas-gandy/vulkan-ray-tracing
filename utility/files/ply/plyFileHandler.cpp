//
// Created by Thomas Gandy on 22/02/2023.
//

#include <iostream>
#include <stdexcept>
#include <random>

#include "plyFileHandler.h"
#include "plyHelper.h"

PlyFileHandler::PlyFileHandler(const std::string &filepath) {
    int numberOfElementsInObject;
    char** elementNames;
    int fileType;
    float version;
    PlyFile* plyFile = ply_open_for_reading(
            const_cast<char*>(filepath.c_str()),
            &numberOfElementsInObject,
            &elementNames,
            &fileType,
            &version
    );
    if (!plyFile) throw std::runtime_error("Failed to open PLY file: " + filepath);

    std::random_device randomDevice;
    std::mt19937 randomGenerator(randomDevice());
    std::uniform_real_distribution floatGenerator(0.f, 1.f);

    for (int i = 0; i < numberOfElementsInObject; i++) {
        int numberOfElementInstances, numberOfPropertiesForElement;
        ply_get_element_description(plyFile, elementNames[i], &numberOfElementInstances, &numberOfPropertiesForElement);

        if (std::string(elementNames[i]) == "vertex") {
            ply_get_property(plyFile, elementNames[i], vertexProperties + 0);
            ply_get_property(plyFile, elementNames[i], vertexProperties + 1);
            ply_get_property(plyFile, elementNames[i], vertexProperties + 2);

            PlyVertex plyVertex{};
            Vertex finalVertex{{}, {}};
            for (int j = 0; j < numberOfElementInstances; j++) {
                const auto randomFloat = floatGenerator(randomGenerator);
                const auto shade = glm::vec3{randomFloat, randomFloat, randomFloat};

                ply_get_element(plyFile, (void*)&plyVertex);
                finalVertex.position.x = plyVertex.x;
                finalVertex.position.y = plyVertex.y;
                finalVertex.position.z = plyVertex.z;
                finalVertex.color = {1, 1, 1};
//                finalVertex.color = shade;
//                finalVertex.color = generateRandomColor();

                this->vertices.push_back(finalVertex);
            }
        }
        else if (std::string(elementNames[i]) == "face") {
            ply_get_property(plyFile, elementNames[i], faceProperties + 0);

            PlyFace plyFace{};
            for (int j = 0; j < numberOfElementInstances; j++) {
                ply_get_element(plyFile, (void*)&plyFace);
                Triangle triangle{};
                triangle.v0 = this->vertices[plyFace.vertices[0]];
                triangle.v1 = this->vertices[plyFace.vertices[1]];
                triangle.v2 = this->vertices[plyFace.vertices[2]];

                this->triangles.push_back(triangle);
            }
        }
    }
    ply_close(plyFile);
}

VertexModel PlyFileHandler::getVertexModel() const {
    VertexModel vertexModel;
    vertexModel.triangles = this->triangles;

    return vertexModel;
}
