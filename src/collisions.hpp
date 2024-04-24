#ifndef COLLISIONS_HPP
#define COLLISIONS_HPP

#include "hw.hpp"
#include <iostream>
#include <vector>
#include "massspring.hpp"

namespace COL781 {
    class Obstacle {
    public:
        Obstacle();
        float rs_coeff;
        float fr_coeff;
        virtual void render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) = 0;
    };

    class Sphere : public Obstacle {
    public:
        glm::vec3 center;
        float radius;
        glm::vec3 velocity;
        glm::vec3 omega;
        Sphere(glm::vec3 center, float radius, float rs_coeff, float fr_coeff);
        void setVelocity(glm::vec3 velocity);
        void setOmega(glm::vec3 omega);
        void update(float dt);
        void render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) override;
    };

    class Plane : public Obstacle {
    public:
        glm::vec3 normal;
        float d;
        Plane(glm::vec3 normal, float d, float rs_coeff, float fr_coeff);
        void render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) override;
    };

    class Cloth {
    public:
        Grid *g;
        Cloth(float width, float height, int n, int m);
        void update(float dt);
        void render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals);
    };

    class CollisionSystem {
    public:
        std::vector<Obstacle*> obstacles;
        std::vector<Particle*> particles;
        CollisionSystem();
        void addObstacle(Obstacle *obstacle);
        void addCloth(Cloth *cloth);
        void update(float dt);
    };
}

#endif