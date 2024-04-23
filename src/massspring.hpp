#ifndef MASSSPRING_HPP
#define MASSSPRING_HPP

#include "hw.hpp"
#include <iostream>
#include <vector>

namespace COL781 {
    class Particle {
    public:
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 force;
        float mass;
        bool isFixed;
        Particle(glm::vec3 position, glm::vec3 velocity, float mass, bool isFixed);
        void update(float dt);
    };

    class Spring {
    public:
        Particle *p1;
        Particle *p2;
        float restLength;
        float stiffness;
        float damping;
        Spring(Particle *p1, Particle *p2, float restLength, float stiffness, float damping);
        void update(float dt);
    };

    class Grid {
    public:
        float width;
        float height;
        int nw;
        int nh;
        std::vector<std::vector<Particle>> particles;
        std::vector<Spring> edgesprings;
        std::vector<Spring> facesprings;
        std::vector<Spring> ghostsprings;
        Grid(float width, float height, int nw, int nh);
        void update(float dt);
        void render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles);
    };
}

#endif