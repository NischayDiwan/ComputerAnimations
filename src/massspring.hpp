#ifndef MASSSPRING_HPP
#define MASSSPRING_HPP

#include "hw.hpp"
#include <iostream>
#include <vector>

namespace COL781 {
    class Particle {
    private:
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 force;
        float mass;
        bool isFixed;
    public:
        Particle(glm::vec3 position, glm::vec3 velocity, float mass, bool isFixed);
    };

    class Spring {
    private:
        Particle *p1;
        Particle *p2;
        float restLength;
        float stiffness;
        float damping;
    public:
        Spring(Particle *p1, Particle *p2, float restLength, float stiffness, float damping);
    };

    class Grid {
    private:
        float width;
        float height;
        int nw;
        int nh;
    public:
        std::vector<std::vector<Particle>> particles;
        Grid(int n);
        // void update(float dt);
    };

    // class MassSpring {
    // public:
    //     glm::vec3 position;
    //     glm::vec3 velocity;
    //     glm::vec3 force;
    //     float mass;
    //     float damping;
    //     float restLength;
    //     float stiffness;
    //     MassSpring();
    //     MassSpring(glm::vec3 position, glm::vec3 velocity, float mass, float damping, float restLength, float stiffness);
    //     void update(float dt);
    // };
}

#endif