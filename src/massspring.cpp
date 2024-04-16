#include "massspring.hpp"

namespace COL781 {
    Particle::Particle(glm::vec3 position, glm::vec3 velocity, float mass, bool isFixed) {
        this->position = position;
        this->velocity = velocity;
        this->mass = mass;
        this->isFixed = isFixed;
        force = glm::vec3(0.0f);
    }

}