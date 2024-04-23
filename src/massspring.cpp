#include "massspring.hpp"

namespace COL781 {
    Particle::Particle(glm::vec3 position, glm::vec3 velocity, float mass, bool isFixed) {
        this->position = position;
        this->velocity = velocity;
        this->mass = mass;
        this->isFixed = isFixed;
        force = glm::vec3(0.0f);
    }

    void Particle::update(float dt) {
        if (!isFixed) {
            glm::vec3 acceleration = force/mass;
            velocity += dt*acceleration;
            position += dt*velocity;
        }
    }

    Spring::Spring(Particle *p1, Particle *p2, float restLength, float stiffness, float damping) {
        this->p1 = p1;
        this->p2 = p2;
        this->restLength = restLength;
        this->stiffness = stiffness;
        this->damping = damping;
    }

    void Spring::update(float dt) {
        glm::vec3 direction = p2->position - p1->position;
        float length = glm::length(direction);
        direction = glm::normalize(direction);
        glm::vec3 vel = p2->velocity - p1->velocity;
        glm::vec3 force = (-stiffness*(length - restLength) - damping*(glm::dot(vel, direction))) * direction;
        p1->force -= force;
        p2->force += force;
    }

    Grid::Grid(float width, float height, int nw, int nh) {
        this->width = width;
        this->height = height;
        this->nw = nw;
        this->nh = nh;
        particles = std::vector<std::vector<Particle>>(nw, std::vector<Particle>(nh, Particle(glm::vec3(0.0f), glm::vec3(0.0f), 0.0f, false)));
        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                particles[i][j] = Particle(glm::vec3(i*width/(nw-1) - width/2, 0.0f, j*height/(nh-1) - height/2), glm::vec3(0.0f), 1.0f, false);
            }
        }

        // fixing edge particles
        for (int i = 0; i < nw; i++) {
            particles[i][0].isFixed = true;
        }

        float ke = 100.0f;
        float kd = 10.0f;
        float kf = 50.0f;
        float kg = 20.0f;


        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                if (i < nw-1) {
                    edgesprings.push_back(Spring(&particles[i][j], &particles[i+1][j], width/(nw-1), ke, kd));
                }
                if (j < nh-1) {
                    edgesprings.push_back(Spring(&particles[i][j], &particles[i][j+1], height/(nh-1), ke, kd));
                }
                if (i < nw-1 && j < nh-1) {
                    facesprings.push_back(Spring(&particles[i][j], &particles[i+1][j+1], sqrt(width*width/(nw-1)/(nw-1) + height*height/(nh-1)/(nh-1)), kf, kd));
                    facesprings.push_back(Spring(&particles[i+1][j], &particles[i][j+1], sqrt(width*width/(nw-1)/(nw-1) + height*height/(nh-1)/(nh-1)), kf, kd));
                }
            }
        }

        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh-2; j++) {
                ghostsprings.push_back(Spring(&particles[i][j], &particles[i][j+2], 2*height/(nh-1), kg, kd));
            }
        }

        for (int i = 0; i < nw-2; i++) {
            for (int j = 0; j < nh; j++) {
                ghostsprings.push_back(Spring(&particles[i][j], &particles[i+2][j], 2*width/(nw-1), kg, kd));
            }
        }
    }

    void Grid::render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles) {
        vertices.clear();
        triangles.clear();
        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                vertices.push_back(particles[i][j].position);
            }
        }
        for (int i = 0; i < nw-1; i++) {
            for (int j = 0; j < nh-1; j++) {
                triangles.push_back(glm::ivec3(i*nh+j, i*nh+j+1, (i+1)*nh+j));
                triangles.push_back(glm::ivec3(i*nh+j+1, (i+1)*nh+j+1, (i+1)*nh+j));
            }
        }
    }

    void Grid::update(float dt) {
        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                particles[i][j].force = this->gravity;
            }
        }
        for (Spring &spring : edgesprings) {
            spring.update(dt);
        }
        for (Spring &spring : facesprings) {
            spring.update(dt);
        }
        for (Spring &spring : ghostsprings) {
            spring.update(dt);
        }
        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                // std::cout << "force on particle " << i << " " << j << " " << particles[i][j].force.x << " " << particles[i][j].force.y << " " << particles[i][j].force.z << "\n";
                particles[i][j].update(dt);
            }
        }
    }
}