#include "massspring.hpp"

namespace COL781 {
    Particle::Particle(glm::vec3 position, glm::vec3 velocity, float mass, bool isFixed) {
        this->position = position;
        this->velocity = velocity;
        this->mass = mass;
        this->isFixed = isFixed;
        this->force = glm::vec3(0.0f);
    }

    void Particle::update(float dt) {
        if (!isFixed) {
            glm::vec3 acceleration = force/mass + glm::vec3(0.0f, -9.8f, 0.0f);
            velocity += dt*acceleration;
            oldpos = position;
            position += dt*velocity;
        }
    }

    Spring::Spring(Particle *p1, Particle *p2, float restLength, float stiffness, float damping) {
        this->p1 = p1;
        this->p2 = p2;
        this->restLength = restLength;
        this->stiffness = stiffness;
        this->damping = damping;
        this->isConstraint = false;
    }

    void Spring::update(float dt) {
        if (!isConstraint) {
            glm::vec3 direction = p2->position - p1->position;
            float length = glm::length(direction);
            direction = glm::normalize(direction);
            glm::vec3 vel = p2->velocity - p1->velocity;
            glm::vec3 force = (-stiffness*(length - restLength) - damping*(glm::dot(vel, direction))) * direction;
            p1->force -= force;
            p2->force += force;
        }
    }

    void Spring::solveConstraint() {
        if (isConstraint) {
            glm::vec3 direction = p1->position - p2->position;
            float length = glm::length(direction);
            direction = glm::normalize(direction);
            float sc = 0.5f * (length - restLength);
            glm::vec3 dstproj = sc * direction * 0.1f;
            if (!p1->isFixed) {
                p1->position -= dstproj;
            }
            if (!p2->isFixed) {
                p2->position += dstproj;
            }
        }
    }

    Grid::Grid(float width, float height, int nw, int nh, bool pbd_val) {
        this->width = width;
        this->height = height;
        this->nw = nw;
        this->nh = nh;
        this->pbd = pbd_val;
        particles = std::vector<std::vector<Particle>>(nw, std::vector<Particle>(nh, Particle(glm::vec3(0.0f), glm::vec3(0.0f), 0.0f, false)));
        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                particles[i][j] = Particle(glm::vec3(i*width/(nw-1) - width/2, 0.0f, j*height/(nh-1) - height/2), glm::vec3(0.0f), 1.0f, false);
            }
        }

        // fixing edge particles
        // for (int i = 0; i < nw; i++) {
        //     particles[i][0].isFixed = true;
        //     // particles[i][nh-1].isFixed = true;
        // }
        particles[0][0].isFixed = true;
        particles[nw-1][0].isFixed = true;

        float scale = 1;
        float ke = 400.0f * sqrt(nw*nh) * scale;
        float kf = 350.0f * sqrt(nw*nh) * scale;
        float kg = 200.0f * sqrt(nw*nh) * scale;
        float kd = 150.0f * scale;


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
        if (pbd) {
            for (int i = 0; i < edgesprings.size(); i++) {
                edgesprings[i].isConstraint = true;
            }
        }
    }

    void Grid::render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) {
        vertices.clear();
        triangles.clear();
        normals.clear();
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

        int nv = vertices.size();
        int nt = triangles.size();
        normals = std::vector<glm::vec3>(nv, glm::vec3(0.0f));
        for (int i = 0; i < nt; i++) {
            glm::vec3 normal = glm::normalize(glm::cross(vertices[triangles[i].z] - vertices[triangles[i].y], vertices[triangles[i].x] - vertices[triangles[i].y]));
            for (int j = 0; j < 3; j++) {
                normals[triangles[i][j]] += normal;
            }
        }
        for (int i = 0; i < nv; i++) {
            normals[i] = glm::normalize(normals[i]);
        }
    }

    void Grid::update(float dt) {
        if (pbd == false) {
            for (int i = 0; i < nw; i++) {
                for (int j = 0; j < nh; j++) {
                    particles[i][j].force = glm::vec3(0.0f);
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
                    particles[i][j].update(dt);
                }
            }
        } else {
            for (int i = 0; i < nw; i++) {
                for (int j = 0; j < nh; j++) {
                    particles[i][j].force = glm::vec3(0.0f);
                }
            }
            for (int i = 0; i < nw; i++) {
                for (int j = 0; j < nh; j++) {
                    particles[i][j].force += glm::vec3(0.0f, -9.8f, 0.0f);
                }
            }
            for (Spring &spring : facesprings) {
                spring.update(dt);
            }
            for (Spring &spring : ghostsprings) {
                spring.update(dt);
            }
            for (int i = 0; i < nw; i++) {
                for (int j = 0; j < nh; j++) {
                    particles[i][j].update(dt);
                }
            }
            for (int i = 0; i < 50; i++) {
                for (Spring &spring : edgesprings) {
                    spring.solveConstraint();
                }
            }
            for (int i = 0; i < nw; i++) {
                for (int j = 0; j < nh; j++) {
                    if (!particles[i][j].isFixed) {
                        particles[i][j].velocity = (particles[i][j].position - particles[i][j].oldpos)/dt;
                    }
                }
            }
        }
    }
}