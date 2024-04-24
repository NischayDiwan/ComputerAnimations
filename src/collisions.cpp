#include "collisions.hpp"

namespace COL781 {
    Obstacle::Obstacle() {
        rs_coeff = 0.0;
        fr_coeff = 0.0;
    }

    Sphere::Sphere(glm::vec3 center, float radius, float rs_coeff, float fr_coeff) {
        this->center = center;
        this->radius = radius;
        this->rs_coeff = rs_coeff;
        this->fr_coeff = fr_coeff;
        this->velocity = glm::vec3(0.0f);
        this->omega = glm::vec3(0.0f);
    }

    void Sphere::setVelocity(glm::vec3 velocity) {
        this->velocity = velocity;
    }

    void Sphere::setOmega(glm::vec3 omega) {
        this->omega = omega;
    }

    void Sphere::update(float dt) {
        center += dt * velocity;
    }

    void Sphere::render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) {
        vertices.clear();
        triangles.clear();
        normals.clear();
        int longitudes = 36;
        int latitudes = 18;

        vertices = std::vector<glm::vec3>((latitudes - 1) * longitudes + 2);
        normals = std::vector<glm::vec3>((latitudes - 1) * longitudes + 2);
        // triangles = std::vector<glm::ivec3>(latitudes * longitudes);

        // vertex calculation
        vertices[0] = glm::vec3(0.0f, 0.0f, radius);
        normals[0] = glm::vec3(0.0f, 0.0f, 1.0f);
        for (int i = 1; i < latitudes; ++i) {
            float phi = (float)i / (float)(latitudes) * M_PI;
            for (int j = 0; j < longitudes; ++j) {
                float theta = (float)j / (float)(longitudes) * 2.0f * M_PI;
                glm::vec3 vertex;
                vertex.x = radius * cos(theta) * cos((M_PI / 2.0f) - phi);       /* x = r * cos(phi) * cos(theta)  */
                vertex.y = radius * sin(theta) * cos((M_PI / 2.0f) - phi);        /* y = r * cos(phi) * sin(theta) */
                vertex.z = radius * sin((M_PI / 2.0f) - phi);                /* z = r * sin(phi) */
                vertices[((i-1) * longitudes) + j + 1] = vertex;
                // normalized vertex normal
                normals[((i-1) * longitudes) + j + 1] = glm::normalize(vertex);
            }
        }
        vertices[vertices.size()-1] = glm::vec3(0.0f, 0.0f, -radius);
        normals[normals.size()-1] = glm::vec3(0.0f, 0.0f, -1.0f);

        for (int i = 0; i < longitudes; ++i) {
            triangles.push_back(glm::ivec3(0, i + 1, (i + 1) % longitudes + 1));
        }

        for (int i = 0; i < latitudes - 2; ++i) {
            for (int j = 0; j < longitudes; ++j) {
                int k1 = i * longitudes + j + 1;
                int k2 = i * longitudes + (j + 1) % longitudes + 1;
                triangles.push_back(glm::ivec3(k1, k1 + longitudes, k2));
                triangles.push_back(glm::ivec3(k1 + longitudes, k2 + longitudes, k2));
            }
        }

        for (int i = 0; i < longitudes; ++i) {
            triangles.push_back(glm::ivec3(vertices.size() - 1, vertices.size() - 2 - i, vertices.size() - 2 - ((i + 1) % longitudes)));
        }

        for (int i = 0; i < vertices.size(); i++) {
            vertices[i] = center + vertices[i];
        }
    }

    Plane::Plane(glm::vec3 normal, float d, float rs_coeff, float fr_coeff) {
        this->normal = normal;
        this->d = d;
        this->rs_coeff = rs_coeff;
        this->fr_coeff = fr_coeff;
    }

    void Plane::render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) {
        vertices.clear();
        triangles.clear();
        normals.clear();
        float width = 10.0f;
        float height = 10.0f;
        int nw = 2;
        int nh = 2;
        for (int i = 0; i < nw; i++) {
            for (int j = 0; j < nh; j++) {
                if (glm::dot(normal, glm::vec3(0.0f, 1.0f, 0.0f)) == 1.0f) {
                    vertices.push_back(glm::vec3(i*width/(nw-1) - width/2, d, j*height/(nh-1) - height/2));
                } else if (glm::dot(normal, glm::vec3(0.0f, 1.0f, 0.0f)) == -1.0f) {
                    vertices.push_back(glm::vec3(i*width/(nw-1) - width/2, -d, j*height/(nh-1) - height/2));
                } else {
                    vertices.push_back(glm::vec3(i*width/(nw-1) - width/2, j*height/(nh-1) - height/2, (d - i*width/(nw-1) - j*height/(nh-1))/normal.z));
                }
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
        normals = std::vector<glm::vec3>(nv, normal);
    }

    Cloth::Cloth(float width, float height, int n, int m) {
        g = new Grid(width, height, n, m, false);
    }

    void Cloth::render(std::vector<glm::vec3> &vertices, std::vector<glm::ivec3> &triangles, std::vector<glm::vec3> &normals) {
        g->render(vertices, triangles, normals);
    }

    void Cloth::update(float dt) {
        g->update(dt);
    }

    CollisionSystem::CollisionSystem() {}

    void CollisionSystem::addObstacle(Obstacle *obstacle) {
        obstacles.push_back(obstacle);
    }

    void CollisionSystem::addCloth(Cloth *cloth) {
        for (std::vector<Particle> &row : cloth->g->particles) {
            for (Particle &p : row) {
                particles.push_back(&p);
            }
        }
    }

    void CollisionSystem::update(float dt) {
        for (Particle *p : particles) {
            for (Obstacle *o : obstacles) {
                if (Sphere *s = dynamic_cast<Sphere*>(o)) {
                    float d = glm::length(p->position - s->center) - s->radius - 0.01f;
                    if (d < 0.0f) {
                        glm::vec3 n = glm::normalize(p->position - s->center);
                        glm::vec3 v_rel = p->velocity - s->velocity - glm::cross(s->omega, p->position - s->center);
                        glm::vec3 v_n = glm::dot(v_rel, n) * n;
                        glm::vec3 v_t = v_rel - v_n;
                        float i_n = - (1.0f + s->rs_coeff) * p->mass * glm::dot(v_rel, n);
                        glm::vec3 i_t = - glm::min(glm::abs(s->fr_coeff * i_n), p->mass * glm::length(v_t)) * glm::normalize(v_t);
                        p->velocity += (i_n * n + i_t)/ p->mass;
                        p->position = p->position - d * n;
                    }
                } else if (Plane *pl = dynamic_cast<Plane*>(o)) {
                    float d = glm::dot(p->position, pl->normal) - pl->d - 0.001f;
                    if (d < 0.0f) {
                        glm::vec3 n = pl->normal;
                        glm::vec3 v_rel = p->velocity;
                        glm::vec3 v_n = glm::dot(v_rel, n) * n;
                        glm::vec3 v_t = v_rel - v_n;
                        float i_n = - (1.0f + pl->rs_coeff) * p->mass * glm::dot(v_rel, n);
                        glm::vec3 i_t = - glm::min(glm::abs(pl->fr_coeff * i_n), p->mass * glm::length(v_t)) * glm::normalize(v_t);
                        p->velocity += (i_n * n + i_t)/ p->mass;
                        p->position = p->position - d * n;
                    }
                }
            }
        }
    }
}