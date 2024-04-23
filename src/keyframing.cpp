#include <cmath>
#include <cassert>
#include <utility>
#include "keyframing.hpp"

using namespace std;
using namespace COL781::OpenGL;

// Quaternion definitions
Quaternion::Quaternion() : scalar(1.0f), vec(glm::vec3(0.0f)) {}

Quaternion::Quaternion(float angle, glm::vec3 axis) {
    float angle_in_radians = angle * static_cast<float>(M_PI) / 180.0f;
    scalar = cos(angle_in_radians / 2.0f);
    vec = sin(angle_in_radians / 2.0f) * axis;
    this->normalize();
}

void Quaternion::init(float s, glm::vec3 v) {
    scalar = s;
    vec = v;
    this->normalize();
}

Quaternion Quaternion::normalize() {
    auto mag = sqrt(scalar * scalar + dot(vec, vec));
    scalar /= mag;
    vec /= mag;

    return *this;
}

float Quaternion::get_scalar() const {
    return scalar;
}

glm::vec3 Quaternion::get_vector() const {
    return vec;
}

Quaternion Quaternion::conj() {
    Quaternion q;
    q.init(scalar, -vec);
    return q;
}

Quaternion Quaternion::operator+(const Quaternion &op) const {
    Quaternion res;
    res.init(scalar + op.get_scalar(), vec + op.get_vector());
    return res;
}

Quaternion Quaternion::operator-(const Quaternion &op) const {
    Quaternion res;
    res.init(scalar - op.get_scalar(), vec - op.get_vector());
    return res;
}

Quaternion Quaternion::operator*(const Quaternion &op) const {
    Quaternion res;
    float s = scalar * op.get_scalar() - glm::dot(vec, op.get_vector());
    glm::vec3 v = scalar * op.get_vector() + op.get_scalar() * vec + glm::cross(vec, op.get_vector());

    res.init(s, v);
    return res;
}

Quaternion Quaternion::operator*(float s) const {
    Quaternion res;
    res.init(s * scalar, s * vec);
    return res;
}

Quaternion Quaternion::operator/(float s) const {
    Quaternion res;
    res.init(scalar / s, vec / s);
    return res;
}

Quaternion Quaternion::interpolate(Quaternion q0, Quaternion q1, Quaternion q2, Quaternion q3, float t) {
    auto dq1 = (q2 - q0) / 2.0f;
    auto dq2 = (q3 - q1) / 2.0f;
    auto t2 = t * t;
    auto t3 = t2 * t;

    auto q = q1 * (2 * t3 - 3 * t2 + 1) + dq1 * (t3 - 2 * t2 + t) + q2 * (-2 * t3 + 3 * t2) + dq2 * (t3 - t2);
    return q.normalize();
}

// Mesh definitions
Mesh::Mesh(int nv, int nt, glm::vec3 *v, glm::vec3 *n, glm::ivec3 *t) : nv(nv), nt(nt) {
    vertices = new glm::vec3[nv];
    normals = new glm::vec3[nv];
    triangles = new glm::ivec3[nt];

    for (int i = 0; i < nv; i++) {
        vertices[i] = v[i];
        normals[i] = n[i];
    }

    for (int i = 0; i < nt; i++)
        triangles[i] = t[i];
}

int Mesh::get_num_vertices() const {
    return nv;
}

int Mesh::get_num_triangles() const {
    return nt;
}

glm::vec3* Mesh::get_vertices() const {
    return vertices;
}

glm::vec3* Mesh::get_normals() const {
    return normals;
}

glm::ivec3* Mesh::get_triangles() const {
    return triangles;
}

//void Mesh::transform(glm::mat4 t) {
//    for (int i = 0; i < nv; i++) {
//        glm::vec4 v = glm::vec4(vertices[i], 1.0f);
//        v = t * v;
//        vertices[i] = glm::vec3(v.x, v.y, v.z);
//    }
//}
//
//pair<glm::vec3*, glm::ivec3*> Mesh::transform(Quaternion q) {
//    Quaternion v;
//    auto *new_vertices = new glm::vec3[nv];
//    for (int i = 0; i < nv; i++) {
//        v.init(0.0f, vertices[i]);
//        v = q * v * q.conj();
//        new_vertices[i] = v.get_vector();
//    }
//
//    return make_pair(new_vertices, triangles);
//}

// Joint definitions
Joint::Joint(Joint *parent, glm::vec3 axis, glm::mat4 transform) : parent(parent), axis(axis),
                                                                   position(transform) {
    rotation.init(1.0f, glm::vec3(0.0f));
}

Joint* Joint::get_parent() const {
    return parent;
}

glm::vec3 Joint::get_axis() const {
    return axis;
}

void Joint::create_mesh(int nv, int nt, glm::vec3 *vertices, glm::vec3 *normals, glm::ivec3 *triangles) {
    mesh = new Mesh(nv, nt, vertices, normals, triangles);
}

glm::mat4 Joint::get_translation_mat() const {
    return position;
}

Quaternion Joint::get_rotation_quat() const {
    return rotation;
}

int Joint::get_num_vertices() const {
    return mesh->get_num_vertices();
}

int Joint::get_num_triangles() const {
    return mesh->get_num_triangles();
}

void Joint::get_transformed_mesh(Quaternion q, int &nv, int &nt, glm::vec3 **vertices, glm::vec3 **normals, glm::ivec3 **triangles) {
    rotation = q.normalize();

    nv = mesh->get_num_vertices();
    nt = mesh->get_num_triangles();

    auto orig_vertices = mesh->get_vertices();
    *triangles = mesh->get_triangles();

    *vertices = new glm::vec3[nv];
    Quaternion v;
    Joint* p = parent;
    for (int i = 0; i < nv; i++) {
        v.init(0.0f, orig_vertices[i]);
        auto translation_mat = position;
        do {
            v = q * v * q.conj();
            glm::vec4 v2(v.get_vector(), 1.0f);
            v2 = translation_mat * v2;
            v.init(0.0f, {v2.x, v2.y, v2.z});
            if (p)
                p = p->get_parent();
            if (p) {
                translation_mat = p->get_translation_mat();
                q = p->get_rotation_quat().normalize();
            }
        } while (p);
        (*vertices)[i] = v.get_vector();
    }
}

//KeyFrame definitions
KeyFrame::KeyFrame(float time) : time(time) {}

float KeyFrame::get_time() const {
    return time;
}

void KeyFrame::add_rotation(float angle) {
    rotations.push_back(angle);
}

float KeyFrame::get_rotation(int i) const {
    return rotations[i];
}

// Animation definitions
Animation::Animation(vector<Joint> joints): prev_keyframe(0), joints(std::move(joints)) {}

void Animation::add_keyframe(const KeyFrame& kf) {
    keyframes.push_back(kf);
}

void Animation::get_frame(float timestamp, vector<glm::vec3*> &vertices, vector<glm::vec3*> &normals,
                          vector<glm::ivec3*> &triangles) {
    while (prev_keyframe < keyframes.size() && keyframes[prev_keyframe + 1].get_time() <= timestamp)
        prev_keyframe++;

    // check that the timestamp does not surpass the animation length
    assert(prev_keyframe < keyframes.size() - 1);

    float t = (timestamp - keyframes[prev_keyframe].get_time()) /
            (keyframes[prev_keyframe + 1].get_time() - keyframes[prev_keyframe].get_time());

    for (int i = 0; i < joints.size(); i++) {
        auto joint = joints[i];

        auto theta1 = keyframes[prev_keyframe].get_rotation(i);
        auto theta2 = keyframes[prev_keyframe + 1].get_rotation(i);
        auto theta0 = keyframes[(prev_keyframe > 0) ? prev_keyframe - 1 : prev_keyframe].get_rotation(i);
        auto theta3 = keyframes[(prev_keyframe + 1 < keyframes.size()) ? prev_keyframe + 1 : prev_keyframe].get_rotation(i);
        auto axis = joint.get_axis();

        auto nv = joint.get_num_vertices();
        auto nt = joint.get_num_triangles();
        auto *transformed_vertices = new glm::vec3[nv];
        auto *transformed_normals = new glm::vec3[nv];
        auto *joint_triangles = new glm::ivec3[nt];

        Quaternion q0(theta0, axis), q1(theta1, axis), q2(theta2, axis), q3(theta3, axis);
        auto q = Quaternion::interpolate(q0, q1, q2, q3, t);

        joint.get_transformed_mesh(q, nv, nt, &transformed_vertices, &transformed_normals, &joint_triangles);

        vertices.push_back(transformed_vertices);
        normals.push_back(transformed_normals);
        triangles.push_back(joint_triangles);
    }
}
