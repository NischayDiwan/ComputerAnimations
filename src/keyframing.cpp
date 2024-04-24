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
    res.init(op.get_scalar() - scalar, op.get_vector() - vec);
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

// MeshData definitions
MeshData::MeshData(int nv, int nt, glm::vec3 *v, glm::ivec3 *t) : nv(nv), nt(nt) {
    vertices = new glm::vec3[nv];
    triangles = new glm::ivec3[nt];

    for (int i = 0; i < nv; i++)
        vertices[i] = v[i];

    for (int i = 0; i < nt; i++)
        triangles[i] = t[i];
}

int MeshData::get_num_vertices() const {
    return nv;
}

int MeshData::get_num_triangles() const {
    return nt;
}

glm::vec3* MeshData::get_vertices() const {
    return vertices;
}

glm::ivec3* MeshData::get_triangles() const {
    return triangles;
}

// Joint definitions
Joint::Joint(Joint *parent, glm::vec3 axis, glm::mat4 position) : parent(parent), axis(axis),
                                                                  position(position) {
    rotation.init(1.0f, glm::vec3(0.0f));
}

Joint* Joint::get_parent() const {
    return parent;
}

glm::vec3 Joint::get_axis() const {
    return axis;
}

glm::mat4 Joint::get_translation_mat() const {
    return translation * position;
}

Quaternion Joint::get_rotation_quat() const {
    return rotation;
}

void Joint::get_transformed_mesh(const MeshData &mesh, Quaternion q, glm::mat4 t, int &nv, int &nt,
                                 glm::vec3 **vertices, glm::vec3 **normals, glm::ivec3 **triangles) {
    rotation = q.normalize();
    translation = t;

    nv = mesh.get_num_vertices();
    nt = mesh.get_num_triangles();

    auto orig_vertices = mesh.get_vertices();
    *triangles = mesh.get_triangles();

    *vertices = new glm::vec3[nv];
    Quaternion v;
    for (int i = 0; i < nv; i++) {
        v.init(0.0f, orig_vertices[i]);
        q = rotation;
        auto translation_mat = translation * position;
        Joint* p = this;
        do {
            v = q * v * q.conj();
            glm::vec4 v2(v.get_vector(), 1.0f);
            v2 = translation_mat * v2;
            v.init(0.0f, {v2.x, v2.y, v2.z});
            p = p->get_parent();
            if (p) {
                translation_mat = p->get_translation_mat();
                q = p->get_rotation_quat().normalize();
            }
        } while (p != nullptr);
        (*vertices)[i] = v.get_vector();
    }

    *normals = new glm::vec3[nv];
    for (int i = 0; i < nt; i++) {
        glm::vec3 normal = glm::normalize(glm::cross((*vertices)[(*triangles)[i].z] - (*vertices)[(*triangles)[i].y], (*vertices)[(*triangles)[i].x] - (*vertices)[(*triangles)[i].y]));
        for (int j = 0; j < 3; j++) {
            (*normals)[(*triangles)[i][j]] += normal;
        }
    }
    for (int i = 0; i < nv; i++) {
        (*normals)[i] = glm::normalize((*normals)[i]);
    }
}

//KeyFrame definitions
KeyFrame::KeyFrame(float time) : time(time) {}

float KeyFrame::get_time() const {
    return time;
}

void KeyFrame::add_rotations(const vector<float>& angles) {
    for (auto angle : angles)
        rotations.push_back(angle);
}

void KeyFrame::add_translations(const vector<glm::vec3> &t) {
    for (auto item : t)
        translations.push_back(item);
}

float KeyFrame::get_rotation(int i) const {
    return rotations[i];
}

glm::vec3 KeyFrame::get_translation(int i) const {
    return translations[i];
}

// Animation definitions
Animation::Animation(int len) : len(len) {}

void Animation::add_joint(int parent_id, glm::vec3 axis, glm::mat4 transform) {
    Joint *parent = (parent_id < 0) ? nullptr : joints[parent_id];
    joints.push_back(new Joint(parent, axis, transform));
}

void Animation::add_mesh(int nv, int nt, glm::vec3 *vertices, glm::ivec3 *triangles) {
    meshes.emplace_back(nv, nt, vertices, triangles);
}

void Animation::add_keyframe(const KeyFrame& kf) {
    keyframes.push_back(kf);
}

void Animation::get_frame(float timestamp, vector<glm::vec3*> &vertices, vector<glm::vec3*> &normals,
                          vector<glm::ivec3*> &triangles) {
    int prev_keyframe = 0;
    timestamp = timestamp - static_cast<float>(len) * floor((timestamp / static_cast<float>(len)));

    while (prev_keyframe < keyframes.size() && keyframes[prev_keyframe + 1].get_time() <= timestamp)
        prev_keyframe++;

    float t1 = keyframes[prev_keyframe].get_time();
    float t2 = keyframes[prev_keyframe + 1].get_time();
    float t = (timestamp - t1) / (t2 - t1);

    for (int i = 0; i < joints.size(); i++) {
        auto theta1 = keyframes[prev_keyframe].get_rotation(i);
        auto theta2 = keyframes[prev_keyframe + 1].get_rotation(i);
        auto theta0 = keyframes[(prev_keyframe > 0) ? prev_keyframe - 1 : prev_keyframe].get_rotation(i);
        auto theta3 = keyframes[(prev_keyframe + 1 < keyframes.size()) ? prev_keyframe + 1 : prev_keyframe].get_rotation(i);
        auto axis = joints[i]->get_axis();

        auto nv = meshes[i].get_num_vertices();
        auto nt = meshes[i].get_num_triangles();
        auto *transformed_vertices = new glm::vec3[nv];
        auto *transformed_normals = new glm::vec3[nv];
        auto *joint_triangles = new glm::ivec3[nt];

        Quaternion q0(theta0, axis), q1(theta1, axis), q2(theta2, axis), q3(theta3, axis);
        auto q = Quaternion::interpolate(q0, q1, q2, q3, t);

        auto trans_1 = keyframes[prev_keyframe].get_translation(i);
        auto trans_2 = keyframes[prev_keyframe + 1].get_translation(i);
        auto trans = t * trans_2 + (1 - t) * trans_1;
        auto trans_mat = glm::translate(glm::mat4(1.0f), trans);

        joints[i]->get_transformed_mesh(meshes[i], q, trans_mat, nv, nt, &transformed_vertices,
                                        &transformed_normals, &joint_triangles);

        vertices.push_back(transformed_vertices);
        normals.push_back(transformed_normals);
        triangles.push_back(joint_triangles);
    }
}
