#ifndef KEYFRAMING_HPP
#define KEYFRAMING_HPP

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "hw.hpp"

using namespace std;
using namespace COL781::OpenGL;

class Quaternion {
private:
    float scalar;
    glm::vec3 vec;

public:
    Quaternion();
    Quaternion(float, glm::vec3);
    void init(float, glm::vec3);
    Quaternion normalize();
    float get_scalar() const;
    glm::vec3 get_vector() const;
    Quaternion conj();
    Quaternion operator+(const Quaternion&) const;
    Quaternion operator-(const Quaternion&) const;
    Quaternion operator*(const Quaternion&) const;
    Quaternion operator*(float) const;
    Quaternion operator/(float) const;
    static Quaternion interpolate(Quaternion , Quaternion, Quaternion, Quaternion, float);
};

class MeshData {
private:
    int nv, nt;
    glm::vec3 *vertices;
    glm::ivec3 *triangles;

public:
    MeshData(int, int, glm::vec3*, glm::ivec3*);
    int get_num_vertices() const;
    int get_num_triangles() const;
    glm::vec3* get_vertices() const;
    glm::ivec3* get_triangles() const;
};

class Joint {
private:
    Joint *parent;
    glm::vec3 axis;
    glm::mat4 position;
    glm::mat4 translation;
    Quaternion rotation;

public:
    Joint(Joint*, glm::vec3, glm::mat4);
    Joint* get_parent() const;
    glm::vec3 get_axis() const;
    glm::mat4 get_translation_mat() const;
    Quaternion get_rotation_quat() const;
    void get_transformed_mesh(const MeshData&, Quaternion, glm::mat4, int&, int&, glm::vec3**, glm::vec3**, glm::ivec3**);
};

class KeyFrame {
private:
    vector<float> rotations;
    vector<glm::vec3> translations;
    float time;                 // in milliseconds

public:
    explicit KeyFrame(float);
    float get_time() const;
    void add_rotations(const vector<float>&);
    void add_translations(const vector<glm::vec3>&);
    float get_rotation(int) const;
    glm::vec3 get_translation(int) const;
};

class Animation {
private:
    int len;
    vector<Joint*> joints;
    vector<MeshData> meshes;
    vector<KeyFrame> keyframes;

public:
    Animation(int);
    void add_joint(int, glm::vec3, glm::mat4);
    void add_mesh(int, int, glm::vec3*, glm::ivec3*);
    void add_keyframe(const KeyFrame&);
    void get_frame(float, vector<glm::vec3*>&, vector<glm::vec3*>&, vector<glm::ivec3*>&);
};

#endif //KEYFRAMING_HPP
