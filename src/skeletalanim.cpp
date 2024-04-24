#include "camera.hpp"
#include "keyframing.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include <iostream>

using namespace std;

using namespace COL781;
namespace GL = COL781::OpenGL;
using namespace glm;

GL::Rasterizer r;
GL::ShaderProgram program;

const int nv = 8;
const int nt = 12;
vec3 vertices[nv];
vec3 normals[nv];
ivec3 triangles[nt];

GL::Object object;
//GL::AttribBuf vertexBuf, normalBuf;

CameraControl camCtl;

Animation initializeScene() {
    Animation anim(12000);
    glm::mat4 translation_mat;

    triangles[0] = ivec3(0, 1, 2);
    triangles[1] = ivec3(0, 2, 3);
    triangles[2] = ivec3(1, 5, 6);
    triangles[3] = ivec3(1, 6, 2);
    triangles[4] = ivec3(5, 4, 7);
    triangles[5] = ivec3(5, 7, 6);
    triangles[6] = ivec3(4, 0, 3);
    triangles[7] = ivec3(4, 3, 7);
    triangles[8] = ivec3(4, 5, 1);
    triangles[9] = ivec3(4, 1, 0);
    triangles[10] = ivec3(3, 2, 6);
    triangles[11] = ivec3(3, 6, 7);

    // backbone
    vertices[0] = vec3(0.0, 0.0, 0.0);
    vertices[1] = vec3(0.05, 0.0, 0.0);
    vertices[2] = vec3(0.05, 0.6, 0.0);
    vertices[3] = vec3(0.0, 0.6, 0.0);
    vertices[4] = vec3(0.0, 0.0, -0.05);
    vertices[5] = vec3(0.05, 0.0, -0.05);
    vertices[6] = vec3(0.05, 0.6, -0.05);
    vertices[7] = vec3(0.0, 0.6, -0.05);

    anim.add_joint(-1, {1.0f, 0.0f, 0.0f}, mat4(1.0f));
    anim.add_mesh(nv, nt, vertices, triangles);

    // shoulder and hip bones
    vertices[0] = vec3(-0.15, 0.0, 0.025);
    vertices[1] = vec3(0.0, 0.0, 0.025);
    vertices[2] = vec3(0.0, 0.05, 0.025);
    vertices[3] = vec3(-0.15, 0.05, 0.025);
    vertices[4] = vec3(-0.15, 0.0, -0.025);
    vertices[5] = vec3(0.0, 0.0, -0.025);
    vertices[6] = vec3(0.0, 0.05, -0.025);
    vertices[7] = vec3(-0.15, 0.05, -0.025);

    // left shoulder
    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, 0.55f, -0.025f));
    anim.add_joint(0, {0.0f, 1.0f, 0.0f}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // left hip
    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, 0.0f, -0.025f));
    anim.add_joint(0, {0.0f, 1.0f, 0.0f}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    vertices[0] = vec3(0.0, 0.0, 0.025);
    vertices[1] = vec3(0.15, 0.0, 0.025);
    vertices[2] = vec3(0.15, 0.05, 0.025);
    vertices[3] = vec3(0.0, 0.05, 0.025);
    vertices[4] = vec3(0.0, 0.0, -0.025);
    vertices[5] = vec3(0.15, 0.0, -0.025);
    vertices[6] = vec3(0.15, 0.05, -0.025);
    vertices[7] = vec3(0.0, 0.05, -0.025);

    // right shoulder
    translation_mat = glm::translate(mat4(1.0f), vec3(0.05f, 0.55f, -0.025f));
    anim.add_joint(0, {0.0f, -1.0f, 0.0f}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // right hip
    translation_mat = glm::translate(mat4(1.0f), vec3(0.05f, 0.0f, -0.025f));
    anim.add_joint(0, {0.0f, -1.0f, 0.0f}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // arms
    vertices[0] = vec3(0.0, -0.2, 0.025);
    vertices[1] = vec3(0.05, -0.2, 0.025);
    vertices[2] = vec3(0.05, 0.0, 0.025);
    vertices[3] = vec3(0.0, 0.0, 0.025);
    vertices[4] = vec3(0.0, -0.2, -0.025);
    vertices[5] = vec3(0.05, -0.2, -0.025);
    vertices[6] = vec3(0.05, 0.0, -0.025);
    vertices[7] = vec3(0.0, 0.0, -0.025);

    // left upper arm
    translation_mat = glm::translate(mat4(1.0f), vec3(-0.2f, 0.05f, 0.0f));
    anim.add_joint(1, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // left lower arm
    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, -0.2f, 0.0f));
    anim.add_joint(5, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // right upper arm
    translation_mat = glm::translate(mat4(1.0f), vec3(0.15f, 0.05f, 0.0f));
    anim.add_joint(3, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // right lower arm
    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, -0.2f, 0.0f));
    anim.add_joint(7, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // legs
    vertices[0] = vec3(0.0, -0.3, 0.025);
    vertices[1] = vec3(0.05, -0.3, 0.025);
    vertices[2] = vec3(0.05, 0.0, 0.025);
    vertices[3] = vec3(0.0, 0.0, 0.025);
    vertices[4] = vec3(0.0, -0.3, -0.025);
    vertices[5] = vec3(0.05, -0.3, -0.025);
    vertices[6] = vec3(0.05, 0.0, -0.025);
    vertices[7] = vec3(0.0, 0.0, -0.025);

    // left upper leg
    translation_mat = glm::translate(mat4(1.0f), vec3(-0.2f, 0.05f, 0.0f));
    anim.add_joint(2, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // left lower leg
    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, -0.3f, 0.0f));
    anim.add_joint(9, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // right upper leg
    translation_mat = glm::translate(mat4(1.0f), vec3(0.15f, 0.05f, 0.0f));
    anim.add_joint(4, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // right lower leg
    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, -0.3f, 0.0f));
    anim.add_joint(11, {-1.0, 0.0, 0.0}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    // head
    vertices[0] = vec3(-0.025, 0.0, 0.025);
    vertices[1] = vec3(0.025, 0.0, 0.025);
    vertices[2] = vec3(0.025, 0.15, 0.025);
    vertices[3] = vec3(-0.025, 0.15, 0.025);
    vertices[4] = vec3(-0.025, 0.0, -0.025);
    vertices[5] = vec3(0.025, 0.0, -0.025);
    vertices[6] = vec3(0.025, 0.15, -0.025);
    vertices[7] = vec3(-0.025, 0.15, -0.025);

    translation_mat = glm::translate(mat4(1.0f), vec3(0.025f, 0.6f, -0.025f));
    anim.add_joint(0, {0.0f, 1.0f, 0.0f}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    vertices[0] = vec3(-0.025, -0.05, 0.15);
    vertices[1] = vec3(0.025, -0.05, 0.15);
    vertices[2] = vec3(0.025, 0.0, 0.15);
    vertices[3] = vec3(-0.025, 0.0, 0.15);
    vertices[4] = vec3(-0.025, -0.05, 0.0);
    vertices[5] = vec3(0.025, -0.05, 0.0);
    vertices[6] = vec3(0.025, 0.0, 0.0);
    vertices[7] = vec3(-0.025, 0.0, 0.0);

    translation_mat = glm::translate(mat4(1.0f), vec3(0.0f, 0.15f, 0.025f));
    anim.add_joint(13, {1.0f, 0.0f, 0.0f}, translation_mat);
    anim.add_mesh(nv, nt, vertices, triangles);

    KeyFrame f1(0.0f), f2(3000.0f), f3(6000.0f), f4(9000.0f), f5(12000);
    f1.add_rotations({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -30.0f, 0.0f, 30.0f, 30.0f, 30.0f, 0.0f, -30.0f, -30.0f, 0.0f, 45.0f});
    f2.add_rotations({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 30.0f, -30.0f, 0.0f, -30.0f, -30.0f, 30.0f, 0.0f, 20.0f, 45.0f});
    f3.add_rotations({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -30.0f, 0.0f, 30.0f, 30.0f, 30.0f, 0.0f, -30.0f, -30.0f, -20.0f, 45.0f});
    f4.add_rotations({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f, 30.0f, -30.0f, 0.0f, -30.0f, -30.0f, 30.0f, 0.0f, 0.0f, 45.0f});
    f5.add_rotations({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -30.0f, 0.0f, 30.0f, 30.0f, 30.0f, 0.0f, -30.0f, -30.0f, 0.0f, 45.0f});

    anim.add_keyframe(f1);
    anim.add_keyframe(f2);
    anim.add_keyframe(f3);
    anim.add_keyframe(f4);
    anim.add_keyframe(f5);

    return anim;
}

int main() {
    int width = 640, height = 480;
    if (!r.initialize("Animation", width, height)) {
        return EXIT_FAILURE;
    }
    camCtl.initialize(width, height);
    camCtl.camera.setCameraView(vec3(0.0, 0.0, 1.5), vec3(0.0, 0.0, -1.0), vec3(0.0, 1.0, 0.0));
    program = r.createShaderProgram(
            r.vsBlinnPhong(),
            r.fsBlinnPhong()
    );

    auto anim = initializeScene();
    vector<glm::vec3*> vertices_vec, normals_vec;
    vector<glm::ivec3*> triangles_vec;

    while (!r.shouldQuit()) {
        r.clear(vec4(1.0f));
        r.enableDepthTest();
        r.useShaderProgram(program);

        camCtl.update();
        Camera &camera = camCtl.camera;

        r.setUniform(program, "model", glm::mat4(1.0));
        r.setUniform(program, "view", camera.getViewMatrix());
        r.setUniform(program, "projection", camera.getProjectionMatrix());
        r.setUniform(program, "lightPos", camera.position);
        r.setUniform(program, "viewPos", camera.position);
        r.setUniform(program, "lightColor", vec3(1.0f, 1.0f, 1.0f));

        anim.get_frame(static_cast<float>(SDL_GetTicks64()), vertices_vec, normals_vec, triangles_vec);

        for (int i = 0; i < vertices_vec.size(); i++) {
            object = r.createObject();

            r.createVertexAttribs(object, 0, 8, vertices_vec[i]);
            r.createVertexAttribs(object, 1, 8, normals_vec[i]);
            r.createTriangleIndices(object, 12, triangles_vec[i]);

            r.setupFilledFaces();
            glm::vec3 orange(1.0f, 0.6f, 0.2f);
            glm::vec3 white(1.0f, 1.0f, 1.0f);
            r.setUniform(program, "ambientColor", 0.4f * orange);
            r.setUniform(program, "diffuseColor", 0.9f * orange);
            r.setUniform(program, "specularColor", 0.8f * white);
            r.setUniform(program, "phongExponent", 100.f);
            r.drawObject(object);

            r.setupWireFrame();
            glm::vec3 black(0.0f, 0.0f, 0.0f);
            r.setUniform(program, "ambientColor", black);
            r.setUniform(program, "diffuseColor", black);
            r.setUniform(program, "specularColor", black);
            r.setUniform(program, "phongExponent", 0.f);
            r.drawObject(object);
        }

        r.show();

        vertices_vec.clear();
        normals_vec.clear();
        triangles_vec.clear();
    }
}
