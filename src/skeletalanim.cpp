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

const int nv = 4;
const int nt = 2;
vec3 vertices[nv];
vec3 normals[nv];
ivec3 triangles[nt];

GL::Object object;
GL::AttribBuf vertexBuf, normalBuf;

CameraControl camCtl;

Animation initializeScene() {
    vertices[0] = vec3(0.4, -0.5, 0);
    vertices[1] = vec3(0.6, -0.5, 0);
    vertices[2] = vec3(0.6, 0, 0);
    vertices[3] = vec3(0.4, 0, 0);
    triangles[0] = ivec3(0, 1, 2);
    triangles[1] = ivec3(0, 2, 3);

    auto translation_mat = glm::translate(glm::mat4(1.0f), glm::vec3(0.0, -0.5, 0.0));

    Animation anim;
    anim.add_joint(-1, {-1.0f, 0.0f, 0.0f}, mat4(1.0f));
    anim.add_joint(0, {-1.0f, 0.0f, 0.0f}, translation_mat);
    anim.add_mesh(4, 2, vertices, normals, triangles);
    anim.add_mesh(4, 2, vertices, normals, triangles);

    KeyFrame f1(0.0f), f2(3000.0f), f3(6000.0f);
    f1.add_rotations({0.0f, 0.0f});
    f2.add_rotations({90.0f, 90.0f});
    f3.add_rotations({0.0f, 0.0f});
//    f1.add_rotation(0.0f);
//    f2.add_rotation(90.0f);
    f3.add_rotation(0.0f);

    anim.add_keyframe(f1);
    anim.add_keyframe(f2);
    anim.add_keyframe(f3);

    return anim;
}

int main() {
    int width = 640, height = 480;
    if (!r.initialize("Animation", width, height)) {
        return EXIT_FAILURE;
    }
    camCtl.initialize(width, height);
    camCtl.camera.setCameraView(vec3(0.5, -0.5, 1.5), vec3(0.5, -0.5, 0.0), vec3(0.0, 1.0, 0.0));
    program = r.createShaderProgram(
            r.vsBlinnPhong(),
            r.fsBlinnPhong()
    );

    auto anim = initializeScene();
    vector<glm::vec3*> vertices_vec, normals_vec;
    vector<glm::ivec3*> triangles_vec;

    while (!r.shouldQuit() && SDL_GetTicks64() < 6000) {
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

            r.createVertexAttribs(object, 0, 4, vertices_vec[i]);
            r.createTriangleIndices(object, 2, triangles_vec[i]);
            //        r.createVertexAttribs(object, 0, 4, vertices);
            //        r.createTriangleIndices(object, 2, triangles);

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
