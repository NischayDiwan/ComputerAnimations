#include "camera.hpp"
#include "massspring.hpp"

#include <iostream>

using namespace COL781;
namespace GL = COL781::OpenGL;
using namespace glm;

GL::Rasterizer r;
GL::ShaderProgram program;

std::vector<glm::vec3> vertices_vec;
std::vector<glm::ivec3> triangles_vec;

GL::Object object;
GL::AttribBuf vertexBuf, normalBuf;

CameraControl camCtl;

void initializeScene() {
	const int nv = vertices_vec.size();
	const int nt = triangles_vec.size();
	vec3 vertices[nv];
	vec3 normals[nv];
	ivec3 triangles[nt];

	object = r.createObject();
	for (int i = 0; i < nv; i++) {
		vertices[i] = vertices_vec[i];
	}	
	vertexBuf = r.createVertexAttribs(object, 0, nv, vertices);
	for (int i = 0; i < nv; i++) {
		normals[i] = vec3(0.0, 0.0, 1.0);
	}
	normalBuf = r.createVertexAttribs(object, 1, nv, normals);
	for (int i = 0; i < nt; i++) {
		triangles[i] = triangles_vec[i];
	}
	r.createTriangleIndices(object, nt, triangles);
}

void updateScene(float t, Grid &grid) {
	grid.update(0.001);
	grid.render(vertices_vec, triangles_vec);
	const int nv = vertices_vec.size();
	const int nt = triangles_vec.size();
	vec3 vertices[nv];
	vec3 normals[nv];
	for (int i = 0; i < nv; i++) {
		vertices[i] = vertices_vec[i];
	}
	r.updateVertexAttribs(vertexBuf, nv, vertices);
	for (int i = 0; i < nt; i++) {
		glm::vec3 normal = glm::normalize(glm::cross(vertices[triangles_vec[i].z] - vertices[triangles_vec[i].y], vertices[triangles_vec[i].x] - vertices[triangles_vec[i].y]));
		for (int j = 0; j < 3; j++) {
			normals[triangles_vec[i][j]] += normal;
		}
	}
	for (int i = 0; i < nv; i++) {
		normals[i] = glm::normalize(normals[i]);
	}
	r.updateVertexAttribs(normalBuf, nv, normals);
}

int main() {
	int width = 640, height = 480;
	if (!r.initialize("Animation", width, height)) {
		return EXIT_FAILURE;
	}
	camCtl.initialize(width, height);
	camCtl.camera.setCameraView(vec3(0.0, 0.0, 1.0), vec3(0.0, -0.5, 0.0), vec3(0.0, 1.0, 0.0));
	program = r.createShaderProgram(
		r.vsBlinnPhong(),
		r.fsBlinnPhong()
	);

	Grid grid(1.0, 1.0, 15, 15);
	grid.render(vertices_vec, triangles_vec);
	initializeScene();

	while (!r.shouldQuit()) {
        float t = SDL_GetTicks64()*1e-3;
		updateScene(t, grid);

		camCtl.update();
		Camera &camera = camCtl.camera;

		r.clear(vec4(1.0, 1.0, 1.0, 1.0));
		r.enableDepthTest();
		r.useShaderProgram(program);

		r.setUniform(program, "model", glm::mat4(1.0));
		r.setUniform(program, "view", camera.getViewMatrix());
		r.setUniform(program, "projection", camera.getProjectionMatrix());
		r.setUniform(program, "lightPos", camera.position);
		r.setUniform(program, "viewPos", camera.position);
		r.setUniform(program, "lightColor", vec3(1.0f, 1.0f, 1.0f));

		r.setupFilledFaces();
        glm::vec3 orange(1.0f, 0.6f, 0.2f);
        glm::vec3 white(1.0f, 1.0f, 1.0f);
        r.setUniform(program, "ambientColor", 0.4f*orange);
        r.setUniform(program, "diffuseColor", 0.9f*orange);
        r.setUniform(program, "specularColor", 0.8f*white);
        r.setUniform(program, "phongExponent", 100.f);
		r.drawObject(object);

		r.setupWireFrame();
        glm::vec3 black(0.0f, 0.0f, 0.0f);
        r.setUniform(program, "ambientColor", black);
        r.setUniform(program, "diffuseColor", black);
        r.setUniform(program, "specularColor", black);
        r.setUniform(program, "phongExponent", 0.f);
		r.drawObject(object);

		r.show();
	}
}
