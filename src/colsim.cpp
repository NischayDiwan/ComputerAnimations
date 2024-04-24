#include "camera.hpp"
#include "massspring.hpp"
#include "collisions.hpp"

#include <iostream>

using namespace COL781;
namespace GL = COL781::OpenGL;
using namespace glm;

GL::Rasterizer r;
GL::ShaderProgram program;

std::vector<glm::vec3> vertices_vec;
std::vector<glm::ivec3> triangles_vec;
std::vector<glm::vec3> normals_vec;
Cloth cloth = Cloth(1.0, 1.0, 20, 20);
Sphere sphere = Sphere(glm::vec3(0.0, -0.8, -0.2), 0.2, 0.1, 1.0);
Plane ground = Plane(glm::vec3(0.0, 1.0, 0.0), -1.0, 0.2, 0.3);
CollisionSystem cs = CollisionSystem();

// GL::Object object;
// GL::AttribBuf vertexBuf, normalBuf;
std::vector<GL::Object> objects;
std::vector<GL::AttribBuf> vertexBufs, normalBufs;

CameraControl camCtl;

void initializeScene() {
	cloth.render(vertices_vec, triangles_vec, normals_vec);
	const int nv = vertices_vec.size();
	const int nt = triangles_vec.size();
	vec3 vertices[nv];
	vec3 normals[nv];
	ivec3 triangles[nt];

	GL::Object clothmesh = r.createObject();
	for (int i = 0; i < nv; i++) {
		vertices[i] = vertices_vec[i];
	}	
	GL::AttribBuf vertexBuf = r.createVertexAttribs(clothmesh, 0, nv, vertices);
	for (int i = 0; i < nv; i++) {
		normals[i] = normals_vec[i];
	}
	GL::AttribBuf normalBuf = r.createVertexAttribs(clothmesh, 1, nv, normals);
	for (int i = 0; i < nt; i++) {
		triangles[i] = triangles_vec[i];
	}
	r.createTriangleIndices(clothmesh, nt, triangles);
	objects.push_back(clothmesh);
	vertexBufs.push_back(vertexBuf);
	normalBufs.push_back(normalBuf);
	
	GL::Object floormesh = r.createObject();
	std::vector<glm::vec3> floor_vertices;
	std::vector<glm::ivec3> floor_triangles;
	std::vector<glm::vec3> floor_normals;
	ground.render(floor_vertices, floor_triangles, floor_normals);
	const int nfv = floor_vertices.size();
	const int nft = floor_triangles.size();
	vec3 floor_vertices_arr[nfv];
	vec3 floor_normals_arr[nfv];
	ivec3 floor_triangles_arr[nft];
	for (int i = 0; i < nfv; i++) {
		floor_vertices_arr[i] = floor_vertices[i];
	}
	GL::AttribBuf floor_vertexBuf = r.createVertexAttribs(floormesh, 0, nfv, floor_vertices_arr);
	for (int i = 0; i < nfv; i++) {
		floor_normals_arr[i] = floor_normals[i];
	}
	GL::AttribBuf floor_normalBuf = r.createVertexAttribs(floormesh, 1, nfv, floor_normals_arr);
	for (int i = 0; i < nft; i++) {
		floor_triangles_arr[i] = floor_triangles[i];
	}
	r.createTriangleIndices(floormesh, nft, floor_triangles_arr);
	objects.push_back(floormesh);
	vertexBufs.push_back(floor_vertexBuf);
	normalBufs.push_back(floor_normalBuf);

	GL::Object spheremesh = r.createObject();
	std::vector<glm::vec3> sphere_vertices;
	std::vector<glm::ivec3> sphere_triangles;
	std::vector<glm::vec3> sphere_normals;
	sphere.render(sphere_vertices, sphere_triangles, sphere_normals);
	const int nsv = sphere_vertices.size();
	const int nst = sphere_triangles.size();
	vec3 sphere_vertices_arr[nsv];
	vec3 sphere_normals_arr[nsv];
	ivec3 sphere_triangles_arr[nst];
	for (int i = 0; i < nsv; i++) {
		sphere_vertices_arr[i] = sphere_vertices[i];
	}
	GL::AttribBuf sphere_vertexBuf = r.createVertexAttribs(spheremesh, 0, nsv, sphere_vertices_arr);
	for (int i = 0; i < nsv; i++) {
		sphere_normals_arr[i] = sphere_normals[i];
	}
	GL::AttribBuf sphere_normalBuf = r.createVertexAttribs(spheremesh, 1, nsv, sphere_normals_arr);
	for (int i = 0; i < nst; i++) {
		sphere_triangles_arr[i] = sphere_triangles[i];
	}
	r.createTriangleIndices(spheremesh, nst, sphere_triangles_arr);
	objects.push_back(spheremesh);
	vertexBufs.push_back(sphere_vertexBuf);
	normalBufs.push_back(sphere_normalBuf);
	
	cs.addObstacle(&sphere);
	cs.addObstacle(&ground);
	cs.addCloth(&cloth);
}

void updateScene(float dt) {
	cloth.update(dt);
	sphere.update(dt);
	cs.update(dt);
	cloth.render(vertices_vec, triangles_vec, normals_vec);
	const int nv = vertices_vec.size();
	const int nt = triangles_vec.size();
	vec3 vertices[nv];
	vec3 normals[nv];
	for (int i = 0; i < nv; i++) {
		vertices[i] = vertices_vec[i];
	}
	r.updateVertexAttribs(vertexBufs[0], nv, vertices);
	for (int i = 0; i < nv; i++) {
		normals[i] = normals_vec[i];
	}
	r.updateVertexAttribs(normalBufs[0], nv, normals);

	std::vector<glm::vec3> floor_vertices;
	std::vector<glm::ivec3> floor_triangles;
	std::vector<glm::vec3> floor_normals;
	ground.render(floor_vertices, floor_triangles, floor_normals);
	const int nfv = floor_vertices.size();
	const int nft = floor_triangles.size();
	vec3 floor_vertices_arr[nfv];
	vec3 floor_normals_arr[nfv];
	for (int i = 0; i < nfv; i++) {
		floor_vertices_arr[i] = floor_vertices[i];
	}
	r.updateVertexAttribs(vertexBufs[1], nfv, floor_vertices_arr);
	for (int i = 0; i < nfv; i++) {
		floor_normals_arr[i] = floor_normals[i];
	}
	r.updateVertexAttribs(normalBufs[1], nfv, floor_normals_arr);

	std::vector<glm::vec3> sphere_vertices;
	std::vector<glm::ivec3> sphere_triangles;
	std::vector<glm::vec3> sphere_normals;
	sphere.render(sphere_vertices, sphere_triangles, sphere_normals);
	const int nsv = sphere_vertices.size();
	const int nst = sphere_triangles.size();
	vec3 sphere_vertices_arr[nsv];
	vec3 sphere_normals_arr[nsv];
	for (int i = 0; i < nsv; i++) {
		sphere_vertices_arr[i] = sphere_vertices[i];
	}
	r.updateVertexAttribs(vertexBufs[2], nsv, sphere_vertices_arr);
	for (int i = 0; i < nsv; i++) {
		sphere_normals_arr[i] = sphere_normals[i];
	}
	r.updateVertexAttribs(normalBufs[2], nsv, sphere_normals_arr);
}

int main() {
	int width = 640, height = 480;
	if (!r.initialize("Animation", width, height)) {
		return EXIT_FAILURE;
	}
	camCtl.initialize(width, height);
	camCtl.camera.setCameraView(vec3(0.0, 0.0, 1.5), vec3(0.0, -0.5, 0.0), vec3(0.0, 1.0, 0.0));
	program = r.createShaderProgram(
		r.vsBlinnPhong(),
		r.fsBlinnPhong()
	);
	
	initializeScene();
	sphere.setVelocity(vec3(0.0, 0.0, -0.5));
	sphere.setOmega(vec3(0.0, 5.0, 0.0));
	float st = SDL_GetTicks64()*1e-3;
	while (!r.shouldQuit()) {
        float t = SDL_GetTicks64()*1e-3;
		if (abs(t-st - 15.0) < 0.01) {
			std::cout << "Changing velocity" << std::endl;
			sphere.setVelocity(vec3(0.0, 0.0, 1.0));
		} else if (abs(t-st - 25.0) < 0.01) {
			std::cout << "Changing velocity" << std::endl;
			sphere.setVelocity(vec3(0.0, 0.0, -1.0));
		} else if ( abs(t-st - 35.0) < 0.01) {
			std::cout << "Changing velocity" << std::endl;
			sphere.setVelocity(vec3(0.0, 0.0, 1.0));
		}
		updateScene(0.001);

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
		r.drawObject(objects[0]);

		r.setupWireFrame();
        glm::vec3 black(0.0f, 0.0f, 0.0f);
        r.setUniform(program, "ambientColor", black);
        r.setUniform(program, "diffuseColor", black);
        r.setUniform(program, "specularColor", black);
        r.setUniform(program, "phongExponent", 0.f);
		r.drawObject(objects[0]);

		r.setupFilledFaces();
        glm::vec3 green(0.2f, 1.0f, 0.2f);
        r.setUniform(program, "ambientColor", 0.4f*green);
        r.setUniform(program, "diffuseColor", 0.9f*green);
        r.setUniform(program, "specularColor", 0.8f*white);
        r.setUniform(program, "phongExponent", 100.f);
		r.drawObject(objects[1]);

		// r.setupWireFrame();
        // r.setUniform(program, "ambientColor", black);
        // r.setUniform(program, "diffuseColor", black);
        // r.setUniform(program, "specularColor", black);
        // r.setUniform(program, "phongExponent", 0.f);
		// r.drawObject(objects[1]);

		r.setupFilledFaces();
		glm::vec3 blue(0.2f, 0.2f, 1.0f);
		r.setUniform(program, "ambientColor", 0.4f*blue);
		r.setUniform(program, "diffuseColor", 0.9f*blue);
		r.setUniform(program, "specularColor", 0.8f*white);
		r.setUniform(program, "phongExponent", 100.f);
		r.drawObject(objects[2]);

		// r.setupWireFrame();
		// r.setUniform(program, "ambientColor", black);
		// r.setUniform(program, "diffuseColor", black);
		// r.setUniform(program, "specularColor", black);
		// r.setUniform(program, "phongExponent", 0.f);
		// r.drawObject(objects[2]);

		r.show();
	}
}
