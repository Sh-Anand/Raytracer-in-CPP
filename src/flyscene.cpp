#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include <math.h>
#include<iostream>
#include <thread>
#include <mutex>
#include <ctime>
#define BACKGROUND Eigen::Vector3f(1.f, 1.f, 1.f)
//#define SHADOW Eigen::Vector3f(0.0, 0.0, 0.0)
float progress;
float total_num_of_rays;
unsigned int ray_done_counter;
bool done_ray_tracing;
std::mutex mtx;           // mutex for critical section
clock_t startTime;
clock_t endTime;

void Flyscene::initialize(int width, int height) {
  // initiliaze the Phong Shading effect for the Opengl Previewer
  phong.initialize();

  // set the camera's projection matrix
  flycamera.setPerspectiveMatrix(60.0, width / (float)height, 0.1f, 100.0f);
  flycamera.setViewport(Eigen::Vector2f((float)width, (float)height));

  // load the OBJ file and materials
  Tucano::MeshImporter::loadObjFile(mesh, materials,
                                    "resources/models/cube.obj");


  // normalize the model (scale to unit cube and center at origin)
  mesh.normalizeModelMatrix();

  // pass all the materials to the Phong Shader
  for (int i = 0; i < materials.size(); ++i)
    phong.addMaterial(materials[i]);



  // set the color and size of the sphere to represent the light sources
  // same sphere is used for all sources
  lightrep.setColor(Eigen::Vector4f(1.0, 1.0, 0.0, 1.0));
  lightrep.setSize(0.15);

  // create a first ray-tracing light source at some random position
  lights.push_back(Eigen::Vector3f(-1.0, 1.0, 1.0));

  // scale the camera representation (frustum) for the ray debug
  camerarep.shapeMatrix()->scale(0.2);

  // the debug ray is a cylinder, set the radius and length of the cylinder
  ray.setSize(0.005, 10.0);

  // craete a first debug ray pointing at the center of the screen
  createDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

  glEnable(GL_DEPTH_TEST);

  // for (int i = 0; i<mesh.getNumberOfFaces(); ++i){
  //   Tucano::Face face = mesh.getFace(i);    
  //   for (int j =0; j<face.vertex_ids.size(); ++j){
  //     std::cout<<"vid "<<j<<" "<<face.vertex_ids[j]<<std::endl;
  //     std::cout<<"vertex "<<mesh.getVertex(face.vertex_ids[j]).transpose()<<std::endl;
  //     std::cout<<"normal "<<mesh.getNormal(face.vertex_ids[j]).transpose()<<std::endl;
  //   }
  //   std::cout<<"mat id "<<face.material_id<<std::endl<<std::endl;
  //   std::cout<<"face   normal "<<face.normal.transpose() << std::endl << std::endl;
  // }



}

void Flyscene::paintGL(void) {

  // update the camera view matrix with the last mouse interactions
  flycamera.updateViewMatrix();
  Eigen::Vector4f viewport = flycamera.getViewport();

  // clear the screen and set background color
  glClearColor(0.9, 0.9, 0.9, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // position the scene light at the last ray-tracing light source
  scene_light.resetViewMatrix();
  scene_light.viewMatrix()->translate(-lights.back());

  hitCircle.render(flycamera, scene_light);

  // render the scene using OpenGL and one light source
  phong.render(mesh, flycamera, scene_light);

  // render the ray and camera representation for ray debug
  ray.render(flycamera, scene_light);
  camerarep.render(flycamera, scene_light);

  // render ray tracing light sources as yellow spheres
  for (int i = 0; i < lights.size(); ++i) {
    lightrep.resetModelMatrix();
    lightrep.modelMatrix()->translate(lights[i]);
    lightrep.render(flycamera, scene_light);
  }

  // render coordinate system at lower right corner
  flycamera.renderAtCorner();
}

void Flyscene::simulate(GLFWwindow *window) {
  // Update the camera.
  // NOTE(mickvangelderen): GLFW 3.2 has a problem on ubuntu where some key
  // events are repeated: https://github.com/glfw/glfw/issues/747. Sucks.
  float dx = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ? 1.0 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ? 1.0 : 0.0);
  float dy = (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS
                  ? 1.0
                  : 0.0) -
             (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS
                  ? 1.0
                  : 0.0);
  float dz = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ? 1.0 : 0.0) -
             (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ? 1.0 : 0.0);
  flycamera.translate(dx, dy, dz);
}

// Creates (technically translates) a sphere at the point provided.
void Flyscene::createHitPoint(Eigen::Vector3f point) {
	hitCircle.resetModelMatrix();
	Eigen::Affine3f modelMatrix = hitCircle.getModelMatrix();
	modelMatrix.translate(point);
	hitCircle.setModelMatrix(modelMatrix);
}

void Flyscene::createDebugRay(const Eigen::Vector2f& mouse_pos) {
	ray.resetModelMatrix();

	std::cout << "DEBUG: " << flycamera.getViewportSize();
	// from pixel position to world coordinates
	Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);
	// direction from camera center to click position
	Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();
	// position and orient the cylinder representing the ray
	ray.setOriginOrientation(flycamera.getCenter(), dir);

	// place the camera representation (frustum) on current camera location, 
	camerarep.resetModelMatrix();
	camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());

	vector<float> intersection;

	bool intersected = false;
	float t = std::numeric_limits<float>::max();
	for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		Tucano::Face currTriangle = mesh.getFace(i);
		intersection =
			rayTriangleIntersection(screen_pos, dir, currTriangle);
		if (intersection.at(0)) {
			intersected = true;
			if (intersection.at(1) < t) {
				t = intersection.at(1);
			}
		}
	}

	if (intersected) {
		Eigen::Vector3f p0 = screen_pos + (t * dir);
		createHitPoint(p0);
	}
	else {
		ray.setSize(0.005, std::numeric_limits<float>::max());
	}
}

void Flyscene::raytraceScene(int width, int height) {
  std::cout << "ray tracing ..." << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  // if no width or height passed, use dimensions of current viewport
  Eigen::Vector2i image_size(width, height);
  if (width == 0 || height == 0) {
    image_size = flycamera.getViewportSize();
  }

  // create 2d vector to hold pixel colors and resize to match image size
  vector<vector<Eigen::Vector3f>> pixel_data;
  pixel_data.resize(image_size[1]);
  for (int i = 0; i < image_size[1]; ++i)
    pixel_data[i].resize(image_size[0]);

  // origin of the ray is always the camera center
  Eigen::Vector3f origin = flycamera.getCenter();
  Eigen::Vector3f screen_coords;

  // for every pixel shoot a ray from the origin through the pixel coords
  for (int j = 0; j < image_size[1]; ++j) {
    for (int i = 0; i < image_size[0]; ++i) {
      // create a ray from the camera passing through the pixel (i,j)
      screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
      // launch raytracing for the given ray and write result to pixel data
      pixel_data[i][j] = traceRay(origin, screen_coords);
	  cout << i << ", " << j << endl;

    }
  }

  // write the ray tracing result to a PPM image
  Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;

  std::cout << "ray tracing done! " << std::endl;

  std::cout << "ELAPSED TIME:" << elapsed.count() << endl;

  std::cout << "ray tracing done! " << std::endl;
}

Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f& origin,
	Eigen::Vector3f& dest) {
	// just some fake random color per pixel until you implement your ray tracing
	// remember to return your RGB values as floats in the range [0, 1]!!!


	Eigen::Vector3f direction = dest - origin;

	int bestIntersectionTriangleIndex = -1;
	vector<float> intersection;
	//Store the best intersection (triangle closest to the camera)
	float t = std::numeric_limits<float>::max();
	//All the triangles which are not reached by the light.

	//Loop through all of the faces
	for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		//get a direction vector
		Tucano::Face currTriangle = mesh.getFace(i);
		intersection = rayTriangleIntersection(origin, direction, currTriangle);
		if (intersection.at(0) && intersection.at(1) < t) {
			t = intersection.at(1);
			bestIntersectionTriangleIndex = i;
		}
	}

	if (bestIntersectionTriangleIndex == -1) {
		return BACKGROUND;
	}
	else {
		return calculateShadow(origin + t * direction, mesh.getFace(bestIntersectionTriangleIndex));
	}
}


// Returns parameter t of r = o + td   of the ray that intersects the plane
float Flyscene::rayPlaneIntersection(Eigen::Vector3f rayPoint, Eigen::Vector3f rayDirection, Eigen::Vector3f planeNormal, Eigen::Vector3f planePoint) {
	if (rayDirection.dot(planeNormal) == 0) {
		return	std::numeric_limits<float>::max();
	}

	float t = (planeNormal.dot(planePoint) - rayPoint.dot(planeNormal)) / rayDirection.dot(planeNormal);
	return t;
}

//Returns a vector with [0] - 1 or 0 meaning: intersection or not. [1] - t: value of light ray to compare distance 
vector<float> Flyscene::rayTriangleIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Tucano::Face& triangle) {
	Eigen::Vector3f vertices[3] = { (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[0])).head<3>() ,
		(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[1])).head<3>(),
		(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[2])).head<3>() };

	Eigen::Vector3f triangleNormal = triangle.normal;
	vector<float> result;
	if (rayDirection.dot(triangleNormal) == 0) {
		result.push_back(0);
		return result;
	}

	float t = rayPlaneIntersection(rayPoint, rayDirection, triangleNormal, vertices[0]);
	Eigen::Vector3f planeIntersection = rayPoint + (t * rayDirection);
	Eigen::Vector3f v0 = vertices[2] - vertices[0];
	Eigen::Vector3f v1 = vertices[1] - vertices[0];
	Eigen::Vector3f v2 = planeIntersection - vertices[0];

	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d02 = v0.dot(v2);
	float d12 = v1.dot(v2);

	float invDenom = 1 / (d00 * d11 - d01 * d01);
	float u = (d11 * d02 - d01 * d12) * invDenom;
	float v = (d00 * d12 - d01 * d02) * invDenom;

	if ((u >= 0) && (v >= 0) && (u + v < 1)) {
		result.push_back(1);
	}
	else {
		result.push_back(0);
	}

	result.push_back(t);
	return result;
}

float calcDistanceV3(Eigen::Vector3f vector) {
	return _CMATH_::pow(vector[0], 2) + _CMATH_::pow(vector[1], 2) + _CMATH_::pow(vector[2], 2);
}

//Given a triangle and a point that the ray intersects with on the triangle, this method tests whether a light ray passes through that point without intersecting any other triangle on the way.
//if yes, return false and do not paint a hard shadow, else return true and paint a hardshadow.
Eigen::Vector3f Flyscene::calculateShadow(Eigen::Vector3f trianglePoint, Tucano::Face triangle) {

	Tucano::Face triangleTest;
	Eigen::Vector3f lightDirection;
	vector<float> intersection;
	Eigen::Vector3f output = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f intensityFactor = Eigen::Vector3f(0.0, 0.0, 0.0);
	bool lightHits = true;

	//For loop going through each light
	for (Eigen::Vector3f lightPoint : lights) {
		lightDirection = trianglePoint - lightPoint;
		float distance = calcDistanceV3(lightDirection);
		float intensity = 1 / distance;
		lightHits = true;

		//For loop going through the meshes to see if the ray to the light is obstructed breaks at the first obscurement.
		for (int j = 0; j < mesh.getNumberOfFaces(); j++) {
			triangleTest = mesh.getFace(j);

			Eigen::Vector3f triangleNormal = triangleTest.normal;
			Eigen::Vector3f vertices = (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[0])).head<3>();

			float t = rayPlaneIntersection(trianglePoint, lightDirection, triangleNormal, vertices);
			float triangleTestDistance = calcDistanceV3(t * lightDirection - lightPoint);

			//Checking if the object is even between de lightsource and the point
			if (triangleTestDistance < distance) {

				intersection = rayTriangleIntersection(lightPoint, lightDirection, triangleTest);

				if (intersection[0] == 1) {
					lightHits = false;
					break;
				}
			}
		}
		if (lightHits) {
			cout << "light hit " << endl;
			output = output + Eigen::Vector3f(intensity, intensity, intensity);
		}

	}
	float x = output[0];
	float y = output[1];
	float z = output[2];
	intensityFactor = Eigen::Vector3f(min(1.f, x), min(1.f, y), min(1.f, z));

	//return materials[triangle.material_id].getAmbient();
	return intensityFactor;
}