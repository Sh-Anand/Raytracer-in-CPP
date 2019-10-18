#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include<iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <ctime>

#define epsilon 0.00001f
#define BACKGROUND Eigen::Vector3f(1.f, 1.f, 1.f)
#define PROGRESS_BAR_STR "=================================================="
#define PROGRESS_BAR_WIDTH 50

// Fields for the progress bar
float progress;
float total_num_of_rays;
unsigned int ray_done_counter;
bool done_ray_tracing;
vector<vector<Eigen::Vector3f>> pixel_data;
std::mutex mtx;           // mutex for critical section

void Flyscene::initialize(int width, int height) {
  // initiliaze the Phong Shading effect for the Opengl Previewer
  phong.initialize();

  //////////////////////////
  done_ray_tracing = false;
  progress = 0.0f;
  ray_done_counter = 0;
  ////////////////////////

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

void Flyscene::createHitPoint(Eigen::Vector3f point) {
	hitCircle.resetModelMatrix();
	Eigen::Affine3f modelMatrix = hitCircle.getModelMatrix();
	modelMatrix.translate(point);
	hitCircle.setModelMatrix(modelMatrix);
}

void Flyscene::createDebugRay(const Eigen::Vector2f &mouse_pos) {
  ray.resetModelMatrix();
  // from pixel position to world coordinates
  Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);
  // direction from camera center to click position
  Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();
  // position and orient the cylinder representing the ray
  ray.setOriginOrientation(flycamera.getCenter(), dir);

  // place the camera representation (frustum) on current camera location, 
  camerarep.resetModelMatrix();
  camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());

  //Try to intersect the triangle to see where to stop drawing the ray
  Eigen::Vector4f bestIntersection = 
	  Eigen::Vector4f(0.f, -1.f, -1.f, std::numeric_limits<float>::max());

  Tucano::Face inter;
  for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
	  Tucano::Face currTriangle = mesh.getFace(i);
	  Eigen::Vector4f intersection = 
		  rayTriangleIntersect(screen_pos, dir, currTriangle);
	  if (intersection.x() != 0.f && intersection.w() < bestIntersection.w()) {
		  bestIntersection = intersection;
		  inter = currTriangle;
	  }
  }

  if (bestIntersection.x() == 1.f) {
	  Eigen::Vector3f p0 = (mesh.getShapeModelMatrix() * mesh.getVertex(inter.vertex_ids[0])).head<3>();
	  createHitPoint(p0);
	  std::cout << p0;
  }
  else {
	  ray.setSize(0.005, std::numeric_limits<float>::max());
  }
}

void printProgress(float percentage) {
	progress = (float)ray_done_counter / (float)total_num_of_rays;
	int val = (int)(percentage * 100);
	int lpad = (int)(percentage * PROGRESS_BAR_WIDTH);
	int rpad = PROGRESS_BAR_WIDTH - lpad;
	printf("\r%3d%% [%.*s%*s]", val, lpad, PROGRESS_BAR_STR, rpad, "");
	fflush(stdout);
}

void progressLoop() {
	printProgress(progress);
	while (!done_ray_tracing) {
		printProgress(progress);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	printProgress(1.0f);
}

void Flyscene::raytraceScene(int width, int height) {
  std::cout << "ray tracing ..." << std::endl;

  //////////////////////////
  done_ray_tracing = false;
  progress = 0.0f;
  ray_done_counter = 0;
  ////////////////////////


  // if no width or height passed, use dimensions of current viewport
  Eigen::Vector2i image_size(width, height);
  if (width == 0 || height == 0) {
    image_size = flycamera.getViewportSize();
  }

  // create 2d vector to hold pixel colors and resize to match image size
  //vector<vector<Eigen::Vector3f>> pixel_data;
  pixel_data.resize(image_size[1]);
  for (int i = 0; i < image_size[1]; ++i)
    pixel_data[i].resize(image_size[0]);

  // origin of the ray is always the camera center
  Eigen::Vector3f origin = flycamera.getCenter();
  Eigen::Vector3f screen_coords;

  total_num_of_rays = (float) (image_size[1] * image_size[0]);


  std::thread progressBarThread(progressLoop);


  // for every pixel shoot a ray from the origin through the pixel coords
  //for (int j = 0; j < image_size[1]; ++j) {
  //  for (int i = 0; i < image_size[0]; ++i) {
  //    // create a ray from the camera passing through the pixel (i,j)
  //    screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
  //    // launch raytracing for the given ray and write result to pixel data
  //    pixel_data[i][j] = traceRay(origin, screen_coords);
	 // 
	 // //Counter for progress
	 // ray_done_counter++;
  //  }
  //}
  int third = (int)(image_size[1] / 3);
  std::thread draw1Thread = createDrawThread(0, third, image_size[0], origin);
  std::thread draw2Thread = createDrawThread(third, 2 * third, image_size[0], origin);
  std::thread draw3Thread = createDrawThread(2 * third, image_size[1], image_size[0], origin);

  draw1Thread.join();
  draw2Thread.join();
  draw3Thread.join();
  //draw(0, image_size[1], image_size[0], origin);

  done_ray_tracing = true;

  progressBarThread.join();

  std::cout << "" << std::endl;
  fflush(stdout);
  // write the ray tracing result to a PPM image

  std::cout << "writing to ppm file ..." << std::endl;
  Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);
  std::cout << "ray tracing done! " << std::endl;
}

void Flyscene::draw(int start, int end, int width, Eigen::Vector3f origin) {
	for (int j = start; j < end; ++j) {
		for (int i = 0; i < width; ++i) {
			// create a ray from the camera passing through the pixel (i,j)
			Eigen::Vector3f screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
			// launch raytracing for the given ray and write result to pixel data
			Eigen::Vector3f col = traceRay(origin, screen_coords);
			
			pixel_data[i][j] = col;
			//Counter for progress
			mtx.lock();
			ray_done_counter++;
			mtx.unlock();
		}
	}
}


Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f& origin,
	Eigen::Vector3f& dest) {
	// just some fake random color per pixel until you implement your ray tracing
	// remember to return your RGB values as floats in the range [0, 1]!!!
		
	  //Store the best intersection (triangle closest to the camera) and the index of the triangle
	  Eigen::Vector4f bestIntersection = Eigen::Vector4f(0.f, -1.f, -1.f, std::numeric_limits<float>::max());
	  int bestIntersectionTriangleIndex = -1;
	  Eigen::Vector3f direction = dest - origin;
	  //Loop through all of the faces
	  for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		//get a direction vector
		Tucano::Face currTriangle = mesh.getFace(i);
		Eigen::Vector4f intersection = rayTriangleIntersect(origin, direction, currTriangle);
		if (intersection.x() != 0.f && intersection.w() < bestIntersection.w()) {
			bestIntersection = intersection;
			bestIntersectionTriangleIndex = i;
		}
	  }
	  if (bestIntersectionTriangleIndex == -1) {
		  return BACKGROUND;
	  }

	  Tucano::Material::Mtl mat = materials[mesh.getFace(bestIntersectionTriangleIndex).material_id];
	  return mat.getAmbient() + mat.getDiffuse();// +mat.getSpecular();
	//return Eigen::Vector3f(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX,
		//rand() / (float)RAND_MAX);

}


//To understand this algorithm, please check this book:
//Möller Tomas, Eric Haines, and Naty Hoffman. Real-Time Rendering. Boca Raton: CRC Press, 2019. Print. Page 746
// It has a very detailed explanation behind the mathematics of the ray-triangle intersection.
Eigen::Vector4f Flyscene::rayTriangleIntersect(Eigen::Vector3f& origin, Eigen::Vector3f& direction, Tucano::Face& triangle)
{
	Eigen::Vector4f result = Eigen::Vector4f(0.f, 0.f, 0.f, 0.f);
	Eigen::Vector3f p0 = (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[0])).head<3>(); //mesh.getVertex(triangle.vertex_ids[0]).head<3>()
	Eigen::Vector3f p1 = (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[1])).head<3>();
	Eigen::Vector3f p2 = (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[2])).head<3>();

	Eigen::Vector3f edge1 = p1 - p0;
	Eigen::Vector3f edge2 = p2 - p0;

	Eigen::Vector3f q = direction.cross(edge2);
	float a = q.dot(edge1);

	if (a > -epsilon && a < epsilon) {
		return result;
	}

	float f = 1 / a;
	Eigen::Vector3f s = origin - p0;
	float u = f * s.dot(q);

	if (u < 0.f) {
		return result;
	}

	Eigen::Vector3f r = s.cross(edge1);
	float v = f * direction.dot(r);

	if (v < 0.f || u + v > 1.f) {
		return result;
	}
	float t = f * edge2.dot(r);
	result = Eigen::Vector4f(1.f, u, v, t);

	return result;
}


bool rayTriangleIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Tucano::Face& triangle, Tucano::Mesh& mesh) {
	Eigen::Vector3f vertices[3] = { mesh.getVertex(triangle.vertex_ids[0]) , mesh.getVertex(triangle.vertex_ids[1]), mesh.getVertex(triangle.vertex_ids[2]) };
	Eigen::Vector3f triangleNormal = triangle.normal;

	if (rayDirection.dot(triangleNormal) == 0) {
		return false;
	}

	return false;
}