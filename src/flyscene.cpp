#include "flyscene.hpp"
#include "ThreadPool.h"
#include <GLFW/glfw3.h>
#include<iostream>
#include <thread>
#include <mutex>
#include <ctime>
#include "../arealight.hpp"
#include "../arealight.cpp"
#define BACKGROUND Eigen::Vector3f(1.f, 1.f, 1.f)
#define SHADOW Eigen::Vector3f(0.f, 0.f, 0.f)
#define PROGRESS_BAR_STR "=================================================="
#define PROGRESS_BAR_WIDTH 50
#define SCALAR 0.0000001f;


// Fields for the progress bar
float progress;
float total_num_of_rays;
unsigned int ray_done_counter;
bool done_ray_tracing;
std::mutex mtx;           // mutex for critical section

typedef vector<pair<Eigen::Vector3f, Eigen::Vector2f>> ThreadPartition;


void Flyscene::initialize(int width, int height) {

	cout << "Enter 0 if Point Lights or 1 if Area Lights : " << endl;
	cin >> areaLight;
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
  //dodgeColorTest

  // normalize the model (scale to unit cube and center at origin)
  mesh.normalizeModelMatrix();

  // pass all the materials to the Phong Shader
  for (int i = 0; i < materials.size(); ++i)
    phong.addMaterial(materials[i]);

  objectBox = BoundingBox::BoundingBox(getMesh());

  // set the color and size of the sphere to represent the light sources
  // same sphere is used for all sources
  lightrep.setColor(Eigen::Vector4f(1.0, 1.0, 0.0, 1.0));
  lightrep.setSize(0.15);

  // create a first ray-tracing light source at some random position
  lights.push_back(Eigen::Vector3f(-1.0, 1.0, 1.0));

  // scale the camera representation (frustum) for the ray debug
  camerarep.shapeMatrix()->scale(0.2);

  // the debug ray is a cylinder, set the radius and length of the cylinder
  ray.setSize(0.005, 1.f);

  //Set up triangle normal from intersection
  intersectNormal.setSize(0.005, 10.0);

  //Set up reflected ray
  reflectedRay.setSize(0.005, 10.0);


  
  intersectionLightRays.push_back(Tucano::Shapes::Cylinder(0.05, 1.0, 16, 64));
  
  // craete a first debug ray pointing at the center of the screen
  createDebugRay(Eigen::Vector2f(width / 2.0, height / 2.0));

  glEnable(GL_DEPTH_TEST);

  // uncomment when boxTree class is fully implemented:


  int capacity = 1000;
  std::cout << "Seting up acceleration data structure ..." << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  
  octree = BoxTree::BoxTree(getMesh(), capacity);

  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Seting up acceleration data structure: done!" << std::endl;
  std::cout << "ELAPSED TIME:" << elapsed.count() << endl;


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
  
  reflectedRay.render(flycamera, scene_light);

  //render intersection of triangle normal
  intersectNormal.render(flycamera, scene_light);

  hitCircle.render(flycamera, scene_light);

  for (Tucano::Shapes::Cylinder light : intersectionLightRays) {
	  light.render(flycamera, scene_light);
  }
  

  // render ray tracing light sources as yellow spheres
  for (int i = 0; i < lights.size(); ++i) {
    lightrep.resetModelMatrix();
    lightrep.modelMatrix()->translate(lights[i]);
    lightrep.render(flycamera, scene_light);
  }

  // render coordinate system at lower right corner
  flycamera.renderAtCorner();

  boxMin.render(flycamera, scene_light);
  boxMax.render(flycamera, scene_light);
  //X_axies.render(flycamera, scene_light);
  //Y_axies.render(flycamera, scene_light);
  //Z_axies.render(flycamera, scene_light);

  
  //create box
  boundingboxVisual.render(flycamera, scene_light);
  
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

// Creates (technically translates) a box at the point provided.
void Flyscene::createBox(Eigen::Vector3f point) {
	boundingboxVisual.resetModelMatrix();
	Eigen::Affine3f modelMatrix = boundingboxVisual.getModelMatrix();
	modelMatrix.translate(point);
	boundingboxVisual.setModelMatrix(modelMatrix);
}


// Creates (technically translates) a sphere at the point provided.
void Flyscene::createHitPoint(Eigen::Vector3f& point) {
	hitCircle.resetModelMatrix();
	Eigen::Affine3f modelMatrix = hitCircle.getModelMatrix();
	modelMatrix.translate(point);
	hitCircle.setModelMatrix(modelMatrix);
}

Tucano::Mesh& Flyscene::getMesh() {
	return mesh;
}

void Flyscene::createDebugRay(const Eigen::Vector2f& mouse_pos) {

	// uncomment to check the results of createRootBox:
	/*boundingBox box = createRootBox();

	/*Eigen::Vector3f minimum = box.getMin();
	Eigen::Vector3f maximum = box.getMax();

	std::cout << "min:" << minimum << std::endl;

	std::cout << "max:" << maximum << std::endl;*/

	ray.resetModelMatrix();


	Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);
	//// direction from camera center to click position
	Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();
	//// position and orient the cylinder representing the ray
	ray.setOriginOrientation(flycamera.getCenter(), dir);


	//// place the camera representation (frustum) on current camera location, 
	camerarep.resetModelMatrix();
	camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());

	float intersection;


	// check whether the debug ray hits the (root) bounding box
	bool hitBox = octree.box.boxIntersect(flycamera.getCenter(), screen_pos);

	bool intersected = false;
	float t = std::numeric_limits<float>::max();
	int intersectedTriangleIndex = -1;
	if(hitBox) {
		ray.setColor(Eigen::Vector4f(0.f, 0.f, 1.f, 1.f)); // if the ray intersects the root box but not the mesh, it should be blue
		for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
			Tucano::Face currTriangle = mesh.getFace(i);
			intersection = rayTriangleIntersection(screen_pos, dir, currTriangle);
			if (intersection != -72.f) {
				intersected = true;
				if (intersection < t) {
					t = intersection;
					intersectedTriangleIndex = i;
				}
				ray.setColor(Eigen::Vector4f(0.f, 1.f, 0.f, 1.f)); // if the ray intersects the mesh, it should  be green
			}
		}
	}
	else {
		ray.setColor(Eigen::Vector4f(1.f, 0.f, 0.f, 1.f)); // if the ray misses, it should be red
	}

  if (intersected) {
	  Eigen::Vector3f p0 = screen_pos + (t * dir);
	  createHitPoint(p0);

	  //Eigen::Vector3f distVec = t * dir;
	  float dist = std::numeric_limits<float>::max();
	  Eigen::Vector3f normalTri = mesh.getFace(intersectedTriangleIndex).normal;
	

	  intersectNormal.setOriginOrientation(p0, normalTri);
	  intersectNormal.setColor(Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
	  intersectNormal.setSize(0.005, 0.3);

	  reflectedRay.setOriginOrientation(p0, dir - 2 * dir.dot(normalTri) * normalTri);
	  reflectedRay.setSize(0.005, dist);

	  ray.setSize(0.005, dist);


	  for (int i = 0; i < intersectionLightRays.size(); i++) {
		  Eigen::Vector3f directionLight = (p0 - lights.at(i));
		  intersectionLightRays.at(i).setOriginOrientation(lights.at(i), directionLight.normalized());
		  intersectionLightRays.at(i).setColor(Eigen::Vector4f(0.5f, 0.5f, 0.f, 1.f));
		  float distLight = sqrt(directionLight.x() * directionLight.x() 
			  + directionLight.y() * directionLight.y() 
			  + directionLight.z() * directionLight.z());
		  intersectionLightRays.at(i).setSize(0.005, distLight);
	  }

	  float width = octree.box.getMax().x() - octree.box.getMin().x();
	  float height = octree.box.getMax().y() - octree.box.getMin().y();
	  float depth = octree.box.getMax().z() - octree.box.getMin().z();


	  boundingboxVisual = Tucano::Shapes::Box(width, height, depth);
	  float g = 192.f / 255.f;
	  float b = 203.f / 255.f;
	  boundingboxVisual.setColor(Eigen::Vector4f(1.f, g, b, 0.2f));
	  Eigen::Vector3f diff = (octree.box.getMax() - octree.box.getMin());
	  diff *= SCALAR;
	  createBox(diff);
  }
  else {
	  boundingboxVisual = Tucano::Shapes::Box(0.f, 0.f, 0.f);

	  Eigen::Vector3f p0 = screen_pos + (std::numeric_limits<float>::max() * dir);
	  createHitPoint(p0);

	  intersectNormal.setSize(0.005, 0);
	  reflectedRay.setSize(0.005, 0);

	  ray.setSize(0.005, std::numeric_limits<float>::max());


	  for (int i = 0; i < intersectionLightRays.size(); i++) {
		  intersectionLightRays.at(i).setOriginOrientation(lights.at(i), Eigen::Vector3f(0.f, 0.f, 1.f));
		  intersectionLightRays.at(i).setSize(0.005, 0);
	  }
  }

  boxMin.resetModelMatrix();
  Eigen::Affine3f boxMinModelMatrix = boxMin.getModelMatrix();
  boxMinModelMatrix.translate(objectBox.getMin());
  boxMin.setModelMatrix(boxMinModelMatrix);
  boxMin.setColor(Eigen::Vector4f(0.f, 1.f, 0.f, 1.f));

  boxMax.resetModelMatrix();
  Eigen::Affine3f boxMaxModelMatrix = boxMax.getModelMatrix();
  boxMaxModelMatrix.translate(objectBox.getMax());
  boxMax.setModelMatrix(boxMaxModelMatrix);
  boxMax.setColor(Eigen::Vector4f(0.f, 1.f, 0.f, 1.f));


  //Show X, Y and Z axies

  /*X_axies.setOriginOrientation(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(1.f, 0.f, 0.f));
  Y_axies.setOriginOrientation(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 1.f, 0.f));
  Z_axies.setOriginOrientation(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f(0.f, 0.f, 1.f));

  X_axies.setSize(0.005, 5);
  Y_axies.setSize(0.005, 5);
  Z_axies.setSize(0.005, 5);

  X_axies.setColor(Eigen::Vector4f(0.f, 1.f, 0.f, 1.f));
  Y_axies.setColor(Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
  Z_axies.setColor(Eigen::Vector4f(0.f, 0.f, 1.f, 1.f));*/
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

	auto start = std::chrono::high_resolution_clock::now();

	// create 2d vector to hold pixel colors and resize to match image size
	vector<vector<Eigen::Vector3f>> pixel_data;
	pixel_data.resize(raytracing_image_size[1]);
	for (int i = 0; i < raytracing_image_size[1]; ++i)
		pixel_data[i].resize(raytracing_image_size[0]);

	// if no width or height passed, use dimensions of current viewport
	Eigen::Vector2i raytracing_image_size(width, height);
	if (width == 0 || height == 0) {
		raytracing_image_size = flycamera.getViewportSize();
	}

	//////////////////////////
	//Set up for progress bar
	done_ray_tracing = false;
	progress = 0.f;
	ray_done_counter = 0;
	////////////////////////

	// create 2d vector to hold pixel colors and resize to match image size
	//vector<vector<Eigen::Vector3f>> pixel_data;
	pixel_data.resize(raytracing_image_size[1]);
	for (int i = 0; i < raytracing_image_size[1]; ++i)
		pixel_data[i].resize(raytracing_image_size[0]);

	vector<vector<Eigen::Vector3f>>& pixel_data_copy = pixel_data;

	// origin of the ray is always the camera center
	Eigen::Vector3f origin = flycamera.getCenter();
	Eigen::Vector3f& origina = origin;
	Eigen::Vector3f screen_coords;

	total_num_of_rays = (float)(raytracing_image_size[1] * raytracing_image_size[0]);


	int num_threads = std::thread::hardware_concurrency() - 1;
	int total_pixels = raytracing_image_size[0] * raytracing_image_size[1];
	int partition_size = ceil(ceil(total_pixels / num_threads) / 1000);

	
	int partition_counter = 0;
	std::cout << "Preparing threads ..."<< endl;
	std::cout << "Threads utilized:      " << num_threads << endl;
	std::cout << "Thread Partition size: " << partition_size << endl;

	vector<ThreadPartition> partitions;
	ThreadPartition partition;

	float percentage = 0.25f;
	unsigned int counter_ray = 0;
	for (int i = 0; i < raytracing_image_size[0]; i++) {
		for (int j = 0; j < raytracing_image_size[1]; j++) {
			screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
			bool hitBox = octree.box.boxIntersect(origina, screen_coords);
			//No hit? Then return background. No need to check individual triangles!
			if (!hitBox) {
				pixel_data_copy[i][j] = BACKGROUND;
				counter_ray++;
			}
			else {
				partition.push_back(std::make_pair(screen_coords, Eigen::Vector2f(i, j)));
				partition_counter++;
			}
			//If we hit the partition size, we add to thread partitions vector
			if (partition_counter == partition_size || (i == raytracing_image_size[0] -1 && j == raytracing_image_size[1] - 1)) {
				partition_counter = 0;
				partitions.push_back(partition);
				partition.clear();
				//if a chunk of rays are not hitting the box, decrease partition size
				if (counter_ray / (raytracing_image_size[0] * raytracing_image_size[1]) > percentage) {
					percentage += 0.1f;
					partition_size = ceil(partition_size/4);
				}
			}
		}
	}

	std::cout << "Thread preperation done!" << endl;
	std::cout << "Thread patitions number: " << partitions.size()<< endl;
	std::cout << "Thread patition smallest size: " << partitions.at(partitions.size()-1).size() << endl;
	std::cout << "Ray tracing ..." << std::endl;
	//start a progress bar thread
	std::thread progressBarThread(progressLoop);
	
	ray_done_counter += counter_ray;
	
	ThreadPool pool(num_threads);
	std::vector<std::future<void>> results;

	
	for (int id = 0; id < partitions.size(); id++) {
		ThreadPartition& current_partition = partitions[id];
		auto f = [&origina, &current_partition, this, &pixel_data_copy]() {
			ThreadPartition thread_partition = current_partition;
			for (int k = 0; k < thread_partition.size(); k++) {
				pair<Eigen::Vector3f, Eigen::Vector2f> partition_element = thread_partition[k];
				Eigen::Vector3f direction = partition_element.first - origina;
				pixel_data_copy[partition_element.second[0]][partition_element.second[1]] =
					traceRay(origina, direction, 0, lights, areaLight, true);
			}
		};
		results.emplace_back(pool.enqueue(f));
	}

	for (auto&& result : results) {
		result.get();
	}

	done_ray_tracing = true;

	progressBarThread.join();
	pool.~ThreadPool();

	pixel_data = pixel_data_copy;

	std::cout << "" << endl;
  // write the ray tracing result to a PPM image
	std::cout << "Writting to restult.ppm ... " << std::endl;
  Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
  std::cout << "Writting to restult.ppm done!" << std::endl;
  std::cout << "" << endl;
  std::cout << "ray tracing done! " << std::endl;

  std::cout << "ELAPSED TIME:" << elapsed.count() << endl;
}


Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f& origin,
	Eigen::Vector3f& direction, int level, vector<Eigen::Vector3f>& lights, bool areaLight, bool countRay) {

	//Check whether the ray hits the (root) bounding box
	bool hitBox = octree.box.boxIntersect(origin, origin+direction);


	if (!hitBox) {
		if (countRay) {
			mtx.lock();
			ray_done_counter++;
			mtx.unlock();
		}
		return BACKGROUND;
	}

	int bestIntersectionTriangleIndex = -1;
	float intersection;
	//Store the best intersection (triangle closest to the camera)
	float t = std::numeric_limits<float>::max();

	std::set<int> faces = octree.intersect(origin, direction+origin);
	
	//Loop through the reduced list of faces
	for (int i : faces) {
		//get a direction vector
		Tucano::Face currTriangle = mesh.getFace(i);
		intersection = rayTriangleIntersection(origin, direction, currTriangle);
		if (intersection != -72 && intersection < t && intersection > 0.00001f) {
			t = intersection;
			bestIntersectionTriangleIndex = i;
		}
	}
	if (bestIntersectionTriangleIndex == -1) {
		if (countRay) {
			mtx.lock();
			ray_done_counter++;
			mtx.unlock();
		}
		return BACKGROUND;
	}
	
	Tucano::Face intersectTriangle = mesh.getFace(bestIntersectionTriangleIndex);

	Eigen::Vector3f hitPoint = origin + t * direction;
	Eigen::Vector3f Color = Eigen::Vector3f(-1, -1, -1);
	Eigen::Vector3f faceNormal = intersectTriangle.normal;

	bool visibleLights[25];
	bool lightStrikesHitPoint = lightStrikes(hitPoint, lights, visibleLights);

	//Return Shadow
	if (!lightStrikesHitPoint) {
		if (countRay) {
			mtx.lock();
			ray_done_counter++;
			mtx.unlock();
		}
		return SHADOW;
	}

	Tucano::Material::Mtl material = materials[intersectTriangle.material_id];
	int imodel = material.getIlluminationModel();

	float fresnelIndex = 1;

	if (imodel == 9) {
		Color = 0.2 * phongShade(origin, hitPoint, intersectTriangle, lights, visibleLights, areaLight) + 0.8 * traceRay(hitPoint, direction, level + 1, lights, areaLight, false);
	}

	else if (imodel == 6 || imodel == 7) {
		float c1 = abs(direction.dot(faceNormal));
		float c2 = sqrt(1 - (pow((1 / material.getOpticalDensity()), 2)) * (1 - pow(c1, 2)));
		Eigen::Vector3f refractedRay = (1 / material.getOpticalDensity()) * direction + ((1 / material.getOpticalDensity()) * c1 - c2) * faceNormal;
		if (imodel == 7) {
			Color = fresnelIndex * Color + (1 - fresnelIndex) * traceRay(hitPoint, refractedRay, level + 1, lights, areaLight, false);
		}
		else {
			Color = traceRay(hitPoint, refractedRay, level + 1, lights, areaLight, false);
		}
	}

	else if (imodel > 2 && imodel < 7) {
		Eigen::Vector3f reflectedDirection = direction - 2 * (direction.dot(faceNormal)) * faceNormal;
		vector<Eigen::Vector3f> reflectedLights;
		reflectedLights.push_back(hitPoint);

		Color = 0.15 * phongShade(origin, hitPoint, intersectTriangle, lights, visibleLights, areaLight) + 0.85 * traceRay(hitPoint, reflectedDirection, level + 1, reflectedLights, areaLight, false);
		if (imodel == 5) {
			float opticalDensity = material.getOpticalDensity();
			fresnelIndex = fresnel(reflectedDirection, faceNormal, opticalDensity);
			return fresnelIndex * Color;
		}
	}
	else if (Color == Eigen::Vector3f(-1, -1, -1)) {
		Color = phongShade(origin, hitPoint, intersectTriangle, lights, visibleLights, areaLight);
	}


	if (countRay) {
		mtx.lock();
		ray_done_counter++;
		mtx.unlock();
	}

	return Color;

}


// Returns parameter t of r = o + td   of the ray that intersects the plane
float Flyscene::rayPlaneIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Eigen::Vector3f& planeNormal, Eigen::Vector3f& planePoint) {
	if (rayDirection.dot(planeNormal) == 0) {
		return	std::numeric_limits<float>::max();
	}

	float t = (planeNormal.dot(planePoint) - rayPoint.dot(planeNormal)) / rayDirection.dot(planeNormal);
	return t;
}


//Returns a vector with [0] - 1 or 0 meaning: intersection or not. [1] - t: value of light ray to compare distance 

float Flyscene::rayTriangleIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Tucano::Face& triangle) {
	Eigen::Vector3f vertices[3] = { (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[0])).head<3>() ,
		(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[1])).head<3>(),
		(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[2])).head<3>() };

	Eigen::Vector3f triangleNormal = triangle.normal;
	if (rayDirection.dot(triangleNormal) == 0) {
		return -72;
	}

	float t = (triangleNormal.dot(vertices[0]) - rayPoint.dot(triangleNormal)) / rayDirection.dot(triangleNormal);
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
		return t;
	}
	else {
		return -72;
	}
}

//Computes phong shading at the given point with interpolated normals.
Eigen::Vector3f Flyscene::phongShade(Eigen::Vector3f& origin, Eigen::Vector3f& hitPoint, Tucano::Face& triangle, vector<Eigen::Vector3f>& lights, bool visibleLights[], bool areaLight) {

	Eigen::Vector3f lightIntensity = lightrep.getColor().head<3>();

	Tucano::Material::Mtl material = materials[triangle.material_id];
	//Eigen::Vector3f ambient = lightIntensity.cwiseProduct(material.getAmbient());
	Eigen::Vector3f colour = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f normal = (mesh.getModelMatrix() * getInterpolatedNormal(hitPoint, triangle)).normalized();

	float sum = 0;

	for (int i = 0; i < lights.size(); i++) {

		if (!areaLight && !visibleLights[i]) {
			continue;
		}
		else if (visibleLights[i]) {
			sum++;
		}
		Eigen::Vector3f lightDirection = (lights.at(i) - hitPoint).normalized();

		float costheta = max(0.0f, lightDirection.dot(normal));
		Eigen::Vector3f diffuse = lightIntensity.cwiseProduct(material.getDiffuse()) * costheta;

		Eigen::Vector3f reflectedLight = (lightDirection - ((2 * lightDirection.dot(normal)) * normal)).normalized();
		Eigen::Vector3f eyeToHitPoint = (-1 * (hitPoint - origin)).normalized();
		float cosphi = std::max(0.0f, eyeToHitPoint.dot(-1 * reflectedLight));
		Eigen::Vector3f specular = lightIntensity.cwiseProduct(material.getSpecular()) * pow(cosphi, material.getShininess());

		colour += diffuse + specular;
	}
	if (areaLight)
		return colour * (sum / lights.size()) * (1.3f / lights.size());
	return colour;
}

//Computes the interpolated normal for the given point on the triangle.
Eigen::Vector3f Flyscene::getInterpolatedNormal(Eigen::Vector3f& trianglePoint, Tucano::Face& triangle) {
	Eigen::Vector3f vertices[3] = { (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[0])).head<3>() ,
	(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[1])).head<3>(),
	(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[2])).head<3>() };
	Eigen::Vector3f v0 = vertices[1] - vertices[0];
	Eigen::Vector3f v1 = vertices[2] - vertices[0];
	Eigen::Vector3f v2 = trianglePoint - vertices[0];
	Eigen::Vector3f normalA = mesh.getNormal(triangle.vertex_ids[0]);
	Eigen::Vector3f normalB = mesh.getNormal(triangle.vertex_ids[1]);
	Eigen::Vector3f normalC = mesh.getNormal(triangle.vertex_ids[2]);

	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);
	float denom = d00 * d11 - d01 * d01;

	float v = (d11 * d20 - d01 * d21) / denom;
	float w = (d00 * d21 - d01 * d20) / denom;
	float u = 1.0f - v - w;

	return u * normalA + v * normalB + w * normalC;

}

float Flyscene::fresnel(Eigen::Vector3f& I, Eigen::Vector3f& N, float& ior)
{
	float cosi = I.dot(N);
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sini using Snell's law
	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
	// Total internal reflection
	if (sint >= 1) {
		return 1;
	}
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		return (Rs * Rs + Rp * Rp) / 2;
	}
	// As a consequence of the conservation of energy, transmittance is given by:
	// kt = 1 - kr;
}

bool Flyscene::lightStrikes(Eigen::Vector3f& hitPoint, vector<Eigen::Vector3f>& lights, bool visibleLights[]) {
	bool hit = false;
	float intersection, t;
	Eigen::Vector3f origin, direction;
	Tucano::Face currTriangle;

	for (int l = 0; l < lights.size(); l++) {
		t = std::numeric_limits<float>::max();
		origin = lights[l];
		direction = hitPoint - origin;
		//Loop through all of the faces
		for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
			//get a direction vector
			currTriangle = mesh.getFace(i);
			if (materials[currTriangle.material_id].getIlluminationModel() == 9) {
				continue;
			}
			intersection = rayTriangleIntersection(origin, direction, currTriangle);
			if (intersection != -72 && intersection < t && intersection > 0.00001) {
				t = intersection;
			}
		}

		if (t >= 0.98) {
			hit = true;
			visibleLights[l] = true;
		}
		else {
			visibleLights[l] = false;
		}
	}
	return hit;
}

arealight Flyscene::createAreaLight(Eigen::Vector3f corner, float lengthX, float lengthY, int usteps, int vsteps) {
	Eigen::Vector3f uvec = corner + lengthX * (Eigen::Vector3f(1, 0, 0));
	Eigen::Vector3f vvec = corner + lengthY * (Eigen::Vector3f(0, 1, 0));
	return arealight(corner, uvec, usteps, vvec, vsteps);
}
