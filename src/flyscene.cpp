#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include<iostream>
#include <thread>
#include <mutex>
#include <ctime>
#define BACKGROUND Eigen::Vector3f(1.f, 1.f, 1.f)
#define PROGRESS_BAR_STR "=================================================="
#define PROGRESS_BAR_WIDTH 50

// Fields for the progress bar
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
  ray.setSize(0.005, 1.f);

  //Set up triangle normal from intersection
  intersectNormal.setSize(0.005, 10.0);

  //Set up reflected ray
  reflectedRay.setSize(0.005, 10.0);


  
  intersectionLightRays.push_back(Tucano::Shapes::Cylinder(0.05, 1.0, 16, 64));
  
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
	std::cout <<"Supported Threads: "<< std::thread::hardware_concurrency();
	// from pixel position to world coordinates
	Eigen::Vector3f screen_pos = flycamera.screenToWorld(mouse_pos);
	// direction from camera center to click position
	Eigen::Vector3f dir = (screen_pos - flycamera.getCenter()).normalized();
	// position and orient the cylinder representing the ray
	ray.setOriginOrientation(flycamera.getCenter(), dir);

  bool intersected = false;
  int intersectedTri = -1;
  float t = std::numeric_limits<float>::max();
  for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
	  Tucano::Face currTriangle = mesh.getFace(i);
	  intersection =
		  rayTriangleIntersection(screen_pos, dir, currTriangle);
	  if (intersection.at(0)) {
		  intersected = true;
		  if (intersection.at(1) < t) {
			  t = intersection.at(1);
			  intersectedTri = i;
		  }
	  }
  }

  if (intersected) {
	  Eigen::Vector3f p0 = screen_pos + (t * dir);
	  createHitPoint(p0);

	  Eigen::Vector3f distVec = t * dir;
	  float dist = sqrt(distVec.x() * distVec.x() + distVec.y() * distVec.y() + distVec.z() * distVec.z());
	  Eigen::Vector3f normalTri = mesh.getFace(intersectedTri).normal;
	float intersection;
	Tucano::Face riangle;
	bool intersected = false;
	float t = std::numeric_limits<float>::max();
	for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		Tucano::Face currTriangle = mesh.getFace(i);
		intersection =
			rayTriangleIntersection(screen_pos, dir, currTriangle);
		if (intersection != std::numeric_limits<float>::min()) {
			intersected = true;
			if (intersection < t && intersection >= 0) {
				t = intersection;
			}
		}
	}

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
  }

  else {
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
}



void printProgress(float percentage) {
	progress = (float)ray_done_counter / (float)total_num_of_rays;
	int val = (int)(percentage * 100);
	int lpad = (int)(percentage * PROGRESS_BAR_WIDTH);
	int rpad = PROGRESS_BAR_WIDTH - lpad;
	int secondsPassed = (int) (clock() - startTime) / CLOCKS_PER_SEC;
	printf("\r%3d%% [%.*s%*s] %4d seconds passed", val, lpad, PROGRESS_BAR_STR, rpad, "", secondsPassed);
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

void printProgress(float percentage) {
	progress = (float)ray_done_counter / (float)total_num_of_rays;
	int val = (int)(percentage * 100);
	int lpad = (int)(percentage * PROGRESS_BAR_WIDTH);
	int rpad = PROGRESS_BAR_WIDTH - lpad;
	int secondsPassed = (int)(clock() - startTime) / CLOCKS_PER_SEC;
	printf("\r%3d%% [%.*s%*s] %4d seconds passed", val, lpad, PROGRESS_BAR_STR, rpad, "", secondsPassed);
	fflush(stdout);
}

void progressLoop() {
	printProgress(progress);
	while (!done_ray_tracing) {
		printProgress(progress);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	printProgress(1.0f);
	endTime = clock();
}

/*void Flyscene::draw(int start, int end, int width, Eigen::Vector3f origin) {
	for (int j = start; j < end; ++j) {
		for (int i = 0; i < width; ++i) {
			// create a ray from the camera passing through the pixel (i,j)
			Eigen::Vector3f screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
			// launch raytracing for the given ray and write result to pixel data
			Eigen::Vector3f col = traceRay(origin, screen_coords);
			// launch raytracing for the given ray and write result to pixel data
			pixel_data[i][j] = traceRay(origin, screen_coords);
			//index++;
			//Counter for progress which is critical section as it can 
			//be overwritten by other threads which are trying to increment 
			//the same number
			mtx.lock();
			ray_done_counter++;
			mtx.unlock();
		}
	}
}*/

void Flyscene::raytraceScene(int width, int height) {
	std::cout << "ray tracing ..." << std::endl;
	auto start = std::chrono::high_resolution_clock::now();

	//////////////////////////
	//Set up for progress bar
	done_ray_tracing = false;
	progress = 0.0f;
	ray_done_counter = 0;
	////////////////////////

  // create 2d vector to hold pixel colors and resize to match image size
  vector<vector<Eigen::Vector3f>> pixel_data;
  pixel_data.resize(image_size[1]);
  for (int i = 0; i < image_size[1]; ++i)
    pixel_data[i].resize(image_size[0]);

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

	vector<vector<Eigen::Vector3f>>& pixel_data_copy = pixel_data;

	// origin of the ray is always the camera center
	Eigen::Vector3f origin = flycamera.getCenter();
	Eigen::Vector3f& origina = origin;
	Eigen::Vector3f screen_coords;
	
	int num_threads = std::thread::hardware_concurrency();
	int total_pixels = image_size[0] * image_size[1];
	int partition_size = ceil(total_pixels / num_threads);
	int counter = 0;
	std::cout << num_threads << endl << total_pixels << endl << partition_size << endl;
	

	vector<vector<pair<Eigen::Vector3f, Eigen::Vector2f>>> partitions;
	vector<pair<Eigen::Vector3f, Eigen::Vector2f>> partition;


	for (int i = 0; i < image_size[0]; i++) {
		for (int j = 0; j < image_size[1]; j++) {
			screen_coords = flycamera.screenToWorld(Eigen::Vector2f(i, j));
			partition.push_back(std::make_pair(screen_coords, Eigen::Vector2f(i, j)));
			counter++;
			
			if (counter == partition_size) {
				counter = 0;
				partitions.push_back(partition);
				partition.clear();
			}
		}
	}
	vector<std::thread> threads;
	for (int id = 0; id < partitions.size(); id++) {
		vector<pair<Eigen::Vector3f, Eigen::Vector2f>>& current_partition = partitions[id];
		auto f = [&origina, &current_partition, this, &pixel_data_copy]() {
			vector<pair<Eigen::Vector3f, Eigen::Vector2f>> thread_partition = current_partition;
			pair<Eigen::Vector3f, Eigen::Vector2f> partition_element;
			for (int k = 0; k < thread_partition.size(); k++) {
				partition_element = thread_partition[k];
				pixel_data_copy[partition_element.second[0]][partition_element.second[1]] = traceRay(origina, partition_element.first);
			}
		};
		threads.push_back(std::thread(f));
	}

	for (int i = 0; i < partitions.size(); i++) {
		threads[i].join();
	}

	
	pixel_data = pixel_data_copy;



	// write the ray tracing result to a PPM image
	Tucano::ImageImporter::writePPMImage("result.ppm", pixel_data);

	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() -start;
	
	std::cout << "ray tracing done! " << std::endl;

	std::cout << "ELAPSED TIME:" << elapsed.count() << endl;
}



Eigen::Vector3f Flyscene::traceRay(Eigen::Vector3f &origin,
                                   Eigen::Vector3f &dest) {
  // just some fake random color per pixel until you implement your ray tracing
  // remember to return your RGB values as floats in the range [0, 1]!!!
  


	int bestIntersectionTriangleIndex = -1;
	float intersection;
	//Store the best intersection (triangle closest to the camera)
	float t = std::numeric_limits<float>::max();

	  int bestIntersectionTriangleIndex = -1;
	  vector<float> intersection;
	  //Store the best intersection (triangle closest to the camera)
	  float t = std::numeric_limits<float>::max();
	 
	  //Loop through all of the faces
	  for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		//get a direction vector
		Tucano::Face currTriangle = mesh.getFace(i);
		intersection = rayTriangleIntersection(origin, direction, currTriangle);
		if (intersection != std::numeric_limits<float>::min() && intersection < t && intersection >= 0) {
			t = intersection;
			bestIntersectionTriangleIndex = i;
		}
	  }
	  if (bestIntersectionTriangleIndex == -1) {
		  return BACKGROUND;
	  }

	Eigen::Vector3f hitPoint = origin + intersection * direction;
	return phongShade(origin, hitPoint, mesh.getFace(bestIntersectionTriangleIndex));
	//return BACKGROUND;
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
		return	std::numeric_limits<float>::min();
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
		return	std::numeric_limits<float>::min();
	}

}

Eigen::Vector3f Flyscene::phongShade(Eigen::Vector3f& origin, Eigen::Vector3f& hitPoint, Tucano::Face& triangle) {

	Eigen::Vector3f lightIntensity = lightrep.getColor().head<3>();

	Tucano::Material::Mtl material = materials[triangle.material_id];
	Eigen::Vector3f ambient = lightIntensity.cwiseProduct(material.getAmbient());

	Eigen::Vector3f lightDirection = (hitPoint -lights.at(0)).normalized();
	Eigen::Vector3f normal = (mesh.getShapeModelMatrix() * triangle.normal).normalized();
	float costheta = abs(normal.dot(lightDirection));
	Eigen::Vector3f diffuse = lightIntensity.cwiseProduct(material.getDiffuse()) * costheta;

	Eigen::Vector3f reflectedLight = (lightDirection - 2 * (abs(lightDirection.dot(normal))) * normal).normalized();
	Eigen::Vector3f eyeToHitPoint = (hitPoint - origin).normalized();
	float cosphi = abs(reflectedLight.dot(eyeToHitPoint));
	Eigen::Vector3f specular = lightIntensity.cwiseProduct(material.getSpecular()) * pow(cosphi, material.getShininess());

	return ambient + diffuse + specular;
}

/*
//calculate the numbe rof iterations we make to generate an image
	total_num_of_rays = (float)(image_size[1] * image_size[0]);
	startTime = clock();
	//start a progress bar thread
	std::thread progressBarThread(progressLoop);

	//Divide up pixel regions for the draw method and create threads that will write to pixel_data
	int seventh = (int)(image_size[1] / 7);

	std::thread draw1Thread = createDrawThread(0, seventh, image_size[0], origin);
	std::thread draw2Thread = createDrawThread(seventh, 2 * seventh, image_size[0], origin);
	std::thread draw3Thread = createDrawThread(2 * seventh, 3 * seventh, image_size[0], origin);
	std::thread draw4Thread = createDrawThread(3 * seventh, 4 * seventh, image_size[0], origin);
	std::thread draw5Thread = createDrawThread(4 * seventh, 5 * seventh, image_size[0], origin);
	std::thread draw6Thread = createDrawThread(5 * seventh, 6 * seventh, image_size[0], origin);
	std::thread draw7Thread = createDrawThread(6 * seventh, image_size[1], image_size[0], origin);

	//Join all of the threads and determine when you are done
	draw1Thread.join();
	draw2Thread.join();
	draw3Thread.join();
	draw4Thread.join();
	draw5Thread.join();
	draw6Thread.join();
	draw7Thread.join();


	done_ray_tracing = true;

	progressBarThread.join();*/