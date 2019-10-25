#include "flyscene.hpp"
#include <GLFW/glfw3.h>
#include<iostream>
#include <thread>
#include <mutex>
#include <ctime>
#define BACKGROUND Eigen::Vector3f(1.f, 1.f, 1.f)
#define SHADOW Eigen::Vector3f(0.f, 0.f, 0.f)
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
                                    "resources/models/glassstraw.obj");


  // normalize the model (scale to unit cube and center at origin)
  mesh.normalizeModelMatrix();

  // pass all the materials to the Phong Shader
  for (int i = 0; i < materials.size(); ++i)
    phong.addMaterial(materials[i]);



  // set the color and size of the sphere to represent the light sources
  // same sphere is used for all sources
  lightrep.setColor(Eigen::Vector4f(1.0, 1.0, 1.0, 1.0));
  lightrep.setSize(0.15);

  // create a first ray-tracing light source at some random position
  lights.push_back(Eigen::Vector3f(-1.0, 1.0, 1.0));

  // scale the camera representation (frustum) for the ray debug
  camerarep.shapeMatrix()->scale(0.2);

  // the debug ray is a cylinder, set the radius and length of the cylinder
  ray.setSize(0.005, 10.0);

  reflectedRay.setSize(0.005, 10.0);

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

  reflectedRay.render(flycamera, scene_light);

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
void Flyscene::createHitPoint(Eigen::Vector3f& point) {
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

	// place the camera representation (frustum) on current camera location, 
	camerarep.resetModelMatrix();
	camerarep.setModelMatrix(flycamera.getViewMatrix().inverse());

	float intersection;
	Tucano::Face riangle;
	bool intersected = false;
	float t = std::numeric_limits<float>::max();
	for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		Tucano::Face currTriangle = mesh.getFace(i);
		intersection =
			rayTriangleIntersection(screen_pos, dir, currTriangle);
		if (intersection != -72) {
			intersected = true;
			if (intersection < t) {
				t = intersection;
				riangle = currTriangle;
			}
		}
	}

	if (intersected) {
		//std::cout <<  endl << "Illuminationmodel: " << materials[riangle.material_id].getIlluminationModel()<<endl;
		Eigen::Vector3f hitPoint = screen_pos + (t * dir);
		createHitPoint(hitPoint);
		Eigen::Vector3f normal = riangle.normal;
		Eigen::Vector3f reflectionDir = (dir - (2 * (dir.dot(normal)) * normal));
		std::cout << reflectionDir;
		reflectedRay.setOriginOrientation(hitPoint, reflectionDir);
		float intersection_r;
		bool intersected_r = false;
		float t_r = std::numeric_limits<float>::max();
		for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
			Tucano::Face currTriangle = mesh.getFace(i);
			intersection_r =
				rayTriangleIntersection(hitPoint, reflectionDir, currTriangle);
			std::cout << "t: " << intersection_r << endl;
			if (intersection_r != -72 && intersection_r > 0.00001) {
				intersected_r = true;
				if (intersection_r < t_r) {
					t_r = intersection_r;
				}
			}
		}

		Eigen::Vector3f points = hitPoint + 2 * reflectionDir;
		std::cout << intersected_r;
		Eigen::Vector3f hitPoint_r = hitPoint + (t_r * reflectionDir);
		createHitPoint(hitPoint_r);
		if (intersected_r) {
			Eigen::Vector3f hitPoint_r = hitPoint + (t_r * reflectionDir);
		}
		
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
				Eigen::Vector3f direction = partition_element.first - origina;
				pixel_data_copy[partition_element.second[0]][partition_element.second[1]] = traceRay(origina, direction , 0, lights);
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
                                   Eigen::Vector3f &direction, int level, vector<Eigen::Vector3f>& lights) {
  // just some fake random color per pixel until you implement your ray tracing
  // remember to return your RGB values as floats in the range [0, 1]!!!
	
	if (level == 2)
		return Eigen::Vector3f(-1, -1, -1);


	int bestIntersectionTriangleIndex = -1;
	float intersection;
	//Store the best intersection (triangle closest to the camera)
	float t = std::numeric_limits<float>::max();
	Tucano::Face currTriangle, intersectTriangle;

	//Loop through all of the faces
	for (int i = 0; i < mesh.getNumberOfFaces(); i++) {
		//get a direction vector
		currTriangle = mesh.getFace(i);
		intersection = rayTriangleIntersection(origin, direction, currTriangle);
		if (intersection != -72 && intersection < t && intersection > 0.00001) {
			t = intersection;
			intersectTriangle = currTriangle;
			bestIntersectionTriangleIndex = i;
		}
	}
	if (bestIntersectionTriangleIndex == -1) {
		if (level == 0)
			return BACKGROUND;
		return Eigen::Vector3f(-1, -1, -1);
	}

	Eigen::Vector3f hitPoint = origin + (t * direction);
	Eigen::Vector3f Color = Eigen::Vector3f(-1,-1,-1);
	Eigen::Vector3f faceNormal = intersectTriangle.normal;

	const int numberOfLights = lights.size();
	bool visibleLights[5];
	bool lightStrikesHitPoint = lightStrikes(hitPoint, lights, visibleLights);

	if (!lightStrikesHitPoint) {
		return SHADOW;
	}


	Tucano::Material::Mtl material = materials[intersectTriangle.material_id];
	int imodel = material.getIlluminationModel();

	float fresnelIndex = 1;

	if (imodel > 2 && imodel <= 7) {
		Eigen::Vector3f reflectedDirection = direction - 2 * (direction.dot(faceNormal)) * faceNormal;
		vector<Eigen::Vector3f> reflectedLights;
		reflectedLights.push_back(hitPoint);

		Color = traceRay(hitPoint, reflectedDirection, level + 1, reflectedLights);
		if (imodel == 5) {
			float opticalDensity = material.getOpticalDensity();
			fresnelIndex = fresnel(reflectedDirection, faceNormal, opticalDensity);
			return fresnelIndex * Color;
		}
	}

	if (imodel == 6 || imodel == 7) {
		float c1 = direction.dot(faceNormal);
		if (c1 < 0)
			faceNormal = -1 * faceNormal;
		c1 = abs(c1);
		float c2 = sqrt(1 - (pow((1 / material.getOpticalDensity()), 2)) * (1 - pow(c1, 2)));
		Eigen::Vector3f refractedRay = material.getOpticalDensity() * direction + (material.getOpticalDensity() * c1 - c2) * faceNormal;
		if (imodel == 7) {
			Color = fresnelIndex * Color + (1-fresnelIndex)* traceRay(hitPoint, refractedRay, level + 1, lights);
		}
		else {
			Color = traceRay(hitPoint, refractedRay, level + 1, lights);
		}
	}

	if (Color == Eigen::Vector3f(-1, -1, -1)) {
		Color = phongShade(origin, hitPoint, intersectTriangle, lights, visibleLights);
	}


	return Color;
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

//Returns  t parameter of light ray to get point of intersection. 
float Flyscene::rayTriangleIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Tucano::Face& triangle) {
	Eigen::Vector3f vertices[3] = { (mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[0])).head<3>() ,
		(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[1])).head<3>(),
		(mesh.getShapeModelMatrix() * mesh.getVertex(triangle.vertex_ids[2])).head<3>() };

	Eigen::Vector3f triangleNormal = triangle.normal;
	if (rayDirection.dot(triangleNormal) == 0) {
		return	-72;
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
		return	-72;
	}

}

//Computes phong shading at the given point with interpolated normals.
Eigen::Vector3f Flyscene::phongShade(Eigen::Vector3f& origin, Eigen::Vector3f& hitPoint, Tucano::Face& triangle, vector<Eigen::Vector3f>& lights, bool visibleLights[]) {

	Eigen::Vector3f lightIntensity = Eigen::Vector3f(1, 1, 1);

	Tucano::Material::Mtl material = materials[triangle.material_id];
	Eigen::Vector3f ambient = lightIntensity.cwiseProduct(material.getAmbient());
	Eigen::Vector3f colour = Eigen::Vector3f(0.0, 0.0, 0.0);
	Eigen::Vector3f normal = (mesh.getModelMatrix()*getInterpolatedNormal(hitPoint,triangle)).normalized();

	for (int i = 0; i < lights.size(); i++) {
		if (!visibleLights[i])
			continue;
		Eigen::Vector3f lightDirection = (lights.at(i) - hitPoint).normalized();
		
		float costheta = max(0.0f, lightDirection.dot(normal));
		Eigen::Vector3f diffuse = lightIntensity.cwiseProduct(material.getDiffuse()) * costheta;

		Eigen::Vector3f reflectedLight = (lightDirection - ((2 * lightDirection.dot(normal)) * normal)).normalized();
		Eigen::Vector3f eyeToHitPoint = (-1 * (hitPoint -  origin)).normalized();
		float cosphi = std::max(0.0f, eyeToHitPoint.dot(-1*reflectedLight));
		Eigen::Vector3f specular = lightIntensity.cwiseProduct(material.getSpecular()) * pow(cosphi, material.getShininess());

		colour += ambient + diffuse + specular;
	}
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
	float intersection;
	//Store the best intersection (triangle closest to the camera)
	float t = std::numeric_limits<float>::max();
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
			intersection = rayTriangleIntersection(origin, direction, currTriangle);
			if (intersection != -72 && intersection < t && intersection > 0.00001) {
				t = intersection;
			}
		}

		if (t >= 0.99) {
			hit = true;
			visibleLights[l] = true;
		}
		else {
			visibleLights[l] = false;
		}
	}
	return hit;
}
