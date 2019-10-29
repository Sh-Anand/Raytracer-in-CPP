#ifndef __FLYSCENE__
#define __FLYSCENE__

// Must be included before glfw.
#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <tucano/effects/phongmaterialshader.hpp>
#include <tucano/mesh.hpp>
#include <tucano/shapes/camerarep.hpp>
#include <tucano/shapes/cylinder.hpp>
#include <tucano/shapes/sphere.hpp>
#include <tucano/shapes/box.hpp>
#include <tucano/utils/flycamera.hpp>
#include <tucano/utils/imageIO.hpp>
#include <tucano/utils/mtlIO.hpp>
#include <tucano/utils/objimporter.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include "../arealight.hpp"
#include "../arealight.cpp"

#include "boundingBox.hpp"
#include "boxTree.hpp"

class Flyscene {

public:
  Flyscene(void) {}

  /**
   * @brief Initializes the shader effect
   * @param width Window width in pixels
   * @param height Window height in pixels
   */
  void initialize(int width, int height);

  /**
   * Repaints screen buffer.
   **/
  virtual void paintGL();

  /**
   * Perform a single simulation step.
   **/
  virtual void simulate(GLFWwindow *window);

  /**
   * Returns the pointer to the flycamera instance
   * @return pointer to flycamera
   **/
  Tucano::Flycamera *getCamera(void) { return &flycamera; }

  /**
   * @brief Add a new light source
   */
  void addLight(void) {
	  lights.push_back(flycamera.getCenter());
  }

  /**
   * @brief Create a debug ray at the current camera location and passing
   * through pixel that mouse is over
   * @param mouse_pos Mouse cursor position in pixels
   */
  void createDebugRay(const Eigen::Vector2f &mouse_pos);


  /**
   * @brief raytrace your scene from current camera position   
   */
  void raytraceScene(int width = 0, int height = 0);


 
  /**
   * @brief trace a single ray from the camera passing through dest
   * @param origin Ray origin
   * @param dest Other point on the ray, usually screen coordinates
   * @return a RGB color
   */
  Eigen::Vector3f traceRay(Eigen::Vector3f& origin, Eigen::Vector3f& direction, int level, vector<Eigen::Vector3f>& lights, bool areaLight, bool countRay);


  float rayPlaneIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Eigen::Vector3f& planeNormal, Eigen::Vector3f& planePoint);

  float rayTriangleIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Tucano::Face& triangle);

  void createHitPoint(Eigen::Vector3f& point);

  void createBox(Eigen::Vector3f point);

  Tucano::Mesh& getMesh();

  Eigen::Vector3f getInterpolatedNormal(Eigen::Vector3f& trianglePoint, Tucano::Face& triangle);

  Eigen::Vector3f phongShade(Eigen::Vector3f& origin, Eigen::Vector3f& hitPoint, Tucano::Face& triangle, vector<Eigen::Vector3f>& lights);

  float fresnel(Eigen::Vector3f& I, Eigen::Vector3f& N, float& ior);

  bool lightStrikes(Eigen::Vector3f& hitPoint, vector<Eigen::Vector3f>& lights, bool visibleLights[]);

  arealight createAreaLight(Eigen::Vector3f corner, float lengthX, float lengthY, int usteps, int vsteps);

  vector<Eigen::Vector3f> createSpherePoint(Eigen::Vector3f lightPoint);

  void modifyTriangle();

private:
  // A simple phong shader for rendering meshes
  Tucano::Effects::PhongMaterial phong;

  /// A small debug sphere to see where ray intersects
  Tucano::Shapes::Sphere hitCircle = Tucano::Shapes::Sphere(0.02);

  // A fly through camera
  Tucano::Flycamera flycamera;

  // the size of the image generated by ray tracing
  Eigen::Vector2i raytracing_image_size;

  // A camera representation for animating path (false means that we do not
  // render front face)
  Tucano::Shapes::CameraRep camerarep = Tucano::Shapes::CameraRep(false);

  // a frustum to represent the camera in the scene
  Tucano::Shapes::Sphere lightrep;

  // light sources for ray tracing
  vector<Eigen::Vector3f> lights;

  // Scene light represented as a camera
  Tucano::Camera scene_light;

  /// A very thin cylinder to draw a debug ray
  Tucano::Shapes::Cylinder ray = Tucano::Shapes::Cylinder(0.05, 1.0, 16, 64);

  /// Intersected normal
  Tucano::Shapes::Cylinder reflectedRay = Tucano::Shapes::Cylinder(0.05, 1.0, 16, 64);

  /// Intersected normal
  Tucano::Shapes::Cylinder intersectNormal = Tucano::Shapes::Cylinder(0.05, 1.0, 16, 64);

  // light sources for ray tracing
  vector<Tucano::Shapes::Cylinder> intersectionLightRays;

  // Scene meshes
  Tucano::Mesh mesh;

  /// MTL materials
  vector<Tucano::Material::Mtl> materials;
  
  // Bounding box for the object
  BoundingBox objectBox;

  // Octree containing indices of the faces of our mesh
  BoxTree octree;
  
  //Visual aid object for a bounding box
  Tucano::Shapes::Box boundingboxVisual = Tucano::Shapes::Box(0.f, 0.f, 0.f);


  Tucano::Shapes::Sphere boxMin = Tucano::Shapes::Sphere(0.02);
  Tucano::Shapes::Sphere boxMax = Tucano::Shapes::Sphere(0.02);

  Tucano::Shapes::Cylinder X_axies = Tucano::Shapes::Cylinder(0.1, 1.0, 16, 64);
  Tucano::Shapes::Cylinder Y_axies = Tucano::Shapes::Cylinder(0.1, 1.0, 16, 64);
  Tucano::Shapes::Cylinder Z_axies = Tucano::Shapes::Cylinder(0.1, 1.0, 16, 64);

  bool areaLight;

  bool pointLight;
  
  Tucano::Face triangleToModify;

};

#endif // FLYSCENE
