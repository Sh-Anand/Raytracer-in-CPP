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
#include <tucano/utils/flycamera.hpp>
#include <tucano/utils/imageIO.hpp>
#include <tucano/utils/mtlIO.hpp>
#include <tucano/utils/objimporter.hpp>
#include <thread>

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
  void addLight(void) { lights.push_back(flycamera.getCenter()); }

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
  Eigen::Vector3f traceRay(Eigen::Vector3f& origin, Eigen::Vector3f& dest);

  float rayPlaneIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Eigen::Vector3f& planeNormal, Eigen::Vector3f& planePoint);

  float rayTriangleIntersection(Eigen::Vector3f& rayPoint, Eigen::Vector3f& rayDirection, Tucano::Face& triangle);

  void createHitPoint(Eigen::Vector3f& point);

  Eigen::Vector3f phongShade(Eigen::Vector3f& origin, Eigen::Vector3f& hitPoint, Tucano::Face& triangle);


  

private:
  // A simple phong shader for rendering meshes
  Tucano::Effects::PhongMaterial phong;

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
  Tucano::Shapes::Cylinder ray = Tucano::Shapes::Cylinder(0.1, 1.0, 16, 64);

  // Scene meshes
  Tucano::Mesh mesh;

  /// MTL materials
  vector<Tucano::Material::Mtl> materials;

  vector<vector<Eigen::Vector3f>> pixel_data;
};

#endif // FLYSCENE
