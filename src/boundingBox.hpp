#pragma once
#include "tucano/utils/misc.hpp"
#include <Eigen/Dense>
#include <memory>
#include <GL/glew.h>

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
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class BoundingBox
{
private:
	Eigen::Vector3f vmin;
	Eigen::Vector3f vmax;

public:
	BoundingBox(void){}

	BoundingBox(const Eigen::Vector3f &minv, const Eigen::Vector3f &maxv);

	BoundingBox(Tucano::Mesh& mesh);

	bool boxIntersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &dest);

	Eigen::Vector3f getMin();

	Eigen::Vector3f getMax();
};

