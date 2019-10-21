#pragma once
#include "tucano/utils/misc.hpp"
#include <Eigen/Dense>
#include <memory>
#include <GL/glew.h>
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class boundingBox
{
private:
	Eigen::Vector3f vmin;
	Eigen::Vector3f vmax;
	//Tucano::Box box;

public:
	boundingBox(Eigen::Vector3f &minv, Eigen::Vector3f &maxv);

	bool boxIntersect(Eigen::Vector3f &origin, Eigen::Vector3f &dest);
};

