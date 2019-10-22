#pragma once
#include "tucano/utils/misc.hpp"
#include <Eigen/Dense>
#include <memory>
#include <GL/glew.h>
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class BoundingBox
{
private:
	Eigen::Vector3f vmin;
	Eigen::Vector3f vmax;
	//Tucano::Box box;

public:
	BoundingBox(void){}

	BoundingBox(const Eigen::Vector3f &minv, const Eigen::Vector3f &maxv);

	bool boxIntersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &dest);

	Eigen::Vector3f getMin();

	Eigen::Vector3f getMax();
};

