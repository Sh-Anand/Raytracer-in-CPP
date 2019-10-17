#pragma once
#include "tucano/utils/misc.hpp"
#include <Eigen/Dense>
#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class box
{
public:
	box(const Eigen::Vector3f &vmin, const Eigen::Vector3f &vmax)
	{
		bounds[0] = vmin;
		bounds[1] = vmax;
	}
	Eigen::Vector3f bounds[2];
};

