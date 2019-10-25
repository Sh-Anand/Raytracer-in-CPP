#pragma once
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

class arealight {
public:
	arealight(Eigen::Vector3f lightPos, Eigen::Vector3f corner, Eigen::Vector3f uvec, float usteps, Eigen::Vector3f vvec, float vsteps);

private:
	Eigen::Vector3f corner;

	Eigen::Vector3f uvec;

	float usteps;

	Eigen::Vector3f vvec;

	float vsteps;

	float samples;

	Eigen::Vector3f pos;
};
