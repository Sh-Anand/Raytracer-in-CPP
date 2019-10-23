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
	//Tucano::Box box;
	std::vector<unsigned int> facesIndexes;

public:
	BoundingBox(void){}

	BoundingBox(const Eigen::Vector3f &minv, const Eigen::Vector3f &maxv);

	BoundingBox(Tucano::Mesh& mesh);

	bool boxIntersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &dest);

	//fill in faces
	void clasifyFaces(std::vector<unsigned int> rootFaces, Tucano::Mesh mesh);

	Eigen::Vector3f getMin();

	Eigen::Vector3f getMax();

	void findMinMax(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f& min, Eigen::Vector3f& max);

	bool planeBoxOverlap(Eigen::Vector3f normal, Eigen::Vector3f vert, Eigen::Vector3f maxbox);

	bool axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestY02(float a, float b, float fa, float fb, const Eigen::Vector3f & v0,
		const Eigen::Vector3f & v2, const Eigen::Vector3f & boxhalfsize);

	bool axisTestZ12(float a, float b, float fa, float fb, const Eigen::Vector3f& v1,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestZ0(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	bool axisTestX2(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	bool axisTestY1(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);
	
};

