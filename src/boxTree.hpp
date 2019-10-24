#pragma once
#include "tucano/utils/misc.hpp"
#include <Eigen/Dense>
#include <memory>
#include <GL/glew.h>
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "boundingBox.hpp"
#include <list>
#include <tucano/mesh.hpp>

class BoxTree {
public:
	BoundingBox box;
	int capacity;
	bool isLeaf;
	bool isEmpty;
	std::vector<BoxTree> children;
	std::vector<int> faces;

	BoxTree(void) {}

	BoxTree(BoundingBox box, int capacity);

	BoxTree(Tucano::Mesh& mesh, int capacity);

	void split(Tucano::Mesh& meshRef);

	std::vector<int> intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& dest);

	//fill in faces
	bool clasifyFace(int faceIndex, Tucano::Mesh& mesh);

	std::pair<Eigen::Vector3f, Eigen::Vector3f> findMinMax(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c);

	bool planeBoxOverlap(Eigen::Vector3f normal, Eigen::Vector3f vert, Eigen::Vector3f maxbox);

	bool axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestY02(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestZ12(float a, float b, float fa, float fb, const Eigen::Vector3f& v1,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestZ0(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	bool axisTestX2(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	bool axisTestY1(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);
};
