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
#include <queue>
#include <tucano/mesh.hpp>

class BoxTree {
public:
	BoundingBox box;
	int capacity;
	bool isLeaf;
	bool isEmpty;
	std::vector<BoxTree> children;
	std::vector<int> faces;
	std::vector<int> vertecies;
	Eigen::Vector3f firstQuartileAxies;
	Eigen::Vector3f centerAxies;
	Eigen::Vector3f secondQuartileAxies;

	BoxTree(void) {}

	BoxTree(BoundingBox box, int capacity);

	BoxTree(Tucano::Mesh& mesh, int capacity);

	void split(Tucano::Mesh& meshRef, int depth);

	std::set<int> intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& dest);

	//fill in faces
	bool clasifyFace(int faceIndex, Tucano::Mesh& mesh);

	std::pair<float, float> findMinMax(float a, float b, float c);

	bool planeBoxOverlap(Eigen::Vector3f normal, Eigen::Vector3f vert, Eigen::Vector3f maxbox);

	bool axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestY02(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestZ12(float a, float b, float fa, float fb, const Eigen::Vector3f& v1,
		const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize);

	bool axisTestZ0(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	bool axisTestX02(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	bool axisTestY1(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
		Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize);

	void makeBoxSmaller(Tucano::Mesh& meshRef);
};
