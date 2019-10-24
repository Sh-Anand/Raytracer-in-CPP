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
	std::list<BoxTree> children;
<<<<<<< HEAD
	std::list<int> faces;
=======
	int triangleLimit;
	
>>>>>>> triangleClassifier

	BoxTree(void) {}

<<<<<<< HEAD
	BoxTree(BoundingBox box, int capacity);

	BoxTree(Tucano::Mesh& mesh, int capacity);

	void split();

	std::list<int> intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& dest);
=======
	//create root
	BoxTree(BoundingBox box);

	//create childrean
	BoxTree(BoundingBox box, std::list<BoxTree> children);
>>>>>>> triangleClassifier

};
