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
	std::list<int> faces;

	BoxTree(void) {}

	BoxTree(BoundingBox box, int capacity);

	BoxTree(Tucano::Mesh& mesh, int capacity);

	void split();

};
