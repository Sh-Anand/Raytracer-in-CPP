#include "boxTree.hpp"

BoxTree::BoxTree(BoundingBox box, std::list<BoxTree> children, std::list<Tucano::Face> faces) {
	this->box = box;
	this->children = children;
	this->faces = faces;
}