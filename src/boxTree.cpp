#include "boxTree.hpp"


BoxTree::BoxTree(BoundingBox box) {

}


BoxTree::BoxTree(BoundingBox box, std::list<BoxTree> children) {
	this->box = box;
	this->children = children;
}