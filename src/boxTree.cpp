#include "boxTree.hpp"

<<<<<<< HEAD
BoxTree::BoxTree(BoundingBox box, int capacity) {
	this->box = box;
	this->capacity = capacity;
	isLeaf = true;
}

BoxTree::BoxTree(Tucano::Mesh& mesh, int capacity) {
	box = BoundingBox::BoundingBox(mesh);
	this->capacity = capacity;
	isLeaf = true;

	// loop trough all faces
	for (int i = 0; i < mesh.getNumberOfFaces(); ++i) {
		Tucano::Face face = mesh.getFace(i);    // get current face
		// TODO: if a face intersects or lies within the bounding box, append its ID to the list of faces
	}

	if (faces.size() > capacity) {
		split();
	}
}

void BoxTree::split() {
	// the node is now an inner node
	isLeaf = false;

	// get the difference between min and max for each dimension
	float dx = (box.getMax.x - box.getMin.x) / 2;
	float dy = (box.getMax.y - box.getMin.y) / 2;
	float dz = (box.getMax.z - box.getMin.z) / 2;

	// pass the difference into a 3D vector
	Eigen::Vector3f vx = Eigen::Vector3f(dx, 0, 0);
	Eigen::Vector3f vy = Eigen::Vector3f(0, dy, 0);
	Eigen::Vector3f vz = Eigen::Vector3f(0, 0, dz);

	// compute the 8 child nodes
	BoundingBox b000 = BoundingBox::BoundingBox(box.getMin, box.getMin + vx + vy + vz);
	BoundingBox b001 = BoundingBox::BoundingBox(box.getMin + vz, box.getMin + vx + vy + 2*vz);
	BoundingBox b010 = BoundingBox::BoundingBox(box.getMin + vy, box.getMin + vx +2*vy + vz);
	BoundingBox b011 = BoundingBox::BoundingBox(box.getMin + vy + vz, box.getMax -vx );
	BoundingBox b100 = BoundingBox::BoundingBox(box.getMin + vx, box.getMin + 2*vx + vy + vz);
	BoundingBox b101 = BoundingBox::BoundingBox(box.getMin + vx + vz, box.getMax - vy);
	BoundingBox b110 = BoundingBox::BoundingBox(box.getMin + vz + vy, box.getMax - vz);
	BoundingBox b111 = BoundingBox::BoundingBox(box.getMin + vx + vy + vz, box.getMax);

	// add the children to the parent node
	children.push_back(BoxTree::BoxTree(b000, capacity));
	children.push_back(BoxTree::BoxTree(b001, capacity));
	children.push_back(BoxTree::BoxTree(b010, capacity));
	children.push_back(BoxTree::BoxTree(b011, capacity));
	children.push_back(BoxTree::BoxTree(b100, capacity));
	children.push_back(BoxTree::BoxTree(b101, capacity));
	children.push_back(BoxTree::BoxTree(b110, capacity));
	children.push_back(BoxTree::BoxTree(b111, capacity));
	
	// distribute the faces of the parent node over the children
	for (BoxTree child : children) {
		for (int i : faces) {
			// TODO: check whether the face intersects the bounding box of the child.
			//		If it does, add the face to the list of faces of that child.
		}
	}

	// now finally empy the list of faces of the parent node (since this is an inner node)
	faces.clear();

	// and repeat the same process for all children
	for (BoxTree child : children) {
		if (child.faces.size() > child.capacity) {
			child.split();
		}
	}
}

// returns the indices of faces that need to be checked by the raytracer
std::list<int> BoxTree::intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& dest) {
	std::list<int> list;

	if (box.boxIntersect(origin, dest)) {

		if (isLeaf) {
			return faces;
		}

		else {
			for (BoxTree child : children) {
				list.merge(child.intersect(origin, dest));
			}
		}
	}

	return list;
=======

BoxTree::BoxTree(BoundingBox box) {

}


BoxTree::BoxTree(BoundingBox box, std::list<BoxTree> children) {
	this->box = box;
	this->children = children;
>>>>>>> triangleClassifier
}