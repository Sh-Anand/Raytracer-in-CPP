#include "boundingBox.hpp"
// Must be included before glfw.
//#include <GL/glew.h>
//#include <GLFW/glfw3.h>


	/**
	* @brief Default Constructor
	*/
BoundingBox::BoundingBox(const Eigen::Vector3f &minv, const Eigen::Vector3f &maxv): vmin(minv), vmax(maxv)
{}

// Create a bounding box around the edges of the mesh, to serve as a root for the tree of boxes
BoundingBox::BoundingBox(Tucano::Mesh& mesh) {

	// instantiate min and max coordinates as very large and small floats, respectively
	float xmin = std::numeric_limits<float>::max();
	float ymin = std::numeric_limits<float>::max();
	float zmin = std::numeric_limits<float>::max();
	float xmax = std::numeric_limits<float>::min(); // or set to 0 /origin?
	float ymax = std::numeric_limits<float>::min();
	float zmax = std::numeric_limits<float>::min();

	//loop trough the mesh (get each vertex of each face)
	for (int i = 0; i < mesh.getNumberOfFaces(); ++i) {
		Tucano::Face face = mesh.getFace(i);    // get current face
		for (int j = 0; j < face.vertex_ids.size(); ++j) {
			Eigen::Vector4f vert = mesh.getShapeModelMatrix() * mesh.getVertex(face.vertex_ids[j]); // get current vertex
			float x = vert.x();
			float y = vert.y();
			float z = vert.z();
			// update min and max values by comparing against current vertex
			xmin = std::min(xmin, x);
			ymin = std::min(ymin, y);
			zmin = std::min(zmin, z);
			xmax = std::max(xmax, x);
			ymax = std::max(ymax, y);
			zmax = std::max(zmax, z);
		}
	}
	vmin = Eigen::Vector3f(xmin, ymin, zmin);
	vmax = Eigen::Vector3f(xmax, ymax, zmax);
}

/**
* @brief Intersection test
*/
bool BoundingBox::boxIntersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &dest)
{
	//Calculate direction
	Eigen::Vector3f dir = dest - origin;

	//Calculate intersection parameter for six sides of box
	float txmin = (vmin.x() - origin.x()) / dir.x();
	float txmax = (vmax.x() - origin.x()) / dir.x();
	float tymin = (vmin.y() - origin.y()) / dir.y();
	float tymax = (vmax.y() - origin.y()) / dir.y();
	float tzmin = (vmin.z() - origin.z()) / dir.z();
	float tzmax = (vmax.z() - origin.z()) / dir.z();

	//Determine when we first cross (in point) and last cross (out) the correspondong
	//planes along each axis
	float tinx = std::min(txmin, txmax);
	float toutx = std::max(txmin, txmax);
	float tiny = std::min(tymin, tymax);
	float touty = std::max(tymin, tymax);
	float tinz = std::min(tzmin, tzmax);
	float toutz = std::max(tzmin, tzmax);

	//In and out points for x, y, z
	float tin = std::max(std::max(tinx, tiny), tinz);
	float tout = std::min(std::min(toutx, touty), toutz);


	//Conditions
	if ((tin > tout) || (tout < 0)) {
		return false;
	}
	else {
		return true;
	}

}

Eigen::Vector3f BoundingBox::getMin() {
	return vmin;
}

Eigen::Vector3f BoundingBox::getMax() {
	return vmax;
}

void BoundingBox::setMin(Eigen::Vector3f min) {
	vmin = min;
}

void BoundingBox::setMax(Eigen::Vector3f max) {
	vmax = max;
}

