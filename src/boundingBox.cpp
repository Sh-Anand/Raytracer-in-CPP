#include "boundingBox.hpp"
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>


	/**
	* @brief Default Constructor
	*/
boundingBox::boundingBox(Eigen::Vector3f& minv, Eigen::Vector3f& maxv): vmin(minv), vmax(maxv)
{}

/**
* @brief Intersection test
*/
bool boundingBox::boxIntersect(Eigen::Vector3f &origin, Eigen::Vector3f &dest)
{
	//Calculate direction
	Eigen::Vector3f dir = dest - origin;

	//Calculate intersection parameter for six sides of box
	float txmin = (vmin.x - origin.x) / dir.x;
	float txmax = (vmax.x - origin.x) / dir.x;
	float tymin = (vmin.y - origin.y) / dir.y;
	float tymax = (vmax.y - origin.y) / dir.y;
	float tzmin = (vmin.z - origin.z) / dir.z;
	float tzmax = (vmax.z - origin.z) / dir.z;

	//Determine when we first cross (in point) and last cross (out) the correspondong
	//planes along each axis
	float tinx = std::min(txmin, txmax);
	float toutx = std::max(txmin, txmax);
	float tiny = std::min(tymin, tymax);
	float touty = std::max(tymin, tymax);
	float tinz = std::min(tzmin, tzmax);
	float toutz = std::max(tzmin, tzmax);

	//In and out points for x, y, z
	float tin = std::max(tinx, tiny, tinz);
	float tout = std::min(toutx, touty, toutz);

	//Conditions
	if ((tin > tout) || (tout < 0)) {
		return false;
	}
	else {
		return true;
	}
}