#include "boundingBox.hpp"
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>

boundingBox::boundingBox(const Eigen::Vector3f &vmin, const Eigen::Vector3f &vmax)
	{
		bounds[0] = vmin;
		bounds[1] = vmax;
	}	

bool boundingBox::boxIntersect(Eigen::Vector3f &origin, Eigen::Vector3f &dir)
	{
		//Get the bounds of the box
		Eigen::Vector3f vmin = bounds[0];
		Eigen::Vector3f vmax = bounds[1];

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
		return true;
	}	