#include "box.hpp"
// Must be included before glfw.
#include <GL/glew.h>
#include <GLFW/glfw3.h>


public:
	void box(const Eigen::Vector3f &vmin, const Eigen::Vector3f &vmax)
	{
		bounds[0] = vmin;
		bounds[1] = vmax;
	}

	bool boxIntersect(Eigen::Vector3f& origin, Eigen::Vector3f& dest)
	{
		//float tmin = (min.x - origin.x) / dest.x;
		//float tmax = (max.x - origin.x) / dest.x;

		//if (tmin > tmax) swap(tmin, tmax);

		//float tymin = (min.y - origin.y) / dest.y;
		//float tymax = (max.y - origin.y) / dest.y;

		//if (tymin > tymax) swap(tymin, tymax);

		//if ((tmin > tymax) || (tymin > tmax))
		//	return false;

		//if (tymin > tmin)
		//	tmin = tymin;

		//if (tymax < tmax)
		//	tmax = tymax;

		//float tzmin = (min.z - origin.z) / dest.z;
		//float tzmax = (max.z - origin.z) / dest.z;

		//if (tzmin > tzmax) swap(tzmin, tzmax);

		//if ((tmin > tzmax) || (tzmin > tmax))
		//	return false;

		//if (tzmin > tmin)
		//	tmin = tzmin;

		//if (tzmax < tmax)
		//	tmax = tzmax;

		//return true;
		Eigen::Vector3f dir = normalize(destination - origin);
		Eigen::Vector3f vmin = bounds[0];
		Eigen::Vector3f vmax = bounds[1];

		float txmin = vmin.x - dir.x;
		float txmax = vmax.x - dir.x;
		float tymin = vmin.y - dir.y;
		float txmax = vmax.y - dir.y;
		float tzmin = vmin.z - dir.z;
		float tzmax = vmax.z - dir.z;

		
	}