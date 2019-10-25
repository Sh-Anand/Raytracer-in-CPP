#include "arealight.hpp"

arealight::arealight(Eigen::Vector3f lightPos, Eigen::Vector3f corner2, Eigen::Vector3f uvec2, float usteps2, Eigen::Vector3f vvec2, float vsteps2) {
	corner = corner2;
	uvec = uvec2 / usteps2;
	usteps = usteps2;
	vvec = vvec2 / vsteps2;
	vsteps = vsteps2;
	samples = usteps2 * vsteps2;
	pos = lightPos;
}