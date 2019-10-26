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

//Eigen::Vector3f arealight::getCorner() {
//	return arealight::corner;
//}
//
//Eigen::Vector3f getUVec() {
//	return arealight::uvec;
//}
//
//float getUSteps() {
//	return usteps
//}
//
//Eigen::Vector3f getVVec() {
//	return vvec;
//}
//
//float getVSteps() {
//	return vsteps;
//}
//
//float getSamples() {
//	return samples;
//}
//
//Eigen::Vector3f getPos() {
//	return pos;
//}