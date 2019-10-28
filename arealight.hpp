#pragma once

#include <thread>

class arealight {
public:
	arealight(Eigen::Vector3f& corner, Eigen::Vector3f& uvec, int usteps, Eigen::Vector3f& vvec, int vsteps) {
		this->corner = corner;
		this->uvec = uvec;
		this->usteps = usteps;
		this->vvec = vvec;
		this->vsteps = vsteps;
	}

	vector<Eigen::Vector3f> getPointLights() {
		Eigen::Vector3f middleCell;
		vector<Eigen::Vector3f> lights;
		for (int i = 0; i < usteps; i++) {
			for (int j = 0; j < vsteps; j++) {
				middleCell = Eigen::Vector3f(((i + 0.5) * (uvec/usteps)).x(), ((j + 0.5) * (vvec/vsteps)).y(), uvec.z());
				lights.push_back(middleCell);
			}
		}
		return lights;
	}

	Eigen::Vector3f corner;

	Eigen::Vector3f uvec;

	int usteps;

	Eigen::Vector3f vvec;

	int vsteps;

};
