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
	for (unsigned int i = 0; i < mesh.getNumberOfFaces(); i++) {
		facesIndexes.push_back(i);
	}
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

/**
* @brief get current box object and check if these triangles from the parent box intersect this box that we are creating
https://gist.github.com/jflipts/fc68d4eeacfcc04fbdb2bf38e0911850
http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
Möller Tomas, Eric Haines, and Naty Hoffman. Real-Time Rendering. Boca Raton: CRC Press, 2019. Print. Page 760
*/
void BoundingBox::clasifyFaces(std::vector<unsigned int> rootFaces, Tucano::Mesh mesh) {
	std::vector<unsigned int> finalTrianglesIndex;
	for (unsigned int triangleIndex : rootFaces) {
		Tucano::Face currentTriangle = mesh.getFace(triangleIndex);
		Eigen::Vector3f vertices[3] = { (mesh.getShapeModelMatrix() * mesh.getVertex(currentTriangle.vertex_ids[0])).head<3>() ,
		(mesh.getShapeModelMatrix() * mesh.getVertex(currentTriangle.vertex_ids[1])).head<3>(),
		(mesh.getShapeModelMatrix() * mesh.getVertex(currentTriangle.vertex_ids[2])).head<3>() };

		int countVertexesInBox = 0;
		for (Eigen::Vector3f vertex : vertices) {

			if (vmin.x() <= vertex.x() && vmax.x() >= vertex.x()
				&& vmin.y() <= vertex.y() && vmax.y() >= vertex.y()
				&& vmin.z() <= vertex.z() && vmax.z() >= vertex.z()) {
				countVertexesInBox++;
			}
		}

		if (countVertexesInBox > 0) {
			finalTrianglesIndex.push_back(triangleIndex);
		}
		else {
				/*    use separating axis theorem to test overlap between triangle and box */
				/*    need to test for overlap in these directions: */
				
				/*    2) normal of the triangle */
				
			Eigen::Vector3f boxCenter =
				Eigen::Vector3f(vmin.x() + (vmin.x() + vmax.x()) / 2,
					vmin.y() + (vmin.y() + vmax.y()) / 2,
					vmin.z() + (vmin.z() + vmax.z()) / 2);


			Eigen::Vector3f boxhalfsize = Eigen::Vector3f(vmax.x(), vmax.x(), vmax.z());

			Eigen::Vector3f A_origin = vertices[0] - boxCenter;
			Eigen::Vector3f B_origin = vertices[1] - boxCenter;
			Eigen::Vector3f C_origin = vertices[2] - boxCenter;


			Eigen::Vector3f e_0 = B_origin - A_origin;
			Eigen::Vector3f e_1 = C_origin - B_origin;
			Eigen::Vector3f e_2 = A_origin - C_origin;
			
			/*    3) crossproduct(edge from tri, {x,y,z}-directin) */
			/*       this gives 3x3=9 more tests */
			float fex;
			float fey;
			float fez;


			fex = fabsf(e_0.x());
			fey = fabsf(e_0.y());
			fez = fabsf(e_0.z());

			if (!axisTestX01(e_0.z(), e_0.y(), fez, fey, A_origin, C_origin, boxhalfsize)) {
				continue;
			}
			if (!axisTestY02(e_0.z(), e_0.x(), fez, fex, A_origin, C_origin, boxhalfsize)) {
				continue;
			}
			if (!axisTestZ12(e_0.y(), e_0.x(), fey, fex, B_origin, C_origin, boxhalfsize)) {
				continue;
			}

			fex = fabsf(e_1.x());
			fey = fabsf(e_1.y());
			fez = fabsf(e_1.z());

			if (!axisTestX01(e_1.z(), e_1.y(), fez, fey, A_origin, C_origin, boxhalfsize)){
				continue;
			}
			if (!axisTestY02(e_1.z(), e_1.x(), fez, fex, A_origin, C_origin, boxhalfsize)){
				continue;
			}
			if (!axisTestZ0(e_1.y(), e_1.x(), fey, fex, A_origin, B_origin, boxhalfsize)){
				continue;
			}

			fex = fabsf(e_2.x());
			fey = fabsf(e_2.y());
			fez = fabsf(e_2.z());

			if (!axisTestX2(e_2.z(), e_2.y(), fez, fey, A_origin, B_origin, boxhalfsize)) {
				continue;
			}
			if (!axisTestY1(e_2.z(), e_2.x(), fez, fex, A_origin, B_origin, boxhalfsize)) {
				continue;
			}
			if (!axisTestZ12(e_2.y(), e_2.x(), fey, fex, B_origin, C_origin, boxhalfsize)) {
				continue;
			}

			Eigen::Vector3f min;
			Eigen::Vector3f max;
			findMinMax(A_origin, B_origin, C_origin, min, max);

			/*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
			/*       we do not even need to test these) */

			/* test in X-direction */
			if (min.x() > boxhalfsize.x() || max.x() < -boxhalfsize.x()) {
				continue;
			}
			/* test in Y-direction */
			if (min.y() > boxhalfsize.y() || max.y() < -boxhalfsize.y()) {
				continue;
			}
			/* test in Z-direction */
			if (min.z() > boxhalfsize.z() || max.z() < -boxhalfsize.z()) {
				continue;
			}
			Eigen::Vector3f normal = e_0.cross(e_1);
			if (!planeBoxOverlap(normal, A_origin, boxhalfsize)) {
				continue;
			}
			 /* box and triangle overlaps */
			finalTrianglesIndex.push_back(triangleIndex);
		}
	}
	this->facesIndexes = finalTrianglesIndex;
}

void BoundingBox::findMinMax(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f& min, Eigen::Vector3f& max) {
	min = Eigen::Vector3f(std::min(a.x(), std::min(b.x(), c.x())),
		std::min(a.y(), std::min(b.y(), c.y())),
		std::min(a.z(), std::min(b.z(), c.z())));
	max = Eigen::Vector3f(std::max(a.x(), std::max(b.x(), c.x())),
		std::max(a.y(), std::max(b.y(), c.y())),
		std::max(a.z(), std::max(b.z(), c.z())));
}

bool BoundingBox::planeBoxOverlap(Eigen::Vector3f normal, Eigen::Vector3f vert, Eigen::Vector3f maxbox) {
	Eigen::Vector3f vmin, vmax;
	float v;
	for (int i = 0; i < 3; i++) {
		v = vert[i];
		if (normal[i] > 0.0f) {
			vmin[i] = -maxbox[i] - v;
			vmax[i] = maxbox[i] - v;
		}
		else {
			vmin[i] = maxbox[i] - v;
			vmax[i] = -maxbox[i] - v;
		}
	}
	if (normal.dot(vmin) > 0.0f)
		return false;
	if (normal.dot(vmax) >= 0.0f)
		return true;

	return false;
}

/*======================== X-tests ========================*/
bool BoundingBox::axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
	const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize) {
	float p0 = a * v0.y() - b * v0.z();
	float p2 = a * v2.y() - b * v2.z();

	float max = std::max(p2, p0);
	float min = std::min(p2, p0);;
	
	float rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoundingBox::axisTestX2(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
	Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize) {
	float p0 = a * v0.y() - b * v0.z();
	float p1 = a * v1.y() - b * v1.z();

	float max = std::max(p1, p0);
	float min = std::min(p1, p0);;
	
	float rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}
/*======================== Y-tests ========================*/
bool BoundingBox::axisTestY1(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
	Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize) {
	float p0 = -a * v0.x() + b * v0.z();
	float p1 = -a * v1.x() + b * v1.z();

	float max = std::max(p1, p0);
	float min = std::min(p1, p0);;

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoundingBox::axisTestY02(float a, float b, float fa, float fb, const Eigen::Vector3f& v0,
	const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize) {
	float p0 = -a * v0.x() + b * v0.z();
	float p2 = -a * v2.x() + b * v2.z();

	float max = std::max(p2, p0);
	float min = std::min(p2, p0);;

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

/*======================== Z-tests ========================*/
bool BoundingBox::axisTestZ0(float a, float b, float fa, float fb, Eigen::Vector3f& v0,
	Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize) {
	float p0 = a * v0.x() - b * v0.y();
	float p1 = a * v1.x() - b * v1.y();

	float max = std::max(p1, p0);
	float min = std::min(p1, p0);;

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoundingBox::axisTestZ12(float a, float b, float fa, float fb, const Eigen::Vector3f& v1,
	const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize) {
	float p1 = a * v1.x() - b * v1.y();
	float p2 = a * v2.x() - b * v2.y();

	float max = std::max(p1, p2);
	float min = std::min(p1, p2);;

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}



Eigen::Vector3f BoundingBox::getMin() {
	return vmin;
}

Eigen::Vector3f BoundingBox::getMax() {
	return vmax;
}

