#include "boxTree.hpp"

BoxTree::BoxTree(BoundingBox box, int capacity) {
	this->box = box;
	this->capacity = capacity;
	isLeaf = true;
}

BoxTree::BoxTree(Tucano::Mesh& mesh, int capacity) {
	box = BoundingBox::BoundingBox(mesh);
	this->capacity = capacity;
	isLeaf = true;

	// add all indices to list of faces
	for (int i = 0; i < mesh.getNumberOfFaces(); ++i) {
		faces.push_back(i);
	}

	if (faces.size() > capacity) {
		split();
	}
}

void BoxTree::split() {
	// the node is now an inner node
	isLeaf = false;

	// get half of the difference between min and max for each dimension
	float dx = (box.getMax().x() - box.getMin().x()) / 2;
	float dy = (box.getMax().y() - box.getMin().y()) / 2;
	float dz = (box.getMax().z() - box.getMin().z()) / 2;

	// pass the difference into a 3D vector
	Eigen::Vector3f vx = Eigen::Vector3f(dx, 0, 0);
	Eigen::Vector3f vy = Eigen::Vector3f(0, dy, 0);
	Eigen::Vector3f vz = Eigen::Vector3f(0, 0, dz);

	// compute the 8 child nodes
	BoundingBox b000 = BoundingBox::BoundingBox(box.getMin(), box.getMin() + vx + vy + vz);
	BoundingBox b001 = BoundingBox::BoundingBox(box.getMin() + vz, box.getMin() + vx + vy + 2*vz);
	BoundingBox b010 = BoundingBox::BoundingBox(box.getMin() + vy, box.getMin() + vx +2*vy + vz);
	BoundingBox b011 = BoundingBox::BoundingBox(box.getMin() + vy + vz, box.getMax() -vx );
	BoundingBox b100 = BoundingBox::BoundingBox(box.getMin() + vx, box.getMin() + 2*vx + vy + vz);
	BoundingBox b101 = BoundingBox::BoundingBox(box.getMin() + vx + vz, box.getMax() - vy);
	BoundingBox b110 = BoundingBox::BoundingBox(box.getMin() + vz + vy, box.getMax() - vz);
	BoundingBox b111 = BoundingBox::BoundingBox(box.getMin() + vx + vy + vz, box.getMax());

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
		if (child.faces.empty()) {
			children.remove(child);
		}
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
}


/**
* @brief get current box object and check if these triangles from the parent box intersect this box that we are creating
https://gist.github.com/jflipts/fc68d4eeacfcc04fbdb2bf38e0911850
http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
Möller Tomas, Eric Haines, and Naty Hoffman. Real-Time Rendering. Boca Raton: CRC Press, 2019. Print. Page 760
*/
bool BoxTree::clasifyFace(int faceIndex, Tucano::Mesh& mesh)
{
	Tucano::Face currentTriangle = mesh.getFace(faceIndex);
	Eigen::Vector3f vertices[3] = { (mesh.getShapeModelMatrix() * mesh.getVertex(currentTriangle.vertex_ids[0])).head<3>() ,
	(mesh.getShapeModelMatrix() * mesh.getVertex(currentTriangle.vertex_ids[1])).head<3>(),
	(mesh.getShapeModelMatrix() * mesh.getVertex(currentTriangle.vertex_ids[2])).head<3>() };

	int countVertexesInBox = 0;
	for (Eigen::Vector3f vertex : vertices) {

		if (box.getMin().x() <= vertex.x() && box.getMax().x() >= vertex.x()
			&& box.getMin().y() <= vertex.y() && box.getMax().y() >= vertex.y()
			&& box.getMin().z() <= vertex.z() && box.getMax().z() >= vertex.z()
			) {
			countVertexesInBox++;
		}
	}

	if (countVertexesInBox > 0) {
		return true;
	}
	else {
		/*    use separating axis theorem to test overlap between triangle and box */
		/*    need to test for overlap in these directions: */

		/*    2) normal of the triangle */

		Eigen::Vector3f boxCenter =
			Eigen::Vector3f(box.getMin().x() + (box.getMin().x() + box.getMax().x()) / 2,
				box.getMin().y() + (box.getMin().y() + box.getMax().y()) / 2,
				box.getMin().z() + (box.getMin().z() + box.getMax().z()) / 2);


		Eigen::Vector3f boxhalfsize = Eigen::Vector3f(box.getMax().x(), box.getMax().x(), box.getMax().z());

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
			return true;
		}
		if (!axisTestY02(e_0.z(), e_0.x(), fez, fex, A_origin, C_origin, boxhalfsize)) {
			return true;
		}
		if (!axisTestZ12(e_0.y(), e_0.x(), fey, fex, B_origin, C_origin, boxhalfsize)) {
			return true;
		}

		fex = fabsf(e_1.x());
		fey = fabsf(e_1.y());
		fez = fabsf(e_1.z());

		if (!axisTestX01(e_1.z(), e_1.y(), fez, fey, A_origin, C_origin, boxhalfsize)) {
			return true;
		}
		if (!axisTestY02(e_1.z(), e_1.x(), fez, fex, A_origin, C_origin, boxhalfsize)) {
			return true;
		}
		if (!axisTestZ0(e_1.y(), e_1.x(), fey, fex, A_origin, B_origin, boxhalfsize)) {
			return true;
		}

		fex = fabsf(e_2.x());
		fey = fabsf(e_2.y());
		fez = fabsf(e_2.z());

		if (!axisTestX2(e_2.z(), e_2.y(), fez, fey, A_origin, B_origin, boxhalfsize)) {
			return true;
		}
		if (!axisTestY1(e_2.z(), e_2.x(), fez, fex, A_origin, B_origin, boxhalfsize)) {
			return true;
		}
		if (!axisTestZ12(e_2.y(), e_2.x(), fey, fex, B_origin, C_origin, boxhalfsize)) {
			return true;
		}

		pair<Eigen::Vector3f, Eigen::Vector3f> minMaxPair = findMinMax(A_origin, B_origin, C_origin);
		Eigen::Vector3f min = minMaxPair.first;
		Eigen::Vector3f max = minMaxPair.second;

		/*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
		/*       we do not even need to test these) */

		/* test in X-direction */
		if (min.x() > boxhalfsize.x() || max.x() < -boxhalfsize.x()) {
			return true;
		}
		/* test in Y-direction */
		if (min.y() > boxhalfsize.y() || max.y() < -boxhalfsize.y()) {
			return true;
		}
		/* test in Z-direction */
		if (min.z() > boxhalfsize.z() || max.z() < -boxhalfsize.z()) {
			return true;
		}
		Eigen::Vector3f normal = e_0.cross(e_1);
		if (!planeBoxOverlap(normal, A_origin, boxhalfsize)) {
			return false;
		}
		return true;
	}
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> BoxTree::findMinMax(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c)
{
	Eigen::Vector3f min = Eigen::Vector3f(std::min(a.x(), std::min(b.x(), c.x())),
		std::min(a.y(), std::min(b.y(), c.y())),
		std::min(a.z(), std::min(b.z(), c.z())));
	Eigen::Vector3f max = Eigen::Vector3f(std::max(a.x(), std::max(b.x(), c.x())),
		std::max(a.y(), std::max(b.y(), c.y())),
		std::max(a.z(), std::max(b.z(), c.z())));
	return std::make_pair(min, max);
}

bool BoxTree::planeBoxOverlap(Eigen::Vector3f normal, Eigen::Vector3f vert, Eigen::Vector3f maxbox)
{
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

bool BoxTree::axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3f& v0, const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize)
{
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

bool BoxTree::axisTestY02(float a, float b, float fa, float fb, const Eigen::Vector3f& v0, const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize)
{
	float p0 = -a * v0.x() + b * v0.z();
	float p2 = -a * v2.x() + b * v2.z();

	float max = std::max(p2, p0);
	float min = std::min(p2, p0);

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoxTree::axisTestZ12(float a, float b, float fa, float fb, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& boxhalfsize)
{
	float p1 = a * v1.x() - b * v1.y();
	float p2 = a * v2.x() - b * v2.y();

	float max = std::max(p1, p2);
	float min = std::min(p1, p2);

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoxTree::axisTestZ0(float a, float b, float fa, float fb, Eigen::Vector3f& v0, Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize)
{
	float p0 = a * v0.x() - b * v0.y();
	float p1 = a * v1.x() - b * v1.y();

	float max = std::max(p1, p0);
	float min = std::min(p1, p0);

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoxTree::axisTestX2(float a, float b, float fa, float fb, Eigen::Vector3f& v0, Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize)
{
	float p0 = a * v0.y() - b * v0.z();
	float p1 = a * v1.y() - b * v1.z();

	float max = std::max(p1, p0);
	float min = std::min(p1, p0);

	float rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}

bool BoxTree::axisTestY1(float a, float b, float fa, float fb, Eigen::Vector3f& v0, Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize)
{
	float p0 = -a * v0.x() + b * v0.z();
	float p1 = -a * v1.x() + b * v1.z();

	float max = std::max(p1, p0);
	float min = std::min(p1, p0);

	float rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();
	if (min > rad || max < -rad) {
		return false;
	}
	return true;
}
