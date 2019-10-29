#include "boxTree.hpp"

#define MAX_DEPTH 10
BoxTree::BoxTree(BoundingBox box, int capacity) {
	this->box = box;
	this->capacity = capacity;
	isLeaf = false;
	isEmpty = false;
}

BoxTree::BoxTree(Tucano::Mesh& mesh, int capacity) {
	box = BoundingBox::BoundingBox(mesh);
	this->capacity = capacity;
	isLeaf = false;
	isEmpty = false;

	// add all indices to list of faces
	for (int i = 0; i < mesh.getNumberOfFaces(); ++i) {
		faces.push_back(i);
	}

	if (faces.size() > capacity) {
		this->split(mesh, MAX_DEPTH);
	}
	else if (faces.empty()) {
		isEmpty = true;
	}
	else {
		isLeaf = true;
	}
}

void BoxTree::makeBoxSmaller(Tucano::Mesh& meshRef) {
	Eigen::Vector3f min = this->box.getMin();
	Eigen::Vector3f max = this->box.getMax();
	float newMinX = min.x();
	float newMaxX = max.x();
	float newMinY = min.y();
	float newMaxY = max.y();
	float newMinZ = min.z();
	float newMaxZ = max.z();

	float newMinXtemp = min.x();
	float newMaxXtemp = max.x();
	float newMinYtemp = min.y();
	float newMaxYtemp = max.y();
	float newMinZtemp = min.z();
	float newMaxZtemp = max.z();
	for (int i = 0; i < this->faces.size(); i++) {
		Eigen::Vector3f vertices[3] = { (meshRef.getShapeModelMatrix() * meshRef.getVertex(meshRef.getFace(i).vertex_ids[0])).head<3>() ,
	(meshRef.getShapeModelMatrix() * meshRef.getVertex(meshRef.getFace(i).vertex_ids[1])).head<3>(),
	(meshRef.getShapeModelMatrix() * meshRef.getVertex(meshRef.getFace(i).vertex_ids[2])).head<3>() };
		bool triangleFullyInside = true;
		for (Eigen::Vector3f vertex : vertices) {

			if (this->box.getMin().x() <= vertex.x() && this->box.getMax().x() >= vertex.x()
				&& this->box.getMin().y() <= vertex.y() && this->box.getMax().y() >= vertex.y()
				&& this->box.getMin().z() <= vertex.z() && this->box.getMax().z() >= vertex.z()
				) {
				newMinXtemp = std::min(newMinX, vertex.x());
				newMaxXtemp = std::max(newMaxX, vertex.x());
				newMinYtemp = std::min(newMinY, vertex.y());
				newMaxYtemp = std::max(newMaxY, vertex.y());
				newMinZtemp = std::min(newMinZ, vertex.z());
				newMaxZtemp = std::max(newMaxZ, vertex.z());
			}
			else {
				triangleFullyInside = false;
				break;
			}
		}
		if (triangleFullyInside) {
			newMinX = newMinXtemp;
			newMaxX = newMaxXtemp;
			newMinY = newMinYtemp;
			newMaxY = newMaxYtemp;
			newMinZ = newMinZtemp;
			newMaxZ = newMaxZtemp;
		}
	}
	Eigen::Vector3f newMin = Eigen::Vector3f(std::min(newMinX, min.x()), std::min(newMinY, min.y()), std::min(newMinZ, min.z()));
	Eigen::Vector3f newMax = Eigen::Vector3f(std::max(newMaxX, max.x()), std::max(newMaxY, max.y()), std::max(newMaxZ, max.z()));
	
	this->box.setMin(newMin);
	this->box.setMax(newMax);
}

void BoxTree::split(Tucano::Mesh& meshRef, int depth) {
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
	BoundingBox b001 = BoundingBox::BoundingBox(box.getMin() + vz, box.getMin() + vx + vy + 2 * vz);
	BoundingBox b010 = BoundingBox::BoundingBox(box.getMin() + vy, box.getMin() + vx + 2 * vy + vz);
	BoundingBox b011 = BoundingBox::BoundingBox(box.getMin() + vy + vz, box.getMin() + vx + 2 * vy + 2 * vz);
	BoundingBox b100 = BoundingBox::BoundingBox(box.getMin() + vx, box.getMin() + 2 * vx + vy + vz);
	BoundingBox b101 = BoundingBox::BoundingBox(box.getMin() + vx + vz, box.getMax() - vy);
	BoundingBox b110 = BoundingBox::BoundingBox(box.getMin() + vx + vy, box.getMax() - vz);
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
	for (int boxIndex = 0; boxIndex < children.size(); boxIndex++) {
		for (int i : this->faces) {
			if (children.at(boxIndex).clasifyFace(i, meshRef)) {
				children.at(boxIndex).faces.push_back(i);
			}
		}
	}
	/*std::cout << "Parent faces number: "<< this->faces.size()<< std::endl;
	std::cout << "Used faces: " << used.size() << std::endl;*/
	// now finally empy the list of faces of the parent node (since this is an inner node)
	faces.clear();

	//// and repeat the same process for all children
	for (int boxIndex = 0; boxIndex < children.size(); boxIndex++) {
		if (children.at(boxIndex).faces.empty() && children.at(boxIndex).children.empty()) {
			children.at(boxIndex).isEmpty = true;
		}
		if (children.at(boxIndex).faces.size() < children.at(boxIndex).capacity || depth <=0) {
			children.at(boxIndex).isLeaf = true;
		}
		if (children.at(boxIndex).faces.size() > children.at(boxIndex).capacity && depth > 0) {
			children.at(boxIndex).split(meshRef, depth - 1);
		}
	}
}

// returns the indices of faces that need to be checked by the raytracer
std::set<int> BoxTree::intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& dest) {
	std::set<int> list;

	std::queue<BoxTree> toVisit;
	toVisit.push(*this);
	while (!toVisit.empty()) {
		BoxTree currentBoxTree = toVisit.front();
		toVisit.pop();
		if (currentBoxTree.box.boxIntersect(origin, dest)) {
			if (currentBoxTree.isLeaf && !currentBoxTree.isEmpty) {
				list.insert(currentBoxTree.faces.begin(), currentBoxTree.faces.end());
			}
			else if (!currentBoxTree.isEmpty){
				for (BoxTree tree : currentBoxTree.children) {
					if (!tree.isEmpty && tree.box.boxIntersect(origin, dest)) {
						toVisit.push(tree);
					}
				}
			}
		}
	}

	return list;
}

std::vector<pair<Eigen::Vector3f, Eigen::Vector3f>> BoxTree::leafBoxes() {
	std::vector<pair<Eigen::Vector3f, Eigen::Vector3f>> finalList;

	std::queue<BoxTree> toVisit;
	toVisit.push(*this);
	while (!toVisit.empty()) {
		BoxTree currentBoxTree = toVisit.front();
		toVisit.pop();
		if (currentBoxTree.isLeaf && !currentBoxTree.isEmpty) {
			finalList.push_back(std::make_pair(currentBoxTree.box.getMin(), currentBoxTree.box.getMax()));
		}
		else if (!currentBoxTree.isEmpty) {
			for (BoxTree tree : currentBoxTree.children) {
				toVisit.push(tree);
			}
		}
	}

	return finalList;
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

		if (this->box.getMin().x() <= vertex.x() && this->box.getMax().x() >= vertex.x()
			&& this->box.getMin().y() <= vertex.y() && this->box.getMax().y() >= vertex.y()
			&& this->box.getMin().z() <= vertex.z() && this->box.getMax().z() >= vertex.z()
			) {
			countVertexesInBox++;
		}
	}

	if (countVertexesInBox > 0) {
		return true;
	}
	else {
	//	/*    use separating axis theorem to test overlap between triangle and box */
	//	/*    need to test for overlap in these directions: */

	//	/*    2) normal of the triangle */

		Eigen::Vector3f boxcenter =
			Eigen::Vector3f(this->box.getMin().x() + (this->box.getMax().x() - this->box.getMin().x()) / 2.f,
				this->box.getMin().y() + (this->box.getMax().y() - this->box.getMin().y()) / 2.f,
				this->box.getMin().z() + (this->box.getMax().z() - this->box.getMin().z()) / 2.f);


		Eigen::Vector3f boxhalfsize = (this->box.getMax() - boxcenter).normalized();

		Eigen::Vector3f a_origin = (vertices[0] - boxcenter).normalized();
		Eigen::Vector3f b_origin = (vertices[1] - boxcenter).normalized();
		Eigen::Vector3f c_origin = (vertices[2] - boxcenter).normalized();


		Eigen::Vector3f e_0 = b_origin - a_origin;
		Eigen::Vector3f e_1 = c_origin - b_origin;
		Eigen::Vector3f e_2 = a_origin - c_origin;

		/*    3) crossproduct(edge from tri, {x,y,z}-directin) */
		/*       this gives 3x3=9 more tests */
		float fex;
		float fey;
		float fez;


		fex = fabsf(e_0.x());
		fey = fabsf(e_0.y());
		fez = fabsf(e_0.z());

		if (!axisTestX01(e_0.z(), e_0.y(), fez, fey, a_origin, c_origin, boxhalfsize)) {
			return false;
		}
		if (!axisTestY02(e_0.z(), e_0.x(), fez, fex, a_origin, c_origin, boxhalfsize)) {
			return false;
		}
		if (!axisTestZ12(e_0.y(), e_0.x(), fey, fex, b_origin, c_origin, boxhalfsize)) {
			return false;
		}

		fex = fabsf(e_1.x());
		fey = fabsf(e_1.y());
		fez = fabsf(e_1.z());

		if (!axisTestX01(e_1.z(), e_1.y(), fez, fey, a_origin, c_origin, boxhalfsize)) {
			return false;
		}
		if (!axisTestY02(e_1.z(), e_1.x(), fez, fex, a_origin, c_origin, boxhalfsize)) {
			return false;
		}
		if (!axisTestZ0(e_1.y(), e_1.x(), fey, fex, a_origin, b_origin, boxhalfsize)) {
			return false;
		}

		fex = fabsf(e_2.x());
		fey = fabsf(e_2.y());
		fez = fabsf(e_2.z());

		if (!axisTestX02(e_2.z(), e_2.y(), fez, fey, a_origin, b_origin, boxhalfsize)) {
			return false;
		}
		if (!axisTestY1(e_2.z(), e_2.x(), fez, fex, a_origin, b_origin, boxhalfsize)) {
			return false;
		}
		if (!axisTestZ12(e_2.y(), e_2.x(), fey, fex, b_origin, c_origin, boxhalfsize)) {
			return false;
		}
		

		/*    1) the {x,y,z}-directions (actually, since we use the aabb of the triangle */
		/*       we do not even need to test these) */
		

		/* test in x-direction */
		pair<float, float> minmaxpair = findMinMax(a_origin.x(), b_origin.x(), c_origin.x());
		float min = minmaxpair.first;
		float max = minmaxpair.second;

		if (min > boxhalfsize.x() || max < -boxhalfsize.x()) {
			return false;
		}
		/* test in y-direction */
		minmaxpair = findMinMax(a_origin.y(), b_origin.y(), c_origin.y());
		min = minmaxpair.first;
		max = minmaxpair.second;
		if (min > boxhalfsize.y() || max < -boxhalfsize.y()) {
			return false;
		}
		/* test in z-direction */
		minmaxpair = findMinMax(a_origin.z(), b_origin.z(), c_origin.z());
		min = minmaxpair.first;
		max = minmaxpair.second;
		if (min > boxhalfsize.z() || max < -boxhalfsize.z()) {
			return false;
		}

		//get edges of triangle
		Eigen::Vector3f edge1trianglea = a_origin - b_origin;
		Eigen::Vector3f edge2trianglea = a_origin - c_origin;
		//get the normal
		Eigen::Vector3f normaltrianglea = (edge1trianglea.cross(edge2trianglea)).normalized();
		//Eigen::Vector3f normal = e_0.cross(e_1);

		if (!planeBoxOverlap(normaltrianglea, a_origin, boxhalfsize)) {
			return false;
		}
		return true;
	}
}

std::pair<float, float> BoxTree::findMinMax(float a, float b, float c)
{
	float min = std::min(std::min(a, b), c);
	float max = std::max(std::max(a, b), c);
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
/*checked*/
bool BoxTree::axisTestX01(float a, float b, float fa, float fb, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& boxhalfsize)
{
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
/*checked*/
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
/*checked*/
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
/*checked*/
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
/*checked*/
bool BoxTree::axisTestX02(float a, float b, float fa, float fb, Eigen::Vector3f& v0, Eigen::Vector3f& v1, Eigen::Vector3f& boxhalfsize)
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
/*checked*/
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
