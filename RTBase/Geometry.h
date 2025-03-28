﻿#pragma once
#include <algorithm>
#include "Core.h"
#include "Sampling.h"
#include <iostream>


class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d; // d = - n dot p0
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		//float denom = n.dot(r.dir); // denominator
		//if (fabs(denom) < 1e-6) return false; // ray is parallel to plane
		// 
		//t = -(d + n.dot(r.o)) / denom; //???????????????

		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }

		t = (d - Dot(n, r.o)) / denom;
		return (t>=0); // t is how many step to reach plane
	}
};

//#define EPSILON 0.001f
#define EPSILON 0.0000001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p); // d = - normal dot p0 originally but here it changed to normal dot p0 ad -d
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }

		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }

		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;

	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		//3.13
		float r1 = sampler->next();
		float r2 = sampler->next();

		float alpha = 1 - sqrtf(r1);
		float beta = r2 * sqrtf(r1);
		float gamma = 1.0f - (alpha + beta);
		pdf = 1.0f / area;
		Vec3 p = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
		return p;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// re
	void extend(const AABB& box)
	{
		extend(box.min);
		extend(box.max);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 t_B_min = (min - r.o) * r.invDir;
		Vec3 t_B_max = (max - r.o) * r.invDir;
		Vec3 t_entry = Min(t_B_min, t_B_max); // see what is MIn mean!
		Vec3 t_exit = Max(t_B_min, t_B_max);
		float t_entry_f = std::max(std::max(t_entry.x, t_entry.y), t_entry.z);
		float t_exit_f = std::min(std::min(t_exit.x, t_exit.y), t_exit.z);

		if (t_exit_f < t_entry_f || t_exit_f < 0) return false;
		t = (t_entry_f < 0) ? t_exit_f : t_entry_f;
		return true;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		Vec3 t_B_min = (min - r.o) * r.invDir;
		Vec3 t_B_max = (max - r.o) * r.invDir;
		Vec3 t_entry = Min(t_B_min, t_B_max); // see what is MIn mean!
		Vec3 t_exit = Max(t_B_min, t_B_max);
		float t_entry_f = std::max(std::max(t_entry.x, t_entry.y), t_entry.z);
		float t_exit_f = std::min(std::min(t_exit.x, t_exit.y), t_exit.z);

		if (t_exit_f < t_entry_f || t_exit_f < 0) return false;
		return true;
	}

	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 l = r.o - centre;
		float b = l.dot(r.dir);
		float c = l.dot(l) - SQ(radius);
		float dis = SQ(b) - c;

		if(dis<0) return false;

		float dissqrt = sqrtf(dis);
		float t1 = -b + dissqrt;
		float t2 = -b - dissqrt;

		if (dis > 0) {
			if (t2 > 0) t = t2;
			else if (t1 > 0) t = t1;
			else return false;

			return true;
		}
		t = -b;

		return t > 0;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

struct TriangleComparator {
	int axis;
	TriangleComparator(int a) :axis(a){}

	bool operator()(const Triangle& a, const Triangle& b) {
		if (axis == 0) return a.centre().x < b.centre().x;
		if (axis == 1) return a.centre().y < b.centre().y;
		if (axis == 2) return a.centre().z < b.centre().z;
	}
};

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;

	int startIndex;
	int endIndex;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
		startIndex = 0;
		endIndex = 0;
	}

	static AABB triangleBound(const Triangle& tri)
	{
		AABB box;
		box.reset();
		box.extend(tri.vertices[0].p);
		box.extend(tri.vertices[1].p);
		box.extend(tri.vertices[2].p);
		return box;
	}

	void buildRecursive(std::vector<Triangle>& triangles, int start, int end) {
		//std::cout << "BVH extent: " << start << " - " << end << "\n";
		// 1 calculate AABB of current node
		bounds.reset();
		for (int i = start; i < end; i++) {
			bounds.extend(triangles[i].vertices[0].p);
			bounds.extend(triangles[i].vertices[1].p);
			bounds.extend(triangles[i].vertices[2].p);
		}

		// 2 if num of tris less than threshold, then this node is a leaf node
		int numTri = end - start;
		if (numTri <= MAXNODE_TRIANGLES) {
			this->startIndex = start;
			this->endIndex = end;
			//std::cout << "BVH extent: " << startIndex << " - " << endIndex << "\n";
			return;
		}
		// 3 else split
		// 3-1 choose longest axis as split axis
		int axis = 0; // choose x axis first
		Vec3 axis_size = bounds.max - bounds.min;
		if (axis_size.y >= axis_size.x && axis_size.y >= axis_size.z) axis = 1; // y axis
		else if (axis_size.z >= axis_size.x && axis_size.z >= axis_size.y) axis = 2; // z axis

		// 3-2 sort triangles on the axis
		std::sort(triangles.begin() + start, triangles.begin() + end, TriangleComparator(axis));

		// 3-3 pre-compute ans store left-to-right and right-to-left AABBs - avoid redundant calculations
		// left prefix and right suffix array
		std::vector<AABB> leftBounds(numTri), rightBounds(numTri);
		leftBounds[0] = triangleBound(triangles[start]);
		rightBounds[numTri-1] = triangleBound(triangles[end-1]);
		// store left-to-right AABBs
		for (int i = 1; i < numTri; i++) {
			leftBounds[i] = leftBounds[i - 1];
			leftBounds[i].extend(triangleBound(triangles[start + i]));
		}
		// store right-to-left AABBs
		for (int i = numTri - 2; i >= 0; i--) {
			rightBounds[i] = rightBounds[i + 1];
			rightBounds[i].extend(triangleBound(triangles[start + i]));
		}
		// 3-4 iterate split position i, and calculate,compare SAH to get the best split position
		// cost_split = cost_bounds + (left area * left num + right area * right num) * cost_sect / parent area
		float cost_min = FLT_MAX;
		int split_index = 0;

		for (int i = 1; i < numTri; i++) {
			float area_left = leftBounds[i-1].area();
			float area_right = rightBounds[i].area();
			float num_left = i;
			float num_right = numTri - i;
			float cost = area_left * num_left + area_right * num_right;

			if (cost < cost_min) {
				cost_min = cost;
				split_index = i;
			}
		}
		// 3-5 split to left and right node
		int mid = start + split_index;
		l = new BVHNode();
		r = new BVHNode();
		l->buildRecursive(triangles, start, mid);
		r->buildRecursive(triangles, mid, end);

	}
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// Add BVH building code here
		outputTriangles = inputTriangles;
		buildRecursive(outputTriangles, 0, static_cast<int>(outputTriangles.size()));
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		// stack ?
		// 1 if ray intersec the bound
		if (!bounds.rayAABB(ray)) {
			return;
		}
		// 2 if node is leaf, intersect tris in this node
		if (!l && !r) {
			for (int i = startIndex; i < endIndex; i++) {
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v)) {
					if (t < intersection.t && t > 1e-4f)
					{
						intersection.t = t;  //update smallest t
						intersection.ID = i;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}
		// 3 if node is not a leaf, call traverse on left and right childe nodes
		if (l) l->traverse(ray, triangles, intersection);
		if (r) r->traverse(ray, triangles, intersection);
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		float t;
		// if ray not intersect bound - visible
		if (!bounds.rayAABB(ray, t)) {
			return true;
		}
		// or p2-p1 length less than t
		if(t > maxT) return true;
		// iterate all tris in this node
		if (!l && !r) {
			for (int i = startIndex; i < endIndex; i++) {
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v)) {
					if (t < maxT && t > 1e-4f)
					{
						return false;
					}
				}
			}
			return true;
		}
		bool leftVisible = (l ? l->traverseVisible(ray, triangles, maxT) : true);
		bool rightVisible = (r ? r->traverseVisible(ray, triangles, maxT) : true);
		return (leftVisible && rightVisible);
	}
};

class BVHNode2
{
public:
	AABB bounds;
	BVHNode2* l;
	BVHNode2* r;

	int startIndex;
	int endIndex;

	BVHNode2()
		: l(nullptr)
		, r(nullptr)
		, startIndex(0)
		, endIndex(0)
	{
	}

	static AABB triangleBounds(const Triangle& tri)
	{
		AABB box;
		box.reset();
		box.extend(tri.vertices[0].p);
		box.extend(tri.vertices[1].p);
		box.extend(tri.vertices[2].p);
		return box;
	}
	// using indices to sort
	void buildRecursive(std::vector<Triangle>& triangles, int start, int end)
	{
		bounds.reset();
		for (int i = start; i < end; i++) {
			bounds.extend(triangles[i].vertices[0].p);
			bounds.extend(triangles[i].vertices[1].p);
			bounds.extend(triangles[i].vertices[2].p);
		}

		int numTriangles = end - start;
		if (numTriangles <= MAXNODE_TRIANGLES)
		{
			this->startIndex = start;
			this->endIndex = end;
			return;
		}


		Vec3 size = bounds.max - bounds.min;
		int axis = 0;
		if (size.y > size.x && size.y > size.z)
			axis = 1;
		else if (size.z > size.x && size.z > size.y)
			axis = 2;

		std::sort(triangles.begin() + start, triangles.begin() + end,
			[axis](const Triangle& a, const Triangle& b) {
				float ca = (axis == 0) ? a.centre().x : (axis == 1) ? a.centre().y : a.centre().z;
				float cb = (axis == 0) ? b.centre().x : (axis == 1) ? b.centre().y : b.centre().z;
				return ca < cb;
			});

		int n = numTriangles;
		std::vector<AABB> leftBounds(n);
		std::vector<AABB> rightBounds(n);
		leftBounds[0] = triangleBounds(triangles[start]);
		for (int i = 1; i < n; i++) {
			leftBounds[i] = leftBounds[i - 1];
			leftBounds[i].extend(triangleBounds(triangles[start + i]));
		}
		rightBounds[n - 1] = triangleBounds(triangles[end - 1]);
		for (int i = n - 2; i >= 0; i--) {
			rightBounds[i] = rightBounds[i + 1];
			rightBounds[i].extend(triangleBounds(triangles[start + i]));
		}

		float totalArea = bounds.area();
		float bestCost = FLT_MAX;
		int bestSplit = -1;

		for (int i = 1; i < n; i++) {
			float leftArea = leftBounds[i - 1].area();
			float rightArea = rightBounds[i].area();
			int leftCount = i;
			int rightCount = n - i;
			float cost = 1.0f + (leftArea * leftCount + rightArea * rightCount) / totalArea;
			if (cost < bestCost) {
				bestCost = cost;
				bestSplit = i;
			}
		}

		if (bestCost >= static_cast<float>(numTriangles)) {
			this->startIndex = start;
			this->endIndex = end;
			return;
		}

		int mid = start + bestSplit;
		l = new BVHNode2();
		r = new BVHNode2();
		l->buildRecursive(triangles, start, mid);
		r->buildRecursive(triangles, mid, end);
	}

	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		triangles = inputTriangles;
		buildRecursive(triangles, 0, static_cast<int>(triangles.size()));
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		float tBox;
		if (!bounds.rayAABB(ray, tBox)) {
			return;
		}

		if (!l && !r)
		{
			for (int i = startIndex; i < endIndex; i++)
			{
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v) && t > 1e-4f && t < intersection.t)
				{
					/*intersection.t = t;
					intersection.alpha = 1 - u - v;
					intersection.beta = u;
					intersection.gamma = v;
					intersection.ID = i;*/

					intersection.t = t;
					intersection.ID = i;
					intersection.alpha = u;
					intersection.beta = u;
					intersection.gamma = 1.0f - (u + v);
				}
			}
			return;
		}

		if (l) l->traverse(ray, triangles, intersection);
		if (r) r->traverse(ray, triangles, intersection);
	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}

	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, float maxT)
	{
		float tBox;
		float tMin, tMax;
		if (!bounds.rayAABB(ray, tMin)) {
			return true;
		}

		if (tMin > maxT) {
			return true;
		}

		if (!l && !r)
		{
			for (int i = startIndex; i < endIndex; i++) // missing
			{
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v) && t > 1e-4f && t < maxT)
				{
					return false;
				}
			}
			return true;
		}

		bool leftVis = l ? l->traverseVisible(ray, triangles, maxT) : true;
		bool rightVis = r ? r->traverseVisible(ray, triangles, maxT) : true;
		return leftVis && rightVis;
		/*bool leftVis = l ? l->traverseVisible(ray, triangles, maxT) : true;
		if (leftVis)
			return true;
		bool rightVis = r ? r->traverseVisible(ray, triangles, maxT) : true;
		return rightVis;*/
	}

};



