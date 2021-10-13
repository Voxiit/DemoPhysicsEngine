//
//	ShapeConvex.h
//
#pragma once
#include "ShapeBase.h"

struct tri_t {
	int a;
	int b;
	int c;
};

struct edge_t {
	int a;
	int b;

	bool operator == ( const edge_t & rhs ) const {
		return ( ( a == rhs.a && b == rhs.b ) || ( a == rhs.b && b == rhs.a ) );
	}
};

void BuildConvexHull( const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< tri_t > & hullTris );

/*
====================================================
ShapeConvex
====================================================
*/
class ShapeConvex : public Shape {
public:
	explicit ShapeConvex( const Vec3 * pts, const int num ) {
		Build( pts, num );
	}
	void Build( const Vec3 * pts, const int num ) override;

	Vec3 Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const override;

	Mat3 InertiaTensor() const override { return m_inertiaTensor; }

	Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const override;
	Bounds GetBounds() const override { return m_bounds; }

	float FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const override;

	shapeType_t GetType() const override { return SHAPE_CONVEX; }

public:
	std::vector< Vec3 > m_points;
	Bounds m_bounds;
	Mat3 m_inertiaTensor;
};

// Convex Hulls function
int FindPointFurthestInDir(const Vec3* pts, const int num, const Vec3& dir);
float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pt);
Vec3 FindPointFurthestFromLine(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB);
float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt);
Vec3 FindPointFurthestFromTriangle(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB, const Vec3& ptC);
void BuildTetrahedron(const Vec3* verts, const int num, std::vector<Vec3>& hullPts, std::vector<tri_t>& hullTri);
void ExpandConvexHulls(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri, const std::vector<Vec3>& verts);
void RemoveInternalPoints(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri, std::vector<Vec3>& checkPts);
bool IsEdgeUnique(std::vector<tri_t>& tris, std::vector<int>& facingTris, const int ignoreTri, const edge_t& edge);
void AddPoint(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri, const Vec3& pt);
void RemoveUnreferencedVerts(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri);

// Intertia tensor functions
bool IsExternal(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris, const Vec3& pt);
Vec3 CalculateCenterOfMass(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris);
Mat3 CalculateInertiaTensor(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris, const Vec3& cm);