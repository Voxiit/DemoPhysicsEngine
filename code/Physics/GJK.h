//
//	GJK.h
//
#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"
#include "../Math/Bounds.h"
#include "Body.h"
#include "Shapes.h"


struct point_t
{
	Vec3 xyz; // The point on the minkowski sum
	Vec3 ptA; // The point on bodyA
	Vec3 ptB; // The point on bodyB

	point_t() : xyz(0.0f), ptA(0.0f), ptB(0.0f) {}
	const point_t& operator = (const point_t& rhs)
	{
		xyz = rhs.xyz;
		ptA = rhs.ptA;
		ptB = rhs.ptB;
		return *this;
	}

	bool operator == (const point_t& rhs) const
	{
		return ((ptA == rhs.ptA) && (ptB == rhs.ptB) && (xyz == rhs.xyz));
	}
};

// Signed volumes
Vec2 SignedVolume1D(const Vec3& s1, const Vec3& s2);
Vec3 SignedVolume2D(const Vec3& s1, const Vec3& s2, const Vec3& s3);
Vec4 SignedVolume3D(const Vec3& s1, const Vec3& s2, const Vec3& s3, const Vec3& s4);

int CompareSigns(float a, float b);

void TestSignedVolumeProjection();

// GJK
class Body;
point_t Support(const Body* bodyA, const Body* bodyB, Vec3 dir, const float bias);
bool SimplexSignedVolumes(point_t* pts, const int num, Vec3& newDir, Vec4& lambdasOut);
bool HasPoint(const point_t simplexPoints[4], const point_t& newPt);
void SortValids(point_t simplexPoints[4], Vec4& lambdas);
static int NumValids(const Vec4& lambdas);

bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB );
bool GJK_DoesIntersect( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB );
void GJK_ClosestPoints( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB );


// EPA
Vec3 BarycentricCoordinates(Vec3 s1, Vec3 s2, Vec3 s3, const Vec3& pt);
Vec3 NormalDirection(const tri_t& tri, const std::vector<point_t>& points);
float SignedDistanceToTriangle(const tri_t& tri, const Vec3& pt, const std::vector<point_t>& points);
int ClosestTriangle(const std::vector<tri_t>& triangles, const std::vector<point_t>& points);
bool HasPoint(const Vec3& w, const std::vector<tri_t> triangles, const std::vector<point_t>& points);
int RemoveTrianglesFacingPoint(const Vec3& pt, std::vector<tri_t>& triangles, const std::vector<point_t>& points);
void FindDanglingEdges(std::vector<edge_t>& danglingEdges, const std::vector<tri_t>& triangles);
float EPA_Expand(const Body* bodyA, const Body* bodyB, const float bias, const point_t simplexPoint[4], Vec3& ptOnA, Vec3& ptOnB);