//
//  ShapeConvex.cpp
//
#include "ShapeConvex.h"
#include "../../Math/Random.h"

/*
========================================================================================================

ShapeConvex

========================================================================================================
*/
void BuildConvexHull( const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< tri_t > & hullTris ) {
	if (verts.size() < 4)
	{
		return;
	}

	// Build tetrahedron
	BuildTetrahedron(verts.data(), (int)verts.size(), hullPts, hullTris);

	ExpandConvexHulls(hullPts, hullTris, verts);
}


/*
====================================================
ShapeConvex::Build
====================================================
*/
void ShapeConvex::Build( const Vec3 * pts, const int num ) {
	m_points.clear();
	m_points.reserve(num);
	for (int i = 0; i < num; i++)
	{
		m_points.push_back(pts[i]);
	}

	// Expand into a convex hull
	std::vector<Vec3> hullPoints;
	std::vector<tri_t> hullTriangles;
	BuildConvexHull(m_points, hullPoints, hullTriangles);
	m_points = hullPoints;

	// Expand the bounds
	m_bounds.Clear();
	m_bounds.Expand(m_points.data(), m_points.size());

	//m_centerOfMass = CalculateCenterOfMass(hullPoints, hullTriangles);
	//m_centerOfMass = CalculateCenterOfMassMonteCarlo(hullPoints, hullTriangles);
	m_centerOfMass = CalculateCenterOfMassTetrahedron(hullPoints, hullTriangles);

	//m_inertiaTensor = CalculateInertiaTensor(hullPoints, hullTriangles, m_centerOfMass);
	//m_inertiaTensor = CalculateInertiaTensorMonteCarlo(hullPoints, hullTriangles, m_centerOfMass);
	m_inertiaTensor = CalculateInertiaTensorTetrahedron(hullPoints, hullTriangles, m_centerOfMass);
}

/*
====================================================
ShapeConvex::Support
====================================================
*/
Vec3 ShapeConvex::Support( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	// Finf the point in the furthest direction
	Vec3 maxPt = orient.RotatePoint(m_points[0]) + pos;
	float maxDist = dir.Dot(maxPt);
	for (int i = 1; i < m_points.size(); i++)
	{
		const Vec3 pt = orient.RotatePoint(m_points[i]) + pos;
		const float dist = dir.Dot(pt);

		if (dist > maxDist)
		{
			maxDist = dist;
			maxPt = pt;
		}
	}

	Vec3 norm = dir;
	norm.Normalize();
	norm *= bias;
	return maxPt + norm;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Vec3 corners[8];
	corners[0] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z);
	corners[1] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[3] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);

	corners[4] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z);
	corners[5] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[6] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[7] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);

	Bounds bounds;
	for (int i = 0; i < 8; i++)
	{
		corners[i] = orient.RotatePoint(corners[i]) + pos;
		bounds.Expand(corners[i]);
	}

	return bounds;
}

/*
====================================================
ShapeConvex::FastestLinearSpeed
====================================================
*/
float ShapeConvex::FastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxSpeed = 0.0f;

	for (int i = 0; i < m_points.size(); i++)
	{
		Vec3 r = m_points[i] - m_centerOfMass;
		Vec3 linearVelocity = angularVelocity.Cross(r);
		float speed = dir.Dot(linearVelocity);
		if (speed > maxSpeed)
		{
			maxSpeed = speed;
		}
	}

	return maxSpeed;
}

/*
====================================================
FindPointFurthestInDir
====================================================
*/
int FindPointFurthestInDir(const Vec3* pts, const int num, const Vec3& dir)
{
	int maxIdx = 0;
	float maxDist = dir.Dot(pts[0]);
	for (int i = 1; i < num; i++)
	{
		float dist = dir.Dot(pts[i]);
		if (dist > maxDist)
		{
			maxDist = dist;
			maxIdx = i;
		}
	}

	return maxIdx;
}

/*
====================================================
DistanceFromLine
====================================================
*/
float DistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pt)
{
	Vec3 ab = b - a;
	ab.Normalize();

	Vec3 ray = pt - a;
	Vec3 projection = ab * ray.Dot(ab); // project the ray onto ab
	Vec3 perpendicular = ray - projection;
	return perpendicular.GetMagnitude();
}

/*
====================================================
FindPointFurthestFromLine
====================================================
*/
Vec3 FindPointFurthestFromLine(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB)
{
	int maxIdx = 0;
	float maxDist = DistanceFromLine(ptA, ptB, pts[0]);
	for (int i = 1; i < num; i++)
	{
		float dist = DistanceFromLine(ptA, ptB, pts[i]);
		if (dist > maxDist)
		{
			maxDist = dist;
			maxIdx = i;
		}
	}
	return pts[maxIdx];
}

/*
====================================================
DistanceFromTriangle
====================================================
*/
float DistanceFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt)
{
	Vec3 ab = b - a;
	Vec3 ac = c - a;
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	Vec3 ray = pt - a;
	float dist = ray.Dot(normal);
	return dist;
}

/*
====================================================
FindPointFurthestFromTriangle
====================================================
*/
Vec3 FindPointFurthestFromTriangle(const Vec3* pts, const int num, const Vec3& ptA, const Vec3& ptB, const Vec3& ptC)
{
	int maxIdx = 0;
	float maxDist = DistanceFromTriangle(ptA, ptB, ptC, pts[0]);
	for (int i = 1; i < num; i++)
	{
		float dist = DistanceFromTriangle(ptA, ptB, ptC, pts[i]);
		if (dist * dist > maxDist * maxDist)
		{
			maxDist = dist;
			maxIdx = i;
		}
	}
	return pts[maxIdx];
}

/*
====================================================
BuildTetrahedron
====================================================
*/
void BuildTetrahedron(const Vec3* verts, const int num, std::vector<Vec3>& hullPts, std::vector<tri_t>& hullTri)
{
	hullPts.clear();
	hullTri.clear();

	Vec3 points[4];

	int idX = FindPointFurthestInDir(verts, num, Vec3(1, 0, 0));
	points[0] = verts[idX];
	idX = FindPointFurthestInDir(verts, num, points[0] * -1.0f);
	points[1] = verts[idX];
	points[2] = FindPointFurthestFromLine(verts, num, points[0], points[1]);
	points[3] = FindPointFurthestFromTriangle(verts, num, points[0], points[1], points[2]);

	// This is important for making sure ordering is CCW for all faces
	float dist = DistanceFromTriangle(points[0], points[1], points[2], points[3]);
	if (dist > 0.0f)
	{
		std::swap(points[0], points[1]);
	}

	// Build the tetrahedron
	hullPts.push_back(points[0]);
	hullPts.push_back(points[1]);
	hullPts.push_back(points[2]);
	hullPts.push_back(points[3]);

	tri_t tri;
	tri.a = 0;
	tri.b = 1;
	tri.c = 2;
	hullTri.push_back(tri);


	tri.a = 0;
	tri.b = 2;
	tri.c = 3;
	hullTri.push_back(tri);


	tri.a = 2;
	tri.b = 1;
	tri.c = 3;
	hullTri.push_back(tri);


	tri.a = 1;
	tri.b = 0;
	tri.c = 3;
	hullTri.push_back(tri);
}

/*
====================================================
ExpandConvexHulls
====================================================
*/
void ExpandConvexHulls(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri, const std::vector<Vec3>& verts)
{
	std::vector<Vec3> externalsVerts = verts;
	RemoveInternalPoints(hullPoints, hullTri, externalsVerts);

	while (externalsVerts.size() > 0)
	{
		int ptIdx = FindPointFurthestInDir(externalsVerts.data(), (int) externalsVerts.size(), externalsVerts[0]);

		Vec3 pt = externalsVerts[ptIdx];

		// remove this element
		externalsVerts.erase(externalsVerts.begin() + ptIdx);

		AddPoint(hullPoints, hullTri, pt);

		RemoveInternalPoints(hullPoints, hullTri, externalsVerts);
	}
	RemoveUnreferencedVerts(hullPoints, hullTri);
}

/*
====================================================
RemoveInternalPoints
====================================================
*/
void RemoveInternalPoints(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri, std::vector<Vec3>& checkPts)
{
	for (int i = 0; i < checkPts.size(); i++)
	{
		const Vec3& pt = checkPts[i];

		bool isExternal = false;
		for (int t = 0; t < hullTri.size(); t++)
		{
			const tri_t& tri = hullTri[t];
			const Vec3& a = hullPoints[tri.a];
			const Vec3& b = hullPoints[tri.b];
			const Vec3& c = hullPoints[tri.c];

			// If the point is in front of any triangle then it's external
			float dist = DistanceFromTriangle(a, b, c, pt);
			if (dist > 0.0f)
			{
				isExternal = true;
				break;
			}
		}

		// If it's not external, ther it's inside the plyhedron and should be removed
		if (!isExternal)
		{
			checkPts.erase(checkPts.begin() + i);
			i--;
		}
	}

	// Also remove any points that are just a little too close to the hull points
	for (int i = 0; i < checkPts.size(); i++)
	{
		const Vec3& pt = checkPts[i];

		bool isTooClose = false;
		for (int j = 0; j < hullPoints.size(); j++)
		{
			Vec3 hullPt = hullPoints[j];
			Vec3 ray = hullPt - pt;
			if (ray.GetLengthSqr() < 0.01f * 0.01f)
			{
				isTooClose = true;
				break;
			}
		}

		if (isTooClose)
		{
			checkPts.erase(checkPts.begin() + i);
			i--;
		}
	}
}

/*
====================================================
IsEdgeUnique
This will compare the incoming edges will all the edges in the facing tris and then 
	return true if it's unique
====================================================
*/
bool IsEdgeUnique(std::vector<tri_t>& tris, std::vector<int>& facingTris, const int ignoreTri, const edge_t& edge)
{
	for (int i = 0; i < facingTris.size(); i++)
	{
		const int triIdx = facingTris[i];
		if (ignoreTri == triIdx)
		{
			continue;
		}

		const tri_t& tri = tris[triIdx];

		edge_t edges[3];
		edges[0].a = tri.a;
		edges[0].b = tri.b;


		edges[1].a = tri.b;
		edges[1].b = tri.c;


		edges[2].a = tri.c;
		edges[2].b = tri.a;

		for (int e = 0; e < 3; e++)
		{
			if (edge == edges[e])
			{
				return false;
			}
		}
	}
	return true;
}

/*
====================================================
AddPoint
====================================================
*/
void AddPoint(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri, const Vec3& pt)
{
	// This point is outside
	// Now we need to remove old triangles and build new ones

	// Find all triangles that face this point
	std::vector<int> facingTris;
	for (int i = (int)hullTri.size() - 1; i >= 0; i--)
	{
		const tri_t& tri = hullTri[i];

		const Vec3& a = hullPoints[tri.a];
		const Vec3& b = hullPoints[tri.b];
		const Vec3& c = hullPoints[tri.c];

		const float dist = DistanceFromTriangle(a, b, c, pt);
		if (dist > 0.0f)
		{
			facingTris.push_back(i);
		}
	}

	// Now find all edges that are unique to the tris, these will be the edges that form the new triangles
	std::vector<edge_t> uniquesEdges;
	for (int i = 0; i < facingTris.size(); i++)
	{
		const int triIdx = facingTris[i];
		const tri_t& tri = hullTri[triIdx];

		edge_t edges[3];
		edges[0].a = tri.a;
		edges[0].b = tri.b;


		edges[1].a = tri.b;
		edges[1].b = tri.c;


		edges[2].a = tri.c;
		edges[2].b = tri.a;

		for (int e = 0; e < 3; e++)
		{
			if (IsEdgeUnique(hullTri, facingTris, triIdx, edges[e]))
			{
				uniquesEdges.push_back(edges[e]);
			}
		}
	}

	// Now remove the old facing tris
	for (int i = 0; i < facingTris.size(); i++)
	{
		hullTri.erase(hullTri.begin() + facingTris[i]);
	}

	// Now add the new point
	hullPoints.push_back(pt);
	const int newPtIdx = (int)hullPoints.size() - 1;

	// Now add triangles for each unique edge
	for (int i = 0; i < uniquesEdges.size(); i++)
	{
		const edge_t& edge = uniquesEdges[i];

		tri_t tri;
		tri.a = edge.a;
		tri.b = edge.b;
		tri.c = newPtIdx;
		hullTri.push_back(tri);
	}
}

/*
====================================================
AddPoint
====================================================
*/
void RemoveUnreferencedVerts(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTri)
{
	for (int i = 0; i < hullPoints.size(); i++)
	{
		bool isUsed = false;
		for (int j = 0; j < hullTri.size(); j++)
		{
			const tri_t& tri = hullTri[j];
			
			if (tri.a == i || tri.b == i || tri.c == i)
			{
				isUsed = true;
				break;
			}
		}

		if (isUsed)
		{
			continue;
		}

		for (int j = 0; j < hullTri.size(); j++)
		{
			tri_t& tri = hullTri[j];
			if (tri.a > i)
			{
				tri.a--;
			}
			if (tri.b > i)
			{
				tri.b--;
			}
			if (tri.c > i)
			{
				tri.c--;
			}
		}

		hullPoints.erase(hullPoints.begin() + i);
		i--;
	}
}





/*
====================================================
IsExternal
====================================================
*/
bool IsExternal(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris, const Vec3& pt)
{
	bool isExternal = false;
	
	for (int t = 0; t < tris.size(); t++)
	{
		const tri_t& tri = tris[t];
		const Vec3& a = pts[tri.a];
		const Vec3& b = pts[tri.b];
		const Vec3& c = pts[tri.c];

		// If the point is in front of any triangles, then it's external
		float dist = DistanceFromTriangle(a, b, c, pt);
		if (dist > 0.0f)
		{
			isExternal = true;
			break;
		}
	}

	return isExternal;
}

/*
====================================================
CalculateCenterOfMass
====================================================
*/
Vec3 CalculateCenterOfMass(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris)
{
	const int numSamples = 100;

	Bounds bounds;
	bounds.Expand(pts.data(), pts.size());

	Vec3 cm(0.0f);
	const float dx = bounds.WidthX() / (float)numSamples;
	const float dy = bounds.WidthY() / (float)numSamples;
	const float dz = bounds.WidthZ() / (float)numSamples;

	int sampleCount = 0;
	//100*100*100 = 1 000 000 000 ... It's insane
	for (float x = bounds.mins.x; x < bounds.maxs.x; x += dx)
	{
		for (float y = bounds.mins.y; y < bounds.maxs.y; y += dy)
		{
			for (float z = bounds.mins.z; z < bounds.maxs.z; z += dz)
			{
				Vec3 pt(x, y, z);
				if (IsExternal(pts, tris, pt))
				{
					continue;
				}

				cm += pt;
				sampleCount++;
			}
		}
	}

	cm /= (float)sampleCount;
	return cm;
}

/*
====================================================
CalculateInertiaTensor
====================================================
*/
Mat3 CalculateInertiaTensor(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris, const Vec3& cm)
{
	const int numSamples = 100;

	Bounds bounds;
	bounds.Expand(pts.data(), (int)pts.size());

	Mat3 tensor;
	tensor.Zero();

	const float dx = bounds.WidthX() / (float)numSamples;
	const float dy = bounds.WidthY() / (float)numSamples;
	const float dz = bounds.WidthZ() / (float)numSamples;

	int sampleCount = 0;
	for (float x = bounds.mins.x; x < bounds.maxs.x; x += dx)
	{
		for (float y = bounds.mins.y; y < bounds.maxs.y; y += dy)
		{
			for (float z = bounds.mins.z; z < bounds.maxs.z; z += dz)
			{
				Vec3 pt(x, y, z);
				if (IsExternal(pts, tris, pt))
				{
					continue;
				}
				
				// Get the point relative to the center of mass
				pt -= cm;

				tensor.rows[0][0] += pt.y * pt.y + pt.z * pt.z;
				tensor.rows[1][1] += pt.z * pt.z + pt.x * pt.x;
				tensor.rows[2][2] += pt.x * pt.x + pt.y * pt.y;

				tensor.rows[0][1] += -1.0f * pt.x * pt.y;
				tensor.rows[0][2] += -1.0f * pt.x * pt.z;
				tensor.rows[1][2] += -1.0f * pt.y * pt.z;

				tensor.rows[1][0] += -1.0f * pt.x * pt.y;
				tensor.rows[2][0] += -1.0f * pt.x * pt.z;
				tensor.rows[2][1] += -1.0f * pt.y * pt.z;

				sampleCount++;
			}
		}
	}

	tensor *= 1.0f / (float)sampleCount;
	return tensor;
}




// Monte Carlo
/*
====================================================
CalculateCenterOfMassMonteCarlo
====================================================
*/
static Vec3 CalculateCenterOfMassMonteCarlo(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris)
{
	const int numSamples = 10000;

	Bounds bounds;
	bounds.Expand(pts.data(), (int)pts.size());

	Vec3 cm(0.0f);
	int sampleCount = 0;
	for (int i = 0; i < numSamples; i++)
	{
		Vec3 pt;
		pt.x = bounds.mins.x + Random::Get() * bounds.WidthX();
		pt.y = bounds.mins.y + Random::Get() * bounds.WidthY();
		pt.z = bounds.mins.z + Random::Get() * bounds.WidthZ();
		
		if (IsExternal(pts, tris, pt))
		{
			continue;
		}

		cm += pt;
		sampleCount++;
	}

	cm /= (float)sampleCount;
	return cm;
}

/*
====================================================
CalculateInertiaTensorMonteCarlo
====================================================
*/
Mat3 CalculateInertiaTensorMonteCarlo(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris, const Vec3& cm)
{
	const int numSamples = 10000;
	Bounds bounds;
	bounds.Expand(pts.data(), (int)pts.size());

	Mat3 tensor;
	tensor.Zero();

	int sampleCount = 0;
	for (int i = 0; i < numSamples; i++)
	{
		Vec3 pt;
		pt.x = bounds.mins.x + Random::Get() * bounds.WidthX();
		pt.y = bounds.mins.y + Random::Get() * bounds.WidthY();
		pt.z = bounds.mins.z + Random::Get() * bounds.WidthZ();

		if (IsExternal(pts, tris, pt))
		{
			continue;
		}

		// Get the point relative to the center of mass
		pt -=cm;

		tensor.rows[0][0] += pt.y * pt.y + pt.z * pt.z;
		tensor.rows[1][1] += pt.z * pt.z + pt.x * pt.x;
		tensor.rows[2][2] += pt.x * pt.x + pt.z * pt.z;
		
		tensor.rows[0][1] += -1.0f * pt.x * pt.y;
		tensor.rows[0][2] += -1.0f * pt.x * pt.z;
		tensor.rows[1][2] += -1.0f * pt.y * pt.z;

		tensor.rows[1][0] += -1.0f * pt.x * pt.y;
		tensor.rows[2][0] += -1.0f * pt.x * pt.z;
		tensor.rows[2][1] += -1.0f * pt.y * pt.z;

		sampleCount++;
	}

	tensor *= 1.0f / (float)sampleCount;
	return tensor;
}





// Exact inertia tensor for convex hulls
/*
====================================================
TetrahedronVolume
====================================================
*/
float TetrahedronVolume(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
	const Vec3 ad = d - a;
	const Vec3 bd = d - b;
	const Vec3 cd = d - c;
	float numerator = ad.Dot(bd.Cross(cd));
	float volume = numerator / 6.0f;

	return volume;
}

/*
====================================================
CalculateCenterOfMassTetrahedron
====================================================
*/
Vec3 CalculateCenterOfMassTetrahedron(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris)
{
	std::vector<Vec3> cms;
	std::vector<float> volumes;
	float totalVolume = 0.0f;
	cms.reserve(tris.size());
	volumes.reserve(tris.size());

	Vec3 centerish(0.0f);
	for(int i = 0; i < pts.size(); i++)
	{
		centerish += pts[i];
	}
	centerish *= 1.0f / float(pts.size());

	for (int i = 0; i < tris.size(); i++)
	{
		const tri_t& tri = tris[i];

		const Vec3& ptA = centerish;
		const Vec3& ptB = pts[tri.a];
		const Vec3& ptC = pts[tri.b];
		const Vec3& ptD = pts[tri.c];

		const Vec3 centerOfMassOfThisSimplex = (ptA + ptB + ptC + ptD) * 0.25f;
		const float volume = TetrahedronVolume(ptA, ptB, ptC, ptD);
		cms.push_back(centerOfMassOfThisSimplex);
		volumes.push_back(volume);
		totalVolume += volume;
	}

	Vec3 cm(0.0f);
	for (int i = 0; i < cms.size(); i++)
	{
		cm += cms[i] * volumes[i];
	}
	cm *= 1.0f / totalVolume;
	
	return cm;
}

/*
====================================================
InertiaTensorTetrahedron
====================================================
*/
Mat3 InertiaTensorTetrahedron(const Vec3& ptA, const Vec3& ptB, const Vec3& ptC, const Vec3& ptD)
{
	const Vec3 pts[4] = {ptA, ptB, ptC, ptD};

	Mat3 mat;
	mat.rows[0] = Vec3(pts[1].x - pts[0].x, pts[2].x - pts[0].x, pts[3].x - pts[0].x);
	mat.rows[1] = Vec3(pts[1].y - pts[0].y, pts[2].y - pts[0].y, pts[3].y - pts[0].y);
	mat.rows[2] = Vec3(pts[1].z - pts[0].z, pts[2].z - pts[0].z, pts[3].z - pts[0].z);
	const float DetJ = fabsf(mat.Determinant());

	const float density = 1.0f;
	const float mu = density;

	float xx = 0;
	float yy = 0;
	float zz = 0;

	float xy = 0;
	float xz = 0;
	float yz = 0;

	for (int i = 0; i < 4; i++)
	{
		for (int j = i; j < 4; j++)
		{
			// diagonales
			xx += pts[i].x * pts[j].x;
			yy += pts[i].y * pts[j].y;
			zz += pts[i].z * pts[j].z;

			// off diagonals
			xy += pts[i].x * pts[j].y + pts[j].x * pts[i].y;
			xz += pts[i].x * pts[j].z + pts[j].x * pts[i].z;
			yz += pts[i].y * pts[j].z + pts[j].y * pts[i].z;
		}
	}

	const float a = mu * DetJ * (yy + zz) / 60.0f;
	const float b = mu * DetJ * (xx + zz) / 60.0f;
	const float c = mu * DetJ * (xx + yy) / 60.0f;

	const float aprime = mu * DetJ * yz / 120.0f;
	const float bprime = mu * DetJ * xz / 120.0f;
	const float cprime = mu * DetJ * xy / 120.0f;

	Mat3 inertiaTensor;
	inertiaTensor.rows[0] = Vec3(a, -cprime, -bprime);
	inertiaTensor.rows[1] = Vec3(-cprime, b, -aprime);
	inertiaTensor.rows[2] = Vec3(-bprime, -aprime, c);

	return inertiaTensor;
}

/*
====================================================
CalculateInertiaTensorTetrahedron
====================================================
*/
Mat3 CalculateInertiaTensorTetrahedron(const std::vector<Vec3>& pts, const std::vector<tri_t>& tris, const Vec3& cm)
{
	Mat3 inertiaTensor;
	inertiaTensor.Zero();
	float totalVolume = 0.0f;
	for (int i = 0; i < tris.size(); i++)
	{
		const tri_t& tri = tris[i];

		const Vec3 ptA = cm - cm;
		const Vec3 ptB = pts[tri.a] - cm;
		const Vec3 ptC = pts[tri.b] - cm;
		const Vec3 ptD = pts[tri.c] - cm;

		Mat3 tensor = InertiaTensorTetrahedron(ptA, ptB, ptC, ptD);
		inertiaTensor += tensor;

		const float volume = TetrahedronVolume(ptA, ptB, ptC, ptD);
		totalVolume += volume;
	}
	
	inertiaTensor *= 1.0f / totalVolume;
	return inertiaTensor;
}