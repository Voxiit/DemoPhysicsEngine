//
//  Broadphase.cpp
//
#include "Broadphase.h"

/*
====================================================
BroadPhase
====================================================
*/
int CompareSAP(const void* a, const void* b)
{
	const pseudoBody_t* ea = (const pseudoBody_t*)a;
	const pseudoBody_t* eb = (const pseudoBody_t*)b;

	if (ea->value < eb->value)
	{
		return -1;
	}
	return 1;
}

void SortBodiesBounds(const Body* bodies, const int num, pseudoBody_t* sortedArray, const float dt_sec)
{
	Vec3 axis = Vec3(1, 1, 1);
	axis.Normalize();

	for (int i = 0; i < num; i++)
	{
		const Body& body = bodies[i];
		Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

		// Expand the bounds by the linear velocity
		bounds.Expand(bounds.mins + body.m_linearVelocity * dt_sec);
		bounds.Expand(bounds.maxs + body.m_linearVelocity * dt_sec);

		const float elipson = 0.01f;
		bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * elipson);
		bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * elipson);

		sortedArray[i * 2 + 0].id = i;
		sortedArray[i * 2 + 0].value = axis.Dot(bounds.mins);
		sortedArray[i * 2 + 0].isMin = true;

		sortedArray[i * 2 + 1].id = i;
		sortedArray[i * 2 + 1].value = axis.Dot(bounds.maxs);
		sortedArray[i * 2 + 1].isMin = false;
	}

	qsort(sortedArray, num * 2, sizeof(pseudoBody_t), CompareSAP);
}

void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const pseudoBody_t* sortedBodies, const int num)
{
	collisionPairs.clear();

	// Now that the bodies are sorted, build the collision pairs
	for (int i = 0; i < num * 2; i++)
	{
		const pseudoBody_t& a = sortedBodies[i];
		if (!a.isMin)
		{
			continue;
		}

		collisionPair_t pair;
		pair.a = a.id;

		for (int j = i + 1; j < num * 2; j++)
		{
			const pseudoBody_t b = sortedBodies[j];
			// if we've hit the end of a element, then we're done creating a pair with a
			if (b.id == a.id)
			{
				break;
			}

			if (!b.isMin)
			{
				continue;
			}

			pair.b = b.id;
			collisionPairs.push_back(pair);
		}
	}
}

void SweetAndPrune1D(const Body* bodies, const int num, std::vector<collisionPair_t>& finalPairs, const float dt_sec)
{
	pseudoBody_t* sortedBodies = (pseudoBody_t*)alloca(sizeof(pseudoBody_t) * num * 2);

	SortBodiesBounds(bodies, num, sortedBodies, dt_sec);
	BuildPairs(finalPairs, sortedBodies, num);
}

void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float dt_sec ) {
	finalPairs.clear();

	SweetAndPrune1D(bodies, num, finalPairs, dt_sec);
}