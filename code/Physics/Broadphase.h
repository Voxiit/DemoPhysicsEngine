//
//	Broadphase.h
//
#pragma once
#include "Body.h"
#include <vector>


struct collisionPair_t {
	int a;
	int b;

	bool operator == ( const collisionPair_t & rhs ) const {
		return ( ( ( a == rhs.a ) && ( b == rhs.b ) ) || ( ( a == rhs.b ) && ( b == rhs.a ) ) );
	}
	bool operator != ( const collisionPair_t & rhs ) const {
		return !( *this == rhs );
	}
};

struct pseudoBody_t
{
	int id;
	float value;
	bool isMin;
};

int CompareSAP(const void* a, const void* b);
void SortBodiesBounds(const Body* bodies, const int num, pseudoBody_t* sortedArray, const float dt_sec);

void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const pseudoBody_t* sortedBodies, const int num);
void SweetAndPrune1D(const Body* bodies, const int num, std::vector<collisionPair_t>& finalPairs, const float dt_sec);
void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float dt_sec );