//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Shapes.h"
#include "Physics/Body.h"
#include "Physics/Constraints.h"
#include "Physics/Manifold.h"

/*
====================================================
Scene
====================================================
*/
class Scene {
public:
	Scene() { m_bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector< Body > m_bodies;
	std::vector< Constraint * >	m_constraints;
	ManifoldCollector m_manifolds;

	// Differents type of Initialization
	void InitializationRandomSphere();
	void InitializationFallingRope();
	void InitializationMerryGoRound();
	void InitializationStackBoxes();
	void InitializationRagdoll();
	void InitializationMotors();
	void InitializationMovers();
	void InitializationFinal();
};

void AddStandardSandBox(std::vector<Body>& bodies);
void AddStandardSandBoxBis(std::vector<Body>& bodies);

