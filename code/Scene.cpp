//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"
#include <string>
#include <iostream>

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	for (int i = 0; i < m_constraints.size(); i++)
	{
		delete m_constraints[i];
	}
	m_constraints.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {


	//InitializationRandomSphere();
	//InitializationFallingRope();
	InitializationStackBoxes();
	
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) 
{
	std::vector<ConstraintPenetration> penetrationConstraints;

	// Gravity impulse
	for (int i = 0; i < m_bodies.size(); i++)
	{
		Body* body = &m_bodies[i];

		// Gravity need to be an impulse
		// I = dp, F = dp/dt => dp = F * dt => I = F *dt
		// F = mgs
		float mass = 1.0f / body->m_invMass;
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body->ApplyImpulseLinear(impulseGravity);
	}

	//
	// Broadphase
	//
	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

	//
	// Narrowphase (perfom actual collision detection)
	//
	int numContacts = 0;
	//const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = (contact_t*) alloca(sizeof(contact_t) * collisionPairs.size());
	// Check for collision woth other bodies
	for (int i = 0; i < collisionPairs.size(); i++)
	{
		const collisionPair_t& pair = collisionPairs[i];
		Body* bodyA = &m_bodies[pair.a];
		Body* bodyB = &m_bodies[pair.b];

		//Skip body pairs with infinite mass
		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
		{
			continue;
		}

		// Check for intersection
		contact_t contact;
		if (Intersect(bodyA, bodyB, dt_sec, contact))
		{
			if (0.0f == contact.timeOfImpact)
			{
				m_manifolds.AddContact(contact);
			}
			else
			{
				// Balistic contact
				contacts[numContacts] = contact;
				numContacts++;
			}
		}
	}

	// Sort the time from earliest to lastest
	if (numContacts > 1)
	{
		qsort( contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

	//
	// Solve Constraints
	//
	for (int i = 0; i < m_constraints.size(); i++)
	{
		m_constraints[i]->PreSolve(dt_sec);
	}
	m_manifolds.PreSolve(dt_sec);
	

	const int maxIters = 5;
	for (int iters = 0; iters < maxIters; iters++)
	{
		for (int i = 0; i < m_constraints.size(); i++)
		{
			m_constraints[i]->Solve();
		}
		m_manifolds.Solve();
	}
	
	for (int i = 0; i < m_constraints.size(); i++)
	{
		m_constraints[i]->PostSolve();
	}
	m_manifolds.PostSolve();
	

	//
	// Apply balistic impulses
	//
	float accumulateTime = 0.0f;
	for (int i = 0; i < numContacts; i++)
	{
		contact_t& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulateTime;

		// Position Update
		for (int j = 0; j < m_bodies.size(); j++)
		{
			m_bodies[j].Update(dt);
		}

		ResolveContact(contact);
		accumulateTime += dt;
	}

	// Update the position for the rest of this frame's time
	const float timeRemaining = dt_sec - accumulateTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < m_bodies.size(); i++)
		{
			m_bodies[i].Update(timeRemaining);
		}
	}
}




/*
====================================================
AddStandardSandBox
====================================================
*/
void AddStandardSandBox(std::vector<Body>& bodies)
{
	Body body;

	// box ground
	body.m_position = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeBox(g_boxGround, sizeof(g_boxGround) / sizeof(Vec3));
	body.m_name = "Ground";
	bodies.push_back(body);

	// box wall 0
 	body.m_position = Vec3(50, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	body.m_name = "Wall10 1";
	bodies.push_back(body);

	// box wall 0
	body.m_position = Vec3(-50, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	body.m_name = "Wall10 2";
	bodies.push_back(body);

	// box wall 1
	body.m_position = Vec3(0, 25, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	body.m_name = "Wall11 1";
	bodies.push_back(body);

	// box wall 1
	body.m_position = Vec3(0, -25, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	body.m_name = "Wall11 2";
	bodies.push_back(body);
}


void AddStandardSandBoxBis(std::vector<Body>& bodies)
{
	Body body;

	// box ground
	body.m_position = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeBox(g_boxGroundBis, sizeof(g_boxGroundBis) / sizeof(Vec3));
	body.m_name = "Ground";
	bodies.push_back(body);

	// box wall 0
	body.m_position = Vec3(10, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0Bis, sizeof(g_boxWall0Bis) / sizeof(Vec3));
	body.m_name = "Wall10 1";
	bodies.push_back(body);

	// box wall 0
	body.m_position = Vec3(-10, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0Bis, sizeof(g_boxWall0Bis) / sizeof(Vec3));
	body.m_name = "Wall10 2";
	bodies.push_back(body);

	// box wall 1
	body.m_position = Vec3(0, 10, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1Bis, sizeof(g_boxWall1Bis) / sizeof(Vec3));
	body.m_name = "Wall11 1";
	bodies.push_back(body);

	// box wall 1
	body.m_position = Vec3(0, -10, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1Bis, sizeof(g_boxWall1Bis) / sizeof(Vec3));
	body.m_name = "Wall11 2";
	bodies.push_back(body);
}


/*
====================================================
Differents type of Initialization
====================================================
*/
void Scene::InitializationRandomSphere()
{
	Body body;

	body.m_position = Vec3(10, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_linearVelocity = Vec3(-100, 0, 0);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeSphere(0.5f);
	body.m_name = "Sphere";
	m_bodies.push_back(body);

	body.m_position = Vec3(-10, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(100, 0, 0);
	body.m_angularVelocity = Vec3(0, 10, 0);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
	body.m_name = "Diamond";
	m_bodies.push_back(body);
}

void Scene::InitializationFallingRope()
{
	Body body;

	//
	// Build a chain for funsies
	//

	const int numJoints = 5;
	for (int i = 0; i < numJoints; i++)
	{
		if (i == 0)
		{
			body.m_position = Vec3(0.0f, 5.0f, (float)numJoints + 3.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
			body.m_invMass = 1.0f;
			body.m_elasticity = 1.0f;
			body.m_name = "MainCube";
			m_bodies.push_back(body);
		}
		else
		{
			body.m_invMass = 1.0f;
		}

		body.m_linearVelocity = Vec3(0, 0, 0);

		Body* bodyA = &m_bodies[m_bodies.size() - 1];
		const Vec3 jointWorldSpaceAnchor = bodyA->m_position;

		ConstraintDistance* joint = new ConstraintDistance();

		joint->m_bodyA = &m_bodies[m_bodies.size() - 1];
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		body.m_position = joint->m_bodyA->m_position + Vec3(1, 0, 0);
		body.m_orientation = Quat(0, 0, 0, 1);
		body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		body.m_name = "Cube";
		m_bodies.push_back(body);


		joint->m_bodyB = &m_bodies[m_bodies.size() - 1];
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		m_constraints.push_back(joint);
	}
}

void Scene::InitializationMerryGoRound()
{
	Body body;

	// box ground
	body.m_position = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity = Vec3(0, 0, 0);
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.0f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxGroundBis, sizeof(g_boxGroundBis) / sizeof(Vec3));
	body.m_name = "Ground";
	m_bodies.push_back(body);

	// Cube 1 
	body.m_position = Vec3(8, 0, 2);
	body.m_angularVelocity = Vec3(0, 1, 0);
	body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
	body.m_name = "Cube1";
	m_bodies.push_back(body);


	/*
	// Cube 2
	body.m_position = Vec3(-8, 0, 2);
	body.m_angularVelocity = Vec3(0, -1, 0);
	body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
	body.m_name = "Cube2";
	m_bodies.push_back(body);

	// Cube 3
	body.m_position = Vec3(0, 8, 2);
	body.m_angularVelocity = Vec3(-1, 0, 0);
	body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
	body.m_name = "Cube3";
	m_bodies.push_back(body);

	// Cube 4 
	body.m_position = Vec3(0, -8, 2);
	body.m_angularVelocity = Vec3(1, 0, 0);
	body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
	body.m_name = "Cube4";
	m_bodies.push_back(body);
	*/
}

void Scene::InitializationStackBoxes()
{
	Body body;

	int x = 0;
	int y = 0;
	const int stackHeights = 5;
	for (int z = 0; z < stackHeights; z++)
	{
		float offset = ((z & 1) == 0) ? 0.0f : 0.15f;
		float xx = (float)x + offset;
		float yy = (float)y + offset;
		float delta = 0.04f;
		float scaleHeight = 2.0f + delta;
		float deltaHeight = 1.0f + delta;
		body.m_position = Vec3((float)xx * scaleHeight, (float)yy * scaleHeight, deltaHeight + (float)z * scaleHeight);
		body.m_orientation = Quat(0, 0, 0, 1);
		body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
		body.m_invMass = 1.0f;
		body.m_elasticity = 0.5f;
		body.m_friction = 0.5f;
		body.m_name = "MainCube";
		m_bodies.push_back(body);
	}

	AddStandardSandBox(m_bodies);
}
