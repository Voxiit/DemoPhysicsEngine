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

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	
	body.m_position = Vec3(10, 0, 3);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0,0);
	body.m_linearVelocity = Vec3(-100, 0,0);
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


	AddStandardSandBox(m_bodies);

	/*
	// Dynamic Bodies
	for (int x = 0; x < 6; x++)
	{
		for (int y = 0; y < 6; y++)
		{
			float radius = 0.5f;
			float xx = float(x - 1) * radius * 1.5f;
			float yy = float(y - 1) * radius * 1.5f;
			body.m_position = Vec3(xx, yy, 10);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}

	// Static "floor"
	for (int x = 0; x < 3; x++)
	{
		for (int y = 0; y < 3; y++)
		{
			float radius = 80.0f;
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_angularVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			body.m_name == "GroundSphere";
			m_bodies.push_back(body);
		}
	}
	*/

}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) 
{
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
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = (contact_t*) alloca(sizeof(contact_t) * maxContacts);
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

		contact_t contact;
		if (Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			numContacts++;

			std::string print = bodyA->m_name + " - " + bodyB->m_name;
			std::cout << print << std::endl;
		}
	}

	// Sort the time from earliest to lastest
	if (numContacts > 1)
	{
		qsort( contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

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
