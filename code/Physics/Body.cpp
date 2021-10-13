//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_linearVelocity(0.0f, 0.0f, 0.0f),
m_angularVelocity(0.0f, 0.0f, 0.0f),
m_invMass(0.0f),
m_elasticity(0.0f),
m_friction(0.0f),
m_shape( NULL ){
}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 CenterOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(CenterOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const
{
	Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrientat = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrientat.RotatePoint(tmp);
	return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& worldPt) const
{
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
	return worldSpace;
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	Mat3 orient = m_orientation.ToMat3();
	// We transform it from local space into world space
	invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}

void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (0.0f == m_invMass)
	{
		return;
	}



	// impulsePoint is the world space location of the application of the impulse
	// impulse is the world space direction and magnitude of the impulse

	ApplyImpulseLinear(impulse);

	Vec3 position = GetCenterOfMassWorldSpace(); // applying impulses must produce torques through the center of mass
	Vec3 r = impulsePoint - position;
	Vec3 dl = r.Cross(impulse); // this is in world space
	ApplyImpulseAngular(dl);
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (0.0f == m_invMass)
	{
		return;
	}

	//printf("APPLY IMPUSLE LINEAR: %f, %f, %f\n", impulse.x, impulse.y, impulse.z);

	// p = mv
	// dp = m dv = J
	// => dv = J / m
	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (0.0f == m_invMass)
	{
		return;
	}

	// L = I w = r x p
	// dL = I dw = r x J
	// => dw = I^-1 . (r x J)
	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float maxAngularSpeed = 30.0f; // in rad/s
	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
	{
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}

void Body::Update(const float dt_sec)
{
	// linear velocity
	m_position += m_linearVelocity * dt_sec;
	 
	// ok, we have an angular velocity around the center of mass, this need to be converted somehow
	// relative to the model position. This way we can properly update the orientation of the model
	Vec3 positionCm = GetCenterOfMassWorldSpace();
	Vec3 cmToPos = m_position - positionCm;

	// Total Torque is equal to external applied toques + internal torque (precession)
	// T = T_external + omega x I * omega
	// T_external = 0 because it was applied in the collision response function
	// T = Ia = w x I * w
	// a = I^-1 (W x I * w)
	Mat3 orientation = m_orientation.ToMat3();
	Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
	Vec3 alpha = inertiaTensor.Inverse() * (m_angularVelocity.Cross(inertiaTensor* m_angularVelocity));
	m_angularVelocity += alpha * dt_sec;

	// Update orientation
	Vec3 dAngle = m_angularVelocity * dt_sec;
	Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	m_orientation = dq * m_orientation;
	m_orientation.Normalize();

	// Now, get the new model position
	m_position = positionCm + dq.RotatePoint(cmToPos);

}
