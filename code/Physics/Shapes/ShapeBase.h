//
//	ShapeBase.h
//
#pragma once
#include "../../Math/Vector.h"
#include "../../Math/Quat.h"
#include "../../Math/Matrix.h"
#include "../../Math/Bounds.h"
#include <vector>

/*
====================================================
Shape
====================================================
*/
class Shape {
public:
	virtual void Build(const Vec3 * pts, const int num){}

	virtual Vec3 Support(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const = 0;

	virtual Mat3 InertiaTensor() const = 0;

	virtual Bounds GetBounds( const Vec3 & pos, const Quat & orient ) const = 0;
	virtual Bounds GetBounds() const = 0;

	virtual Vec3 GetCenterOfMass() const { return m_centerOfMass; }

	virtual float FastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const { return 0.0f; }

	enum shapeType_t {
		SHAPE_SPHERE,
		SHAPE_BOX,
		SHAPE_CONVEX,
	};
	virtual shapeType_t GetType() const = 0;


protected:
	Vec3 m_centerOfMass;
};
