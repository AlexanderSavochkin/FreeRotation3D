#include "RotationalMechanics.h"
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/quaternion.hpp>

QuaternionDirectIntegrationRotationalMechanics::QuaternionDirectIntegrationRotationalMechanics():
	orientation(1.0f, 0.0f, 0.0f, 0.0f), d_orientation_dt(0.0f, 0.02f, 0.5f, 0.01f)
{
}


QuaternionDirectIntegrationRotationalMechanics::~QuaternionDirectIntegrationRotationalMechanics()
{
}

void QuaternionDirectIntegrationRotationalMechanics::setAngularMomentum(const vec3& _angularMomentum)
{
	angularMomentum = _angularMomentum;
}

void QuaternionDirectIntegrationRotationalMechanics::update(float dt)
{
	orientation = orientation + d_orientation_dt * dt;
	d_orientation_dt = d_orientation_dt + d2_Orientation_d2t(orientation, d_orientation_dt) * dt;
	orientation = normalize(orientation);
}

const quat& QuaternionDirectIntegrationRotationalMechanics::getOrientation() const
{ 
	return orientation; 
}

const mat4& QuaternionDirectIntegrationRotationalMechanics::getOrientationMatrix() const
{
	return glm::toMat4(orientation);
}

void QuaternionDirectIntegrationRotationalMechanics::setInertia(vec3 diag)
{
	tensorOfInertia = diag;
	iTensorOfInertia = vec3(1.0f/diag.x, 1.0f / diag.y, 1.0f / diag.z);
}

quat QuaternionDirectIntegrationRotationalMechanics::d2_Orientation_d2t(const quat& orientation,
	const quat& d_orientation_dt)
{
	quat q_qbarqdot = conjugate(orientation) * d_orientation_dt;
	vec3 v_qbarqdot(q_qbarqdot.x, q_qbarqdot.y, q_qbarqdot.z);
	vec3 Iqbarqdot = tensorOfInertia * v_qbarqdot;
	vec3 iIqbarqdot = iTensorOfInertia * v_qbarqdot;
	vec3 dynamic_part = -4.0f * cross(iIqbarqdot, Iqbarqdot);

	quat dynamic_part_quat(0.0f, dynamic_part.x, dynamic_part.y, dynamic_part.z);

	return d_orientation_dt * q_qbarqdot
		+ 0.5f * orientation * dynamic_part_quat;
}