#include "QuaternionDirectIntegrationRotationalMechanics.h"
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/quaternion.hpp>

QuaternionDirectIntegrationRotationalMechanics::QuaternionDirectIntegrationRotationalMechanics():
	orientation(1.0f, 0.0f, 0.0f, 0.0f), d_orientation_dt(0.0f, 0.02f, 0.5f, 0.01f), first_iteration(true)
{
	//Angular velocity in body frame
	quat orientation_bar = conjugate(orientation);
	quat q_qbarqdot_2 = 2.0f *	orientation_bar * d_orientation_dt;
	vec3 w_body_frame(q_qbarqdot_2.x, q_qbarqdot_2.y, q_qbarqdot_2.z);

	//Angular momentum in body frame
	vec3 angular_momentum_body_frame(
		w_body_frame.x * tensorOfInertia.x,
		w_body_frame.y * tensorOfInertia.y,
		w_body_frame.z * tensorOfInertia.z);

	//Angular momentum in world frame
	angularMomentum = orientation * angular_momentum_body_frame * orientation_bar;

	//energy
	energy = 0.5f * dot(w_body_frame, angular_momentum_body_frame);
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
	if (first_iteration)
	{
		prev_orientation = orientation;
		orientation = orientation + d_orientation_dt * dt;
		d_orientation_dt = d_orientation_dt + d2_Orientation_d2t(orientation, d_orientation_dt) * dt;
		first_iteration = false;
	}
	else
	{
		quat predicted_orientation = 2.0f * orientation
			+ (-1.0f) * prev_orientation;

		d_orientation_dt = 0.5f * (predicted_orientation + (-1.0f) * prev_orientation) / dt;

		quat next_orientation = 2.0f * orientation
			+ (-1.0f) * prev_orientation
			+ 0.5f * d2_Orientation_d2t(orientation, d_orientation_dt) * dt * dt;

		prev_orientation = orientation;
		orientation = next_orientation;
	}
	orientation = normalize(orientation);
	orientation_matrix = glm::toMat4(orientation);
}

const mat4& QuaternionDirectIntegrationRotationalMechanics::getOrientationMatrix() const
{
	return orientation_matrix;
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