#include "MatrixRotationalMechanics.h"
#include <glm/gtx/matrix_operation.hpp>

using glm::vec4;

MatrixRotationalMechanics::MatrixRotationalMechanics()
{
}


MatrixRotationalMechanics::~MatrixRotationalMechanics()
{
}

void MatrixRotationalMechanics::setAngularMomentum(const vec3& _angularMomentum)
{
	angularMomentum = _angularMomentum;
}

void MatrixRotationalMechanics::update(float dt)
{
	vec3 angVelocity;
	angVelocity = vec3(iTensorOfInertia * vec4(angularMomentum, 0));

	float angle = glm::length(angVelocity) * dt;
	vec3 axis = glm::normalize(angVelocity);

	mat4 Q = glm::rotate(mat4(1.0f), angle, axis);
	mat4 iQ = glm::rotate(mat4(1.0f), -angle, axis);

	orientation = Q * orientation;
	iTensorOfInertia = Q * iTensorOfInertia * iQ;
	tensorOfInertia = iQ * tensorOfInertia * Q;
}

const mat4& MatrixRotationalMechanics::getOrientationMatrix() const
{ 
	return orientation; 
}

void MatrixRotationalMechanics::setInertia(vec3 diag)
{
	tensorOfInertia = glm::diagonal4x4( vec4(diag, 0.0f) );
	iTensorOfInertia = mat4(0.0f);
	for (int i = 0; i < 3; ++i)
		iTensorOfInertia[i][i] = 1.0f / tensorOfInertia[i][i];
}
