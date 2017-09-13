#pragma once

#include "RotationalMechanics.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

using glm::mat4;
using glm::vec3;

class MatrixRotationalMechanics : public RotationalMechanics
{
	mat4 orientation;			
	mat4 tensorOfInertia;	
	mat4 iTensorOfInertia;	
	vec3  angularMomentum;
	double width, height, depth;

public:
	MatrixRotationalMechanics();
	virtual ~MatrixRotationalMechanics();

	virtual void setAngularMomentum(const vec3& _angularMomentum);

	virtual void setInertia(vec3 diag);

	virtual void update(float dt);

	virtual const mat4& getOrientationMatrix() const;
};

