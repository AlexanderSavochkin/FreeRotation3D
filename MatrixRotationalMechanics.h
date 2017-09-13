#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

using glm::mat4;
using glm::vec3;
using glm::vec4;

class RotationalMechanics
{
	mat4 orientation;			
	mat4 tensorOfInertia;	
	mat4 iTensorOfInertia;	
	vec3  angularMomentum;
	double width, height, depth;

public:
	RotationalMechanics();
	virtual ~RotationalMechanics();

	void setAngularMomentum(const vec3& _angularMomentum);

	void setInertia(vec3 diag);

	void update(float dt);

	const mat4& getOrientation() const;
};

