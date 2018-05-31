#pragma once

#include "RotationalMechanics.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

/*
Simulation based on http://www.mare.ee/indrek/varphi/vardyn.pdf
*/

using glm::mat4;
using glm::vec3;
using glm::vec4;
using glm::quat;

class QuaternionDirectIntegrationRotationalMechanics : public RotationalMechanics
{
	quat orientation;
	quat prev_orientation;
	quat d_orientation_dt;
	
	vec3  angularMomentum;
	double energy;

	vec3 tensorOfInertia;	
	vec3 iTensorOfInertia;

	mat4 orientation_matrix;

	double width, height, depth;
	bool first_iteration;
public:
	QuaternionDirectIntegrationRotationalMechanics();
	virtual ~QuaternionDirectIntegrationRotationalMechanics();

	void setAngularMomentum(const vec3& _angularMomentum);

	void setInertia(vec3 diag);

	void update(float dt);

	const mat4& getOrientationMatrix() const;

	//const quat& getOrientation() const;
private:
	quat d2_Orientation_d2t(const quat& orientation, const quat& d_orientation_dt);
};

