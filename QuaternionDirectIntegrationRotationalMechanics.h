#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

using glm::mat4;
using glm::vec3;
using glm::vec4;
using glm::quat;

class QuaternionDirectIntegrationRotationalMechanics : public RotationalMechanics
{
	quat orientation;
	quat d_orientation_dt;
			
	vec3 tensorOfInertia;	
	vec3 iTensorOfInertia;

	vec3  angularMomentum;

	double width, height, depth;

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

