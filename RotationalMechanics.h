#include <glm/glm.hpp>

using glm::mat4;
using glm::vec3;

class RotationalMechanics
{
public:
    virtual const mat4& getOrientationMatrix() const = 0;
    virtual void setAngularMomentum(const vec3& _angularMomentum) = 0;
    virtual void setInertia(vec3 diag) = 0;
    virtual void update(float dt) = 0;
}