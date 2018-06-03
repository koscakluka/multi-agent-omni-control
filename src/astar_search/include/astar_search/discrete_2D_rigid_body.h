#include <utility>
#include <vector>
#include <set>

#define _USE_MATH_DEFINES
#include <math.h>

class Discrete2DRigidBody
{
private:
    std::vector<std::pair<int, int> > rigid_body_;

public:
//  TODO: figure out how to return vector of coordinates relative to center of area
    Discrete2DRigidBody(float, float, float);
    Discrete2DRigidBody(float, float);

    std::vector<std::pair<int, int> > getRigidBodyAtRadAngle(float);
    std::vector<std::pair<int, int> > getRigidBodyAtAngle(float);

};
