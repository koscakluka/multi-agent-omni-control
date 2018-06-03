#include "astar_search/discrete_2D_rigid_body.h"

std::pair<int, int> applyTransform(float x, float y, float angle)
{
  float new_x = x * cos(angle) - y * sin(angle);
  float new_y = x * sin(angle) + y * cos(angle);
  return std::make_pair<int, int>(int(new_x), int(new_y));
}

Discrete2DRigidBody::Discrete2DRigidBody(float length, float precision)
{
  Discrete2DRigidBody(length, length, precision);
}

/*!
 * Constructs a Discrete2DRigidBody defined by it's length and height.
 * 
 * \param length Full size of rigid body in x direction.
 * \param height Full size of rigid body in y direction.
 */
Discrete2DRigidBody::Discrete2DRigidBody(float length, float height, float precision)
{
  int low_limit_x, high_limit_x, low_limit_y, high_limit_y;
  if(int(length) % 2 == 0)
  {
    low_limit_x = -int(length) / 2 + 1;
    high_limit_x = int(length) / 2;
  }
  else
  {
    low_limit_x = -int(length) / 2;
    high_limit_x = int(length) / 2;
  }

  if(int(height) % 2 == 0)
  {
    low_limit_y = -int(height) / 2 + 1;
    high_limit_y = int(height) / 2;
  }
  else
  {
    low_limit_y = -int(height) / 2;
    high_limit_y = int(height) / 2;
  }

  for(int i = low_limit_x; i <= high_limit_x; i++)
  {
    for(int j = low_limit_y; j <= high_limit_y; j++)
    {
      rigid_body_.push_back(std::pair<int, int>(i,j));
    }
  }
}

std::vector<std::pair<int, int> > Discrete2DRigidBody::getRigidBodyAtRadAngle(float angle)
{
  std::set<std::pair<int, int> > rotated_rigid_body;
  for(auto point:rigid_body_)
  {
    for(int i = -1; i <= 1; i++)
    {
      for(int j = -1; j <= 1; j++)
      {
        rotated_rigid_body.insert(applyTransform(
          point.first + i/2.0,
          point.second + j/2.0,
          angle
        ));
      }
    }
  }
  return std::vector<std::pair<int, int> >(
    rotated_rigid_body.begin(),
    rotated_rigid_body.end()
  );
}

std::vector<std::pair<int, int> > Discrete2DRigidBody::getRigidBodyAtAngle(float angle)
{
  return getRigidBodyAtRadAngle(angle * M_PI / 180.0);
}
