#include <ecs_core/ecs.hpp>

/* 
 *  Environment detection system
 *
 *  Class EnvDetection is an entry point of the system.
 *
 *
 */

namespace ecs
{

ECS::ECS(ros::NodeHandle &nodeHandle)
    : nh_(nodeHandle)
{
    read_parameters();
}

ECS::~ECS()
{
}

void ECS::read_parameters()
{
}

void ECS::run()
{
}

} // namespace ecs