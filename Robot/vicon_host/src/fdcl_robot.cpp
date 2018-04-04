#include "fdcl_robot.h"

fdcl_robot::fdcl_robot()
{
	clock_gettime(CLOCK_REALTIME, &tspec_INIT);
}

void fdcl_robot::callback_VICON(Vector3 x_v, Vector4 q_v, Matrix3 R_vm)
{
	this->x_v=x_v;
	this->q_v=q_v;
	this->R_vm=R_vm;
}
