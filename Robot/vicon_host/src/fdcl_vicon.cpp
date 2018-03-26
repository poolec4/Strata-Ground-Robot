#include "fdcl_vicon.h"

fdcl_vicon::fdcl_vicon()
{
}

fdcl_vicon::~fdcl_vicon()
{
}

void fdcl_vicon::close()
{
	delete tracker;
}

void fdcl_vicon::open(string object)
{
	tracker = new vrpn_Tracker_Remote(object.c_str());
	tracker -> register_change_handler(NULL, fdcl_vicon::callback);
}

void fdcl_vicon::load_config(fdcl_param& cfg)
{
	cfg.read("VICON.object",object);
	cout << "VICON: object " << object << endl;
}

void fdcl_vicon::open()
{
	open(object);
}

void fdcl_vicon::loop()
{
	tracker->mainloop();
}

void fdcl_vicon::callback(void* userdata, const vrpn_TRACKERCB tdata)
{
	Vector3 x_v;
	Vector4 q_v;
	Matrix3 R_vm;

	x_v(0)=tdata.pos[0];
	x_v(1)=tdata.pos[1];
	x_v(2)=tdata.pos[2];

	q_v(0) = tdata.quat[0]; //ox
	q_v(1) = tdata.quat[1]; //oy
	q_v(2) = tdata.quat[2]; //oz
	q_v(3) = tdata.quat[3]; //ow

	R_vm(0,0) = 1-(2*(tdata.quat[1])*(tdata.quat[1]))-(2*(tdata.quat[2])*(tdata.quat[2]));
    R_vm(0,1) = (2*tdata.quat[0]*tdata.quat[1])-(2*tdata.quat[3]*tdata.quat[2]);
    R_vm(0,2) = (2*tdata.quat[0]*tdata.quat[2])+(2*tdata.quat[3]*tdata.quat[1]);
    R_vm(1,0) = (2*tdata.quat[0]*tdata.quat[1])+(2*tdata.quat[3]*tdata.quat[2]);
    R_vm(1,1) = 1-(2*(tdata.quat[0])*(tdata.quat[0]))-(2*(tdata.quat[2])*(tdata.quat[2]));
    R_vm(1,2) = (2*(tdata.quat[1])*(tdata.quat[2]))-(2*(tdata.quat[3])*(tdata.quat[0]));
    R_vm(2,0) = (2*tdata.quat[0]*tdata.quat[2])-(2*tdata.quat[3]*tdata.quat[1]);
    R_vm(2,1) = (2*tdata.quat[0]*tdata.quat[3])+(2*tdata.quat[2]*tdata.quat[1]);
    R_vm(2,2) = 1-(2*(tdata.quat[0])*(tdata.quat[0]))-(2*(tdata.quat[1])*(tdata.quat[1]));

	ROBOT.callback_VICON(x_v,q_v,R_vm);
}
