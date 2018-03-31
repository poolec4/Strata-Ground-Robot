#ifndef _FDCL_VICON_H
#define _FDCL_VICON_H

#include <stdio.h>
#include <iostream>
#include <vrpn_Tracker.h>

#include "fdcl_param.h"
#include "fdcl_robot.h"
#include "misc_matrix_func.h"

extern fdcl_robot ROBOT;

class fdcl_vicon
{
public:
	fdcl_vicon();
	~fdcl_vicon();
	
	string object;
	static void callback(void* userdata, const vrpn_TRACKERCB tdata);

	void load_config(fdcl_param& ); 
	void open();
	void open(string object);
	void close();
	void loop();
private:
	vrpn_Tracker_Remote *tracker;
};

#endif
