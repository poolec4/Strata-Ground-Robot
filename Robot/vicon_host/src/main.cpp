#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <iostream>
#include <fstream>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "Eigen/Dense"

#include "main.h"
#include "fdcl_param.h"
#include "fdcl_vicon.h"
#include "fdcl_robot.h"
#include "misc_matrix_func.h"

using Eigen::MatrixXd;

fdcl_param CFG;
fdcl_vicon VICON;
fdcl_robot ROBOT;

bool RUN_PROGRAM = true;

// TCP Variables
int port = 2200;
string server_ip_addr = "192.168.10.31";

int main(){

	CFG.open("../robot_SEH.cfg");
	VICON.load_config(CFG);

	VICON.open();
	printf("VICON: thread initialized..\n\n");

	while(RUN_PROGRAM == true){
		VICON.loop();

       	printf("tx: %.3f  ty: %.3f  tz: %.3f ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r", ROBOT.x_v(0), ROBOT.x_v(1), ROBOT.x_v(2), ROBOT.q_v(0), ROBOT.q_v(1), ROBOT.q_v(2), ROBOT.q_v(3));

	}

	VICON.close();
	printf("VICON: thread closing\n");
}