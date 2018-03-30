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
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PORT 5000

#include "Eigen/Dense"

#include "main.h"
#include "fdcl_param.h"
#include "fdcl_vicon.h"
#include "fdcl_robot.h"
#include "misc_matrix_func.h"
#include "char_parse.h"

using Eigen::MatrixXd;

fdcl_param CFG;
fdcl_vicon VICON;
fdcl_robot ROBOT;

bool RUN_PROGRAM = true;

int server_fd, new_socket, valread;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);
char send_buffer[1024] = {0};
char *hello = "Hello from server";

int main(){

	CFG.open("../robot_SEH.cfg");
	VICON.load_config(CFG);

	// OPEN TCP

	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
	                                      &opt, sizeof(opt)))
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );
	
	if (bind(server_fd, (struct sockaddr *)&address, 
	                     sizeof(address))<0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	
	send(new_socket , hello , strlen(hello) , 0 );
	printf("Hello message sent\n");

	VICON.open();
	printf("VICON: thread initialized..\n\n");

	while(RUN_PROGRAM == true){
		VICON.loop();

       	printf("tx: %.3f  ty: %.3f  tz: %.3f ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r", ROBOT.x_v(0), ROBOT.x_v(1), ROBOT.x_v(2), ROBOT.q_v(0), ROBOT.q_v(1), ROBOT.q_v(2), ROBOT.q_v(3));
  	    
        clear_string(send_buffer);

        add_double_to_string(send_buffer, ROBOT.x_v(0), "tx", false);
        add_double_to_string(send_buffer, ROBOT.x_v(1), "ty", false);
        add_double_to_string(send_buffer, ROBOT.x_v(2), "tz", false);
        add_double_to_string(send_buffer, ROBOT.q_v(0), "ox", false);
        add_double_to_string(send_buffer, ROBOT.q_v(1), "oy", false);
        add_double_to_string(send_buffer, ROBOT.q_v(2), "oz", false);
        add_double_to_string(send_buffer, ROBOT.q_v(3), "ow", true);

		// Forcefully attaching socket to the port 8080
		
		if (listen(server_fd, 3) < 0)
		{
			perror("listen");
			exit(EXIT_FAILURE);
		}
		if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
		           (socklen_t*)&addrlen))<0)
		{
			perror("accept");
			exit(EXIT_FAILURE);
		}

	    send(new_socket, send_buffer, strlen(send_buffer), 0);

	}

	VICON.close();
	printf("VICON: thread closing\n");
}