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
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

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

void error(const char *msg)
{
    perror(msg);
}

int main(){

	CFG.open("../robot_SEH.cfg");
	VICON.load_config(CFG);

	int sockfd, portno = 50002, n, client_ln;
	struct sockaddr_in serv_addr;
	struct hostent *server;

	char buffer[1000];
	char ip_address[] = "127.0.0.1";

    // printf("Enter the IP of the client: ");
    // scanf("%s", ip_address);
    // printf("Please enter the port number: ");
    // scanf("%d", &portno);

	VICON.open();
	printf("VICON: thread initialized..\n\n");

	while(RUN_PROGRAM == true){
		VICON.loop();

       	printf("tx: %.3f  ty: %.3f  tz: %.3f ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r", ROBOT.x_v(0), ROBOT.x_v(1), ROBOT.x_v(2), ROBOT.q_v(0), ROBOT.q_v(1), ROBOT.q_v(2), ROBOT.q_v(3));
  	    

  	    sockfd = socket(AF_INET, SOCK_STREAM, 0);

        if (sockfd < 0)
            error("ERROR opening socket");

        server = gethostbyname(ip_address);

        if (server == NULL) {
            fprintf(stderr,"ERROR, no such host\n");
            exit(0);
        }

        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;

        bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

        serv_addr.sin_port = htons(portno);

        if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
            error("ERROR connecting");

        bzero(buffer,1000);
        listen(sockfd, 5);

        clear_string(buffer);

        add_double_to_string(buffer, ROBOT.x_v(0), "tx", false);
        add_double_to_string(buffer, ROBOT.x_v(1), "ty", false);
        add_double_to_string(buffer, ROBOT.x_v(2), "tz", false);
        add_double_to_string(buffer, ROBOT.q_v(0), "ox", false);
        add_double_to_string(buffer, ROBOT.q_v(1), "oy", false);
        add_double_to_string(buffer, ROBOT.q_v(2), "oz", false);
        add_double_to_string(buffer, ROBOT.q_v(3), "ow", true);
        
        n = write(sockfd, buffer, 1000);
		if (n < 0){
	        error("ERROR writing to socket");

	  	    sockfd = socket(AF_INET, SOCK_STREAM, 0);

	        if (sockfd < 0)
	            error("ERROR opening socket");

	        server = gethostbyname(ip_address);

	        if (server == NULL) {
	            fprintf(stderr,"ERROR, no such host\n");
	            exit(0);
	        }

	        bzero((char *) &serv_addr, sizeof(serv_addr));
	        serv_addr.sin_family = AF_INET;

	        bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

	        serv_addr.sin_port = htons(portno);

	        if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
	            error("ERROR connecting");

	        bzero(buffer,1000);
	        listen(sockfd, 5);
		}

        close(sockfd);
        sleep(0.2);
	}

	VICON.close();
	printf("VICON: thread closing\n");
}