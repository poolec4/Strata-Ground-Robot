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

    int sockfd, newsockfd, port_number = 50000, n, count = 0;
    socklen_t client_ln;
    char buffer[1000], buffer_[1000];
    struct sockaddr_in serv_addr, cli_addr;

    printf("\nTCP Server - starting\n");
    printf("Connecting to TCP on port %d\n", port_number);
    //Start TCP socket

    printf("Please connect client to start sending data.\n");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 1){
        printf("\nERROR: failed to open socket\n");
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port_number);
    serv_addr.sin_addr.s_addr = INADDR_ANY;

    int yes = 1;

    if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1){
        printf("\nERROR setsockopt");
    }

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) <0){
        printf("\nERROR: failed to bind to socket\n");
    }


	VICON.open();
	printf("VICON: thread initialized..\n\n");

	while(RUN_PROGRAM == true){
		VICON.loop();

       	printf("tx: %.3f  ty: %.3f  tz: %.3f ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r", ROBOT.x_v(0), ROBOT.x_v(1), ROBOT.x_v(2), ROBOT.q_v(0), ROBOT.q_v(1), ROBOT.q_v(2), ROBOT.q_v(3));
  	    
        clear_string(buffer);

        add_double_to_string(buffer, ROBOT.x_v(0), "tx", false);
        add_double_to_string(buffer, ROBOT.x_v(1), "ty", false);
        add_double_to_string(buffer, ROBOT.x_v(2), "tz", false);
        add_double_to_string(buffer, ROBOT.q_v(0), "ox", false);
        add_double_to_string(buffer, ROBOT.q_v(1), "oy", false);
        add_double_to_string(buffer, ROBOT.q_v(2), "oz", false);
        add_double_to_string(buffer, ROBOT.q_v(3), "ow", true);
    
        listen(sockfd, 5);
        client_ln = sizeof(cli_addr);
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &client_ln);

        if (newsockfd <0){
            printf("\nERROR: failed to accept socket\n");
        }

        n = write(newsockfd, buffer, strlen(buffer));
		if (n < 0){
	        error("ERROR writing to socket");
        }

        close(newsockfd);
        sleep(0.01);
	}

	VICON.close();
	printf("VICON: thread closing\n");
}
