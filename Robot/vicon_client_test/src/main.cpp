// Client side C/C++ program to demonstrate Socket programming
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

int sockfd, newsockfd, port_number = 50002, n, count = 0;
socklen_t client_ln;
char buffer[1000], buffer_[1000];
struct sockaddr_in serv_addr, cli_addr;


int main(){

    printf("\nTCP Server - starting\n");
    printf("Connecting to TCP on port %d\n", port_number);
    //Start TCP socket

    printf("Please start sending Vicon data.\n");

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

    // listen(sockfd, 5);

    // client_ln = sizeof(cli_addr);
    // newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &client_ln);

    // if (newsockfd <0){
    //     printf("\nERROR: failed to accept socket\n");
    //             //exit(1);
    // }

    while(1){
        listen(sockfd, 5);
        client_ln = sizeof(cli_addr);
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &client_ln);

        if (newsockfd <0){
            printf("\nERROR: failed to accept socket\n");
        }

        bzero(buffer, 1000);
        listen(newsockfd, 5);
        n = read(newsockfd, buffer, 1000);

        if (n < 0){
            printf("\nERROR: failed to read from socket\n");
        }

        close(newsockfd);

        printf("%s\n",buffer);
        // x_v[0] = parse_string_to_double(buffer, "x_v0");
        // x_v[1] = parse_string_to_double(buffer, "x_v1");
        // x_v[2] = parse_string_to_double(buffer, "x_v2");

        // quat_vm[0] = parse_string_to_double(buffer, "quat_vm0");
        // quat_vm[1] = parse_string_to_double(buffer, "quat_vm1");
        // quat_vm[2] = parse_string_to_double(buffer, "quat_vm2");
        // quat_vm[3] = parse_string_to_double(buffer, "quat_vm3");
    }
}