#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>

#define SOCKET_PATH "/tmp/example_socket"
#define BUFF_SIZE 1024
#define MAX_CONNECTS 10

void report(const char* msg, int terminate) {
    perror(msg);
    if (terminate) exit(-1); // exit with failure status
}

int main() {
    int fd = socket(AF_UNIX, SOCK_STREAM, 0); // Create UNIX domain socket
    if (fd < 0) report("socket", 1); // terminate if socket creation fails

    struct sockaddr_un saddr;
    memset(&saddr, 0, sizeof(saddr)); // clear the structure
    saddr.sun_family = AF_UNIX; // address family
    strncpy(saddr.sun_path, SOCKET_PATH, sizeof(saddr.sun_path) - 1);

    unlink(SOCKET_PATH); // remove existing socket file if any
    if (bind(fd, (struct sockaddr*) &saddr, sizeof(saddr)) < 0)
        report("bind", 1); // terminate on failure

    if (listen(fd, MAX_CONNECTS) < 0)
        report("listen", 1); // terminate on failure

    fprintf(stderr, "Server is listening for local connections at %s...\n", SOCKET_PATH);

    while (1) {
        struct sockaddr_un caddr; // client address
        socklen_t len = sizeof(caddr); // address length

        int client_fd = accept(fd, (struct sockaddr*)&caddr, &len); // accept connection
        if (client_fd < 0) {
            report("accept", 0); // don't terminate on accept failure
            continue;
        }

        char buffer[BUFF_SIZE + 1];
        memset(buffer, '\0', sizeof(buffer));
        int count = read(client_fd, buffer, BUFF_SIZE);
        if (count > 0) {
            printf("Received: %s\n", buffer);
            write(client_fd, buffer, count); // echo received data
        }
        close(client_fd); // close client connection
    }

    // This point is never reached
    close(fd);
    unlink(SOCKET_PATH); // cleanup socket file
    return 0;
}
