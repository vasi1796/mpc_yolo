#include "Server.h"


int main() {

	Server server;
	server.run();

	while (1) {
		printf("opa\n");
		Sleep(500);
	}
    return 0;
}