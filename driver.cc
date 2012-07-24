/*
 * driver.cc
 *
 * CS456 A2 Driver file
 * Written by Andrew J Klamut (ajklamut)
 *
 * July 2012
 *
 * This file has the main() function that drives the program.
 */

#include "common.h"
#include "ospfnode.h"

#include <iostream>
#include <cstdlib>
#include <string>

using namespace std;

void usage(char *argv[]) {
    cerr << "Usage: "
         << argv[0]
         << " <router_id> <nse_host> <nse_port> <router_port>"
         << endl;

    exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    // Verify there are the right amount of arguments
    if (argc != 5) {
        usage(argv);
    }

    // Parse the arguments with minimal error-checking
    int router_id, router_port, nse_port;
    string nse_address;

    router_id = asciiToInt(string(argv[1]));

    nse_address = string(argv[2]);

    nse_port = asciiToInt(string(argv[3]));

    router_port = asciiToInt(string(argv[4]));

    OSPFNode *router = new OSPFNode(router_id, nse_address, nse_port, router_port);
    router->service();
    delete router;
}
