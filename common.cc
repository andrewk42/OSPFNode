/*
 * common.cc
 *
 * CS456 A2 Common Implementation
 * Written by Andrew J Klamut (ajklamut)
 *
 * July 2012
 *
 * This file has definitions for common functions used by the program.
 */

#include "common.h"
#include <netdb.h>

using namespace std;

int asciiToInt(string s) {
    int ret;
    stringstream ss(s);
    ss >> ret;
    return ret;
}

string intToAscii(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

char *intToChars(int n, char *buffer) {
    int num = htonl(n); // Perform any necessary endian conversion

    for (int i = 0; i < sizeof(int); i++) {
        buffer[i] = (num >> (sizeof(int)-i-1)*8) & 0xFF;
    }

    return buffer;
}

int charsToInt(char *buffer) {
    int ret = 0;

    for (int i = 0; i < sizeof(int); i++) {
        ret |= (buffer[i] & 0xFF) << ((sizeof(int)-i-1)*8);
    }

    return ntohl(ret);
}
