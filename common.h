/*
 * common.h
 *
 * CS456 A2 Common Header
 * Written by Andrew J Klamut (ajklamut)
 *
 * July 2012
 *
 * This file has common classes and declarations used by the program.
 */

#ifndef COMMON_H
#define COMMON_H

#define NBR_ROUTER 5
#define DEBUG false

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

class DebugStream : public std::ostream {
    class DebugBuf : public std::stringbuf {
        std::ofstream out_file;
        bool enabled;

      public:
        DebugBuf(std::string file_name) {
            if (!DEBUG && file_name.find("debug") != std::string::npos) {
                enabled = false;
            } else {
                enabled = true;
            }

            if (enabled) {
                out_file.open(file_name.c_str());
            }
        }
        ~DebugBuf() {
            if (enabled) {
                out_file.close();
            }
        }

        virtual int sync() {
            if (enabled) {
                out_file << str();
                str("");
                out_file.flush();
            }
            return 0;
        }
    };

    DebugBuf buffer;

  public:
    DebugStream(std::string class_name) : std::ostream(&buffer), buffer(class_name) {}
};

int asciiToInt(std::string);
std::string intToAscii(int);
char *intToChars(int, char *);
int charsToInt(char *buffer);

#endif
