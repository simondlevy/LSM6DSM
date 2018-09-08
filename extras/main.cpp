/* 
   main.pp: Generic main() for caling Arduino-style setup(), loop()

   Copyright (c) 2018 Simon D. Levy

   This file is part of MPU.

   MPU is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MPU is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>

extern void setup(), loop();

void error(const char * errmsg) 
{
    fprintf(stderr, "%s\n", errmsg);
    exit(1);
}


int main(int argc, char ** argv)
{
    setup();

    while (true) {
        loop();
    }
}
