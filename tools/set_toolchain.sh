#!/bin/bash
# Copyright (C) 2020 Coding Night Romania
# 
# This file is part of automatic-farm.
# 
# automatic-farm is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# automatic-farm is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with automatic-farm.  If not, see <http://www.gnu.org/licenses/>.


#setting the toolchain to PATH variable

BASEDIR=$(pwd -L)
TOOLCHAIN_NAME="avr8-gnu-toolchain-linux_x86_64"

if [ -d "$BASEDIR/$TOOLCHAIN_NAME" ]; then
    echo "Toolchain found, setting the PATH env variable"
    echo "GNU binaries dir: $BASEDIR/$TOOLCHAIN_NAME/bin"

    if [[ $PATH != *"$BASEDIR/$TOOLCHAIN_NAME/bin"* ]]; then
        echo "Setting PATH env variable: $PATH:$BASEDIR/$TOOLCHAIN_NAME/bin"
        echo export PATH="$PATH:$BASEDIR/$TOOLCHAIN_NAME/bin" >> ~/.bashrc
        echo "Make PATH permanent..."
        echo "$PATH:$BASEDIR/$TOOLCHAIN_NAME/bin" > /etc/environment
    else
        echo "Toolchain's path is already in PATH env variable"
    fi

else
    echo "Toolchain is not installed...Please visit https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers and download the AVR 8-bit Toolchain 3.6.2 - Linux 64-bit"
fi