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




#check if avrdude is installed
#out=$(command -V avrdude)

if [ "$1" == "WSL" ]; then
    avrdude_win32=/mnt/c/avrdude
    echo "Installing tools for WSL support"
    if [ -d "$avrdude_win32" ]; then
        echo "Directory already exists!"
    else
        mkdir /mnt/c/avrdude
    fi
    
    if [ -f "$avrdude_win32/avrdude.exe" ]; then
        echo "Avrdude already installed!"
    else
        cp ./avrdude/avrdude.exe $avrdude_win32/.
        #export this to PATH
        export PATH="$PATH":$avrdude_win32 >> ~/.bashrc
    fi

    ##install avr_flash
    if grep -q "avr_flash" ~/.bashrc; then
        echo "avr_flash is already installed!"
    else
        echo "Install avr_flash!"
        ##generate code
        echo 'avr_flash() {' >> ~/.bashrc
        ##copy the hex code to the win 32 location
        echo "cp \$2 $avrdude_win32/data.hex" >> ~/.bashrc
        echo '    avrdude.exe -c usbasp -p $1 -P usb -U flash:w:"C:\avrdude\data.hex":i' >> ~/.bashrc
        echo '}' >> ~/.bashrc
    fi

    ##install avr_eeprom
    if grep -q "avr_eeprom" ~/.bashrc; then
        echo "avr_eeprom is already installed!"
    else
        echo "Install avr_eeprom!"
        ##generate code
        echo 'avr_eeprom() {' >> ~/.bashrc
        echo "cp \$2 $avrdude_win32/eep.hex" >> ~/.bashrc
        echo '    avrdude.exe -c usbasp -p $1 -P usb -U eeprom:w:"C:\avrdude\eep.hex":i' >> ~/.bashrc
        echo '}' >> ~/.bashrc
    fi

    ##install avr_ota_push
    if grep -q "avr_ota_push" ~/.bashrc; then
        echo "avr_ota_push is already installed!"
    else
        echo "Install avr_ota_push!"
        if [ -f "$avrdude_win32/find_ftdi.cmd" ]; then
            echo "find_ftdi.cmd already exists"
        else
            cp ./win32/find_ftdi.cmd $avrdude_win32/.
        fi
        #check python3
        out=$(command -V python3)
        if [[ $out == *"python3 is"* ]]; then
            ##generate code
            flash_tool_path=$(pwd)/flash_tool/flash-tool.py
            echo 'avr_ota_push() {' >> ~/.bashrc
            echo 'cmd.exe -cmd /K "cd C:\avrdude & START /WAIT /B find_ftdi.cmd Serial0 /all > out.txt  & exit"' >> ~/.bashrc
            echo 'com_port=$(cat /mnt/c/avrdude/out.txt)' >> ~/.bashrc
            echo 'tty_port=/dev/ttyS${com_port:3:1}' >> ~/.bashrc
            echo 'if [ -z "$3" ]; then' >> ~/.bashrc
            echo "    python3 $flash_tool_path -T \$2 -H \$1 -P \$tty_port -B 1152000 " >> ~/.bashrc
            echo 'else' >> ~/.bashrc
            echo "    python3 $flash_tool_path -T \$2 -H \$1 -K \$3 -P /dev/\$usb_dev -B 1152000 " >> ~/.bashrc
            echo 'fi' >> ~/.bashrc
            echo '}' >> ~/.bashrc
        else
            echo "python3 not found, please run apt-get install python3!"
        fi

    fi


else
    if [[ $out == *"avrdude is"* ]]; then
        echo "Avrdude is installed!"

        ##install avr_flash
        if grep -q "avr_flash" ~/.bashrc; then
            echo "avr_flash is already installed!"
        else
            echo "Install avr_flash!"
            ##generate code
            echo 'avr_flash() {' >> ~/.bashrc
            echo '    avrdude -c usbasp -p $1 -P usb -U flash:w:$2' >> ~/.bashrc
            echo '}' >> ~/.bashrc
        fi

        ##install avr_eeprom
        if grep -q "avr_eeprom" ~/.bashrc; then
            echo "avr_eeprom is already installed!"
        else
            echo "Install avr_eeprom!"
            ##generate code
            echo 'avr_eeprom() {' >> ~/.bashrc
            echo '    avrdude -c usbasp -p $1 -P usb -U eeprom:w:$2' >> ~/.bashrc
            echo '}' >> ~/.bashrc
        fi

        ##install avr_ota_push
        if grep -q "avr_ota_push" ~/.bashrc; then
            echo "avr_ota_push is already installed!"
        else
            echo "Install avr_ota_push!"
            #check python3
            out=$(command -V python3)
            if [[ $out == *"python3 is"* ]]; then
                ##generate code
                flash_tool_path=$(pwd)/flash_tool/flash-tool.py
                echo 'avr_ota_push() {' >> ~/.bashrc
                echo "usb_dev=\$(dmesg | grep \"FTDI USB Serial Device converter now attached\" | grep -Eo 'ttyUSB[0-9]+')" >> ~/.bashrc
                echo 'if [ -z "$3" ]; then' >> ~/.bashrc
                echo "    python3 $flash_tool_path -T \$2 -H \$1 -P /dev/\$usb_dev -B 576000 " >> ~/.bashrc
                echo 'else' >> ~/.bashrc
                echo "    python3 $flash_tool_path -T \$2 -H \$1 -K \$3 -P /dev/\$usb_dev -B 576000 " >> ~/.bashrc
                echo 'fi' >> ~/.bashrc
                echo '}' >> ~/.bashrc
            else
                echo "python3 not found, please run apt-get install python3!"
            fi

        fi

        ##check if the functions are already installed
    else
        echo "Avrdude not found, please run apt-get install avrdude!"
    fi
fi