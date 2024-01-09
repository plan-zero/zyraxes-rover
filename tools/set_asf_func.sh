#!/bin/bash

##install sam4l_flash
if grep -q "sam4l_flash" ~/.bashrc; then
    echo "sam4l_flash is already installed!"
else
    echo "Install sam4l_flash!"
    ##generate code
    echo 'sam4l_flash() {' >> ~/.bashrc
    echo 'openocd -f interface/cmsis-dap.cfg -f board/atmel_sam4l8_xplained_pro.cfg -c "program $1 verify reset exit"' >> ~/.bashrc
    echo '}' >> ~/.bashrc
fi