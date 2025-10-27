# ROB450_project

add these lines to /etc/udev/rules.d/99-usb.rules

SUBSYSTEM=="usb", ATTR{idVendor}=="046d", ATTR{idProduct}=="c216", MODE="0666", OPTIONS+="ignore_device"

then run 

sudo udevadm control --reload-rules && sudo udevadm trigger