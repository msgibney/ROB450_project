#!/usr/bin/python
#
# Simple class for the Logitech F310 Gamepad.
# Needs pyusb library.
#

import usb
import struct

USB_VENDOR = 0x046d
USB_PRODUCT = 0xc216
default_state = (0, 20, 0, 0, 0, 0, 123, 251, 128, 0, 128, 0, 128, 0, 0, 0, 0, 0, 0, 0)

class Gamepad(object):

    def __init__(self, serial=None):
        """ Initialize the gamepad.

        Args:
            serial: Serial number of the gamepad. If None, the first gamepad found is used.
        """

        self.dev = usb.core.find(idVendor=USB_VENDOR, idProduct=USB_PRODUCT)

        if self.dev is None:
            raise RuntimeError("F310 not found (is the switch set to D mode?)")

        # If a kernel driver is already attached, detach it
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except (usb.core.USBError, NotImplementedError):
            pass

        self.dev.set_configuration()
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 0)]

        # Find interrupt-IN endpoint automatically
        self.ep_in = [e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) ==
            usb.util.ENDPOINT_IN][0]
        if self.ep_in is None:
            raise RuntimeError("Could not find interrupt IN endpoint")

        self.changed = False
        self._state = default_state
        self._old_state = default_state
        self.is_initialized = True

    def _getState(self, timeout):
       try:
            data = self.dev.read(self.ep_in.bEndpointAddress,
                                 self.ep_in.wMaxPacketSize,
                                 timeout=timeout)
            return data
       except usb.core.USBError as e:
            return None
       except usb.core.USBTimeoutError as e:
           return None
    
    def _applyJoystickTransformations(self, state):
        if state < 138 and state > 120:
            return 0
        return (state - 128) / 128

    def read_gamepad(self, timeout=200):
        state = self._getState(timeout=timeout)
        self.changed = state is not None
        if self.changed:
            self._old_state = self._state
            self._state = state

    def get_analogR_x(self):
        return -1 * self._applyJoystickTransformations(self._state[2])

    def get_analogR_y(self):
        return self._applyJoystickTransformations(self._state[3])

    def get_analogL_x(self):
        return -1 * self._applyJoystickTransformations(self._state[0])

    def get_analogL_y(self):
        return self._applyJoystickTransformations(self._state[1])

    def changed(self):
        return self.changed

    # def __del__(self):
    #     #if not self._dev is None:
    #     if self.is_initialized:
    #         self.dev.releaseInterface()
    #         self.dev.reset()

# Unit test code
if __name__ == '__main__':
    pad = None

    pad = Gamepad()
    while True:
        pad.read_gamepad()
        if pad.changed:
            print(pad._state)
            #print("analog R: {0:3}|{1:3}  analog L: {2:3}|{3:3}".format(pad.get_analogR_x(),pad.get_analogR_y(),pad.get_analogL_x(),pad.get_analogL_y()))
            #pass
