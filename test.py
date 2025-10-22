import usb.core, usb.util, struct, time

dev = usb.core.find(idVendor=0x046d, idProduct=0xc216)
dev.set_configuration()

cfg = dev.get_active_configuration()
# for intf in cfg:
#     print(f"Interface {intf.bInterfaceNumber}:")
#     for ep in intf:
#         direction = "IN" if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN else "OUT"
#         print(f"  Endpoint {hex(ep.bEndpointAddress)} dir={direction} packet={ep.wMaxPacketSize}")

intf = cfg[(0,0)]
ep = [e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) ==
      usb.util.ENDPOINT_IN][0]

# detach kernel driver if needed
try:
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
except Exception as e:
    print(e)
    exit(1)

while True:
    print("Reading one packet...")
    data = dev.read(ep.bEndpointAddress, ep.wMaxPacketSize, timeout=5000)
    print("len =", len(data))
    print(list(data))
