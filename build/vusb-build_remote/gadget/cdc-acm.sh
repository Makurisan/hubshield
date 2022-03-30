mkdir /sys/kernel/config/usb_gadget/u1
cd /sys/kernel/config/usb_gadget/u1

echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

echo "0x4075" > idVendor
echo "0x0001" > idProduct

mkdir strings/0x409
echo "0123456789" > strings/0x409/serialnumber
echo "Virtual Hub" > strings/0x409/manufacturer
echo "Serial Gadget" > strings/0x409/product

mkdir configs/c.1
mkdir configs/c.1/strings/0x409
echo "acm" > configs/c.1/strings/0x409/configuration

mkdir functions/acm.usb0
ln -s functions/acm.usb0 configs/c.1
