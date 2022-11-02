# How to upgrade the firmware

## Download the firmware

Get the *windbird-firmware-x.x.x.bin* file of the latest release, at https://github.com/windbird-sensor/windbird-fox-firmware/releases

## Connect your Windbird

Use any 3.3V USB-to-UART cable (be careful not to use a 5V cable).

We recommend the *FTDI TTL-232R-RPI* (cheaper) or *FTDI TTL-232R-3V3*. You can also find very cheap chinese cables, they will work too.

![FTDI TTL-232R-RPI](https://user-images.githubusercontent.com/1681443/199475597-df15238b-3611-43d0-8b0c-2d804575dbf8.png)

Connect the cable to the board. If you use another brand than FTDI, the colors of the cables might be different.

![wb uart wiring](https://user-images.githubusercontent.com/1681443/199485309-e7c597e3-cab4-4dad-a36c-c2b6f6c09b66.png)

The battery must be plugged in. There is no need to remove the board from the Windbird's plastic body.

## Upgrading from Windows

Download and extract the TDLoader software : https://github.com/windbird-sensor/windbird-fox-firmware/raw/main/usb-flasher/windows/tdloader.zip

Open the *Device Manager* using the provided shortcut.

In the device manager, identify your cable and note the port number. In this example, port number is 4. 

![Capture d’écran du 2022-11-02 13-16-02](https://user-images.githubusercontent.com/1681443/199487441-032c4621-0b38-4532-a9c7-2051d5b8fb94.png)

If your cable doesn't show up, you might need to install the cable's driver. Driver for FTDI cables is included in tdloader.zip. For other brands, please refer to manufacturer's instructions.

Launch TDLoader software.

Enter the port number you previously noted.

Select *TD1808 EVB* as product.

Click *Browse* and select the firmware file.

![Capture d’écran du 2022-11-02 13-21-43](https://user-images.githubusercontent.com/1681443/199488406-7a34f5e2-d1e5-4f06-a38c-b73c3f50e2f5.png)

At this step, make sure that your Windbird is shut down and that the battery is connected.

Then press *Acquire* to launch the upgrade process.

When you read *Synchronizing* on the screen , turn on the Windbird by pressing it's power button.

At this step, the upgrade process will start. Do not touch or close or disconnect anything during the upgrade.

![image1059](https://user-images.githubusercontent.com/1681443/199490009-47fe796f-a90e-4713-8298-b9a75489bc0b.png)

Once you read *Upgrade OK*, the Windbird is ready and will reboot. You can now disconnect everything and enjoy the new software!
