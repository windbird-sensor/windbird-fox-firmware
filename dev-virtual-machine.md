# Virtual machine for development

The setup of a complete development environment can be complex. To make things easier, we provide a ready-to-use [virtual machine](https://www.youtube.com/watch?v=yIVXjl4SwVo).

## Initial setup

### Prerequisites
- A reasonably powerful computer
- 8GB or more RAM
- Windows, Linux or Mac OS
- At least 30GB of free disk space
- For best performance, enable Hardware Virtualization in your BIOS. [Instructions here](https://bce.berkeley.edu/enabling-virtualization-in-your-pc-bios.html)

### Install VirtualBox

- Go to https://www.virtualbox.org/wiki/Downloads
- Install VirtualBox platform package
- After installing VirtualBox, install Oracle VM VirtualBox Extension Pack. This will provide support for USB devices. VirtualBox and the Extension pack must have the same version number.

### Download the virtual machine image

- https://openwindmap.s3.fr-par.scw.cloud/dev/windbird-dev-vm.zip (7.8 GB)
- Extract the zip file (extracted size : 18GB)

### Import the virtual machine into VirtualBox

- Launch VirtualBox
- In the top menu, select Machine->Add
- Select the .vbox file you have just extracted
- It might be necessary to adjust the virtual machine settings, depending on your computer capabilities: Base memory, Video memory, Processors count... (the sliders should be in the green area).

### Start the virtual machine

- Press the Start button
- If everything goes well, the virtual machine should boot and lead you to the Windows desktop
- You might want to adjust the keyboard layout (default: QWERTY). You can do this by clicking on "ENG" next to the taskbar clock.
- For a better experience, it is recommended to [reinstall Guest Additions](https://docs.oracle.com/cd/E36500_01/E36502/html/qs-guest-additions.html) within your VM. This will ensure that the guest Virtualbox drivers package matches your version of Virtualbox
- You can enable shared folders and shared clipboard between your host computer and the VM. That can be quite useful.

## What's in the box?

### Eclipse IDE

Preconfigurated workspace with TD SDK, Windbird firmware, build/flash/debug toolchain, code examples...

**TODO:** tutorial for using the IDE and contributing with Git

### TD SDK

### Documentation

### Git

Git utilities are preinstalled.

Necessary repositories are already cloned in `C:\TD\TD_RF_Module_SDK-v6.0.0\Github`.

Don't forget to pull the latest versions. Ex: Open Git Bash -> `cd windbird-firmware` -> `git pull`

### USB devices

Drivers for J-LINK debug interface and FTDI USB-Serial cable are preinstalled in the VM and should work out-of-the-box.

To get your USB devices into the VM, you must redirect them. On the top menu of the virtual machine screen, click on Device->USB->[YOUR DEVICE]. You can also add filters so your devices will connect automatically every time you plug them.

On linux hosts, it is necessary to add your user to `vboxusers` group : `sudo gpasswd -a $USER vboxusers`, then log-out and log-in again.

### Utilities

- Serial : Putty, Termite
- Flashing : JFlashLite, eACommander
