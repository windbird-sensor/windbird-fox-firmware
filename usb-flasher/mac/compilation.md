I attempted to compile the cflash program on mac. I got it to compile but not to have a successful upgrade.

Install gcc if needed `xcode-select --install`

`curl https://raw.githubusercontent.com/windbird-sensor/windbird-fox-firmware/main/usb-flasher/linux/cflash.c -o cflash.c`

To get it to compile, I have to patch cflash.c:
- line 34 : remove `#include <features.h>` (Not present on osx. Seems not to be needed.)
- line 109 and 113 : remove `460800` / `B460800`
- line 237 : remove `OLCUC` (option to map lowercase characters to uppercase on output)

Compile : `gcc cflash.c -o cflash`

See the available serial ports : `ls -1 /dev/cu.*`

Try to flash : `./cflash -d /dev/cu.usbserial-FTFMF2DI windbird-firmware-0.1.0-with-uart-debug.bin`

The syncronization works, but then the upgrade can't start. It fails with the following error : `Did not get local ACK (00)` (defined on [line 405](https://github.com/windbird-sensor/windbird-fox-firmware/blob/main/usb-flasher/linux/cflash.c#L405))
