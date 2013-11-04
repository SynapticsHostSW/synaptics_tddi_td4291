HOW TO INSTALL THE DRIVER
-------------------------

** Follow the instructions outlined on the TWiki site below to obtain the kernel
   source code.
   http://twiki.synaptics.com/bin/view/Main/PandaJellyBean

** Copy the files contained in the driver tarball under /kernel to the
   equivalent locations in the kernel source tree, replacing the existing files.

** "make distclean".

** "make panda_defconfig".

** "make -j4".

** Install the new kernel on your Android system.

** Reboot your Android system.


USAGE OF SYSFS INTERFACE
------------------------

** Use adb command "adb shell" to go into shell.

** Go to /sys/devices/omapdss/display0 directory.

** Square brackets below represent hexdecimal or decimal values.
   e.g. echo 2 > length
        echo 0x44 > read

** Issue command
   echo [command] > write

** Issue command with parameters
   echo [parameter1] > buffer
   echo [parameter2] > buffer
   ...
   echo [command] > write

** Issue command and read one byte of data
   echo [command] > read
   cat read

** Issue command and read multiple bytes of data
   echo [length] > length
   echo [command] > read
   cat read

** Write value to register
   echo [value] > buffer
   echo [register address] > write_reg

** Read from register
   echo [register address] > read_reg
   cat read_reg

** Reset device
   echo 1 > reset

** Stop video data transmission
   echo 0 > enable_display

** Start video data transmission
   echo 1 > enable_display

** Run initialization sequence for AUO
   echo auo > config

** Run new initialization sequence for AUO
   echo auo_new > config

** Run initialization sequence for YXT
   echo yxt > config

** Run new initialization sequence for YXT
   echo yxt_new > config

** Wait for [value] usecs
   echo [value] > uwait

** Set delay time between commands to [value] msecs
   echo [value] > delay_time_ms

** Set delay time after Sleep Out to [value] msecs
   echo [value] > sleep_out_ms

** Set delay time after Display On to [value] msecs
   echo [value] > display_on_ms

** Disable register access using generic read/write
   echo 0 > use_generic

** Enable register access using generic read/write
   echo 1 > use_generic


INITIALIZATION SEQUENCES
------------------------

Initialization sequence for AUO (echo auo > config)
   - Set register 0xb0 to 0x00 (DSI_CFG_7_0)
   - Set register 0xb3 to 0xf0 (DSI_CFG_31_24)
   - Set register 0x45 to 0x11 (TCH_SL_LSB)
   - Set register 0x55 to 0x00 (BLANK_REG)

Initization sequence for YXT (echo yxt > config)
   - Set register 0xb3 to 0x70 (DSI_CFG_31_24)
   - Set register 0x45 to 0x13 (TCH_SL_LSB)
