HOW TO INSTALL THE DRIVER
-------------------------

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

** Reset device
   echo 1 > reset

** Stop video data transmission
   echo 0 > enable_display

** Start video data transmission
   echo 1 > enable_display

** Wait for [value] usecs
   echo [value] > uwait

** Set delay time between commands to [value] msecs
   echo [value] > delay_time_ms

** Set delay time after Sleep Out to [value] msecs
   echo [value] > sleep_out_ms

** Set delay time after Display On to [value] msecs
   echo [value] > display_on_ms
