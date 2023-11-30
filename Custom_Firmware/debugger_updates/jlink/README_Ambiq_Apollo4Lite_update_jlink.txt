August 18, 2023

This brief installation procedure will update an existing Windows J-Link
installation with support for Ambiq Apollo4 Lite devices.

IMPORTANT - This procedure assumes an existing installation of J-Link v7.88b or
            later which natively supports Apollo4 and Apollo4 Plus devices.

Update Procedure:
-----------------
1. Exit from all usage of all J-Link tools.

2. Depending on the installation location, this step may require administrator
   privileges.

   Copy the entire contents of 'Devices/AmbiqMicro/' including subdirectories
   from this install directory to the Segger install directory:
   C:\Program Files (x86)\SEGGER\JLink_Vxxx\Devices\AmbiqMicro\

   NOTE for Linux: After copying the files, edit JLinkDevices.xml and substitute
                   .elf for .FLM and forward slashes for backslashes.

3. Start J-Flash Lite.
   You should be able to select the following device(s) and perform erase and
   programming functions.

   AMAP42KL-KBR:      For Apollo4 Lite
   AMA4B2KL-KXR:      For Apollo4 Blue Lite

4. You should now be able to run any J-Link debugger environment as usual.
