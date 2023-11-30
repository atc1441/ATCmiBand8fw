********************************************************************************
* install_readme.txt
* 2023 Ambiq Micro Inc.
* All rights reserved.
********************************************************************************

08/18/2023

Once the installation is completed, IAR EWARM will support the Apollo4 Lite
(aka Apollo4L) devices, AMAP42KL-KBR and AMA4B2KL-KXR.

This installation assumes:
  a) A recent version of IAR EWARM is already installed on the target system.
  b) J-Link has been updated to support Apollo4 Lite.
     See README_AmbiqMicro_Apollo4L_update_jlink.txt for more information.
     Note: It is expected that Segger will soon release a version of J-Link
           that will natively support Apollo4 Lite.
           Once available, Ambiq recommends using that version or later
           rather than using this procedure to update an existing installation.
  c) Administrator rights have been addressed as in steps 1 and 2.

1. Administrator notes:
   When EWARM is installed at a standard Windows installation folder, such as
   C:\Program Files (x86)\, Adminstrator rights are usually required in order
   perform this update. If installed elsewhere, Administrator rights may not
   necessarily be required.

2. Start a Command Prompt.
    - Press Windows+R - opens the "Run" box.
    - If Administrator rights are NOT required,
      Type "cmd", Enter. Go to step 3.
    - If Administrator rights are required,
      Type "cmd", Ctrl+Shift+Enter and enter creditials to the system prompt.
    - Run isadmin.bat to determine whether you are logged in as Administrator
      or User (again, Admin may not necessarily be required depending on where
      your current installation location is).

3. Note the target IAR EWARM installation folder.  A typical installation
   might be located as follows (the location will vary).
   e.g. C:\Program Files (x86)\IAR Systems\Embedded Workbench 9.2\

4. From the command prompt, CD to the download directory (where you found
   this readme) and run:
   install_Ambiq2IAR.bat "<target name from step 3>"

   e.g.
   install_Ambiq2IAR.bat "D:\Program Files (x86)\IAR Systems\Embedded Workbench 9.2"
   (Quotation marks must surround the full pathname.)
   (Ending backslash is not necessary.)

5. This should complete the update to IAR EWARM with Apollo4 support.
   Type exit to close the Command Prompt.
