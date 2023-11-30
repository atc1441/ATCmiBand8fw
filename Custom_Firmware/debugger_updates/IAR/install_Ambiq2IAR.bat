:: *****************************************************************************
:: install_Ambiq2IAR.bat
:: 2022 Ambiq Micro Inc.
:: All rights reserved.
:: *****************************************************************************
@Echo off

::
:: These files add Ambiq Apollo4 Plus support to IAR EWARM.
::
:: User must supply the version name of the IAR EWARM installation, which
:: can typically be found at: C:\Program Files (x86)\IAR Systems\
:: and is typically named something like: Embedded Workbench 9.2
::
:: Therefore, a typical command line will be something like:
::    install_Ambiq2IAR.bat "C:\Program Files (x86)\IAR Systems\Embedded Workbench 9.2\"
::
::
:: Administrator notes:
:: When EWARM is installed at a standard Windows installation folder, such as
:: C:\Program Files (x86)\, Adminstrator rights are usually required in order
:: perform this update. If installed elsewhere, Administrator rights may not
:: necessarily be required.
::

:FullPath
:: Full path specified in first argument, it must be fully specified and quoted.
if exist %1 goto fullpathGood
Echo.
Echo. ERROR: The fullpathname specified was not valid.
Echo.        Be sure to put quotes around the entire path.
Echo.        Path must include drive name and the entire path all the way out to 'IAR Systems\Embedded Workbench 9.x'
Echo.        A closing backslash is not required.
Echo.
Echo ERROR: IAR EWARM is not installed.
Echo Ambiq IAR install aborted!
goto Exit

:fullpathGood
set fullpathname=%1
goto ChkIAR

:ChkIAR
if exist %fullpathname% goto IAR_installed
Echo.
Echo. ERROR: Specified IAR install directory is incorrect.
Echo.
Echo ERROR: IAR EWARM is not installed.
Echo Ambiq IAR install aborted!
goto Exit


:IAR_installed
Echo IAR is installed.
if exist %fullpathname%\arm goto VersValid
goto Usage


:VersValid
Echo.
Echo Valid target: %fullpathname%\arm\
Echo.

::
:: Copy AmbiqMicro files to IAR installation.
::
Echo Copying files to target IAR installation directory:
xcopy /v /e /y *.* %fullpathname%\arm\*.*

:: Delete this batch file from the target.
if exist %fullpathname%\install_Ambiq2IAR.bat del %fullpathname%\arm\install_Ambiq2IAR.bat

:: Temporary - clean up (delete) other unneeded files
if exist %fullpathname%\arm\config\debugger\AmbiqMicro\AMAB42KP-KBR.*         del %fullpathname%\arm\config\debugger\AmbiqMicro\AMAB42KP-KBR.*
if exist %fullpathname%\arm\config\devices\AmbiqMicro\Apollo4p\AMAB42KP-KBR.* del %fullpathname%\arm\config\devices\AmbiqMicro\Apollo4p\AMAB42KP-KBR.*


Echo.
Echo Installation SUCCESS!!
Goto Exit


:Usage
Echo.
Echo. ERROR:
Echo.   Must provide the full pathname of the EWARM installation directory,
Echo.   which can typically be found at:  C:\Program Files (x86)\IAR Systems\
Echo.   It is typically named something like: "Embedded Workbench 9.2"
Echo.
Echo.   For the above example, a typical full pathname would look like:
Echo.   "C:\Program Files (x86)\IAR Systems\Embedded Workbench 9.2"
Echo.
Echo.   Installation method:
Echo.   Once the full pathname to EWARM has been determined (see above),
Echo.   specify the full path as in the following example:
Echo.
Echo.    install_Ambiq2IAR.bat "C:\Program Files (x86)\IAR Systems\Embedded Workbench 9.2"
Echo.
Echo Ambiq install aborted!
goto Exit


:Exit
Echo.
Echo Press any key to exit.
pause
set fullpathname=
