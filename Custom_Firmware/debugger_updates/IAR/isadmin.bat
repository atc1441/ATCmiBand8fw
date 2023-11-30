:: *****************************************************************************
:: isadmin.bat
:: 2022 Ambiq Micro Inc.
:: All rights reserved.
:: *****************************************************************************
@Echo off

::
:: Check whether credentialed as an Administrator or User.
::
:: After the next 3 lines, the variable runState will be set as one of:
::  Admin, System, or User.
::
set runState=user
whoami /groups | findstr /b /c:"Mandatory Label\High Mandatory Level"   > nul && set runState=Admin
whoami /groups | findstr /b /c:"Mandatory Label\System Mandatory Level" > nul && set runState=System
whoami /groups | findstr /b /c:"Mandatory Label\Medium Mandatory Level" > nul && set runState=User

if "%runState%"=="Admin"  goto ImAdmin
if "%runState%"=="System" goto ImSystem
if "%runState%"=="User"   goto ImUser

:ImAdmin
:ImSystem
:ImUser
Echo.
Echo The current account level is: %runState%
Echo.
Goto Exit


:Exit
Echo.
set runState=
