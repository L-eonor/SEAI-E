@rem #############################################################################
@rem # Copyright (c) 2004-2020 Universidade do Porto - Faculdade de Engenharia   #
@rem # Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                   #
@rem # All rights reserved.                                                      #
@rem # Rua Dr. Roberto Frias s/n, sala I203, 4200-465 Porto, Portugal            #
@rem #                                                                           #
@rem # This file is part of Neptus, Command and Control Framework.               #
@rem #                                                                           #
@rem # Commercial Licence Usage                                                  #
@rem # Licencees holding valid commercial Neptus licences may use this file      #
@rem # in accordance with the commercial licence agreement provided with the     #
@rem # Software or, alternatively, in accordance with the terms contained in a   #
@rem # written agreement between you and Universidade do Porto. For licensing    #
@rem # terms, conditions, and further information contact lsts@fe.up.pt.         #
@rem #                                                                           #
@rem # Modified European Union Public Licence - EUPL v.1.1 Usage                 #
@rem # Alternatively, this file may be used under the terms of the Modified EUPL,#
@rem # Version 1.1 only (the "Licence"), appearing in the file LICENCE.md        #
@rem # included in the packaging of this file. You may not use this  work        #
@rem # except in compliance with the Licence. Unless required by  applicable     #
@rem # law or agreed to in writing, software distributed under the Licence  is   #
@rem # distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF      #
@rem # ANY KIND, either express or implied. See the Licence for the specific     #
@rem # language governing permissions and limitations at                         #
@rem # https://github.com/LSTS/neptus/blob/develop/LICENSE.md                    #
@rem # and http://ec.europa.eu/idabc/eupl.html.                                  #
@rem #                                                                           #
@rem # For more information please see <http://lsts.fe.up.pt/neptus>.            #
@rem #############################################################################
@rem # Author: Paulo Dias, José Pinto                                            #
@rem #############################################################################

@if "%DEBUG%" == "" @echo off

@rem  neptus startup script for Windows

@rem Set local scope for the variables with windows NT shell
if "%OS%"=="Windows_NT" @setlocal

set DIRNAME=%~dp0
if "%DIRNAME%" == "" set DIRNAME=.
set APP_BASE_NAME=%~n0
set APP_HOME=%DIRNAME%

@rem Resolve any "." and ".." in APP_HOME to make it shorter.
for %%i in ("%APP_HOME%") do set APP_HOME=%%~fi

cd %APP_HOME%

@rem Add default JVM options here. You can also use JAVA_OPTS and NEPTUS_OPTS to pass JVM options to this script.
set DEFAULT_JVM_OPTS="-Xms10m" "-Xmx2g" "-XX:MaxMetaspaceSize=512m" "-XX:+UseG1GC" "-XX:+HeapDumpOnOutOfMemoryError" "-XX:HeapDumpPath=$APP_HOME/log/heap-dump.hprof" "-Xss256k"

set DEFAULT=pt.lsts.neptus.loader.NeptusMain

set CLASSPATH=%APP_HOME%;%APP_HOME%\bin\neptus.jar;%APP_HOME%\conf;%APP_HOME%\cp.jar;%APP_HOME%\plugins\*;%CLASSPATH%

set LIBRARYPATH=.;libJNI

if exist "%APP_HOME%\jre\bin" (
    set JAVA_BIN_FOLDER=%APP_HOME%\jre\bin\
) else (
    @rem Check JAVA_HOME
    set JAVA_HOME=%JAVA_HOME:"=%
    if exist "%JAVA_HOME%\bin" (
        set JAVA_BIN_FOLDER=%JAVA_HOME%\bin\
    ) else (
        set JAVA_BIN_FOLDER=
    )
)

set JAVA_EXE=%JAVA_BIN_FOLDER%java.exe

"%JAVA_EXE%" -version >NUL 2>&1
if not "%ERRORLEVEL%" == "0" (
  echo Java executable was not found in "%JAVA_EXE%"
  exit /b %ERRORLEVEL%
)

for /f "delims=" %%a in ('call "%JAVA_EXE%" -classpath "%APP_HOME%\bin\neptus.jar" pt.lsts.neptus.loader.helper.CheckJavaOSArch') do (@set JAVA_MACHINE_TYPE=%%a)

if %JAVA_MACHINE_TYPE%==windows-x86 (
  if %PROCESSOR_ARCHITECTURE%==x86 (
    if defined vtk.lib.dir (
      set VTKLIB=%vtk.lib.dir%
    ) else (
       set VTKLIB=%PROGRAMFILES%\VTK\bin
    )
  )
  else (
    if defined "vtk.lib.dir(x86)" (
      set VTKLIB=%vtk.lib.dir(x86)%
    ) else (
      if defined vtk.lib.dir (
        set VTKLIB=%vtk.lib.dir%
      ) else (
        set "VTKLIB=%PROGRAMFILES(x86)%\VTK\bin"
      )
    )
  )
) else (
  if defined vtk.lib.dir (
      set VTKLIB=%vtk.lib.dir%
  ) else (
    set VTKLIB=%PROGRAMFILES%\VTK\bin
  )
)

if %JAVA_MACHINE_TYPE%==windows-x86 (
  if %PROCESSOR_ARCHITECTURE%==x86 (
    if defined opencv.lib.dir (
      set OPENCVLIB=%opencv.lib.dir%
    ) else (
       set OPENCVLIB=%PROGRAMFILES%\opencv\bin
    )
  )
  else (
    if defined "opencv.lib.dir(x86)" (
      set OPENCVLIB=%opencv.lib.dir(x86)%
    ) else (
      if defined opencv.lib.dir (
        set OPENCVLIB=%opencv.lib.dir%
      ) else (
        set "OPENCVLIB=%PROGRAMFILES(x86)%\opencv\bin"
      )
    )
  )
) else (
  if defined opencv.lib.dir (
      set OPENCVLIB=%opencv.lib.dir%
  ) else (
    set OPENCVLIB=%PROGRAMFILES%\opencv\bin
  )
)

if %JAVA_MACHINE_TYPE%==windows-x86 (
  set LIBRARYPATH=%APP_HOME%\.;%APP_HOME%\libJNI/gdal/win/x86;%APP_HOME%\libJNI/win;%APP_HOME%\libJNI;/usr/lib/jni;%VTKLIB%;%OPENCVLIB%
) else (
  set LIBRARYPATH=%APP_HOME%\.;%APP_HOME%\libJNI/gdal/win/x64;%APP_HOME%\libJNI/win;%APP_HOME%\libJNI/europa/x64;%APP_HOME%\libJNI/x64;%APP_HOME%\libJNI/gdal/win/x86;%APP_HOME%\libJNI;/usr/lib/jni;%VTKLIB%;%OPENCVLIB%
)

set WORKSPACE=pt.lsts.neptus.loader.NeptusMain ws
set MRA=pt.lsts.neptus.loader.NeptusMain mra
set LAUVCONSOLE=pt.lsts.neptus.mc.lauvconsole.LAUVConsole
set WORLDMAPPANEL=pt.lsts.neptus.app.tiles.WorldMapPanel

if not "%1"=="ws" goto end2
	set DEFAULT=%WORKSPACE%
	shift
:end2
if not "%1"=="v3d" goto end3
	set DEFAULT=%VIEWER3D%
	shift
:end3
if not "%1"=="mra" goto end4
	set DEFAULT=%MRA%
	shift
:end4
if not "%1"=="la" goto end8
	set DEFAULT=%LAUVCONSOLE%
	shift
:end8
if not "%1"=="wm" goto end10
	set DEFAULT=%WORLDMAPPANEL%
	shift
:end10
if not "%1"=="run" goto end10
	set DEFAULT=
	shift
:end10


:endCheckApp

if not exist "%APP_HOME%\log\" mkdir "%APP_HOME%\log"

@rem @echo on

set CMD_LINE_ARGS=%*

set OLDPATH=%PATH%
set PATH=%LIBRARYPATH%;%PATH%
@rem Execute neptus
"%JAVA_EXE%" -Dj3d.rend=d3d -Dsun.java2d.d3d=true %DEFAULT_JVM_OPTS% %JAVA_OPTS% %NEPTUS_OPTS% -Djava.library.path="%LIBRARYPATH%" -classpath "%CLASSPATH%" %DEFAULT% %CMD_LINE_ARGS%
set PATH=%OLDPATH%
