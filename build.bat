@echo off
rem Builds Super Mario World Recomp (Release) with MSVC + Ninja.
rem Usage: build.bat [path\to\your\rom.sfc]
rem   Without an argument the ROM is expected at smw.sfc (or smw.smc) in this folder.
rem Requires Visual Studio 2022 or newer with "Desktop development with C++", and Python 3.
setlocal
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (echo Visual Studio not found. & exit /b 1)
set "VS="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS=%%i"
if not defined VS (echo Visual Studio with the C++ workload not found. & exit /b 1)
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>nul
set "CMAKE=cmake"
set "NINJA=ninja"
if exist "%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" set "CMAKE=%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if exist "%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe" set "NINJA=%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
cd /d "%~dp0"
set ROMARG=
if not "%~1"=="" set ROMARG="-DSMW_ROM=%~f1"
"%CMAKE%" -S . -B build -G Ninja "-DCMAKE_MAKE_PROGRAM=%NINJA%" -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=cl %ROMARG% || exit /b 1
"%CMAKE%" --build build || exit /b 1
echo.
echo Built: %~dp0build\smw.exe
