@echo OFF

setlocal ENABLEDELAYEDEXPANSION
FOR /F "delims=" %%i IN ('date /t') DO SET today=%%i
ECHO %today%


ECHO %OS%
:: get OS and GPU
IF %OS%==Windows_NT (
    ECHO Windows_NT
    :: get gpu
    FOR /F "delims=" %%i IN ('wmic path win32_VideoController get name ^| findstr "NVIDIA"') DO set GPU=%%i
    echo GPU = !GPU!
)
@REM echo %GPU%
endlocal

set ARCH=AMD

:: If on a Jetson copy ARM versions
if %ARCH%==ARM (SET DOCKER_ARCH=Jetson) else (SET DOCKER_ARCH=Dev)

echo %DOCKER_ARCH%

copy .docker_templates/zed.%DOCKER_ARCH%.Dockerfile .devcontainer/zed.Dockerfile
copy .docker_templates/zed.%DOCKER_ARCH%.Dockerfile .devcontainer/zed.Dockerfile




:exit

