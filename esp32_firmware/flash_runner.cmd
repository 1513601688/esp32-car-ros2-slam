@echo off
setlocal
chcp 65001 >nul

set "PROJECT=%~1"
set "PORT=%~2"
set "SDK_DEFAULTS=%~3"
set "IDF_EXPORT=%~4"
set "SDK_CONFIG=%~5"
set "BUILD_DIR=%~6"

echo __PHASE_ENV__
call "%IDF_EXPORT%"
if errorlevel 1 exit /b %errorlevel%

cd /d "%PROJECT%"
if errorlevel 1 exit /b %errorlevel%

set "SDKCONFIG_DEFAULTS=%SDK_DEFAULTS%"

echo __PHASE_CONFIG__
idf.py -B "%BUILD_DIR%" -D "SDKCONFIG=%SDK_CONFIG%" -D "SDKCONFIG_DEFAULTS=%SDK_DEFAULTS%" reconfigure
if errorlevel 1 exit /b %errorlevel%

echo __PHASE_BUILD__
idf.py -B "%BUILD_DIR%" -D "SDKCONFIG=%SDK_CONFIG%" -D "SDKCONFIG_DEFAULTS=%SDK_DEFAULTS%" build
if errorlevel 1 exit /b %errorlevel%

echo __PHASE_FLASH__
idf.py -B "%BUILD_DIR%" -D "SDKCONFIG=%SDK_CONFIG%" -D "SDKCONFIG_DEFAULTS=%SDK_DEFAULTS%" -p "%PORT%" flash
if errorlevel 1 exit /b %errorlevel%

echo __PHASE_DONE__
exit /b 0
