@echo off
rem S1 dashboard launcher for Windows (test S1-U23). Double-click it to
rem start the broker and the dashboard: it runs s1_launch.py next to it
rem from the repository root. The window stays open only if the launcher
rem stops with an error, so the message can be read.
cd /d "%~dp0..\.."
py -3 -I -B "%~dp0s1_launch.py" %*
set "S1_LAUNCH_CODE=%ERRORLEVEL%"
if "%S1_LAUNCH_CODE%"=="9009" (
    echo The py launcher was not found: install Python 3 from python.org.
)
if not "%S1_LAUNCH_CODE%"=="0" (
    echo.
    echo s1_launch stopped with code %S1_LAUNCH_CODE%: read the message above.
    pause
)
exit /b %S1_LAUNCH_CODE%
