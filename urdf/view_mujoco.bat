@echo off
REM ===================================================================
REM  Open the wheel-leg robot in the MuJoCo viewer (Windows launcher).
REM
REM  Double-click this file, or run it from cmd / PowerShell.
REM  It forwards into WSL, where MuJoCo and WSLg's display live.
REM ===================================================================

setlocal
set DISTRO=Ubuntu-24.04
set URDIR=/mnt/d/Files/轮腿训练/newstart/urdf

echo.
echo  Launching the MuJoCo viewer...
echo  model: %URDIR%/car.xml
echo.
echo  Viewer controls:
echo    space        pause / resume
echo    backspace    reset to the "home" keyframe
echo    Tab          toggle the UI panels (motor sliders)
echo    left drag    orbit          right drag  pan       scroll  zoom
echo    Esc          quit
echo.

wsl -d %DISTRO% -e bash -lc "cd '%URDIR%' && python3 tools/view.py"

if errorlevel 1 (
  echo.
  echo  [ERROR] the viewer exited with an error.
  echo  Try running the check first:
  echo      wsl -d %DISTRO% -e bash -lc "cd '%URDIR%' ^&^& python3 tools/viewer_check.py"
  echo.
)

endlocal
pause
