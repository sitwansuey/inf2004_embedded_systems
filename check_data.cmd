@echo off
setlocal DisableDelayedExpansion
rem Double-click this file, or use --no-pause from a terminal/automation.
set "EDR_RESULT=1"

if not "%~2"=="" goto usage
if "%~1"=="" goto find_python
if /I "%~1"=="--no-pause" goto find_python
goto usage

:find_python
echo Checking the shared data representation...
echo.
py -3 -c "import sys; sys.exit(0 if sys.version_info.major == 3 else 1)" >nul 2>&1
if errorlevel 1 goto try_python
py -3 -B "%~dp0testing\data_representation\run.py" --report "%~dp0edr-check-report.json"
set "EDR_RESULT=%ERRORLEVEL%"
goto finish

:try_python
python -c "import sys; sys.exit(0 if sys.version_info.major == 3 else 1)" >nul 2>&1
if errorlevel 1 goto missing_python
python -B "%~dp0testing\data_representation\run.py" --report "%~dp0edr-check-report.json"
set "EDR_RESULT=%ERRORLEVEL%"
goto finish

:missing_python
echo ERROR: Python 3 was not found. Install Python 3 and add it to PATH.
goto finish

:usage
echo Usage: check_data.cmd [--no-pause]
set "EDR_RESULT=2"

:finish
echo.
if "%EDR_RESULT%"=="0" goto passed
echo CHECK FAILED. Read the error above before integrating your changes.
goto close

:passed
echo CHECK PASSED. Evidence saved to edr-check-report.json beside this script.

:close
if /I "%~1"=="--no-pause" goto done
echo Press any key to close this window.
pause >nul

:done
exit /b %EDR_RESULT%
