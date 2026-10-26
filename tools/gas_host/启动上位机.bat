@echo off
setlocal
cd /d "%~dp0"
set "PY=%LocalAppData%\Programs\Python\Python312\pythonw.exe"
if not exist "%PY%" set "PY=pythonw.exe"
start "" "%PY%" "%~dp0gas_host_gui.pyw"
endlocal
