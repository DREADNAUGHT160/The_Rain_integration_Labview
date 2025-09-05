@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM ============================================================
REM FINAL_HIWI_PROGRAMS launcher (conda)
REM Launches each script in its own CMD window, activating a conda env first.
REM Edit CONDA_ENV below to your environment name.
REM ============================================================

set "CONDA_ENV=hiwi_env"

set "BASE=%~dp0"
set "RAIN_GUI=%BASE%src\rain_gui_5.py"
set "FOG_GUI=%BASE%src\fog_system_5.py"
set "RAIN_SIM=%BASE%test\labview_rain_sim.py"
set "FOG_SIM=%BASE%test\labview_fog_sim.py"

REM ---- locate conda.bat robustly ----
set "CONDA_BAT="
for /f "delims=" %%A in ('where conda 2^>NUL') do (
  set "CONDA_EXE=%%~fA"
  goto :haveConda
)
:haveConda
if defined CONDA_EXE (
  for %%B in ("%CONDA_EXE%") do (
    set "CONDA_BAT=%%~dpB..\condabin\conda.bat"
  )
)
if not defined CONDA_BAT if defined CONDA_EXE (
  for %%B in ("%CONDA_EXE%") do (
    set "CONDA_BAT=%%~dpB..\Scripts\activate.bat"
  )
)

if exist "%CONDA_BAT%" (
  set "PRE=call "%CONDA_BAT%" activate %CONDA_ENV% ^& "
) else (
  echo WARNING: conda not found. Falling back to system Python.
  set "PRE="
)

start "Rain GUI" cmd /k %PRE% python "%RAIN_GUI%"
start "Fog  GUI" cmd /k %PRE% python "%FOG_GUI%"
start "Rain Sim" cmd /k %PRE% python "%RAIN_SIM%"
start "Fog  Sim" cmd /k %PRE% python "%FOG_SIM%"

endlocal
