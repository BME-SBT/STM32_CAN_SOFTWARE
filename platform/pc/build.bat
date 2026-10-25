@echo off
rem Build the CAN simulator and tests.
rem   build.bat          configure (first time) and build
rem   build.bat test     build, then run all automatic tests
rem   build.bat run      build, then start the interactive console
rem   build.bat clean    delete the build folder
setlocal
cd /d "%~dp0"

if /i "%~1"=="clean" (
  if exist build rmdir /s /q build
  echo build folder removed.
  exit /b 0
)

where gcc >nul 2>nul || (echo ERROR: gcc not found. Install the toolchain, see "CAN simulator.md" section 2, then open a NEW terminal. & exit /b 1)
where cmake >nul 2>nul || (echo ERROR: cmake not found. See "CAN simulator.md" section 2. & exit /b 1)
where ninja >nul 2>nul || (echo ERROR: ninja not found. See "CAN simulator.md" section 2. & exit /b 1)

if not exist build\build.ninja (
  cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=gcc || exit /b 1
)
cmake --build build || exit /b 1

if /i "%~1"=="test" (
  ctest --test-dir build --output-on-failure || exit /b 1
)
if /i "%~1"=="run" (
  build\cansim.exe
)
exit /b 0
