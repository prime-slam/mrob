@REM Copyright (c) 2022, Arthur Saliou, Anastasiia Kornilova
@REM Licensed under the Apache License, Version 2.0 (the "License");
@REM you may not use this file except in compliance with the License.
@REM You may obtain a copy of the License at
@REM     http://www.apache.org/licenses/LICENSE-2.0
@REM Unless required by applicable law or agreed to in writing, software
@REM distributed under the License is distributed on an "AS IS" BASIS,
@REM WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@REM See the License for the specific language governing permissions and
@REM limitations under the License.

SETLOCAL EnableDelayedExpansion

SET plat="x64" || EXIT /B !ERRORLEVEL!

:: Early check for build tools
cmake --version || EXIT /B !ERRORLEVEL!

:: Install build package
python -m pip install --user -q build || popd && EXIT /B !ERRORLEVEL!

:: Build wheel
cmake -B build -G "Visual Studio 16 2019" -A %plat% || EXIT /B !ERRORLEVEL!
cmake --build build --target build-wheel --config Release || EXIT /B !ERRORLEVEL!