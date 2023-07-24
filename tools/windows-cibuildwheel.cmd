cmake -B build || EXIT /B !ERRORLEVEL!
cmake --build build --target python-package --config Release || EXIT /B !ERRORLEVEL!
