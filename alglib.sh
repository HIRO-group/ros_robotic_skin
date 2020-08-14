#!/bin/bash

wget https://www.alglib.net/translator/re/alglib-3.16.0.cpp.gpl.tgz
tar -xzf alglib-3.16.0.cpp.gpl.tgz -C  src/
rm alglib-3.16.0.cpp.gpl.tgz
mv src/cpp/src src/alglib
rm -rf src/cpp



