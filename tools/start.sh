#!/bin/bash

# Exit immediately if command fails
set -e

# Generate Makefile project
cmake -D CMAKE_SKIP_BUILD_RPATH=ON \
      -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D Boost_USE_STATIC_LIBS=${BOOST_USE_STATIC_LIBS} \
       ${PROJECT_DIR}

# Perform building of project
cmake --build . --config ${BUILD_TYPE}
