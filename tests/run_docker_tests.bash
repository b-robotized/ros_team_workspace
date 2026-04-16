#!/bin/bash
set -e

docker build --pull --no-cache -t rtw_test -f tests/Dockerfile.test .

echo "================================================================"
echo "Running RTW CLI tests..."
echo "================================================================"
docker run --rm rtw_test ./tests/run_rtw_cli_tests.bash

echo ""
echo "================================================================"
echo "Running package creation and bringup tests..."
echo "================================================================"
docker run --rm rtw_test ./tests/run_tests.bash
