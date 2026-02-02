#!/bin/bash
docker build --pull --no-cache -t rtw_test -f tests/Dockerfile.test .
docker run --rm rtw_test
