#!/bin/bash
docker build -t rtw_test -f tests/Dockerfile.test .
docker run --rm rtw_test
