#!/bin/env bash
# 
# Basic compile-and-run script for testing dcc C library with Python

root="$(dirname "$(dirname "$(realpath "$0")")")"
dcc_c="$root/fw/dcc.c"
test_py="$root/test/test_dcc.py"

gcc -Wall -shared -DPYTHON_TESTING "$dcc_c" -o dcc.so && python3 "$test_py"