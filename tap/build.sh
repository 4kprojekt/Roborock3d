#!/bin/sh
# Cross-compile for the robot (Allwinner R16, Cortex-A7, glibc 2.19).
# Needs Zig: python -m pip install --target <dir> ziglang ; ZIGPY=<dir>
set -e
cd "$(dirname "$0")"
ZIG="python -m ziglang"
export PYTHONPATH="${ZIGPY:-$PYTHONPATH}"
T="-target arm-linux-gnueabihf.2.19 -mcpu=cortex_a7 -O2 -s"
# glibc 2.19's loader segfaults on lld's default RELRO + BIND_NOW layout -> norelro, lazy, 4K pages
$ZIG cc $T -shared -fPIC -Wl,-z,max-page-size=4096 -Wl,-z,norelro -Wl,-z,lazy -o lidartap.so lidartap.c -ldl
$ZIG cc $T -o lidartap_relay lidartap_relay.c
ls -la lidartap.so lidartap_relay
