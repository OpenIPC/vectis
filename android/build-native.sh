#!/bin/sh
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# build-native.sh — build vectis (vectis.c, usbserial.c, ch34x.c, the web
# terminal's webterm_assets.h and the page's brass fonts) with the NDK for the
# Android app
#
# Usage:
#   android/build-native.sh OUT_DIR [ABI...]    (default ABIs: arm64-v8a armeabi-v7a)
#
# Notes:
#   - The result is an executable, not a JNI library: OUT_DIR/<abi>/libvectis.so.
#     Android installs only lib*.so files from the APK, so the name has to look
#     like a library; the app runs it as a child process with -U fd:@NAME.
#   - The Makefile's flags (-Wall -Wextra -Wpedantic -Os, sections) with -Werror,
#     -U support with the CH34x family; ELF segments aligned to 16 KB pages, as
#     Android 15 devices with 16 KB pages need.
#   - Needs ANDROID_NDK_HOME (the NDK r27 directory) and a Linux or macOS
#     build host (on Windows: WSL).
#

set -eu

OUT=${1:?usage: build-native.sh OUT_DIR [ABI...]}
shift
[ $# -gt 0 ] || set -- arm64-v8a armeabi-v7a
API=21
ROOT=$(cd "$(dirname "$0")/.." && pwd)
# The NDK's prebuilt host folder: linux-x86_64 or darwin-x86_64 (on Windows use WSL)
case $(uname -s) in
Darwin) HOST=darwin-x86_64 ;;
*)      HOST=linux-x86_64 ;;
esac
TC=${ANDROID_NDK_HOME:?ANDROID_NDK_HOME is not set}/toolchains/llvm/prebuilt/$HOST/bin
[ -d "$TC" ] || { echo "build-native.sh: no NDK toolchain in $TC" >&2; exit 1; }

# The page's brass fonts as C arrays (web/gen-fonts.sh), made anew on every
# run and never committed: next to vectis.c, where make puts them too, so
# #include "webfonts_assets.h" can never pick up a stale copy from there
sh "$ROOT/web/gen-fonts.sh" "$ROOT/webfonts_assets.h"

for abi in "$@"; do
	case $abi in
	arm64-v8a)   cc=$TC/aarch64-linux-android$API-clang ;;
	armeabi-v7a) cc=$TC/armv7a-linux-androideabi$API-clang ;;
	x86_64)      cc=$TC/x86_64-linux-android$API-clang ;;
	*) echo "build-native.sh: unknown ABI $abi" >&2; exit 1 ;;
	esac
	mkdir -p "$OUT/$abi"
	"$cc" -std=gnu99 -Wall -Wextra -Wpedantic -Werror -Os -ffunction-sections -fdata-sections \
		-DVECTIS_USB -DVECTIS_CHIP_CH34X -DVECTIS_FONTS \
		-o "$OUT/$abi/libvectis.so" \
		"$ROOT/vectis.c" "$ROOT/usbserial.c" "$ROOT/ch34x.c" \
		-Wl,--gc-sections -Wl,-z,max-page-size=16384 -s
	echo "build-native.sh: $OUT/$abi/libvectis.so"
done
