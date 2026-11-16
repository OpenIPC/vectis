#!/bin/sh
# -PvectisAbis must be a non-empty list of the ABIs android/build-native.sh builds (#91):
# configure the project (gradle help) with bad and good lists. No device, no NDK build.
#   cd android && sh test-abis.sh
set -u
cd "$(dirname "$0")"
known="one or more of arm64-v8a,armeabi-v7a,x86_64"
fail=0
for v in "" "," "arm64" "x86" "arm64-v8a,,armeabi-v7a" "arm64-v8a,"; do
	if out=$(gradle --no-daemon -q help "-PvectisAbis=$v" 2>&1); then
		echo "FAIL: -PvectisAbis='$v' was accepted"; fail=1
	elif ! printf '%s\n' "$out" | grep -q "$known"; then
		echo "FAIL: -PvectisAbis='$v' failed without the ABI list:"; printf '%s\n' "$out" | tail -5; fail=1
	else
		echo "ok: -PvectisAbis='$v' stops the build"
	fi
done
for v in "arm64-v8a" "arm64-v8a, armeabi-v7a,x86_64"; do
	if gradle --no-daemon -q help "-PvectisAbis=$v" >/dev/null 2>&1; then
		echo "ok: -PvectisAbis='$v' is accepted"
	else
		echo "FAIL: -PvectisAbis='$v' was rejected"; fail=1
	fi
done
exit $fail
