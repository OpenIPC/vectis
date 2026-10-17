#!/bin/sh
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# ci/build-static.sh — static musl build of vectis (with -U) for the
# release binary that runs on any Linux of that architecture, whatever
# its libc.  Runs inside alpine:3.22 (build.yml):
#
#   docker run --rm -v "$PWD":/src -w /src alpine:3.22 sh ci/build-static.sh [OUTDIR ARCH]
#
# Fails on any compiler warning, runs the regression tests, and with
# OUTDIR copies the binary there as vectis-linux-ARCH (no version in the
# name, so the release links never change) with a SHA256SUMS file.
#
set -eu

apk add -q build-base linux-headers python3 file

make clean
# No pipe: a failing make must stop the script (sh has no pipefail).
if ! make LDFLAGS="-static -Wl,--gc-sections" all vectis-bootrom > build.log 2>&1; then
    cat build.log
    exit 1
fi
cat build.log
if grep -q 'warning:' build.log; then
    echo "compiler warnings, see above" >&2
    exit 1
fi
rm -f build.log
file vectis vectis-bootrom
./vectis --version
python3 tests/test_vectis.py .

if [ $# -ge 2 ]; then
    out=$1 arch=$2
    mkdir -p "$out"
    cp vectis "$out/vectis-linux-$arch"
    # Only this build's file: an OUTDIR reused for another build must not
    # mix older binaries into the checksums.
    (cd "$out" && sha256sum "vectis-linux-$arch" > SHA256SUMS && cat SHA256SUMS)
fi
