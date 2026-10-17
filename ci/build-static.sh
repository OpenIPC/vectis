#!/bin/sh
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# ci/build-static.sh — static musl build of vectis and vectis-ch34x for
# release binaries that run on any Linux of that architecture, whatever
# its libc.  Runs inside alpine:3.22 (build.yml, release.yml):
#
#   docker run --rm -v "$PWD":/src -w /src alpine:3.22 sh ci/build-static.sh [OUTDIR VERSION ARCH]
#
# Fails on any compiler warning, runs the regression tests, and with
# OUTDIR copies the binaries there as vectis-VERSION-linux-ARCH and
# vectis-ch34x-VERSION-linux-ARCH with a SHA256SUMS file.
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
file vectis vectis-ch34x vectis-bootrom
./vectis --version
./vectis-ch34x --version
python3 tests/test_vectis.py .

if [ $# -ge 3 ]; then
    out=$1 ver=$2 arch=$3
    mkdir -p "$out"
    cp vectis "$out/vectis-$ver-linux-$arch"
    cp vectis-ch34x "$out/vectis-ch34x-$ver-linux-$arch"
    # Only this build's files: an OUTDIR reused for another version
    # must not mix older binaries into the checksums.
    (cd "$out" && sha256sum "vectis-$ver-linux-$arch" "vectis-ch34x-$ver-linux-$arch" \
        > SHA256SUMS && cat SHA256SUMS)
fi
