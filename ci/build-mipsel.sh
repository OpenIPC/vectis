#!/bin/sh
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# ci/build-mipsel.sh — static musl build of vectis (with -U) for MIPS
# little-endian routers: MediaTek MT7620/MT7621/MT7628 and other MIPS32r2
# cores running OpenWrt.  Soft-float, so it runs with or without an FPU
# (kernels of such routers often have no FPU emulation, and hard-float code
# dies with "Illegal instruction"), whatever the router's libc.
#
#   ci/build-mipsel.sh [OUTDIR]
#
# Uses the OpenWrt 23.05.5 ramips/mt7621 toolchain (gcc 12.3.0, musl,
# -march=24kc), downloaded once into $TOOLCHAIN_CACHE and checked by its
# SHA256 before its compiler is run.  Needs curl, xz, make and, for the
# smoke test, qemu-mipsel.  Fails on any compiler warning; with OUTDIR
# copies the binary there as vectis-linux-mipsel with a SHA256SUMS file.
#
set -eu

OUT=${1:-}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CACHE=${TOOLCHAIN_CACHE:-$HOME/.cache/vectis-toolchains}
# Relative paths are the caller's: the build below runs in $ROOT.
case "$OUT" in ""|/*) ;; *) OUT=$PWD/$OUT ;; esac
case "$CACHE" in /*) ;; *) CACHE=$PWD/$CACHE ;; esac

OPENWRT=https://downloads.openwrt.org/releases/23.05.5/targets/ramips/mt7621
NAME=openwrt-toolchain-23.05.5-ramips-mt7621_gcc-12.3.0_musl.Linux-x86_64
SHA=bb08478a46869009a2a8b85f88e266cc6b6d3ccefc34de61907ead2a986e08c9
DIR=$CACHE/$NAME/toolchain-mipsel_24kc_gcc-12.3.0_musl

if [ ! -x "$DIR/bin/mipsel-openwrt-linux-musl-gcc" ]; then
    mkdir -p "$CACHE"
    echo "downloading $OPENWRT/$NAME.tar.xz"
    # download first (with retries), then check and extract: a truncated
    # stream piped into tar looks like a corrupt archive
    curl -fsSL --retry 3 -o "$CACHE/toolchain.tar.xz" "$OPENWRT/$NAME.tar.xz"
    echo "$SHA  $CACHE/toolchain.tar.xz" | sha256sum -c - >/dev/null ||
        { echo "SHA256 mismatch for $NAME.tar.xz" >&2; rm -f "$CACHE/toolchain.tar.xz"; exit 1; }
    tar -xJf "$CACHE/toolchain.tar.xz" -C "$CACHE"
    rm -f "$CACHE/toolchain.tar.xz"
fi
CROSS=$DIR/bin/mipsel-openwrt-linux-musl-
export STAGING_DIR="$DIR"     # OpenWrt's gcc wrapper warns without it

cd "$ROOT"
make clean
# No pipe: a failing make must stop the script (sh has no pipefail).
if ! make CC="${CROSS}gcc" STRIP="${CROSS}strip" LDFLAGS="-static -Wl,--gc-sections" \
        all vectis-bootrom > build.log 2>&1; then
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

# The binary must start and report the version vectis.c defines.
ver=$(sed -n 's/^#define PROGRAM_VERSION *"\(.*\)"/\1/p' vectis.c)
qemu-mipsel ./vectis-bootrom --version
got=$(qemu-mipsel ./vectis --version | head -1 | awk '{print $NF}')
echo "vectis --version: $got, vectis.c: $ver"
[ -n "$ver" ] && [ "$got" = "$ver" ] || { echo "version mismatch" >&2; exit 1; }

if [ -n "$OUT" ]; then
    mkdir -p "$OUT"
    cp vectis "$OUT/vectis-linux-mipsel"
    # Only this build's file: an OUTDIR reused for another build must not
    # mix older binaries into the checksums.
    (cd "$OUT" && sha256sum vectis-linux-mipsel > SHA256SUMS && cat SHA256SUMS)
fi
