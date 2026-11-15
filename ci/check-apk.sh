#!/bin/sh
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# ci/check-apk.sh — check a release APK before it is published: exactly one
# signer, and that signer is the project's certificate (or users of an
# earlier release could not update the app); with VERSION, also that the
# APK's versionName is VERSION.  Used by build.yml (android-release and
# release), the one place the certificate is written down.
#
# Usage: ci/check-apk.sh APK [VERSION]     (needs ANDROID_HOME with build-tools)
#

set -eu

# SHA-256 of the certificate of the vectis release key (alias vectis, RSA 4096)
CERT_SHA256=cc3d24b01d9e509c9d97c4dbbe67de86458ac7edd32e57b5517bad93d7024e5c

apk=${1:?usage: ci/check-apk.sh APK [VERSION]}
ver=${2:-}

# The newest build-tools whose apksigner runs here prints the signers; whole
# lines only ("Signer #1 certificate ...", "Signer (minSdkVersion=N...)
# certificate ..." or "V2 Signer: certificate ..." of build-tools 37), 64 hex
# digits, one signer only.
signer=
checked=
for bt in $(ls -d "$ANDROID_HOME"/build-tools/* | sort -rV); do
	if out=$("$bt/apksigner" verify --print-certs "$apk" 2>&1); then
		signer=$(echo "$out" | sed -nE 's/^(Signer #[0-9]+|Signer \([^)]*\)|V[0-9]+ Signer:) certificate SHA-256 digest: ([0-9a-f]{64})$/\2/p' | sort -u)
		checked=$bt
		break
	fi
	echo "$bt/apksigner: $out" | head -3 >&2
done
[ -n "$checked" ] || { echo "::error::apksigner could not check $apk"; exit 1; }
if [ "$(printf '%s\n' "$signer" | grep -c .)" -ne 1 ]; then
	echo "::error::$apk: expected one signer, apksigner printed:"
	"$checked/apksigner" verify --print-certs "$apk" || true
	exit 1
fi
[ "$signer" = "$CERT_SHA256" ] ||
	{ echo "::error::$apk is signed by $signer, not the project's certificate $CERT_SHA256"; exit 1; }
echo "$apk: signed by the project's certificate"

if [ -n "$ver" ]; then
	"$checked/aapt" dump badging "$apk" | grep -q "versionName='$ver'" ||
		{ echo "::error::$apk is not version $ver"; exit 1; }
	echo "$apk: version $ver"
fi
