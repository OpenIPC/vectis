#!/bin/sh
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# gen-fonts.sh — turn web/fonts/*.woff2 (brand's brass 1.5.0: Brass Serif
# and Brass Mono, web/fonts/OFL.txt) into webfonts_assets.h, the fonts
# vectis serves at /fonts/<file> for its web page
#
# Usage: sh web/gen-fonts.sh OUT_FILE
#
# Notes:
#   - Made at build time (Makefile, android/build-native.sh) and never
#     committed, unlike webterm_assets.h.  Needs od and awk.
#   - woff2 is compressed already: the arrays are the files as they are.
#

set -eu

OUT=${1:?usage: gen-fonts.sh OUT_FILE}
DIR=$(cd "$(dirname "$0")" && pwd)/fonts
set -- "$DIR"/*.woff2
[ -e "$1" ] || { echo "gen-fonts.sh: no fonts in $DIR" >&2; exit 1; }
for f in "$@"; do   # od's own failure would not stop the pipe below
	[ -s "$f" ] && [ -r "$f" ] || { echo "gen-fonts.sh: cannot read $f" >&2; exit 1; }
done

{
	echo "/* Made by web/gen-fonts.sh from web/fonts/ at build time -- do not edit. */"
	echo "/* Brass Serif and Brass Mono (IBM Plex, SIL OFL 1.1): web/fonts/OFL.txt */"
	n=0
	for f in "$@"; do
		echo
		printf 'static const unsigned char wf_font%d[] = {\n' "$n"
		bytes=$(od -An -v -tu1 "$f") || { echo "gen-fonts.sh: cannot read $f" >&2; exit 1; }
		# awk splits on any run of blanks: BSD od pads its last line with them
		printf '%s\n' "$bytes" | awk '{ for (i = 1; i <= NF; i++) { printf "%s,", $i
			if (++n % 16 == 0) printf "\n" } } END { if (n % 16) printf "\n" }'
		echo "};"
		n=$((n + 1))
	done
	echo
	echo "static const struct { const char *name; const unsigned char *data; size_t len; } web_fonts[] = {"
	n=0
	for f in "$@"; do
		printf '    { "%s", wf_font%d, sizeof wf_font%d },\n' "$(basename "$f")" "$n" "$n"
		n=$((n + 1))
	done
	echo "};"
} > "$OUT.tmp"
mv "$OUT.tmp" "$OUT"
echo "gen-fonts.sh: $OUT"
