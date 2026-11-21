#!/bin/bash
#
# Copyright (c) OpenIPC  https://openipc.org  MIT License
#
# vpn-yield-e2e.sh — vectis and another app with a camex tunnel on one emulator: the tunnel
# vectis starts by itself does not take the VPN from the other app (#130)
#
# Android holds one VPN, and an app that has the consent takes it from the app that holds it.
# Two apps here, both with the consent (appops), both joining one camex server:
#   A  the camex app (org.openipc.camex, bundles/camex): its BOOT_COMPLETED receiver starts
#      the tunnel by itself (startWhenFree), its START button is the user's start (start)
#   V  vectis (org.openipc.vectis): its BOOT_COMPLETED receiver starts the service, which
#      starts the tunnel by itself; its Start button is the user's
#
#   1. A connects (its receiver).
#   2. V starts by itself: V waits (WAITING FOR THE VPN), A keeps the VPN, also past V's
#      60 s retry; A stops: V comes up by itself.
#   3. A's START (the user) takes the VPN: V waits and takes nothing back past its retry;
#      V's Stop and Start (the user) take the VPN: A waits; V's Stop: A comes back by itself.
#   4. Save in the settings of a stopped V (Stop on its screen, then Settings, Save) takes
#      nothing: A keeps the VPN (#132).
#
# Usage: tests/android/vpn-yield-e2e.sh [API]   (API 34 by default)
#   ANDROID_IMAGE, CROSS_IMAGE, EMULATOR_IMAGE  Docker images
#   EMULATOR_DIR     the emulator, platform-tools and system images (mounted at /emu)
#   GRADLE_CACHE     Gradle's cache (mounted at /cache/gradle)
#   KVM_GROUP        the group id of /dev/kvm on the Docker host (992 by default)
#
set -e

API=${1:-34}
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CAMEX="$ROOT/bundles/camex"
OUT="$ROOT/build/vpn-yield-e2e"
ANDROID_IMAGE=${ANDROID_IMAGE:-compot/android:2026.10}
CROSS_IMAGE=${CROSS_IMAGE:-compot/cross:2026.11}
EMULATOR_IMAGE=${EMULATOR_IMAGE:-compot/android-emulator:2026.11}
EMULATOR_DIR=${EMULATOR_DIR:-$HOME/.cache/compot/toolchains/android-emulator}
GRADLE_CACHE=${GRADLE_CACHE:-$HOME/.gradle}
KVM_GROUP=${KVM_GROUP:-992}
NAME=vectis-vpn-yield-$$
SRV=$NAME-srv
NET=$NAME-net
A=org.openipc.camex
V=org.openipc.vectis
PSK="yield-secret"
UID_GID="$(id -u):$(id -g)"
RETRY=65    # past the 60 s tunnel retry of vectis (TUNNEL_RETRY_MS)

pass=0
fail=0
ok() { pass=$((pass + 1)); echo "PASS: $*"; }
bad() { fail=$((fail + 1)); echo "FAIL: $*"; }

cleanup() {
    docker rm -f "$NAME" "$SRV" >/dev/null 2>&1 || true
    docker network rm "$NET" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

[ -f "$CAMEX/Makefile" ] || { echo "bundles/camex is empty: git submodule update --init"; exit 1; }
rm -rf "$OUT"
mkdir -p "$OUT/w"

echo "== camex (static), the camex app and vectis (x86_64)"
docker run --rm --pull never -u "$UID_GID" -v "$ROOT:/src" -w /src "$CROSS_IMAGE" sh -c '
    mkdir -p build/vpn-yield-e2e/camex && cp -a bundles/camex/Makefile bundles/camex/src build/vpn-yield-e2e/camex/ &&
    cd build/vpn-yield-e2e/camex && make -s LDFLAGS=-static' \
    > "$OUT/camex.log" 2>&1 || { tail -30 "$OUT/camex.log"; exit 1; }
cp "$OUT/camex/camex" "$OUT/w/"
KEYS=$("$OUT/w/camex" --genkey)
printf '%s\n' "$KEYS" | sed -n 's/^server_key=/server_key=/p' > "$OUT/w/server.key"
PUBKEY=$(printf '%s\n' "$KEYS" | sed -n 's/^public_key=//p')
docker run --rm --pull never -u "$UID_GID" -v "$ROOT:/src" -v "$GRADLE_CACHE:/cache/gradle" \
    -w /src/bundles/camex/android "$ANDROID_IMAGE" gradle --no-daemon -q -PcamexAbis=x86_64 :app:assembleDebug \
    > "$OUT/gradle-camex.log" 2>&1 || { tail -30 "$OUT/gradle-camex.log"; exit 1; }
docker run --rm --pull never -u "$UID_GID" -v "$ROOT:/src" -v "$GRADLE_CACHE:/cache/gradle" \
    -w /src/android "$ANDROID_IMAGE" gradle --no-daemon -q -PvectisAbis=x86_64 assembleDebug \
    > "$OUT/gradle-vectis.log" 2>&1 || { tail -30 "$OUT/gradle-vectis.log"; exit 1; }
cp "$(ls "$CAMEX"/android/app/build/outputs/apk/debug/*.apk | head -1)" "$OUT/w/a.apk"
cp "$ROOT/android/app/build/outputs/apk/debug/app-debug.apk" "$OUT/w/v.apk"

cat > "$OUT/w/avd.sh" <<EOS
#!/bin/sh
export ANDROID_SDK_ROOT=/emu ANDROID_HOME=/emu ANDROID_AVD_HOME=/w/.emu/avd HOME=/w/.emu/home
D=\$ANDROID_AVD_HOME/e2e.avd
mkdir -p \$D \$HOME
printf 'avd.ini.encoding=UTF-8\npath=%s\ntarget=android-$API\n' \$D > \$ANDROID_AVD_HOME/e2e.ini
printf 'abi.type=x86_64\nhw.cpu.arch=x86_64\nimage.sysdir.1=system-images/android-$API/default/x86_64/\ntag.id=default\nhw.ramSize=2048\ndisk.dataPartition.size=2G\nhw.lcd.width=720\nhw.lcd.height=1280\nhw.lcd.density=320\nhw.gpu.enabled=yes\nhw.gpu.mode=swiftshader_indirect\n' > \$D/config.ini
/emu/emulator/emulator -avd e2e -no-window -no-audio -no-boot-anim -no-snapshot -gpu swiftshader_indirect -accel on > /w/emulator.log 2>&1 &
/emu/platform-tools/adb wait-for-device
until [ "\$(/emu/platform-tools/adb shell getprop sys.boot_completed 2>/dev/null | tr -d '\r')" = 1 ]; do sleep 2; done
echo BOOTED
sleep 100000
EOS

echo "== emulator (API $API) and a camex server"
docker network create "$NET" >/dev/null
docker run -d --name "$NAME" --network "$NET" --pull never --device /dev/kvm --group-add "$KVM_GROUP" \
    -u "$UID_GID" -v "$EMULATOR_DIR:/emu:ro" -v "$OUT/w:/w" "$EMULATOR_IMAGE" sh /w/avd.sh >/dev/null
docker run -d --name "$SRV" --network "$NET" --pull never --cap-add NET_ADMIN --device /dev/net/tun \
    -e CAMEX_POOL_HOLD=0 -v "$OUT/w:/w" "$CROSS_IMAGE" sh -c "/w/camex --mode server --port 5800 \
    --transport udp --local-cidr 10.200.0.1/24 --server-key-file /w/server.key --psk $PSK \
    > /w/server.log 2>&1" >/dev/null
SRV_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$SRV")
i=0
until docker logs "$NAME" 2>&1 | grep -q BOOTED; do
    i=$((i + 1))
    if [ $i -ge 120 ] || grep -q 'requires hardware acceleration' "$OUT/w/emulator.log" 2>/dev/null; then
        echo "the emulator did not boot:"; tail -5 "$OUT/w/emulator.log"; exit 1
    fi
    sleep 5
done
ADB="docker exec $NAME /emu/platform-tools/adb"
$ADB root >/dev/null 2>&1 || true    # BOOT_COMPLETED to the receivers needs root
sleep 3
$ADB wait-for-device
$ADB install -r -t /w/a.apk >/dev/null
$ADB install -r -t /w/v.apk >/dev/null
for p in $A $V; do
    [ "$API" -ge 33 ] && $ADB shell pm grant $p android.permission.POST_NOTIFICATIONS
    $ADB shell appops set $p ACTIVATE_VPN allow
done

# The settings, as the settings screens save them
cat > "$OUT/w/camex.xml" <<EOF
<?xml version='1.0' encoding='utf-8' standalone='yes' ?>
<map>
    <string name="server_host">$SRV_IP</string>
    <int name="server_port" value="5800" />
    <string name="name">appA</string>
    <string name="transport">udp</string>
    <string name="server_pubkey">$PUBKEY</string>
    <string name="psk">$PSK</string>
    <boolean name="start_on_boot" value="true" />
</map>
EOF
cat > "$OUT/w/vectis.xml" <<EOF
<?xml version='1.0' encoding='utf-8' standalone='yes' ?>
<map>
    <boolean name="running" value="true" />
    <boolean name="start_on_boot" value="true" />
    <boolean name="camex" value="true" />
    <string name="camex.server">$SRV_IP:5800</string>
    <string name="camex.name">vectisV</string>
    <string name="camex.transport">UDP</string>
    <string name="camex.pubkey">$PUBKEY</string>
    <string name="camex.psk">$PSK</string>
</map>
EOF
for f in camex:$A vectis:$V; do
    n=${f%%:*}; p=${f#*:}
    $ADB push /w/$n.xml /data/local/tmp/$n.xml >/dev/null
    $ADB shell "chmod 644 /data/local/tmp/$n.xml; run-as $p sh -c 'mkdir -p shared_prefs && cp /data/local/tmp/$n.xml shared_prefs/$n.xml'"
done

boot() { $ADB shell am broadcast -a android.intent.action.BOOT_COMPLETED -n "$1/.BootReceiver" >/dev/null; }
# tap <package> <text>: a button of the app's screen, from the UI dump (any case)
tap() {
    local xy _
    $ADB shell am start -n "$1/.MainActivity" >/dev/null 2>&1
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        sleep 1
        $ADB shell rm -f /data/local/tmp/ui.xml
        $ADB shell uiautomator dump /data/local/tmp/ui.xml >/dev/null 2>&1
        xy=$($ADB shell cat /data/local/tmp/ui.xml | tr '>' '\n' | grep -i "text=\"$2\"" | grep 'enabled="true"' | head -1 |
            sed -nE 's/.*bounds="\[([0-9]+),([0-9]+)\]\[([0-9]+),([0-9]+)\]".*/\1 \2 \3 \4/p' |
            awk '{ print int(($1 + $3) / 2), int(($2 + $4) / 2) }')
        [ -n "$xy" ] && { $ADB shell input tap $xy; return 0; }
    done
    echo "no $2 on the screen of $1"
    return 1
}
# v_state: vectis' last word on the tunnel in logcat ("app: camex tunnel: ..."): the screen
# redraws all the time, so uiautomator cannot dump it
v_state() {
    $ADB logcat -d -s vectis 2>/dev/null | tr -d '\r' | grep 'app: camex tunnel: ' | tail -1 | sed 's/.*app: camex tunnel: //'
}
v_wait() {  # v_wait <seconds> <pattern>: vectis' tunnel state matches
    local _
    for _ in $(seq 1 "$1"); do v_state | grep -q "$2" && return 0; sleep 1; done
    return 1
}
# v_user START|STOP: what the screen's Start and Stop send (MainActivity), the user's flag with Start
v_user() {
    if [ "$1" = START ]; then
        $ADB shell am start-foreground-service -n $V/.VectisService -a $V.START --ez $V.USER true >/dev/null
    else
        $ADB shell am startservice -n $V/.VectisService -a $V.STOP >/dev/null
    fi
}
# v_button <n>: tap the n-th visible button of V's screen. uiautomator cannot dump that screen,
# so the place comes from the view tree of `dumpsys activity top` (offsets added up);
# 1 Start, 2 Stop, 3 Terminal, 4 Settings
cat > "$OUT/vbutton.py" <<'EOP'
import re, sys
pkg, n = sys.argv[1], int(sys.argv[2])
found, stack, inside, base, mine = [], [], False, 0, False
for line in sys.stdin:
    ind = len(line) - len(line.lstrip())
    a = re.match(r'\s*ACTIVITY (\S+)/', line)
    if a:                   # every top activity is listed: the other app's MainActivity too
        mine, inside = a.group(1) == pkg, False
        continue
    if 'DecorView@' in line:
        inside, base, stack = mine, ind, [(ind, 0, 0, True)]
        continue
    m = re.search(r'\{\S+ (\S)\S* \S+ (-?\d+),(-?\d+)-(-?\d+),(-?\d+)', line)
    if not inside or ind <= base or not m:
        inside = inside and ind > base
        continue
    while stack[-1][0] >= ind:
        stack.pop()
    x, y, shown = stack[-1][1:]
    l, t, r, b = map(int, m.group(2, 3, 4, 5))
    shown = shown and m.group(1) == 'V'
    stack.append((ind, x + l, y + t, shown))
    if shown and line.lstrip().startswith('android.widget.Button{'):
        found.append((x + (l + r) // 2, y + (t + b) // 2))
if len(found) >= n:
    print('%d %d' % found[n - 1])
EOP
v_button() {
    local xy
    $ADB shell am start -n $V/.MainActivity >/dev/null 2>&1
    sleep 2
    xy=$($ADB shell dumpsys activity top | python3 "$OUT/vbutton.py" $V "$1")
    [ -n "$xy" ] && { $ADB shell input tap $xy; return 0; }
    echo "no button $1 on the screen of $V"
    return 1
}
# v_dialog <text>: tap a button of V's dialog: over the screen, uiautomator can dump it
v_dialog() {
    local xy _
    for _ in 1 2 3 4 5; do
        sleep 1
        $ADB shell rm -f /data/local/tmp/ui.xml
        $ADB shell uiautomator dump /data/local/tmp/ui.xml >/dev/null 2>&1
        xy=$($ADB shell cat /data/local/tmp/ui.xml 2>/dev/null | tr '>' '\n' | grep -i "text=\"$1\"" | head -1 |
            sed -nE 's/.*bounds="\[([0-9]+),([0-9]+)\]\[([0-9]+),([0-9]+)\]".*/\1 \2 \3 \4/p' |
            awk '{ print int(($1 + $3) / 2), int(($2 + $4) / 2) }')
        [ -n "$xy" ] && { $ADB shell input tap $xy; return 0; }
    done
    echo "no $1 in the dialog of $V"
    $ADB shell screencap -p /data/local/tmp/v.png && $ADB pull /data/local/tmp/v.png /w/v-dialog.png >/dev/null 2>&1
    return 1
}
a_board() {  # the text of A's TUNNEL row
    $ADB shell am start -n $A/.MainActivity >/dev/null 2>&1
    sleep 2
    $ADB shell rm -f /data/local/tmp/ui.xml
    $ADB shell uiautomator dump /data/local/tmp/ui.xml >/dev/null 2>&1
    $ADB shell cat /data/local/tmp/ui.xml 2>/dev/null | tr '>' '\n' | grep -oE 'text="(CONNECTED|CONNECTING|WAITING|STOPPED|VPN CONSENT)[^"]*"' | head -1
}
client_up() {  # client_up <ID>: the server lists the client
    docker exec "$SRV" /w/camex --status 2>/dev/null | awk 'NR > 2 { print $1 }' | grep -qx "$1"
}
wait_up() {  # wait_up <ID> <seconds>
    local i=0
    until client_up "$1"; do i=$((i + 1)); [ $i -lt "$2" ] || return 1; sleep 1; done
}
switches() { $ADB logcat -d 2>/dev/null | tr -d '\r' | grep -c "Switched from" || true; }

echo "== 1. A connects by itself (its receiver), no other VPN"
$ADB logcat -c
boot $A
wait_up appA 60 && ok "A connected" || bad "A did not connect"

echo "== 2. V starts by itself while A holds the VPN"
$ADB logcat -c
boot $V
v_wait 30 "waits: another app" && ok "V waits: another app's VPN is on" || bad "V: $(v_state)"
echo "   (past V's retry: $RETRY s)"
sleep $RETRY
[ "$(switches)" = 0 ] && ok "A keeps the VPN: nothing switched" || bad "the VPN switched $(switches) times while V waited"
client_up appA && ok "A's tunnel is still up" || bad "A's tunnel went"
client_up vectisV && bad "V is on the server while it waits" || ok "V is not on the server"
tap $A STOP || true
v_wait 60 "^http://" && wait_up vectisV 30 && ok "A stopped: V came up by itself" \
    || bad "V did not come up after A stopped: $(v_state)"

echo "== 3. A's START takes the VPN; V waits; V's Start takes it back"
$ADB logcat -c
tap $A START || true
i=0; until a_board | grep -q CONNECTED; do i=$((i + 1)); [ $i -lt 20 ] || break; done
a_board | grep -q CONNECTED && ok "A's START connected A" || bad "A after its START: $(a_board)"
v_wait 30 "waits: another app" && ok "V waits after A took the VPN" || bad "V after A's START: $(v_state)"
n=$(switches)
echo "   (past V's retry: $RETRY s)"
sleep $RETRY
[ "$(switches)" = "$n" ] && ok "V took nothing back while it waited" || bad "the VPN switched while V waited"
client_up appA && ok "A's tunnel is still up" || bad "A's tunnel went"
$ADB logcat -c
v_user STOP
v_wait 30 "^stopped" || true
v_user START
v_wait 60 "^http://" && ok "V's Start (the user) took the VPN and connected" || bad "V after its Start: $(v_state)"
case "$(a_board)" in
    *WAITING*) ok "A waits: V took the VPN";;
    *) bad "A's board after V's Start: $(a_board)";;
esac
v_user STOP
i=0; until a_board | grep -q CONNECTED; do i=$((i + 1)); [ $i -lt 20 ] || break; done
a_board | grep -q CONNECTED && ok "V stopped: A came back by itself" || bad "A after V's Stop: $(a_board)"

echo "== 4. Save in the settings of a stopped V takes nothing (#132)"
v_button 2 || bad "V's Stop not reached"   # the screen's: it clears "running", so V stays stopped
sleep 3
$ADB shell run-as $V cat shared_prefs/vectis.xml 2>/dev/null | grep -q 'name="running" value="false"' \
    && ! $ADB shell dumpsys activity services $V | grep -q ServiceRecord \
    && ok "V is stopped: its service is gone, running is false" || bad "V is not stopped before its Save"
i=0; until a_board | grep -q CONNECTED; do i=$((i + 1)); [ $i -lt 20 ] || break; done
a_board | grep -q CONNECTED && ok "V stopped by its Stop: A holds the VPN" || bad "A before V's Save: $(a_board)"
$ADB logcat -c
v_button 4 && v_dialog SAVE && ok "V's settings saved" || bad "V's Save not reached"
sleep 10
[ "$(switches)" = 0 ] && ok "V's Save took nothing: no VPN switch" || bad "the VPN switched $(switches) times after V's Save"
client_up appA && ok "A's tunnel is still up" || bad "A's tunnel went after V's Save"
case "$(a_board)" in
    *CONNECTED*) ok "A is still connected";;
    *) bad "A's board after V's Save: $(a_board)";;
esac

$ADB logcat -d > "$OUT/logcat.txt" 2>&1 || true
grep -q 'FATAL EXCEPTION' "$OUT/logcat.txt" && bad "a crash in logcat ($OUT/logcat.txt)" || ok "no crash"
echo "== $pass passed, $fail failed (API $API)"
[ "$fail" -eq 0 ]
