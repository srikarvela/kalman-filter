#!/usr/bin/env bash
# Run tcl/kalman_synth.tcl inside a Parallels Windows VM from macOS (Apple
# Silicon hosts can't run Vivado natively) and copy reports/ back.
# Adapted from cordic-engine/scripts/vivado_in_parallels.sh.
#
#   ./scripts/vivado_in_parallels.sh                 # 4.000 ns (the XDC constraint)     make vm-synth
#   ./scripts/vivado_in_parallels.sh 4.000 3.000     # one Vivado run per period         make vm-sweep
#
# The working tree (chisel/generated/KalmanFilter.v, tcl/, constraints/ -- no
# commit needed; run `make kf-verilog` first) is zipped into a folder the VM
# sees through Parallels shared folders, unzipped to a local VM disk, built
# with `prlctl exec --current-user`, and reports/ + build/*.dcp are copied
# back the same way. One Vivado process per period, so a flaky run under x86
# emulation ("couldn't read file ...") only repeats that configuration.
#
# Environment overrides:
#   KALMAN_VM          Parallels VM name             (default "Windows 11")
#   KALMAN_VIVADO      vivado.bat inside the VM      (default C:\Xilinx\Vivado\2024.1\bin\vivado.bat)
#   KALMAN_VM_WORKDIR  build directory inside the VM (default C:\kalman)
#   KALMAN_SHARE_MAC   macOS side of a shared folder (default ~/Downloads/kalman-vm-transfer)
#   KALMAN_SHARE_VM    same folder as the VM sees it (default Z:\Downloads\kalman-vm-transfer)
#   KALMAN_TRIES       attempts per configuration    (default 3)
set -euo pipefail
cd "$(dirname "$0")/.."

[ $# -ge 1 ] || set -- 4.000
VM="${KALMAN_VM:-Windows 11}"
VIVADO="${KALMAN_VIVADO:-C:\\Xilinx\\Vivado\\2024.1\\bin\\vivado.bat}"
WORK="${KALMAN_VM_WORKDIR:-C:\\kalman}"
SHARE_MAC="${KALMAN_SHARE_MAC:-$HOME/Downloads/kalman-vm-transfer}"
SHARE_VM="${KALMAN_SHARE_VM:-Z:\\Downloads\\kalman-vm-transfer}"
TRIES="${KALMAN_TRIES:-3}"

[ -f chisel/generated/KalmanFilter.v ] || { echo "chisel/generated/KalmanFilter.v missing: run 'make kf-verilog' first"; exit 1; }
command -v prlctl >/dev/null || { echo "prlctl not found (Parallels Desktop Pro/Business required)"; exit 1; }
state="$(prlctl list -a -o status,name | awk -v vm="$VM" '$0 ~ vm {print $1}')"
case "$state" in
  running)   ;;
  suspended) echo "resuming VM '$VM'"; prlctl resume "$VM" >/dev/null ;;
  stopped)   echo "starting VM '$VM'"; prlctl start  "$VM" >/dev/null ;;
  paused)    prlctl unpause "$VM" >/dev/null ;;
  *) echo "Parallels VM '$VM' not found (set KALMAN_VM)"; exit 1 ;;
esac

vm() { prlctl exec "$VM" --current-user "$@"; }
for _ in $(seq 1 30); do vm cmd /c "echo ready" >/dev/null 2>&1 && break; sleep 5; done

mkdir -p "$SHARE_MAC" reports build
rm -rf "$SHARE_MAC/out" "$SHARE_MAC/kalman.zip"
zip -qr "$SHARE_MAC/kalman.zip" chisel/generated/KalmanFilter.v tcl constraints reports
vm powershell -NoProfile -Command "Remove-Item -Recurse -Force '$WORK' -ErrorAction SilentlyContinue; Expand-Archive -Path '$SHARE_VM\\kalman.zip' -DestinationPath '$WORK'" >/dev/null
echo "Copied working tree to $VM:$WORK"

status=0
for p in "$@"; do
  ok=1
  for i in $(seq 1 "$TRIES"); do
    log="vivado_${p}ns_try$i.log"
    echo "== Vivado OOC KalmanFilter period=$p ns, attempt $i/$TRIES (log: build/$log)"
    vm cmd /c "cd /d $WORK && \"$VIVADO\" -mode batch -nolog -nojournal -source tcl/kalman_synth.tcl -tclargs period:$p > $log 2>&1" || true
    vm cmd /c "mkdir \"$SHARE_VM\\out\" 2>nul & xcopy /e /y /i /q \"$WORK\\reports\" \"$SHARE_VM\\out\\reports\" >nul & xcopy /e /y /i /q \"$WORK\\build\" \"$SHARE_VM\\out\\build\" >nul & copy /y \"$WORK\\$log\" \"$SHARE_VM\\out\\\" >nul" || true
    cp "$SHARE_MAC/out/$log" build/ 2>/dev/null || true
    grep -E "^ERROR|^=== KalmanFilter period" "build/$log" | head -10 || true
    if grep -q "^=== KalmanFilter period=$p: " "build/$log" 2>/dev/null; then ok=0; break; fi
    echo "-- run did not complete, retrying"
  done
  [ "$ok" -eq 0 ] || { echo "-- period=$p failed after $TRIES attempts (see build/)"; status=1; }
done

cp -R "$SHARE_MAC/out/reports/." reports/ 2>/dev/null || true
cp -R "$SHARE_MAC/out/build/." build/ 2>/dev/null || true
rm -rf "$SHARE_MAC"
[ "$status" -eq 0 ] && echo "Done: reports/N_*ns/ (run scripts/ppa_table.py for reports/ppa.csv); routed .dcp in build/"
exit "$status"
