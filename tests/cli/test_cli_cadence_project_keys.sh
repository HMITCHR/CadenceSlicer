#!/usr/bin/env bash
# End-to-end check that the CLI slices a saved Cadence project with the settings the project owns.
#
# A Cadence project keeps its nozzle pair and the per-tool layer height limits at project level, and
# saves each material's values composed for the nozzle it prints on. None of these are listed in
# different_settings_to_system, so the system preset refresh of a project load must not replace them.
# The role filament ids are checked with the process entry as saved and with them left out of it. The
# project saves one retraction column per nozzle while the 2.5 preset lays them out per flow variant. As in the
# app, the 0.6 nozzle retracts like the H2D 0.6 preset (1.4 mm) where the project left the 0.2 preset's value;
# the second copy gives it its own 2.2 mm, which must stay. The project
# also owns the prime tower and beam settings Cadence uses: the second copy narrows the tower to 33 mm and
# sets the beam layer count to 3 without listing either, and both must reach the G-code. Material 7 prints
# on the 0.6 nozzle; the second copy saves a stale 7.7 mm3/s for it under the 0.6 preset's own name, and the
# command line must compose it for that nozzle as the app does (30, the 0.6 mm Bambu PLA Basic value).
# A project saved under a process name its system preset was since renamed from (renamed_from) is refreshed
# from that preset, as the app finds it: two more copies save a stale 37% sparse infill density, which the
# project does not list, one under the old process name and one under the current name; both must print the
# preset's own density.
# The project was saved before the paint depth options existed and paints as deep as its shells: it slices
# with both at 0 (follow the shell count). The second copy saves a depth of its own, which stays. A copy with
# the mode Off takes the process preset's depth, as in OrcaSlicer 2.5.
#
# usage: test_cli_cadence_project_keys.sh <orca-slicer binary> <python3> <tests/data>
set -u

BIN="${1:-}"
PY="${2:-python3}"
DATA="${3:-}"
# 77 is the test's SKIP_RETURN_CODE.
[ -x "$BIN" ] || { echo "SKIP: orca-slicer binary not found: $BIN"; exit 77; }
PROJECT="$DATA/feature-split-L-tower-008.3mf"
[ -f "$PROJECT" ] || { echo "FAIL: project not found: $PROJECT"; exit 1; }

WORK="$(mktemp -d "${TMPDIR:-/tmp}/orca-cli-cadence-keys.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT

# The same project with the role filament ids taken out of the process entry of the change list and the
# 0.6 nozzle retracting 1.4 mm.
"$PY" - "$PROJECT" "$WORK/unlisted.3mf" <<'EOF' || exit $?
import json, sys, zipfile

src, dst = sys.argv[1], sys.argv[2]
roles = {"outer_wall_filament_id", "inner_wall_filament_id", "sparse_infill_filament_id",
         "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"}
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            config = json.loads(data)
            different = config["different_settings_to_system"]
            different[0] = ";".join(k for k in different[0].split(";") if k and k not in roles)
            assert len(config["retraction_length"]) == 2, config["retraction_length"]
            config["retraction_length"] = [config["retraction_length"][0], "2.2"]
            config["prime_tower_width"] = "33"
            config["interlocking_beam_layer_count"] = "3"
            config["top_color_penetration_layers"] = "2"
            config["bottom_color_penetration_layers"] = "4"
            config["filament_max_volumetric_speed"][6] = "7.7"
            config["filament_settings_id"][6] = "Bambu PLA Basic @BBL H2D 0.6 nozzle"
            data = json.dumps(config, indent=4)
        zout.writestr(item, data)
EOF

# Two copies with a stale unlisted sparse infill density: one saved under the process name the preset was renamed
# from, one under its current name.
"$PY" - "$PROJECT" "$WORK" <<'EOF' || exit $?
import json, sys, zipfile

src, work = sys.argv[1], sys.argv[2]
for tag in ("renamed", "current"):
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile("%s/%s.3mf" % (work, tag), "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename == "Metadata/project_settings.config":
                config = json.loads(data)
                assert config["print_settings_id"] == "MN Feature 0.2-0.6 0.10-0.30 Standard @BBL", config["print_settings_id"]
                assert "sparse_infill_density" not in config["different_settings_to_system"][0].split(";")
                if tag == "renamed":
                    config["print_settings_id"] = "MN Feature 0.2-0.6 0.10-0.30 Standard Candidate @BBL"
                config["sparse_infill_density"] = "37%"
                data = json.dumps(config, indent=4)
            zout.writestr(item, data)
EOF

# The same project with the mode Off: no plate uses a mode, so setup has cleared the tower ledger. Off applies
# the stock tower rules, so this copy also turns "No sparse layers" off as a change of its own.
"$PY" - "$PROJECT" "$WORK/off.3mf" <<'EOF' || exit $?
import json, sys, zipfile

src, dst = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == "Metadata/project_settings.config":
            config = json.loads(data)
            assert config["mixed_nozzle_slicing_mode"] == "feature_split", config["mixed_nozzle_slicing_mode"]
            config["mixed_nozzle_slicing_mode"] = "off"
            config["mixed_nozzle_tower_policy_state"] = ""
            config["wipe_tower_no_sparse_layers"] = "0"
            config["different_settings_to_system"][0] += ";wipe_tower_no_sparse_layers"
            data = json.dumps(config, indent=4)
        zout.writestr(item, data)
EOF

# slice <tag> <input>: slice plate 1 into $WORK/<tag>/out.3mf with a fresh data directory.
slice() {
    local out="$WORK/$1" input="$2"
    mkdir -p "$out"
    "$BIN" --datadir "$out/datadir" --slice 1 --arrange 0 --no-thumbnails --outputdir "$out" --export-3mf out.3mf "$input" \
        > "$out/log" 2>&1 || { echo "FAIL: $1: orca-slicer exited $?"; tail -n 40 "$out/log"; exit 1; }
}

# check <tag> <input>: the sliced G-code's settings carry the input project's own values.
check() {
    if ! "$PY" - "$2" "$WORK/$1/out.3mf" "$1" <<'EOF'
import json, sys, zipfile

with zipfile.ZipFile(sys.argv[1]) as z:
    saved = json.loads(z.read("Metadata/project_settings.config"))
with zipfile.ZipFile(sys.argv[2]) as z:
    gcode = z.read("Metadata/plate_1.gcode").decode("utf-8", "replace")
sliced = {}
for line in gcode.splitlines():
    if line.startswith("; ") and " = " in line:
        key, value = line[2:].split(" = ", 1)
        sliced[key] = value

def as_text(value):
    return ",".join(value) if isinstance(value, list) else value

errors = []
expected_depth = ("2", "4") if sys.argv[3] == "unlisted" else ("0", "0")
for key, value in zip(("top_color_penetration_layers", "bottom_color_penetration_layers"), expected_depth):
    if sys.argv[3] != "unlisted" and key in saved:
        errors.append("%s is saved in the input project" % key)
    if sliced.get(key) != value:
        errors.append("%s is %r, expected %r" % (key, sliced.get(key), value))
expected_retraction = "0.4,2.2" if sys.argv[3] == "unlisted" else "0.4,1.4"
if sliced.get("retraction_length") != expected_retraction:
    errors.append("retraction_length is %r, expected %r" % (sliced.get("retraction_length"), expected_retraction))
for key in ("nozzle_diameter", "min_layer_height", "max_layer_height", "outer_wall_filament_id",
            "inner_wall_filament_id", "sparse_infill_filament_id", "internal_solid_filament_id",
            "top_surface_filament_id", "bottom_surface_filament_id", "interlocking_beam_layer_count",
            "prime_tower_width", "enable_tower_interface_features"):
    if sliced.get(key) != as_text(saved[key]):
        errors.append("%s is %r, saved %r" % (key, sliced.get(key), as_text(saved[key])))
# Material 7 is PLA Basic on the 0.6 nozzle: composed for that nozzle, whatever the project saved.
speeds = sliced.get("filament_max_volumetric_speed", "").split(",")
if len(speeds) < 7 or speeds[6] != "30":
    errors.append("filament_max_volumetric_speed is %r, material 7 should be 30 (saved %r)"
                  % (sliced.get("filament_max_volumetric_speed"), saved["filament_max_volumetric_speed"][6]))
for e in errors:
    print("FAIL: %s: %s" % (sys.argv[3], e))
sys.exit(1 if errors else 0)
EOF
    then
        tail -n 40 "$WORK/$1/log"
        exit 1
    fi
}

slice saved "$PROJECT"
check saved "$PROJECT"
slice unlisted "$WORK/unlisted.3mf"
check unlisted "$WORK/unlisted.3mf"

# Off takes the depth of the process preset (fdm_process_dual_0.10_nozzle_0.2: 7 top, 5 bottom).
slice off "$WORK/off.3mf"
"$PY" - "$WORK/off/out.3mf" <<'EOF' || { tail -n 40 "$WORK/off/log"; exit 1; }
import sys, zipfile

with zipfile.ZipFile(sys.argv[1]) as z:
    lines = z.read("Metadata/plate_1.gcode").decode("utf-8", "replace").splitlines()
sliced = dict(line[2:].split(" = ", 1) for line in lines if line.startswith("; ") and " = " in line)
errors = ["%s is %r, expected %r" % (key, sliced.get(key), value)
          for key, value in (("top_color_penetration_layers", "7"), ("bottom_color_penetration_layers", "5"))
          if sliced.get(key) != value]
for e in errors:
    print("FAIL: off: %s" % e)
sys.exit(1 if errors else 0)
EOF

# The old process name and the current one give the same refreshed density, and not the stale one.
slice renamed "$WORK/renamed.3mf"
slice current "$WORK/current.3mf"
"$PY" - "$WORK/renamed/out.3mf" "$WORK/current/out.3mf" <<'EOF' || { tail -n 40 "$WORK/renamed/log"; exit 1; }
import sys, zipfile

def density(path):
    with zipfile.ZipFile(path) as z:
        for line in z.read("Metadata/plate_1.gcode").decode("utf-8", "replace").splitlines():
            if line.startswith("; sparse_infill_density = "):
                return line.split(" = ", 1)[1]
    return None

renamed, current = density(sys.argv[1]), density(sys.argv[2])
if current in (None, "37%"):
    print("FAIL: current: sparse_infill_density is %r, expected the preset's own value" % current)
    sys.exit(1)
if renamed != current:
    print("FAIL: renamed: sparse_infill_density is %r, expected %r as under the current process name" % (renamed, current))
    sys.exit(1)
EOF
echo "PASS"
