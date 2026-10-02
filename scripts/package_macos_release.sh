#!/usr/bin/env bash
# Package a built OrcaSlicer.app (Cadence Slicer, a mixed-nozzle fork of OrcaSlicer) into a redistributable, sealed,
# signed DMG that runs on any supported Mac -- locally, without GitHub's runners.
#
# Mirrors .github/workflows/build_orca.yml "Sign app and notary" and the runtime relocation in
# build_release_macos.sh, so a local release is byte-for-byte the same shape as a CI release.
#
# Usage:
#   scripts/package_macos_release.sh --app <path/to/OrcaSlicer.app> --version <label> --out <dir>
#                                    [--identity "Developer ID Application: Name (TEAMID)"]
#                                    [--notarize-profile <notarytool keychain profile>]
#                                    [--volname <DMG volume name>] [--name <artifact base name>]
#
# Without --identity the bundle is signed ad hoc ("-"): it is sealed and internally consistent,
# but Gatekeeper will still block it on other Macs until the user allows it in
# System Settings > Privacy & Security ("Open Anyway"). A public release needs --identity
# (Developer ID, hardened runtime) and --notarize-profile (notarytool --keychain-profile).
# Credentials never appear on the command line; notarytool reads them from the keychain profile
# created once with:  xcrun notarytool store-credentials <profile> --apple-id ... --team-id ...
#
# The source .app is never modified: everything happens on a copy under --out.
set -euo pipefail

APP="" VERSION="" OUT="" IDENTITY="-" NOTARIZE_PROFILE="" VOLNAME="" NAME=""
while [ $# -gt 0 ]; do
    case "$1" in
        --app) APP="$2"; shift 2 ;;
        --version) VERSION="$2"; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --identity) IDENTITY="$2"; shift 2 ;;
        --notarize-profile) NOTARIZE_PROFILE="$2"; shift 2 ;;
        --volname) VOLNAME="$2"; shift 2 ;;
        --name) NAME="$2"; shift 2 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[ -n "$APP" ] && [ -n "$VERSION" ] && [ -n "$OUT" ] || { echo "--app, --version and --out are required" >&2; exit 2; }
[ -d "$APP" ] && [ -f "$APP/Contents/Info.plist" ] || { echo "not an app bundle: $APP" >&2; exit 2; }
APP_BUNDLE="$(basename "$APP")"
APP_NAME="${APP_BUNDLE%.app}"
EXE_NAME="$(/usr/libexec/PlistBuddy -c 'Print :CFBundleExecutable' "$APP/Contents/Info.plist")"
[ -f "$APP/Contents/MacOS/$EXE_NAME" ] || { echo "missing bundle executable: $EXE_NAME" >&2; exit 2; }
NAME="${NAME:-$APP_NAME}"
VOLNAME="${VOLNAME:-$APP_NAME}"
[ -z "$NOTARIZE_PROFILE" ] || [ "$IDENTITY" != "-" ] || { echo "--notarize-profile requires --identity (Apple rejects ad-hoc signatures)" >&2; exit 2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENTITLEMENTS="$SCRIPT_DIR/disable_validation.entitlements"
[ -f "$ENTITLEMENTS" ] || { echo "missing $ENTITLEMENTS" >&2; exit 2; }
# shellcheck source=scripts/retry.sh
source "$SCRIPT_DIR/retry.sh"

ARCHS="$(lipo -archs "$APP/Contents/MacOS/$EXE_NAME" | tr ' ' '-')"
ARTIFACT="${NAME}_Mac_${ARCHS}_${VERSION}"
STAGE="$OUT/stage"
DMG_DIR="$STAGE/dmg"
LOG="$OUT/package-${VERSION}.log"
mkdir -p "$OUT"
rm -rf "$STAGE"
mkdir -p "$DMG_DIR"
exec > >(tee "$LOG") 2>&1
echo "== package_macos_release: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "   source app : $APP"
echo "   version    : $VERSION   archs: $ARCHS   identity: $IDENTITY   notarize: ${NOTARIZE_PROFILE:-no}"

# 1. Work on a copy; drop Finder metadata.
cp -pR "$APP" "$DMG_DIR/$APP_BUNDLE"
find "$DMG_DIR/$APP_BUNDLE" -name '.DS_Store' -delete
xattr -cr "$DMG_DIR/$APP_BUNDLE"
app="$DMG_DIR/$APP_BUNDLE"
exe="$app/Contents/MacOS/$EXE_NAME"

# 2. Relocate the bundled Python runtime (build_release_macos.sh relocate_python_runtime):
#    codesign cannot seal dotted directories under Contents/MacOS.
if [ -d "$app/Contents/MacOS/python" ] && [ ! -L "$app/Contents/MacOS/python" ]; then
    rm -rf "$app/Contents/Resources/python"
    mv "$app/Contents/MacOS/python" "$app/Contents/Resources/python"
    ln -s ../Resources/python "$app/Contents/MacOS/python"
    echo "   relocated Contents/MacOS/python -> Contents/Resources/python (+ symlink)"
fi

# 3. Remove build-machine rpaths: only @executable_path/@loader_path entries may remain.
while IFS= read -r rp; do
    case "$rp" in
        @executable_path/*|@loader_path/*|@rpath/*) ;;
        *) install_name_tool -delete_rpath "$rp" "$exe"; echo "   removed build-machine rpath: $rp" ;;
    esac
done < <(otool -l "$exe" | awk '/LC_RPATH/{f=1} f&&/path /{print $2; f=0}')
if otool -L "$exe" | tail -n +2 | grep -q -v -E '^\s*(/usr/lib|/System/Library|@rpath|@executable_path|@loader_path)'; then
    echo "ERROR: executable links libraries outside the bundle/system:" >&2
    otool -L "$exe" | tail -n +2 | grep -v -E '^\s*(/usr/lib|/System/Library|@rpath|@executable_path|@loader_path)' >&2
    exit 1
fi

# 4. Sign every Mach-O explicitly (codesign --deep is deprecated and skips the Python runtime),
#    then seal the bundle last, then verify strictly so gaps fail here, not at the user's Mac.
sign_args=(--force --sign "$IDENTITY" --entitlements "$ENTITLEMENTS")
if [ "$IDENTITY" != "-" ]; then sign_args+=(--options runtime --timestamp); fi
machos=()
while IFS= read -r f; do machos+=("$f"); done < <(
    find "$app" -type f -print0 |
        xargs -0 file --no-pad --mime-type -- |
        grep -v ' (for architecture ' |
        grep ': application/x-mach-binary$' |
        sed 's|: application/x-mach-binary$||'
)
[ "${#machos[@]}" -gt 0 ] || { echo "ERROR: no Mach-O files found in $app" >&2; exit 1; }
echo "   signing ${#machos[@]} Mach-O files"
codesign "${sign_args[@]}" "${machos[@]}"
codesign "${sign_args[@]}" --verbose "$app"
codesign --verify --deep --strict --verbose=2 "$app"
echo "   codesign --verify --deep --strict: OK"

# 5. DMG with an /Applications drop target, signed like the app.
ln -sfn /Applications "$DMG_DIR/Applications"
DMG="$OUT/${ARTIFACT}.dmg"
rm -f "$DMG"
retry hdiutil create -volname "$VOLNAME" -srcfolder "$DMG_DIR" -ov -format UDZO "$DMG"
codesign --force --sign "$IDENTITY" "$DMG"

# 6. Optional notarization + stapling (Developer ID only).
if [ -n "$NOTARIZE_PROFILE" ]; then
    xcrun notarytool submit "$DMG" --keychain-profile "$NOTARIZE_PROFILE" --wait
    xcrun stapler staple "$DMG"
    echo "   notarized and stapled"
fi

# 7. Gatekeeper assessment (informational: expected 'rejected' for ad-hoc, 'accepted' when notarized).
spctl --assess --type open --context context:primary-signature -v "$DMG" 2>&1 | sed 's/^/   spctl dmg: /' || true

# 8. Checksum and install notes.
( cd "$OUT" && shasum -a 256 "$(basename "$DMG")" > "${ARTIFACT}.sha256" )
cat > "$OUT/${ARTIFACT}.INSTALL.md" <<EOF
# ${ARTIFACT}

${APP_NAME}, a mixed-nozzle fork of OrcaSlicer. Upstream attribution and GNU AGPLv3 notices are included. Architecture: ${ARCHS}. Minimum macOS: $(otool -l "$exe" | awk '/LC_BUILD_VERSION/{f=1} f&&/minos/{print $2; exit}').

1. Open the DMG and drag **$APP_NAME** to **Applications**.
2. $(if [ -n "$NOTARIZE_PROFILE" ]; then echo "The app is notarized: it opens normally."; elif [ "$IDENTITY" != "-" ]; then echo "The app is Developer ID signed but not notarized: on first launch macOS may ask you to confirm in System Settings > Privacy & Security."; else echo "This build is NOT Developer ID signed. On first launch macOS will refuse it; open **System Settings > Privacy & Security**, scroll to the message about $APP_NAME and click **Open Anyway**, then launch again. (Terminal alternative: \`xattr -dr com.apple.quarantine /Applications/$APP_BUNDLE\`.)"; fi)
3. In the first-run wizard keep **preset sync off** until you have confirmed the Mixed-Nozzle Body presets appear; a cloud sync can replace the bundled printer profiles.

SHA-256: $(cut -d' ' -f1 "$OUT/${ARTIFACT}.sha256")
EOF
rm -rf "$STAGE"
echo "== done"
echo "   dmg     : $DMG"
echo "   sha256  : $(cut -d' ' -f1 "$OUT/${ARTIFACT}.sha256")"
echo "   install : $OUT/${ARTIFACT}.INSTALL.md"
echo "   log     : $LOG"
