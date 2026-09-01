# create_release_hex_files.ps1
#
# frtag2 post-build script. Runs after every CubeIDE build (Debug or
# Release) and produces the two release artefacts we actually care about,
# nothing more.
#
# What it produces:
#
#   In the build's own $mode\ folder (e.g. Debug\) — exactly two files:
#
#     frtag2_<M>_<m>_<p>.hex           app image only (Intel HEX; manual
#                                      programming of an already-loaded
#                                      bootloader)
#     frtag2_<M>_<m>_<p>_bl_<b>.hex    app + bootloader combined (Intel HEX;
#                                      first-time programming of a fresh MCU)
#
#   Also removes CubeIDE's stock frtag2.hex / frtag2.bin from $mode\ so the
#   folder only ever holds the versioned deliverables.
#
#   In each repo listed in $publish_dirs below (the public OTA host repo,
#   plus the in-house mirror) — exactly two files per repo:
#
#     frtag2_<M>_<m>_<p>.bin           the versioned app image (raw binary)
#                                      the OTA flow fetches over HTTP
#     version.json                     manifest naming the .bin above
#                                      {"version","file","size","crc32","xor"}
#
#   The host repos are:
#
#     C:\Code\farmranger-firmware\frtag2-firmware\      field units
#     C:\Code\frtag-firmware-inhouse\frtag2-firmware\   in-house bench units
#
#   Both get byte-identical artefacts. Which host a unit actually fetches
#   from is decided at fr9 build time by SWITCH_TAGFOTA_INHOUSE_HOST
#   (fr9_application, fr_app/inc/tag_fota.h). Publishing here is only a copy
#   into a local working tree - nothing reaches a device until that repo is
#   committed and pushed, so the field host stays under exactly the manual
#   control it always had.
#
#   NO latest*.bin/hex, NO _ota.* variants, NO combined _bl_* copies of the
#   BIN in either publish repo — the OTA path only ever fetches the app-only
#   BIN, and its filename comes from version.json's "file" field (fr9's
#   TAGFOTA_pacManifestFile returns whatever the manifest names).
#
#   This script only ever writes files into those repos' working trees. It
#   never runs git add/commit/push there, and never will — committing and
#   pushing C:\Code\farmranger-firmware and C:\Code\frtag-firmware-inhouse
#   is a manual, deliberate step for a human, not something a build step
#   does on every compile.
#
# For a first-time programming of a fresh MCU: flash frtag2_<M>_<m>_<p>_bl_<b>.hex
# — contains bootloader + application. After that every OTA update ships
# the app-only .bin and the resident bootloader handles the reprogram.
#
# Version numbers come from:
#   frtag2:            fr_app/inc/config/version_config.h (VERSION_SW_MAJOR/MINOR/PATCH)
#   frtag2_bootloader: ../frtag2_bootloader/fr_bootloader/src/main.c (FRTAG_BL_VER)

param (
    [string]$mode = "Debug"
 )

$ErrorActionPreference = "Stop"

$scriptpath = $MyInvocation.MyCommand.Path
$dir        = Split-Path $scriptpath
$modeDir    = "$dir\..\$mode"

# The app hex and bin CubeIDE just built.
$hex_file_path = Resolve-Path "$modeDir\frtag2.hex"
$bin_file_path = Resolve-Path "$modeDir\frtag2.bin"
if (-Not (Test-Path $hex_file_path)) {
    Write-Host "Could not find frtag2.hex, is it compiled?"; exit 1
}
if (-Not (Test-Path $bin_file_path)) {
    Write-Host "Could not find frtag2.bin, is it compiled? (Enable 'Convert to binary' in the project's build settings.)"; exit 1
}

# Bootloader hex - same $mode, same layout as frtag2 (both Debug/).
$bootloader_dir  = "$dir\..\..\frtag2_bootloader"
$bl_hex_filename = "$bootloader_dir\$mode\frtag2_bootloader.hex"
$bl_version_path = Resolve-Path "$bootloader_dir\fr_bootloader\src\main.c"
$bl_version = Select-String $bl_version_path -pattern '_BL_VER\s+(\d+)' | Foreach-Object { $_.Matches.Groups[1].Value }

if (-Not (Test-Path $bl_hex_filename)) {
    Write-Host "Could not find bootloader hex at $bl_hex_filename - is frtag2_bootloader compiled in $mode?"
    Write-Host "The combined 'bl' artifact will be skipped; app-only artifacts still produced."
    $has_bootloader = $false
} else {
    $has_bootloader = $true
}

# App version from version_config.h.
$version_h_path = Resolve-Path "$dir\..\fr_app\inc\config\version_config.h"
$Major = Select-String $version_h_path -pattern '_MAJOR\s+(\d+)' | Foreach-Object { $_.Matches.Groups[1].Value }
$Minor = Select-String $version_h_path -pattern '_MINOR\s+(\d+)' | Foreach-Object { $_.Matches.Groups[1].Value }
$Patch = Select-String $version_h_path -pattern '_PATCH\s+(\d+)' | Foreach-Object { $_.Matches.Groups[1].Value }
$version_str = "${Major}.${Minor}.${Patch}"

Write-Host "frtag2 release: v${version_str} + bootloader v${bl_version} (mode=$mode)"

# Destinations: every repo that mirrors the app-only .bin + version.json.
# Both are separate local git repos this script only ever writes files
# into - see the file-header note on why nothing here ever commits or
# pushes them.
$publish_dirs = @(
    "C:\Code\farmranger-firmware\frtag2-firmware",
    "C:\Code\frtag-firmware-inhouse\frtag2-firmware"
)
foreach ($publish_dir in $publish_dirs) {
    if (-Not (Test-Path $publish_dir)) {
        Write-Host "Publish dir $publish_dir does not exist - creating it (repo not initialised locally?)"
        New-Item -ItemType Directory -Path $publish_dir -Force | Out-Null
    }
}
# Checksums below are read back off the first copy; the others are
# byte-identical copies of the same source bin.
$publish_dir = $publish_dirs[0]

# ---- Versioned app-only HEX in Debug\ (Intel HEX for manual programming) ----
$app_hex_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}.hex"
Copy-Item $hex_file_path -Destination $app_hex_versioned -Force

# ---- Combined bootloader + app HEX in Debug\ (first-time programming) -------
$bl_hex_versioned = $null
if ($has_bootloader) {
    $bl_hex_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.hex"

    # Splice bootloader hex over the app hex's EOF record so the combined
    # file loads with a single "program" action from any HEX flasher.
    $app_hex_raw  = (Get-Content $hex_file_path -Raw).Trim()
    $bl_hex_raw   = (Get-Content $bl_hex_filename -Raw).Trim()
    $combined_hex = $app_hex_raw -replace ":00000001FF", $bl_hex_raw
    Set-Content -Path $bl_hex_versioned -Value $combined_hex
}

# ---- Checksums, computed once against the bin CubeIDE just built ------------
# Same bytes get published to every repo in $publish_dirs, so the CRC/XOR are
# computed once here rather than once per destination.
$app_bin_bytes = [System.IO.File]::ReadAllBytes($bin_file_path)

# ---- Identity gate: does the image agree with the version we are labelling it?
# The linker KEEPs a .fw_info record (major/minor/patch, uint16 LE) at image
# offset 0x200. The tag bootloader reads it to decide what is installed, and
# both fr9 and the tag app now check it before adopting or installing an image.
# Verifying it HERE means a mislabelled artifact cannot be published at all.
#
# Worth the six bytes: on 2026-09-01, 2.1.7/2.1.8/2.1.9 all built to exactly
# 109068 B with an identical xor8 of 0xC5, fr9's "already on the filesystem"
# check is (size, xor8), so it adopted the previous release and served it under
# the new version number. Tags installed it, booted reporting the old version,
# were offered the "new" one again, and looped until a human intervened. A
# stale or wrongly-named bin reaching this script would do the same thing.
$fw_info_offset = 0x200
if ($app_bin_bytes.Length -lt ($fw_info_offset + 6)) {
    Write-Host "ABORT: bin is only $($app_bin_bytes.Length) B - too small to hold fw_info at 0x200"
    exit 1
}
$bin_major = [uint16]$app_bin_bytes[$fw_info_offset]     + ([uint16]$app_bin_bytes[$fw_info_offset + 1] -shl 8)
$bin_minor = [uint16]$app_bin_bytes[$fw_info_offset + 2] + ([uint16]$app_bin_bytes[$fw_info_offset + 3] -shl 8)
$bin_patch = [uint16]$app_bin_bytes[$fw_info_offset + 4] + ([uint16]$app_bin_bytes[$fw_info_offset + 5] -shl 8)
$bin_version_str = "${bin_major}.${bin_minor}.${bin_patch}"

if ($bin_version_str -ne $version_str) {
    Write-Host "ABORT: version mismatch - version_config.h says $version_str but the built"
    Write-Host "       bin's fw_info record says $bin_version_str."
    Write-Host "       $bin_file_path is stale or is not the image for this version."
    Write-Host "       Rebuild in CubeIDE before publishing; nothing has been written."
    exit 1
}
Write-Host "fw_info check OK: bin declares v$bin_version_str, matching version_config.h"

# CRC32 (IEEE 802.3, poly 0xEDB88320) computed manually to avoid PowerShell
# hex-literal quirks (>= 0x80000000 parses as negative Int32; use ToUInt32).
$CRC32_POLY = [Convert]::ToUInt32("EDB88320", 16)
$CRC32_ONES = [Convert]::ToUInt32("FFFFFFFF", 16)

$crc32Table = New-Object 'uint32[]' 256
for ($i = 0; $i -lt 256; $i++) {
    $c = [uint32]$i
    for ($k = 0; $k -lt 8; $k++) {
        if ($c -band 1) { $c = ($c -shr 1) -bxor $CRC32_POLY } else { $c = $c -shr 1 }
    }
    $crc32Table[$i] = $c
}
$crc = $CRC32_ONES
foreach ($b in $app_bin_bytes) {
    $crc = ($crc -shr 8) -bxor $crc32Table[(($crc -bxor $b) -band 0xFF)]
}
$crc = $crc -bxor $CRC32_ONES
$crc_hex = ("{0:X8}" -f $crc)

# Streamed 8-bit XOR fold — the algorithm every downstream hop (fr9 verify,
# frtag2 primary, bootloader) computes over the same bytes, so this is the
# value comparable end-to-end without decoding.
$xor8 = 0
foreach ($b in $app_bin_bytes) { $xor8 = $xor8 -bxor $b }
$xor8_hex = ("{0:X2}" -f $xor8)

$manifest = [ordered]@{
    version = $version_str
    file    = "frtag2_${Major}_${Minor}_${Patch}.bin"
    size    = $app_bin_bytes.Length
    crc32   = $crc_hex
    xor     = $xor8_hex
}
$manifest_json = ($manifest | ConvertTo-Json)

# ---- Publish to every repo in $publish_dirs ----------------------------------
# ONE file each: the versioned app-only .bin the OTA flow actually fetches,
# plus the version.json naming it. version.json's "file" field is what fr9's
# manifest parser (TAGFOTA_pacManifestFile) goes by — no "latest.bin"
# fallback, and both repos get byte-identical bin + manifest.
# ---- Collision warning against the manifest currently live -----------------
# fr9's "is this already on my filesystem?" check is (size, xor8). Two releases
# that agree on both are indistinguishable to it, and the older bytes get
# re-served under the newer version number - the 2026-09-01 install loop.
#
# fr9 and the tag now check the image's fw_info record as well, so a collision
# is harmless once BOTH are deployed. Until then it is still live ammunition,
# and this is the last point where a human can see it coming.
foreach ($publish_dir in $publish_dirs) {
    $live_manifest_path = "$publish_dir\version.json"
    if (Test-Path $live_manifest_path) {
        try {
            $live = Get-Content $live_manifest_path -Raw | ConvertFrom-Json
            if (($live.version -ne $version_str) -and
                ([uint32]$live.size -eq [uint32]$app_bin_bytes.Length) -and
                ($live.xor -eq $xor8_hex)) {
                Write-Host ""
                Write-Host "*** WARNING: (size, xor8) COLLISION with the live manifest ***"
                Write-Host "    $publish_dir"
                Write-Host "    live: v$($live.version)  size=$($live.size)  xor=$($live.xor)"
                Write-Host "    new:  v$version_str  size=$($app_bin_bytes.Length)  xor=$xor8_hex"
                Write-Host "    An fr9 that predates the fw_info identity check will treat the"
                Write-Host "    file it already has as this release and never download the new"
                Write-Host "    bytes. Confirm every fr9 in the fleet is updated before relying"
                Write-Host "    on this release reaching tags."
                Write-Host ""
            }
        } catch {
            Write-Host "Note: could not parse existing $live_manifest_path - skipping collision check"
        }
    }
}

foreach ($publish_dir in $publish_dirs) {
    $app_bin_published = "$publish_dir\frtag2_${Major}_${Minor}_${Patch}.bin"
    Copy-Item $bin_file_path -Destination $app_bin_published -Force

    # Verify the copy landed intact before trusting it as a deliverable.
    $published_len = (Get-Item $app_bin_published).Length
    if ($published_len -ne $app_bin_bytes.Length) {
        Write-Host "Publish to $publish_dir FAILED - size mismatch ($published_len vs $($app_bin_bytes.Length))"
        exit 1
    }

    Set-Content -Path "$publish_dir\version.json" -Value $manifest_json
}

# ---- Cleanup CubeIDE's stock output from Debug\ -----------------------------
# Only the two versioned HEX files are the deliverables. Leaving the stock
# frtag2.hex/frtag2.bin around invites confusion about which file to flash.
Remove-Item $hex_file_path -Force -ErrorAction SilentlyContinue
Remove-Item $bin_file_path -Force -ErrorAction SilentlyContinue

Write-Host "Debug\ (deliverables only):"
Write-Host "  frtag2_${Major}_${Minor}_${Patch}.hex"
if ($has_bootloader) {
    Write-Host "  frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.hex"
}
Write-Host "Published ($($app_bin_bytes.Length) B, CRC32=$crc_hex, XOR8=$xor8_hex):"
foreach ($publish_dir in $publish_dirs) {
    Write-Host "  $publish_dir\frtag2_${Major}_${Minor}_${Patch}.bin"
    Write-Host "  $publish_dir\version.json (v${version_str})"
}
Write-Host "Neither host repo is committed or pushed by this script - do that by hand."
Write-Host "OK"
