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
#   In EACH OTA host repo's frtag2-firmware\ folder - exactly two files:
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
#   BIN in the public repo — the OTA path only ever fetches the app-only
#   BIN, and its filename comes from version.json's "file" field (fr9's
#   TAGFOTA_pacManifestFile returns whatever the manifest names).
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

# Destinations: the OTA host repos (either may not exist yet on a first run).
# Field host first, in-house bench host second - identical artefacts in both.
$publish_dirs = @(
    "C:\Code\farmranger-firmware\frtag2-firmware",     # field units
    "C:\Code\frtag-firmware-inhouse\frtag2-firmware"   # in-house bench units
)
foreach ($d in $publish_dirs) {
    if (-Not (Test-Path $d)) {
        Write-Host "Publish dir $d does not exist - creating it (repo not initialised locally?)"
        New-Item -ItemType Directory -Path $d -Force | Out-Null
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

# ---- Publish the .bin to every host repo ------------------------------------
# ONE file per repo: the versioned app-only .bin the OTA flow actually
# fetches. version.json below names it explicitly so fr9's manifest parser
# (TAGFOTA_pacManifestFile) picks it up as-is - no "latest.bin" fallback.
$bin_name = "frtag2_${Major}_${Minor}_${Patch}.bin"
foreach ($d in $publish_dirs) {
    Copy-Item $bin_file_path -Destination "$d\$bin_name" -Force
}
$app_bin_published = "$publish_dir\$bin_name"

# Read the just-published bin for checksums (identical bytes to the source
# bin, but read from the destination in case anything went wrong in the copy).
$app_bin_bytes = [System.IO.File]::ReadAllBytes($app_bin_published)

# ---- version.json (OTA manifest) --------------------------------------------
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
foreach ($d in $publish_dirs) {
    Set-Content -Path "$d\version.json" -Value $manifest_json
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
foreach ($d in $publish_dirs) {
    Write-Host "Published to $d :"
    Write-Host "  $bin_name  ($($app_bin_bytes.Length) B, CRC32=$crc_hex, XOR8=$xor8_hex)"
    Write-Host "  version.json (v${version_str})"
}
Write-Host "Neither host repo is committed or pushed by this script - do that by hand."
Write-Host "OK"
