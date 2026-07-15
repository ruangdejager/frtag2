# create_release_hex_files.ps1
#
# frtag2 post-build script. Runs after every CubeIDE build (Debug or
# Release) and produces the versioned release artifacts alongside the raw
# ELF/BIN/HEX CubeIDE emits.
#
# What it produces:
#
#   In the build's own $mode\ folder (e.g. Debug\) — versioned artifacts
#   only, matching CubeIDE's own naming convention:
#
#     frtag2_<M>_<m>_<p>.bin           app image, raw binary
#     frtag2_<M>_<m>_<p>.hex           same image as Intel HEX (manual programming)
#     frtag2_<M>_<m>_<p>_bl_<b>.bin    app + bootloader, combined image for
#     frtag2_<M>_<m>_<p>_bl_<b>.hex    initial programming from a debugger
#     frtag2_<M>_<m>_<p>_ota.bin       the image OTA actually ships (== app
#     frtag2_<M>_<m>_<p>_ota.hex       image today; kept as its own name so
#                                      the OTA artifact stays unambiguous even
#                                      if that ever diverges from the app image)
#
#   In C:\Code\farmranger-firmware\frtag2-firmware\ (the public OTA host
#   repo) — the same three versioned pairs, PLUS:
#
#     latest_bl.bin / latest_bl.hex            copy of the bootloader-only image
#     latest_app_bl.bin / latest_app_bl.hex    copy of the combined image
#     latest_ota.bin / latest_ota.hex          copy of the app-only/OTA image
#     latest.bin / latest.hex                  what the OTA flow actually
#                                              fetches (https://<user>.github.io/
#                                              .../latest.bin) — same bytes
#                                              as latest_ota.*
#     version.json                             OTA manifest — {"version","file","size","crc32"}
#
# For a first-time programming of a fresh MCU: flash frtag2_<M>_<m>_<p>_bl_<b>.hex
# (or latest_app_bl.hex) — it contains bootloader + application. After that,
# every OTA update ships the app-only image and the resident bootloader
# handles programming from external flash.
#
# The bootloader-combined images are NOT published as the OTA endpoint —
# only the app-only image is (a bootloader must never write itself over an
# OTA link because a failed transfer bricks the device).
#
# Version numbers come from:
#   frtag2:            fr_app/inc/config/version_config.h (VERSION_SW_MAJOR/MINOR/PATCH)
#   frtag2_bootloader: ../frtag2_bootloader/fr_bootloader/src/main.c (FRTAG_BL_VER)
# Same conventions as fr9_application/create_release_hex_files.ps1.

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

# Destination: the public OTA host repo (may not exist yet on first run).
$publish_dir = "C:\Code\farmranger-firmware\frtag2-firmware"
if (-Not (Test-Path $publish_dir)) {
    Write-Host "Publish dir $publish_dir does not exist - creating it (repo not initialised locally?)"
    New-Item -ItemType Directory -Path $publish_dir -Force | Out-Null
}

# ---- Versioned app artifacts (bin + hex) — Debug\ only ----------------------
$app_bin_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}.bin"
$app_hex_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}.hex"
Copy-Item $bin_file_path -Destination $app_bin_versioned -Force
Copy-Item $hex_file_path -Destination $app_hex_versioned -Force

# ---- Versioned OTA artifacts (bin + hex) — Debug\ only -----------------------
# Byte-identical to the app image today; kept as its own versioned name so
# "the file OTA ships" stays unambiguous if that ever diverges.
$ota_bin_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}_ota.bin"
$ota_hex_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}_ota.hex"
Copy-Item $bin_file_path -Destination $ota_bin_versioned -Force
Copy-Item $hex_file_path -Destination $ota_hex_versioned -Force

# ---- Combined bootloader + app artifacts (bin + hex) — Debug\ only ----------
# The combined image lives in one file for initial programming from a
# debugger; not sent over OTA (see file header).
if ($has_bootloader) {
    $bl_hex_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.hex"
    $bl_bin_versioned = "$modeDir\frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.bin"
    $bl_bin_filename  = $bl_hex_filename -replace '\.hex$', '.bin'

    # Combined HEX: splice bootloader hex over the app hex's EOF record.
    $app_hex_raw = (Get-Content $hex_file_path -Raw).Trim()
    $bl_hex_raw  = (Get-Content $bl_hex_filename -Raw).Trim()
    $combined_hex = $app_hex_raw -replace ":00000001FF", $bl_hex_raw
    Set-Content -Path $bl_hex_versioned -Value $combined_hex

    # Combined BIN: the app image (from OTA_APP_BASE_ADDR = 0x08005000) is
    # offset 20 KB into the internal-flash address space. Place the
    # bootloader bin at offset 0 and the app bin at offset 0x5000, padding
    # any gap with 0xFF (erased-flash value) so a raw-binary flasher can
    # write the whole thing at 0x08000000 in one shot.
    $BOOTLOADER_SIZE = 0x5000
    $bl_bin_bytes    = [System.IO.File]::ReadAllBytes($bl_bin_filename)
    $app_bin_bytes   = [System.IO.File]::ReadAllBytes($bin_file_path)

    if ($bl_bin_bytes.Length -gt $BOOTLOADER_SIZE) {
        Write-Host ("WARNING: bootloader binary is {0} B, exceeds reserved {1} B" -f $bl_bin_bytes.Length, $BOOTLOADER_SIZE)
    }

    $combined_len   = $BOOTLOADER_SIZE + $app_bin_bytes.Length
    $combined_bytes = New-Object 'byte[]' $combined_len
    for ($i = 0; $i -lt $combined_len; $i++) { $combined_bytes[$i] = 0xFF }
    [System.Array]::Copy($bl_bin_bytes,  0, $combined_bytes, 0,               $bl_bin_bytes.Length)
    [System.Array]::Copy($app_bin_bytes, 0, $combined_bytes, $BOOTLOADER_SIZE, $app_bin_bytes.Length)
    [System.IO.File]::WriteAllBytes($bl_bin_versioned, $combined_bytes)
}

# ---- Publish to farmranger-firmware\frtag2-firmware\ ------------------------
# The OTA endpoint the device fetches: version.json + latest.bin (+ .hex).
# Everything below is farmranger-firmware only — none of the "latest_*"
# names are written into Debug\/Release\, which only ever holds the
# versioned frtag2_<M>_<m>_<p>[_ota|_bl_<b>] pairs above.
Copy-Item $bin_file_path -Destination "$publish_dir\latest.bin" -Force
Copy-Item $hex_file_path -Destination "$publish_dir\latest.hex" -Force

Copy-Item $bin_file_path -Destination "$publish_dir\latest_ota.bin" -Force
Copy-Item $hex_file_path -Destination "$publish_dir\latest_ota.hex" -Force

Copy-Item $app_bin_versioned -Destination "$publish_dir\frtag2_${Major}_${Minor}_${Patch}.bin" -Force
Copy-Item $app_hex_versioned -Destination "$publish_dir\frtag2_${Major}_${Minor}_${Patch}.hex" -Force
Copy-Item $ota_bin_versioned -Destination "$publish_dir\frtag2_${Major}_${Minor}_${Patch}_ota.bin" -Force
Copy-Item $ota_hex_versioned -Destination "$publish_dir\frtag2_${Major}_${Minor}_${Patch}_ota.hex" -Force

if ($has_bootloader) {
    Copy-Item $bl_hex_filename  -Destination "$publish_dir\latest_bl.hex" -Force
    Copy-Item $bl_bin_filename  -Destination "$publish_dir\latest_bl.bin" -Force
    Copy-Item $bl_bin_versioned -Destination "$publish_dir\latest_app_bl.bin" -Force
    Copy-Item $bl_hex_versioned -Destination "$publish_dir\latest_app_bl.hex" -Force

    Copy-Item $bl_bin_versioned -Destination "$publish_dir\frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.bin" -Force
    Copy-Item $bl_hex_versioned -Destination "$publish_dir\frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.hex" -Force
}

# ---- version.json (OTA manifest) --------------------------------------------
# Points at latest.bin — device holds the base URL, appends the filename.
# CRC32 over the raw bin: uppercase hex per the handover doc.
$latest_bin_path = "$publish_dir\latest.bin"
$latest_bin_bytes = [System.IO.File]::ReadAllBytes($latest_bin_path)

# Compute CRC32 (IEEE 802.3, poly 0xEDB88320) without any extra dependency.
# NOTE: PowerShell 5.1 parses 8-digit hex literals >= 0x80000000 as a
# negative Int32 (the bit pattern), not UInt32 - so the poly and the
# all-ones seed/mask are built via [Convert]::ToUInt32(hexstring, 16)
# instead of writing 0xEDB88320 / 0xFFFFFFFF directly.
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
foreach ($b in $latest_bin_bytes) {
    $crc = ($crc -shr 8) -bxor $crc32Table[(($crc -bxor $b) -band 0xFF)]
}
$crc = $crc -bxor $CRC32_ONES
$crc_hex = ("{0:X8}" -f $crc)

# Streamed 8-bit XOR fold — matches the algorithm every downstream hop
# (fr9 verify, frtag2 primary, bootloader) already computes over the same
# bytes, so this is the one value comparable end-to-end without decoding.
$xor8 = 0
foreach ($b in $latest_bin_bytes) { $xor8 = $xor8 -bxor $b }
$xor8_hex = ("{0:X2}" -f $xor8)

$manifest = [ordered]@{
    version = $version_str
    file    = "latest.bin"
    size    = $latest_bin_bytes.Length
    crc32   = $crc_hex
    xor     = $xor8_hex
}
$manifest_json = ($manifest | ConvertTo-Json)
Set-Content -Path "$publish_dir\version.json" -Value $manifest_json

Write-Host "Debug\ (versioned only):"
Write-Host "  frtag2_${Major}_${Minor}_${Patch}.{bin,hex}"
Write-Host "  frtag2_${Major}_${Minor}_${Patch}_ota.{bin,hex}"
if ($has_bootloader) {
    Write-Host "  frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.{bin,hex}"
}
Write-Host "Published to $publish_dir :"
Write-Host "  latest.bin  ($($latest_bin_bytes.Length) B, CRC32=$crc_hex, XOR8=$xor8_hex)"
Write-Host "  latest.hex, latest_ota.*$(if ($has_bootloader) { ', latest_bl.*, latest_app_bl.*' })"
Write-Host "  version.json (v${version_str})"
Write-Host "OK"
