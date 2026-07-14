# This script runs after the normal build has run. It creates the hex files
# required for a full release. The script appends the version number from
# fr_app/inc/config/version_config.h, appends the bootloader, and produces
# the OTA hex (VS,<linecount> + Intel HEX text) the LoRa OTA chain expects
# (HexDecode.c / OtaUpdate.c on the receiving end).
#
# Mirrors fr9_application/build_scripts/create_release_hex_files.ps1 —
# see that file for the original pattern this was ported from.

#    [string]$mode = "Debug"
param (
    [string]$mode
 )

$scriptpath = $MyInvocation.MyCommand.Path
$dir = Split-Path $scriptpath

$hex_file_path = Resolve-Path "$dir\..\$mode\frtag2.hex"

# Check the file exists
if (-Not (Test-Path $hex_file_path)){
	Write-Host "Could not found firmware hex file, is it compiled?"
	exit
}

# Find the version number of the latest bootloader. frtag2 and
# frtag2_bootloader are both built to their own Debug/ folder (unlike
# fr9_bootloader's Release/ output) — use the same $mode for both.
$bootloader_dir = "$dir\..\..\frtag2_bootloader"
$bl_version_path = Resolve-Path "$bootloader_dir\fr_bootloader\src\main.c"
$bl_version = Select-String $bl_version_path -pattern '_BL_VER\s+(\d+)' | Foreach-Object {$_.Matches.Groups[1].Value}

# Find the app version number from the file
$version_h_path = Resolve-Path "$dir\..\fr_app\inc\config\version_config.h"
$Major = Select-String $version_h_path -pattern '_MAJOR\s+(\d+)' | Foreach-Object {$_.Matches.Groups[1].Value}
$Minor = Select-String $version_h_path -pattern '_MINOR\s+(\d+)' | Foreach-Object {$_.Matches.Groups[1].Value}
$Patch = Select-String $version_h_path -pattern '_PATCH\s+(\d+)' | Foreach-Object {$_.Matches.Groups[1].Value}
Write-Host "Detected version is ${Major}_${Minor}_${Patch}_bl_${bl_version}"

# Configure all the paths
$firmware_filename = "$dir\..\$mode\frtag2_${Major}_${Minor}_${Patch}.hex"
$ota_firmware_filename = "$dir\..\$mode\frtag2_${Major}_${Minor}_${Patch}_ota.hex"
$bl_firmware_filename = "$dir\..\$mode\frtag2_${Major}_${Minor}_${Patch}_bl_${bl_version}.hex"
$bl_hex_filename = "$bootloader_dir\$mode\frtag2_bootloader.hex"

# Check the bootloader hex exists too
if (-Not (Test-Path $bl_hex_filename)){
	Write-Host "Could not find bootloader hex file at $bl_hex_filename, is frtag2_bootloader compiled?"
	exit
}

# Copy the firmware hex to three different files
Copy-Item $hex_file_path -Destination $firmware_filename
Copy-Item $hex_file_path -Destination $ota_firmware_filename
Copy-Item $hex_file_path -Destination $bl_firmware_filename

# Append the linecount to the OTA firmware (OTA_SCRATCH decode contract:
# "VS,<linecount>\r\n" + Intel HEX text — see OtaStore_Config.h)
$ota_linecount = Get-Content $ota_firmware_filename | Measure-Object -Line | Select-Object -expand Lines
$ota_begin_string = "VS,${ota_linecount}`r`n"
$ota_firmware = $ota_begin_string + (Get-Content $ota_firmware_filename -Raw)
$ota_firmware = $ota_firmware.Trim()
$_ = Set-Content -Path $ota_firmware_filename -Value $ota_firmware
Write-Host "Wrote linecount of ${ota_linecount} to OTA firmware"

# Append the bootloader to the BL firmware (combined image for a full-chip
# initial flash: app records + bootloader records in one hex file)
$bl_firmware = (Get-Content $bl_firmware_filename -Raw)
$bl_firmware = $bl_firmware.Trim()
$bl_hex = (Get-Content $bl_hex_filename -Raw)
$bl_hex = $bl_hex.Trim()
# Remove the app hex's EOF record and splice in the bootloader hex there
$bl_firmware = $bl_firmware -replace ":00000001FF", $bl_hex
$_ = Set-Content -Path $bl_firmware_filename -Value $bl_firmware
Write-Host "Appended bootloader hex to firmware"
