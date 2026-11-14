param(
    [switch]$Flash
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $repoRoot 'build-fw-stm32'
$programmer = 'C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
$appPath = Join-Path $buildDir 'ECU08.bin'
$sector0Backup = Join-Path $buildDir 'bootloader-sector0-backup.bin'
$sector7Backup = Join-Path $buildDir 'sector7-repair-readback.bin'
$patchedSector7 = Join-Path $buildDir 'sector7-with-current-app-metadata.bin'
$appReadback = Join-Path $buildDir 'app-preflight-readback.bin'
$sector0Readback = Join-Path $buildDir 'sector0-preflight-readback.bin'
$sector7Readback = Join-Path $buildDir 'sector7-preflight-readback.bin'
$sector7After = Join-Path $buildDir 'gpsfix-sector7-after.bin'
$sector0After = Join-Path $buildDir 'sector0-repair-readback.bin'
$appAfter = Join-Path $buildDir 'app-repair-readback.bin'

$sectorSize = 0x20000
$appBase = [uint32]0x08020000
$sector0Base = [uint32]0x08000000
$sector7Base = [uint32]0x080E0000
$metadataOffset = 0x1FFE0
$nvmLength = 0x1FFC0
$metadataMagic = [Convert]::ToUInt32('B007C0DE', 16)
$metadataVersion = [uint32]2

function Invoke-Programmer {
    param([string[]]$Arguments)

    & $programmer @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "STM32CubeProgrammer failed with exit code ${LASTEXITCODE}: $($Arguments -join ' ')"
    }
}

function Read-Flash {
    param(
        [uint32]$Address,
        [int]$Length,
        [string]$Path
    )

    $addressArg = '0x{0:X8}' -f $Address
    $lengthArg = '0x{0:X}' -f $Length
    Invoke-Programmer @('-c', 'port=SWD', '-u', $addressArg, $lengthArg, $Path)
}

function Assert-SameBytes {
    param(
        [byte[]]$Expected,
        [byte[]]$Actual,
        [string]$Description
    )

    if ($Expected.Length -ne $Actual.Length) {
        throw "$Description length mismatch: expected $($Expected.Length), got $($Actual.Length)."
    }

    for ($index = 0; $index -lt $Expected.Length; $index++) {
        if ($Expected[$index] -ne $Actual[$index]) {
            throw "$Description differs at offset 0x$('{0:X}' -f $index)."
        }
    }
}

function Get-Crc32 {
    param([byte[]]$Bytes)

    [uint32]$crc = [uint32]::MaxValue
    [uint32]$polynomial = [Convert]::ToUInt32('EDB88320', 16)
    foreach ($value in $Bytes) {
        $crc = [uint32]($crc -bxor [uint32]$value)
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 1) -ne 0) {
                $crc = [uint32](($crc -shr 1) -bxor $polynomial)
            }
            else {
                $crc = [uint32]($crc -shr 1)
            }
        }
    }

    return [uint32]($crc -bxor [uint32]::MaxValue)
}

function Restore-Sector7Backup {
    Write-Warning 'Restoring the original sector-7 backup.'
    Invoke-Programmer @('-c', 'port=SWD', '-e', '7', '-w', $sector7Backup, ('0x{0:X8}' -f $sector7Base), '-v')
    Read-Flash $sector7Base $sectorSize $sector7After
    Assert-SameBytes ([IO.File]::ReadAllBytes($sector7Backup)) ([IO.File]::ReadAllBytes($sector7After)) 'Restored sector 7'
}

foreach ($requiredFile in @($appPath, $sector0Backup, $sector7Backup)) {
    if (-not (Test-Path -LiteralPath $requiredFile -PathType Leaf)) {
        throw "Required input is missing: $requiredFile"
    }
}
if (-not (Test-Path -LiteralPath $programmer -PathType Leaf)) {
    throw "STM32CubeProgrammer was not found at $programmer"
}

$debugProcesses = Get-Process -Name 'ST-LINK_gdbserver', 'arm-none-eabi-gdb' -ErrorAction SilentlyContinue
if ($debugProcesses) {
    throw 'Close the active GDB server/debugger before the flash preflight.'
}

$appBytes = [IO.File]::ReadAllBytes($appPath)
$backup0 = [IO.File]::ReadAllBytes($sector0Backup)
$backup7 = [IO.File]::ReadAllBytes($sector7Backup)
if ($appBytes.Length -eq 0 -or $appBytes.Length -gt (6 * $sectorSize)) {
    throw "App image size is outside sectors 1-6: $($appBytes.Length) bytes."
}
if ($backup0.Length -ne $sectorSize -or $backup7.Length -ne $sectorSize) {
    throw 'The saved sector-0 and sector-7 backups must each be exactly 128 KB.'
}

$crcCheck = Get-Crc32 ([Text.Encoding]::ASCII.GetBytes('123456789'))
if ($crcCheck -ne [Convert]::ToUInt32('CBF43926', 16)) {
    throw ('CRC32 implementation check failed: got 0x{0:X8}.' -f $crcCheck)
}

Read-Flash $appBase $appBytes.Length $appReadback
Read-Flash $sector0Base $sectorSize $sector0Readback
Read-Flash $sector7Base $sectorSize $sector7Readback

Assert-SameBytes $appBytes ([IO.File]::ReadAllBytes($appReadback)) 'Live app image'
Assert-SameBytes $backup0 ([IO.File]::ReadAllBytes($sector0Readback)) 'Bootloader sector 0'
Assert-SameBytes $backup7 ([IO.File]::ReadAllBytes($sector7Readback)) 'NVM sector 7'

$oldMagic = [BitConverter]::ToUInt32($backup7, $metadataOffset)
$oldSize = [BitConverter]::ToUInt32($backup7, $metadataOffset + 4)
$oldCrc = [BitConverter]::ToUInt32($backup7, $metadataOffset + 8)
$oldBase = [BitConverter]::ToUInt32($backup7, $metadataOffset + 12)
if ($oldMagic -ne $metadataMagic -or $oldBase -ne $appBase) {
    throw ('Existing metadata is unexpected (magic=0x{0:X8}, base=0x{1:X8}); refusing to alter sector 7.' -f $oldMagic, $oldBase)
}

$stayKey = $null
for ($offset = 0; $offset -lt $nvmLength; $offset += 32) {
    if ([BitConverter]::ToUInt16($backup7, $offset) -ne 0xABCD) {
        continue
    }
    if ([BitConverter]::ToUInt16($backup7, $offset + 2) -ne 5) {
        continue
    }

    $entry = [PSCustomObject]@{
        Length = [BitConverter]::ToUInt16($backup7, $offset + 4)
        Sequence = [BitConverter]::ToUInt32($backup7, $offset + 8)
        Value = $backup7[$offset + 12]
    }
    if (($null -eq $stayKey) -or ($entry.Sequence -gt $stayKey.Sequence)) {
        $stayKey = $entry
    }
}
if ($null -ne $stayKey -and $stayKey.Length -gt 0 -and $stayKey.Value -ne 0) {
    throw 'Persistent stay-in-bootloader key 0x0005 is active; refusing to reset into the app.'
}

$newCrc = Get-Crc32 $appBytes
$metadata = New-Object byte[] 32
$words = @($metadataMagic, [uint32]$appBytes.Length, $newCrc, $appBase, $metadataVersion, [uint32]0, [uint32]0, [uint32]0)
for ($word = 0; $word -lt $words.Length; $word++) {
    $wordBytes = [BitConverter]::GetBytes([uint32]$words[$word])
    [Array]::Copy($wordBytes, 0, $metadata, 4 * $word, 4)
}

$patched7 = New-Object byte[] $sectorSize
[Array]::Copy($backup7, $patched7, $sectorSize)
[Array]::Copy($metadata, 0, $patched7, $metadataOffset, $metadata.Length)
Assert-SameBytes $backup7[0..($metadataOffset - 1)] $patched7[0..($metadataOffset - 1)] 'NVM/provisioning data'
[IO.File]::WriteAllBytes($patchedSector7, $patched7)

Write-Output ('Preflight passed: app={0} bytes, CRC32=0x{1:X8}.' -f $appBytes.Length, $newCrc)
Write-Output ('Old metadata: size={0}, CRC32=0x{1:X8}; new metadata: size={2}, CRC32=0x{3:X8}.' -f $oldSize, $oldCrc, $appBytes.Length, $newCrc)
Write-Output ('Sector 0 and every sector-7 byte outside metadata match their saved backups. Stay-in-BL key: {0}.' -f $(if ($null -eq $stayKey) { 'absent' } elseif ($stayKey.Length -eq 0 -or $stayKey.Value -eq 0) { 'tombstoned' } else { 'active' }))

if (-not $Flash) {
    Write-Output "DRY RUN ONLY. No flash was erased or written. Patched sector image: $patchedSector7"
    exit 0
}

try {
    Invoke-Programmer @('-c', 'port=SWD', '-e', '7', '-w', $patchedSector7, ('0x{0:X8}' -f $sector7Base), '-v')
    Read-Flash $sector7Base $sectorSize $sector7After
    Assert-SameBytes $patched7 ([IO.File]::ReadAllBytes($sector7After)) 'Updated sector 7'
}
catch {
    Restore-Sector7Backup
    throw
}

Read-Flash $sector0Base $sectorSize $sector0After
Read-Flash $appBase $appBytes.Length $appAfter
Assert-SameBytes $backup0 ([IO.File]::ReadAllBytes($sector0After)) 'Post-flash bootloader sector 0'
Assert-SameBytes $appBytes ([IO.File]::ReadAllBytes($appAfter)) 'Post-flash app image'

Invoke-Programmer @('-c', 'port=SWD', '-rst')
Write-Output 'Sector 7 metadata repaired; bootloader and app readbacks match; MCU reset requested.'