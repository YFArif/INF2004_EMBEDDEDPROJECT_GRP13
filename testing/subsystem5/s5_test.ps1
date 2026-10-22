<#
    s5_test.ps1 - run one subsystem 5 test from start to finish and save a
    complete log: the build, the flash, the Pico's output, your setup and
    your measurements, all in one file in testing\subsystem5\logs\.

    Works the same way as testing\subsystem2\s2_test.ps1 (Arif's runner,
    whose helper functions are reused here).

    Run it from the project folder in a PowerShell terminal (one line):

        powershell -ExecutionPolicy Bypass -File testing\subsystem5\s5_test.ps1 -Test auto

    -Test   what it runs                                   robot
    auto    S5-U06, U07, U09, U10 (automatic)              anywhere, still
    bench   auto + S5-U01, U04, U03 (sensor and servo)     on blocks, box 30 cm ahead
    stop    auto + S5-U08 x3: drive at a box and stop      on the floor, 1.5 m
    avoid   auto + S5-U05 x3: drive round a box            on the floor, 2 m x 1 m

    testing\subsystem5\TEST_GUIDE.md explains the setup and the
    measurements of each test. The test switch in the C source is set only
    for the build and put back straight afterwards, so the code in git
    never changes. Close the VS Code Serial Monitor (and PuTTY) first.
#>
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('auto', 'bench', 'stop', 'avoid')]
    [string]$Test,
    [string]$Distro = 'Ubuntu-24.04'
)

$ErrorActionPreference = 'Stop'

$root     = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$s5Dir    = Join-Path $root 'app_program\subsystem5'
$buildDir = Join-Path $root 'build_make'
$logDir   = Join-Path $PSScriptRoot 'logs'
$image    = Join-Path $buildDir 'mtk3pico_smp1_usb_cdc_wifi_test.uf2'
$testFile = Join-Path $s5Dir 'subsystem5_test.c'

$button = 'Every floor run waits for you to press BUTTON 1 (GP20) on the Robo Pico, then counts down 3 s (with beeps) and drives. Nothing moves before that.'
$tests = @{
    'auto'  = @{ Name = 'S5 automatic checks'; Mode = 0; Minutes = 3; Floor = $false
                 Setup = @('Nothing to set up: the motors stay still and the servo only centres.') }
    'bench' = @{ Name = 'S5-U01 U04 U03 sensor and servo on the bench'; Mode = 1
                 Minutes = 6; Floor = $false
                 Setup = @('Put the robot on a box or book so both wheels are off the table.',
                           'A flat, hard box (a shoe box is fine) about 30 cm straight in front of the ultrasonic sensor, facing it squarely. Nothing else within 1 m in front.',
                           'Have a ruler (or tape measure) and the protractor ready, and your phone to FILM the servo during the last test.') }
    'stop'  = @{ Name = 'S5-U08 drive at a box, stop, scan and plan (x3)'; Mode = 2
                 Minutes = 20; Floor = $true
                 Setup = @('Clear floor: 1.5 m long, 0.6 m wide. A box (at least 15 cm wide and 10 cm tall, flat front) at the far end.',
                           'Before each run: robot 1 m from the box, pointing straight at it. Tape a mark on the floor so every run starts at the same place.',
                           'The robot drives straight at the box (no line following yet), slows down, stops, then the sensor sweeps. It does NOT drive round the box in this test.',
                           $button) }
    'avoid' = @{ Name = 'S5-U05 drive round a box (x3)'; Mode = 3
                 Minutes = 25; Floor = $true
                 Setup = @('Clear floor: 2 m long, 1 m wide. The box in the middle of the far half, at least 0.5 m clear on BOTH sides of it.',
                           'Lay a strip of masking tape on the floor from the start mark straight through the middle of the box: this is the "line" the robot should come back to.',
                           'Before each run: robot on the tape, 1 m before the box, pointing along the tape.',
                           'The robot stops, scans, drives round the box and comes back to the tape, then drives on for 1.5 s and stops.',
                           $button) }
}
$t = $tests[$Test]

# ---------------------------------------------------------------------
#  Small helpers (from s2_test.ps1)
# ---------------------------------------------------------------------

function Write-Step([string]$text)
{
    Write-Host ''
    Write-Host "=== $text ===" -ForegroundColor Cyan
}

function Ask-Number([string]$question)
{
    while ($true)
    {
        $answer = Read-Host "$question (Enter = skip)"
        if ([string]::IsNullOrWhiteSpace($answer))
        {
            return $null
        }
        $value = 0.0
        if ([double]::TryParse($answer.Trim(),
                               [Globalization.NumberStyles]::Float,
                               [Globalization.CultureInfo]::InvariantCulture,
                               [ref]$value))
        {
            return $value
        }
        Write-Host '  Type a number, for example 120 or -4.5' -ForegroundColor Yellow
    }
}

function Ask-YesNo([string]$question)
{
    while ($true)
    {
        $answer = (Read-Host "$question (y/n)").Trim().ToLower()
        if ($answer -eq 'y' -or $answer -eq 'n')
        {
            return $answer
        }
    }
}

function Format-Value($value)
{
    if ($null -eq $value)
    {
        return 'skipped'
    }
    return $value.ToString([Globalization.CultureInfo]::InvariantCulture)
}

function Find-PicoPort
{
    $ports = Get-CimInstance Win32_PnPEntity -Filter "PNPClass='Ports'"
    foreach ($p in $ports)
    {
        if ($p.DeviceID -like 'USB\VID_2E8A&PID_000A*' -and $p.Name -match '\((COM\d+)\)')
        {
            return $Matches[1]
        }
    }
    return $null
}

function Find-PicoDrive
{
    return (Get-CimInstance Win32_LogicalDisk -Filter "VolumeName='RPI-RP2'" |
            Select-Object -First 1)
}

function Invoke-Wsl([string]$command)
{
    # Windows PowerShell 5.1 does not escape double quotes in arguments to
    # native programs, so the commands here use single quotes only.
    return (& wsl.exe -d $Distro -- bash -lc $command)
}

function Look-AtRobot
{
    Write-Host '  Look at the robot now. Question in' -NoNewline -ForegroundColor Yellow
    foreach ($n in 3, 2, 1)
    {
        Write-Host " $n.." -NoNewline -ForegroundColor Yellow
        Start-Sleep -Seconds 1
    }
    Write-Host ''
    [Console]::Beep(660, 120)
}

function Plan-Name([int]$action)
{
    switch ($action)
    {
        0       { return 'CONTINUE (not in its way)' }
        1       { return 'go round on the LEFT' }
        2       { return 'go round on the RIGHT' }
        3       { return 'REVERSE and look again (no side wide enough)' }
        default { return "unknown ($action)" }
    }
}

# ---------------------------------------------------------------------
#  1. Commit and setup
# ---------------------------------------------------------------------

Write-Step "$Test : $($t.Name)"

$commit = (& git -C $root rev-parse --short HEAD).Trim()
$branch = (& git -C $root rev-parse --abbrev-ref HEAD).Trim()
$dirty  = & git -C $root status --porcelain -- app_program
$state  = 'code committed'
if ($dirty)
{
    $state = 'CODE HAS UNCOMMITTED CHANGES'
    Write-Host 'Warning: app_program has uncommitted changes:' -ForegroundColor Yellow
    $dirty | ForEach-Object { Write-Host "  $_" -ForegroundColor Yellow }
    Write-Host 'A result only counts against a commit (testing/README.md).' -ForegroundColor Yellow
    if ((Ask-YesNo 'Run anyway, as a practice run?') -ne 'y')
    {
        exit 1
    }
}
Write-Host "Branch $branch, commit $commit ($state)"

$header = New-Object System.Collections.Generic.List[string]
$header.Add('=== S5 test log ===')
$header.Add("test:     $Test  ($($t.Name))")
$header.Add("date:     $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$header.Add("commit:   $commit on $branch  ($state)")

$tester = Read-Host 'Tester name (Enter = Arif)'
if ([string]::IsNullOrWhiteSpace($tester))
{
    $tester = 'Arif'
}

Write-Host 'Power: 1 = power bank on VIN + USB cable, 2 = USB cable only, 3 = other'
$supply = Read-Host 'Supply (1/2/3)'
switch ($supply)
{
    '1'     { $supply = 'power bank on VIN + USB cable to the laptop' }
    '2'     { $supply = 'USB cable only' }
    default { $supply = Read-Host 'Describe the supply' }
}
$vin = Ask-Number 'VIN voltage from the multimeter, V, motors still'
$header.Add("tester:   $tester")
$header.Add("supply:   $supply; VIN = $(Format-Value $vin) V")

if ($t.Floor)
{
    $surface = Read-Host 'Floor surface (e.g. lab tiles, track paper, vinyl)'
    $box     = Read-Host 'The box: width x height x depth in mm, and material (e.g. 150x120x100 cardboard)'
    $header.Add("surface:  $surface")
    $header.Add("box:      $box")
}
elseif ($Test -eq 'bench')
{
    $box = Read-Host 'The box: width x height in mm, and material (e.g. 150x120 cardboard)'
    $header.Add("box:      $box")
}

$notes = Read-Host 'Anything unusual to note (Enter = none)'
$header.Add("notes:    $notes")

Write-Step 'Before you start'
$t.Setup | ForEach-Object { Write-Host "  - $_" }
Write-Host '  - Close the VS Code Serial Monitor and PuTTY.'
Write-Host '  - Keep a hand near the Robo Pico power switch whenever the motors may run.'
[void](Read-Host 'Press Enter when ready')

# ---------------------------------------------------------------------
#  Log file: logs\<date>_s5_<test>.txt, numbered if it already exists
# ---------------------------------------------------------------------

$stem = '{0}_s5_{1}' -f (Get-Date -Format 'yyyy-MM-dd'), $Test
$log  = Join-Path $logDir "$stem.txt"
$n    = 2
while (Test-Path $log)
{
    $log = Join-Path $logDir "${stem}_$n.txt"
    $n++
}

$writer = New-Object System.IO.StreamWriter($log, $false, (New-Object System.Text.UTF8Encoding($false)))
$writer.AutoFlush = $true
$header | ForEach-Object { $writer.WriteLine($_) }

function Write-Log([string]$line)
{
    $script:writer.WriteLine($line)
}

try
{
    $wslRoot = (Invoke-Wsl "wslpath -a '$($root -replace '\\', '/')'" | Select-Object -First 1)
    if (-not $wslRoot)
    {
        throw "WSL ($Distro) did not answer. Open the Ubuntu terminal once, then run again."
    }
    $wslRoot = $wslRoot.Trim()

    # -----------------------------------------------------------------
    #  2. Build with S5_TEST_MODE set, then put the source back
    # -----------------------------------------------------------------
    Write-Step 'Building the test image (make TEST=1) in WSL'
    $original = [System.IO.File]::ReadAllBytes($testFile)
    $text     = [System.Text.Encoding]::UTF8.GetString($original)
    $pattern  = '(#define\s+S5_TEST_MODE\s+\()\d+(\))'
    if ([regex]::Matches($text, $pattern).Count -ne 1)
    {
        throw 'S5_TEST_MODE not found exactly once in subsystem5_test.c'
    }
    $text = [regex]::Replace($text, $pattern, '${1}' + $t.Mode + '${2}')
    [System.IO.File]::WriteAllBytes($testFile, (New-Object System.Text.UTF8Encoding($false)).GetBytes($text))

    try
    {
        $result = Invoke-Wsl ("cd '$wslRoot/build_make' && " +
                              'export PICO_SDK_PATH=${PICO_SDK_PATH:-$HOME/pico/pico-sdk} && ' +
                              'make TEST=1 -j8 > /tmp/s5_make.log 2>&1; echo rc=$?; ' +
                              'grep -iE ''error|warning'' /tmp/s5_make.log | head -20; ' +
                              'echo ---- last lines of the build:; tail -n 4 /tmp/s5_make.log')
    }
    finally
    {
        [System.IO.File]::WriteAllBytes($testFile, $original)
    }

    $result | ForEach-Object { Write-Host "  $_" }
    $result | ForEach-Object { Write-Log "[build] $_" }
    if (-not ($result -contains 'rc=0') -or -not (Test-Path $image))
    {
        throw 'The build failed (see the lines above, or /tmp/s5_make.log in WSL).'
    }
    $built = (Get-Item $image).LastWriteTime.ToString('HH:mm:ss')
    Write-Host "Using $([System.IO.Path]::GetFileName($image)) (built $built)" -ForegroundColor Green
    Write-Log "switch:   S5_TEST_MODE = $($t.Mode) in subsystem5_test.c, set for this build only"
    Write-Log "image:    $([System.IO.Path]::GetFileName($image)), built $built"

    # -----------------------------------------------------------------
    #  3. Flash
    # -----------------------------------------------------------------
    Write-Step 'Flashing'
    if (Find-PicoPort)
    {
        Write-Host 'Unplug the Pico''s USB cable now.' -ForegroundColor Yellow
        while (Find-PicoPort)
        {
            Start-Sleep -Milliseconds 300
        }
    }

    Write-Host 'Hold the BOOTSEL button on the Pico, plug the USB cable in, then let go.' -ForegroundColor Yellow
    $drive = $null
    $limit = (Get-Date).AddMinutes(2)
    while (-not $drive -and (Get-Date) -lt $limit)
    {
        Start-Sleep -Milliseconds 300
        $drive = Find-PicoDrive
    }
    if (-not $drive)
    {
        throw 'No RPI-RP2 drive appeared within 2 minutes.'
    }

    Write-Host "Found RPI-RP2 at $($drive.DeviceID); copying the image ..."
    try
    {
        Copy-Item $image "$($drive.DeviceID)\" -Force
    }
    catch
    {
        # The Pico reboots as soon as the image is complete, which can cut
        # the copy's last step; the port check below shows if it worked.
    }

    $port  = $null
    $limit = (Get-Date).AddSeconds(30)
    while (-not $port -and (Get-Date) -lt $limit)
    {
        Start-Sleep -Milliseconds 300
        $port = Find-PicoPort
    }
    if (-not $port)
    {
        throw 'The Pico did not come back as a serial port within 30 s. Unplug, replug and run again.'
    }
    Write-Host "Pico running on $port" -ForegroundColor Green
    Write-Log "port:     $port"

    # -----------------------------------------------------------------
    #  4. Read the Pico's output; ask for measurements at the right lines
    # -----------------------------------------------------------------
    $sp = New-Object System.IO.Ports.SerialPort $port, 115200
    $sp.DtrEnable = $true       # The Pico's tests wait until the PC listens.
    $sp.RtsEnable = $true
    try
    {
        $sp.Open()
    }
    catch
    {
        throw "Cannot open $port. Close the VS Code Serial Monitor or PuTTY and run again."
    }

    Write-Step 'Pico output (your measurements are asked for as the test goes)'
    Write-Host 'The other subsystems'' automatic tests run first (about 30 s); S5 starts after "TEST S5 tests start".'
    Write-Log '=== Pico output and [MEASURED] values ==='

    $script:lastPlan = -1
    $script:run      = 0

    function Ask-ForLine([string]$line)
    {
        if ($line -match '^\[S5\] PLAN action out_mm (\d+) ')
        {
            $script:lastPlan = [int]$Matches[1]
        }

        switch ($script:Test)
        {
            'bench'
            {
                if ($line -match '^\[S5\] S5-U01 mean_mm spread_mm (-?\d+) (-?\d+)')
                {
                    $said = $Matches[1]
                    Write-Host "  The robot measured $said mm to the box." -ForegroundColor Yellow
                    $ruler = Ask-Number '  Ruler: distance from the FRONT of the ultrasonic sensor to the box, mm'
                    Write-Log "[MEASURED] U01 ruler $(Format-Value $ruler) mm; robot said $said mm"
                }
                elseif ($line -match '^\[S5\] PROFILE closest_mm width_mm (\d+) (\d+)')
                {
                    $width = $Matches[2]
                    Write-Host "  The robot measured the box as $width mm wide." -ForegroundColor Yellow
                    $real = Ask-Number '  Real width of the box (its front face), mm'
                    Write-Log "[MEASURED] U04 real width $(Format-Value $real) mm; robot said $width mm"
                    Write-Host '  Next: the servo stops at 5 angles for 2 s each. FILM it, or watch with the protractor.' -ForegroundColor Yellow
                }
                elseif ($line -match '^\[TEST\] S5-U03 ')
                {
                    $sides = Ask-YesNo '  Did the FIRST stop (30 deg) point to the robot''s RIGHT and the LAST (150 deg) to its LEFT?'
                    $mid   = Ask-YesNo '  Did the MIDDLE stop (90 deg) point straight ahead?'
                    $err   = Ask-Number '  Largest angle error you saw at any stop, degrees (from the video or protractor)'
                    Write-Log "[MEASURED] U03 30 right / 150 left: $sides; 90 straight ahead: $mid; worst error $(Format-Value $err) deg"
                }
            }
            'stop'
            {
                if ($line -match '^\[S5\] RESULT U08 state driven_mm (-?\d+) (-?\d+)')
                {
                    $script:run++
                    Look-AtRobot
                    Write-Host "  Run $($script:run): the robot drove $($Matches[2]) mm and planned to $(Plan-Name $script:lastPlan)." -ForegroundColor Yellow
                    $gap   = Ask-Number '  Gap between the robot''s FRONT and the box now, mm'
                    $slow  = Ask-YesNo '  Did it slow down clearly before it stopped?'
                    $touch = Ask-YesNo '  Did it TOUCH the box?'
                    $side  = Ask-YesNo "  It chose to $(Plan-Name $script:lastPlan). Is that the side with MORE free space?"
                    Write-Log "[MEASURED] U08 run $($script:run): gap $(Format-Value $gap) mm; slowed $slow; touched $touch; plan '$(Plan-Name $script:lastPlan)' is the roomier side: $side"
                    Write-Host '  Saved. Put the robot back on the start mark (1 m from the box), then press BUTTON 1.' -ForegroundColor Yellow
                    $script:lastPlan = -1
                }
            }
            'avoid'
            {
                if ($line -match '^\[S5\] RESULT U05 state ok (-?\d+) (\d)')
                {
                    $script:run++
                    Look-AtRobot
                    Write-Host "  Run $($script:run): it planned to $(Plan-Name $script:lastPlan)." -ForegroundColor Yellow
                    $touch  = Ask-YesNo '  Did the robot TOUCH the box at any point?'
                    $offset = Ask-Number '  Sideways distance from the robot''s centre to the tape line now, mm (+ = robot is LEFT of the tape, - = right)'
                    $dir    = Ask-YesNo '  Is the robot pointing along the tape again (within about 10 degrees)?'
                    $close  = Ask-Number '  Closest the robot came to the box''s side, roughly, mm'
                    Write-Log "[MEASURED] U05 run $($script:run): touched $touch; offset from tape $(Format-Value $offset) mm; pointing along tape $dir; closest to box $(Format-Value $close) mm; plan '$(Plan-Name $script:lastPlan)'"
                    Write-Host '  Saved. Put the robot back on the tape, 1 m before the box, then press BUTTON 1.' -ForegroundColor Yellow
                    $script:lastPlan = -1
                }
            }
        }
    }

    $buffer   = ''
    $finished = $false
    $deadline = (Get-Date).AddMinutes($t.Minutes)
    $lines    = New-Object System.Collections.Generic.List[string]

    while (-not $finished -and (Get-Date) -lt $deadline)
    {
        try
        {
            $chunk = $sp.ReadExisting()
        }
        catch
        {
            Write-Host 'The Pico disconnected.' -ForegroundColor Red
            break
        }

        if ($chunk.Length -eq 0)
        {
            Start-Sleep -Milliseconds 50
            continue
        }

        $buffer += $chunk
        while (($i = $buffer.IndexOf("`n")) -ge 0)
        {
            $line   = $buffer.Substring(0, $i).TrimEnd("`r")
            $buffer = $buffer.Substring($i + 1)
            $lines.Add($line)
            Write-Log $line

            $color = 'Gray'
            if ($line -match '^\[S5\]') { $color = 'White' }
            if ($line -match '\bPASS\b') { $color = 'Green' }
            if ($line -match '\bFAIL\b|ERROR|fault|FAULT') { $color = 'Red' }

            if ($line -match '^\[S5-STEP\] NEXT: (.*)$')
            {
                Write-Host ''
                Write-Host ('#' * 70) -ForegroundColor Yellow
                Write-Host "  NEXT: $($Matches[1])" -ForegroundColor Yellow
                Write-Host '  Get the robot ready, then press BUTTON 1 on the robot.' -ForegroundColor Yellow
                Write-Host ('#' * 70) -ForegroundColor Yellow
            }
            elseif ($line -match '^\[S5-STEP\] press')
            {
                [Console]::Beep(880, 120)
                [Console]::Beep(880, 120)
            }
            elseif ($line -match '^\[S5-STEP\] ([123])$')
            {
                Write-Host "      $($Matches[1]) ...  WATCH THE ROBOT" -ForegroundColor Magenta
                [Console]::Beep(1000, 150)
            }
            elseif ($line -match '^\[S5-STEP\] GO$')
            {
                Write-Host '      GO' -ForegroundColor Magenta
                [Console]::Beep(1500, 400)
            }
            else
            {
                Write-Host $line -ForegroundColor $color
            }

            Ask-ForLine $line

            if ($line -match '^\[TEST\] subsystem5 SUMMARY')
            {
                $finished = $true
                break
            }
        }
    }

    $sp.Close()

    # -----------------------------------------------------------------
    #  5. Summary
    # -----------------------------------------------------------------
    $pass = @($lines | Where-Object { $_ -match '^\[TEST\] S5-U\d+\w* PASS' }).Count
    $fail = @($lines | Where-Object { $_ -match '^\[TEST\] S5-U\d+\w* FAIL' }).Count

    if ($finished)
    {
        Write-Log "[RESULT] finished: $pass S5 checks passed, $fail failed"
        Write-Step "Done: $pass passed, $fail failed"
    }
    else
    {
        Write-Log "[RESULT] NOT FINISHED (time limit or disconnect): $pass passed, $fail failed so far"
        Write-Step 'Stopped before the end of the S5 tests'
    }
    Write-Host 'Switch the robot off now if the test drove the motors.'
}
catch
{
    Write-Log "[ERROR] $($_.Exception.Message)"
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
}
finally
{
    if ($sp -and $sp.IsOpen)
    {
        $sp.Close()
    }
    $writer.Close()
    Write-Host "Log saved: $log" -ForegroundColor Green
    Write-Host 'Send this file to Anna.'
}
