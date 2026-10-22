<#
    s2_test.ps1 - run one subsystem 2 test from start to finish and save a
    complete log: the build, the flash, the Pico's output, your setup and
    your measurements, all in one file in testing\subsystem2\logs\.

    Run it from the project folder in a PowerShell terminal (one line):

        powershell -ExecutionPolicy Bypass -File testing\subsystem2\s2_test.ps1 -Test unit

    -Test   what it runs                          robot
    unit    S2-U08, U05, U07, U10 (automatic)     anywhere, motors braked
    u06     S2-U06 open-loop motor drive          wheels lifted
    u02     S2-U02 speed step response            wheels lifted
    u01     S2-U01 ticks per wheel revolution     wheels turned by hand
    u11     S2-U11 MOVE distance, straightness    on the floor, ~1.5 m
    u03     S2-U03 turn accuracy                  on the floor
    u12     S2-U12 braking, slow speed, stall     on the floor, ~1 m
    u09     S2-U09 simulation on the PC           no Pico needed

    testing\subsystem2\TEST_GUIDE.md explains the setup and the
    measurements of each test.

    The test switch in the C source is set only for the build and put back
    straight afterwards, so the code in git never changes. Close the VS
    Code Serial Monitor (and PuTTY) first: only one program can open the
    Pico's port.
#>
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('unit', 'u06', 'u02', 'u01', 'u11', 'u03', 'u12', 'u09')]
    [string]$Test,
    [string]$Distro = 'Ubuntu-24.04'
)

$ErrorActionPreference = 'Stop'

$root     = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$s2Dir    = Join-Path $root 'app_program\subsystem2'
$buildDir = Join-Path $root 'build_make'
$logDir   = Join-Path $PSScriptRoot 'logs'
$image    = Join-Path $buildDir 'mtk3pico_smp1_usb_cdc_wifi_test.uf2'

# Test table: the switch to set for the build, how long the Pico part may
# take, and what to check before starting.
$tests = @{
    'unit' = @{ Name = 'S2-U08 U05 U07 U10 automatic checks'
                File = ''; Switch = ''; Minutes = 3; Floor = $false
                Setup = @('The motors stay braked: the robot can stand anywhere.') }
    'u06'  = @{ Name = 'S2-U06 open-loop motor drive'
                File = 'subsystem2_test.c'; Switch = 'S2_U06_BENCH'; Value = 1
                Minutes = 3; Floor = $false
                Setup = @('Put the robot on a box or book so BOTH wheels are off the table.',
                          'Stand where you can see which way each wheel turns.') }
    'u02'  = @{ Name = 'S2-U02 speed step response 0 -> 250 mm/s'
                File = 'subsystem2_test.c'; Switch = 'S2_U02_BENCH'; Value = 1
                Minutes = 3; Floor = $false
                Setup = @('Put the robot on a box or book so BOTH wheels are off the table.') }
    'u01'  = @{ Name = 'S2-U01 encoder ticks per wheel revolution'
                File = 'subsystem2_test.c'; Switch = 'S2_U01_BENCH'; Value = 1
                Minutes = 5; Floor = $false
                Setup = @('Put a small piece of tape on each tyre as a revolution marker.',
                          'Lift the robot so you can turn each wheel by hand.') }
    'u11'  = @{ Name = 'S2-U11 MOVE distance and straightness'
                File = 'subsystem2_floor.c'; Switch = 'S2_FLOOR_TEST'; Value = 1
                Minutes = 12; Floor = $true
                Setup = @('Tape measure laid flat and straight on the floor, 0 mark at the start, taped down.',
                          'A tape marker on the robot front centre; robot centre line over the tape.',
                          'At least 1.5 m clear in front and 0.6 m behind; cable with slack.',
                          'Each step: 30 s countdown. Measure, type the numbers, put the robot back at 0.') }
    'u03'  = @{ Name = 'S2-U03 turn accuracy'
                File = 'subsystem2_floor.c'; Switch = 'S2_FLOOR_TEST'; Value = 2
                Minutes = 15; Floor = $true
                Setup = @('A "+" of two tape lines on the floor at exactly 90 deg (use a paper corner).',
                          'A ruler taped on top of the robot along its centre line, pointing forward.',
                          'Robot axle centre over the crossing, ruler along the FORWARD line.',
                          'Protractor ready. Each turn: 30 s countdown to measure, type, and line up again.') }
    'u12'  = @{ Name = 'S2-U12 braking distance, slow speeds, stall'
                File = 'subsystem2_floor.c'; Switch = 'S2_FLOOR_TEST'; Value = 3
                Minutes = 12; Floor = $true
                Setup = @('At least 1 m clear in front; cable with slack.',
                          'Watch the wheels when it stops: do they skid?',
                          'Last step: when told, LIFT the robot and pinch the LEFT tyre still.') }
    'u09'  = @{ Name = 'S2-U09 motion_task simulation on the PC'
                File = ''; Switch = ''; Minutes = 0; Floor = $false
                Setup = @('No Pico needed.') }
}

$t = $tests[$Test]

# ---------------------------------------------------------------------
#  Small helpers
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
        Write-Host '  Type a number, for example 1003 or -4.5' -ForegroundColor Yellow
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
    # stdout only; the command itself redirects stderr where needed.
    # Windows PowerShell 5.1 does not escape double quotes in arguments
    # to native programs, so the commands here use single quotes only.
    return (& wsl.exe -d $Distro -- bash -lc $command)
}

# ---------------------------------------------------------------------
#  1. Commit and setup
# ---------------------------------------------------------------------

Write-Step "$Test : $($t.Name)"

$commit = (& git -C $root rev-parse --short HEAD).Trim()
$dirty  = & git -C $root status --porcelain -- app_program/subsystem2
$state  = 'S2 code committed'
if ($dirty)
{
    $state = 'S2 CODE HAS UNCOMMITTED CHANGES'
    Write-Host "Warning: app_program/subsystem2 has uncommitted changes:" -ForegroundColor Yellow
    $dirty | ForEach-Object { Write-Host "  $_" -ForegroundColor Yellow }
    Write-Host 'A result only counts against a commit (testing/README.md).' -ForegroundColor Yellow
    if ((Ask-YesNo 'Run anyway, as a practice run?') -ne 'y')
    {
        exit 1
    }
}
Write-Host "Commit: $commit ($state)"

$header = New-Object System.Collections.Generic.List[string]
$header.Add('=== S2 test log ===')
$header.Add("test:     $Test  ($($t.Name))")
$header.Add("date:     $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$header.Add("commit:   $commit  ($state)")

if ($Test -ne 'u09')
{
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
        $header.Add("surface:  $surface")
    }
}

$notes = Read-Host 'Anything unusual to note (Enter = none)'
$header.Add("notes:    $notes")

Write-Step 'Before you start'
$t.Setup | ForEach-Object { Write-Host "  - $_" }
if ($Test -ne 'u09')
{
    Write-Host '  - Close the VS Code Serial Monitor and PuTTY.'
    Write-Host '  - Keep a hand near the Robo Pico power switch whenever the motors may run.'
}
[void](Read-Host 'Press Enter when ready')

# ---------------------------------------------------------------------
#  Log file: logs\<date>_s2_<test>.txt, numbered if it already exists
# ---------------------------------------------------------------------

$stem = '{0}_s2_{1}' -f (Get-Date -Format 'yyyy-MM-dd'), $Test
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
    #  u09: the host simulation, no Pico
    # -----------------------------------------------------------------
    if ($Test -eq 'u09')
    {
        Write-Step 'Running the S2-U09 simulation in WSL'
        $out = Invoke-Wsl "cd '$wslRoot' && sh testing/subsystem2/run_motion_test.sh > /tmp/s2_u09.txt 2>&1; cat /tmp/s2_u09.txt"
        Write-Log '=== Output ==='
        foreach ($line in $out)
        {
            Write-Log $line
            $color = 'Gray'
            if ($line -match '\bPASS\b') { $color = 'Green' }
            if ($line -match '\bFAIL\b') { $color = 'Red' }
            Write-Host $line -ForegroundColor $color
        }
        Write-Step "Log saved: $log"
        return
    }

    # -----------------------------------------------------------------
    #  2. Build with the test's switch set, then put the source back
    # -----------------------------------------------------------------
    Write-Step 'Building the test image (make TEST=1) in WSL'
    $file     = $null
    $original = $null
    $switchNote = 'none (automatic tests only)'

    if ($t.File)
    {
        $file     = Join-Path $s2Dir $t.File
        $original = [System.IO.File]::ReadAllBytes($file)
        $text     = [System.Text.Encoding]::UTF8.GetString($original)
        $pattern  = '(#define\s+' + $t.Switch + '\s+\()\d+(\))'
        if ([regex]::Matches($text, $pattern).Count -ne 1)
        {
            throw "$($t.Switch) not found exactly once in $($t.File)"
        }
        $text = [regex]::Replace($text, $pattern, '${1}' + $t.Value + '${2}')
        [System.IO.File]::WriteAllBytes($file, (New-Object System.Text.UTF8Encoding($false)).GetBytes($text))
        $switchNote = "$($t.Switch) = $($t.Value) in $($t.File), set for this build only"
    }

    $started = Get-Date
    try
    {
        $result = Invoke-Wsl ("cd '$wslRoot/build_make' && " +
                              'export PICO_SDK_PATH=${PICO_SDK_PATH:-$HOME/pico/pico-sdk} && ' +
                              'make TEST=1 -j8 > /tmp/s2_make.log 2>&1; echo rc=$?; ' +
                              'grep -iE ''error|warning'' /tmp/s2_make.log | head -20')
    }
    finally
    {
        if ($null -ne $original)
        {
            [System.IO.File]::WriteAllBytes($file, $original)
        }
    }

    $result | ForEach-Object { Write-Host "  $_" }
    if (-not ($result -contains 'rc=0') -or
        ((Get-Item $image).LastWriteTime -lt $started))
    {
        throw 'The build failed (see the lines above, or /tmp/s2_make.log in WSL).'
    }
    $built = (Get-Item $image).LastWriteTime.ToString('HH:mm:ss')
    Write-Host "Built $([System.IO.Path]::GetFileName($image)) at $built" -ForegroundColor Green
    Write-Log "switch:   $switchNote"
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
    Write-Log '=== Pico output and [MEASURED] values ==='

    $script:moveRun  = 0
    $script:turnRun  = 0
    $script:u06Side  = ''
    $script:u06Asked = @{}

    function Ask-ForLine([string]$line)
    {
        switch ($script:Test)
        {
            'u11'
            {
                if ($line -match '^\[S2-FLOOR\] MOVE (-?\d+):')
                {
                    $script:moveRun++
                    $target = [int]$Matches[1]
                    $start  = 0
                    if ($target -lt 0)
                    {
                        $start = 500
                    }
                    Write-Host "  Run $($script:moveRun): read the tape at the robot's front marker." -ForegroundColor Yellow
                    $end    = Ask-Number "  Tape reading now, mm (it started at $start)"
                    $offset = Ask-Number '  Sideways offset of the marker from the tape line, mm (+ = robot''s left, - = right)'
                    $real   = $null
                    if ($null -ne $end)
                    {
                        $real = $end - $start
                    }
                    Write-Log ("[MEASURED] MOVE {0} run {1}: real distance {2} mm (tape {3}, start {4}), sideways offset {5} mm" -f
                               $target, $script:moveRun, (Format-Value $real), (Format-Value $end), $start, (Format-Value $offset))
                    Write-Host '  Saved. Put the robot back on its start mark now.' -ForegroundColor Yellow
                }
            }
            'u03'
            {
                if ($line -match '^\[S2-FLOOR\] TURN (-?\d+):')
                {
                    $script:turnRun++
                    $target = [int]$Matches[1]
                    Write-Host "  Turn $($script:turnRun): measure with the protractor at the crossing." -ForegroundColor Yellow
                    $angle = Ask-Number "  Angle actually turned, deg (commanded $target; + = left)"
                    Write-Log ("[MEASURED] TURN {0} run {1}: measured {2} deg" -f $target, $script:turnRun, (Format-Value $angle))
                    Write-Host '  Saved. Line the robot up on the FORWARD line again now.' -ForegroundColor Yellow
                }
            }
            'u12'
            {
                if ($line -match 'braking distance')
                {
                    $skid = Ask-YesNo '  Did the wheels skid or slide when it stopped?'
                    Write-Log "[MEASURED] wheels skidded: $skid  (for: $line)"
                }
                elseif ($line -match '^\[S2-FLOOR\] FOLLOW (\d+) mm/s:')
                {
                    $smooth = Ask-YesNo "  At $($Matches[1]) mm/s did it move smoothly (no stopping or jerking)?"
                    Write-Log "[MEASURED] smooth at $($Matches[1]) mm/s: $smooth"
                }
            }
            'u06'
            {
                if ($line -match '^\[S2-U06\] (LEFT|RIGHT) FORWARD:')
                {
                    $script:u06Side = $Matches[1]
                }
                elseif ($line -match '^\[S2-U06\]\s+duty 400:' -and
                        -not $script:u06Asked.ContainsKey($script:u06Side))
                {
                    $script:u06Asked[$script:u06Side] = $true
                    $side = $script:u06Side
                    $fwd  = Ask-YesNo "  In '$side FORWARD', did the $side wheel turn the way that drives the robot FORWARD?"
                    $only = Ask-YesNo "  Did ONLY the $side wheel turn (the other one stayed still)?"
                    Write-Log "[MEASURED] $side FORWARD drives the robot forward: $fwd; only that wheel turned: $only"
                }
            }
            'u01'
            {
                if ($line -match '^\[TEST\] S2-U01 INFO (LEFT|RIGHT):')
                {
                    $revs = Ask-Number "  How many full revolutions did you turn the $($Matches[1]) wheel? (planned 10)"
                    Write-Log "[MEASURED] $($Matches[1]) wheel turned $(Format-Value $revs) revolutions by hand"
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
            if ($line -match '^\[S2-FLOOR\] (NEXT|GO|HOLD)') { $color = 'Cyan' }
            if ($line -match '\bPASS\b') { $color = 'Green' }
            if ($line -match '\bFAIL\b|ERROR|STALL') { $color = 'Red' }
            Write-Host $line -ForegroundColor $color

            Ask-ForLine $line

            if ($line -match '^\[TEST\] subsystem2 SUMMARY')
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
    $pass = @($lines | Where-Object { $_ -match '^\[TEST\] S2-U\d+ PASS' }).Count
    $fail = @($lines | Where-Object { $_ -match '^\[TEST\] S2-U\d+ FAIL' }).Count

    if ($finished)
    {
        Write-Log "[RESULT] finished: $pass S2 checks passed, $fail failed"
        Write-Step "Done: $pass passed, $fail failed"
    }
    else
    {
        Write-Log "[RESULT] NOT FINISHED (time limit or disconnect): $pass passed, $fail failed so far"
        Write-Step 'Stopped before the end of the S2 tests'
    }
    Write-Host 'Unplug the Pico or switch the robot off now if the test drove the motors.'
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
    Write-Host 'Send me this file.'
}
