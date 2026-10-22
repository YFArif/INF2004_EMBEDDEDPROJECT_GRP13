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
# take (the steps wait for Button 1, so this is generous), and what to
# check before starting.
$button = 'Every step waits for you to press BUTTON 1 (GP20) on the Robo Pico, then counts down 3 s (with beeps) and runs. Nothing moves before that.'
$tests = @{
    'unit' = @{ Name = 'S2-U08 U05 U07 U10 automatic checks'
                File = ''; Switch = ''; Minutes = 3; Floor = $false
                Setup = @('The motors stay braked: the robot can stand anywhere.') }
    'u06'  = @{ Name = 'S2-U06 open-loop motor drive'
                File = 'subsystem2_test.c'; Switch = 'S2_U06_BENCH'; Value = 1
                Minutes = 20; Floor = $false
                Setup = @('Put the robot on a box or book so BOTH wheels are off the table.',
                          'Stand BEHIND the robot (its front pointing away from you): forward = top of the tyre moves away from you.',
                          $button) }
    'u02'  = @{ Name = 'S2-U02 speed step response 0 -> 250 mm/s'
                File = 'subsystem2_test.c'; Switch = 'S2_U02_BENCH'; Value = 1
                Minutes = 10; Floor = $false
                Setup = @('Put the robot on a box or book so BOTH wheels are off the table.',
                          'Stand BEHIND the robot: both tyre tops must move away from you.',
                          $button) }
    'u01'  = @{ Name = 'S2-U01 encoder ticks per wheel revolution'
                File = 'subsystem2_test.c'; Switch = 'S2_U01_BENCH'; Value = 1
                Minutes = 10; Floor = $false
                Setup = @('Put a small piece of tape on each tyre as a revolution marker.',
                          'Lift the robot so you can turn each wheel by hand.',
                          $button) }
    'u11'  = @{ Name = 'S2-U11 MOVE distance and straightness'
                File = 'subsystem2_floor.c'; Switch = 'S2_FLOOR_TEST'; Value = 1
                Minutes = 40; Floor = $true
                Setup = @('Clear floor: 1.6 m long x 0.6 m wide, plus the robot''s own length behind the start.',
                          'Tape measure flat and straight on the floor BESIDE the LEFT wheel (not under the car), about 2 cm from the tyre, taped down at both ends and the middle.',
                          'Reference point: the FRONT EDGE of the LEFT tyre where it touches the floor. Ignore the car''s nose and tail.',
                          'Start mark: any reading you like (0 on a floor line, or 100 on the tape). Run 5 (reverse) goes DOWN the tape: start it at 300 or more.',
                          'Before each run the script asks your start reading and the tyre-to-tape gap; after it, the end reading and the gap again.',
                          $button) }
    'u03'  = @{ Name = 'S2-U03 turn accuracy'
                File = 'subsystem2_floor.c'; Switch = 'S2_FLOOR_TEST'; Value = 2
                Minutes = 40; Floor = $true
                Setup = @('A "+" of two tape lines on the floor at exactly 90 deg (use a paper corner); label the arms F, L, R, B.',
                          'A ruler taped on top of the robot along its centre line, pointing forward (the pointer).',
                          'Robot axle centre over the crossing, pointer along arm F, before every turn.',
                          $button) }
    'u12'  = @{ Name = 'S2-U12 braking distance, slow speeds, stall'
                File = 'subsystem2_floor.c'; Switch = 'S2_FLOOR_TEST'; Value = 3
                Minutes = 40; Floor = $true
                Setup = @('At least 1 m clear in front; cable with slack. Put the robot back at the start before each step.',
                          'Watch the wheels when it stops: do they skid?',
                          'Last step (stall): robot on the BOX with the wheels free; after GO pinch the LEFT tyre still.',
                          $button) }
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

    try
    {
        $result = Invoke-Wsl ("cd '$wslRoot/build_make' && " +
                              'export PICO_SDK_PATH=${PICO_SDK_PATH:-$HOME/pico/pico-sdk} && ' +
                              'make TEST=1 -j8 > /tmp/s2_make.log 2>&1; echo rc=$?; ' +
                              'grep -iE ''error|warning'' /tmp/s2_make.log | head -20; ' +
                              'echo ---- last lines of the build:; tail -n 4 /tmp/s2_make.log')
    }
    finally
    {
        if ($null -ne $original)
        {
            [System.IO.File]::WriteAllBytes($file, $original)
        }
    }

    $result | ForEach-Object { Write-Host "  $_" }
    $result | ForEach-Object { Write-Log "[build] $_" }

    # make's own exit code decides. An unchanged source gives "up to date"
    # and an image with an older time stamp, which is correct.
    if (-not ($result -contains 'rc=0') -or -not (Test-Path $image))
    {
        throw 'The build failed (see the lines above, or /tmp/s2_make.log in WSL).'
    }
    $built = (Get-Item $image).LastWriteTime.ToString('HH:mm:ss')
    Write-Host "Using $([System.IO.Path]::GetFileName($image)) (built $built)" -ForegroundColor Green
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
    $script:u11Start = $null
    $script:u11Runs  = New-Object System.Collections.Generic.List[object]
    $script:u03Runs  = New-Object System.Collections.Generic.List[object]
    $script:u11Gap   = $null
    $script:u06Side  = ''

    # Before every question: the robot has just stopped, so give the
    # tester 3 s to look at it before the terminal needs attention. The
    # Pico waits for Button 1 meanwhile, so nothing moves.
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

    function Ask-ForLine([string]$line)
    {
        switch ($script:Test)
        {
            'u11'
            {
                # Reference point: the front edge of the LEFT tyre, where it
                # touches the floor, read on a tape measure lying beside the
                # left wheel. Every point of the robot moves the same distance
                # on a straight run, so the long nose and tail do not matter.
                if ($line -match '^\[S2-STEP\] NEXT: S2-U11')
                {
                    # Any start reading works (0 on a floor line, 100 on the
                    # tape, ...): the distance is end minus start. Use the same
                    # point of the tyre at both ends.
                    $hint = 'any reading, e.g. 0 on your start line'
                    if ($line -match 'REVERSE')
                    {
                        $hint = 'the reverse run goes DOWN the tape: start at 300 or more, e.g. 400'
                    }
                    Write-Host '  Place the robot with the front edge of the LEFT tyre where it touches the floor on your start mark, wheels parallel to the tape.' -ForegroundColor Yellow
                    $script:u11Start = Ask-Number "  Start reading at that point, mm ($hint; Enter = 0)"
                    if ($null -eq $script:u11Start)
                    {
                        $script:u11Start = 0.0
                    }
                    $script:u11Gap   = Ask-Number '  Gap between the tape''s edge and the outside of the LEFT tyre, mm (0 = touching; negative if the tyre is over the tape)'
                    Write-Host '  Now press BUTTON 1.' -ForegroundColor Yellow
                }
                elseif ($line -match '^\[S2-FLOOR\] MOVE (-?\d+): odometry (-?\d+) mm \(L (-?\d+) R (-?\d+)\)')
                {
                    $script:moveRun++
                    $target = [int]$Matches[1]
                    $odomL  = [double]$Matches[3]
                    $odomR  = [double]$Matches[4]
                    $start  = $script:u11Start
                    Look-AtRobot
                    Write-Host "  Run $($script:moveRun): look straight down at the same point of the LEFT tyre as at the start." -ForegroundColor Yellow
                    $end    = Ask-Number "  Reading at that point now, mm (it started at $(Format-Value $start))"
                    $gapEnd = Ask-Number '  Gap between the tape''s edge and the outside of the left tyre now, mm'
                    $real   = $null
                    if (($null -ne $end) -and ($null -ne $start))
                    {
                        # Signed like the command: a reverse run is negative,
                        # whichever way the tape numbers run.
                        $real = [math]::Sign($target) * [math]::Abs($end - $start)
                    }
                    # The tape is on the robot's left: a smaller gap means the
                    # robot moved left (+), a bigger gap means right (-).
                    $side = $null
                    if (($null -ne $gapEnd) -and ($null -ne $script:u11Gap))
                    {
                        $side = $script:u11Gap - $gapEnd
                    }
                    Write-Log ("[MEASURED] MOVE {0} run {1}: real distance {2} mm (left tyre front edge {3} -> {4}); sideways {5} mm, + = left (tyre gap {6} -> {7})" -f
                               $target, $script:moveRun, (Format-Value $real), $start, (Format-Value $end),
                               (Format-Value $side), (Format-Value $script:u11Gap), (Format-Value $gapEnd))
                    Write-Host "  Saved: travelled $(Format-Value $real) mm, sideways $(Format-Value $side) mm." -ForegroundColor Yellow
                    if (($null -ne $real) -and ([math]::Abs($real) -gt 100) -and ($odomL -ne 0) -and ($odomR -ne 0))
                    {
                        $script:u11Runs.Add(@{ D = $real; Y = $side; L = $odomL; R = $odomR })
                    }
                }
            }
            'u03'
            {
                if ($line -match '^\[S2-FLOOR\] TURN (-?\d+):')
                {
                    $script:turnRun++
                    $target = [int]$Matches[1]
                    Look-AtRobot
                    Write-Host "  Turn $($script:turnRun): measure with the protractor at the crossing." -ForegroundColor Yellow
                    $angle = Ask-Number "  Angle actually turned, deg (commanded $target; + = left)"
                    Write-Log ("[MEASURED] TURN {0} run {1}: measured {2} deg" -f $target, $script:turnRun, (Format-Value $angle))
                    if (($null -ne $angle) -and ([math]::Abs($angle) -gt 20))
                    {
                        $script:u03Runs.Add(@{ Cmd = [double]$target; Meas = [double]$angle })
                    }
                    Write-Host '  Saved. Line the pointer up on arm F again, then press BUTTON 1.' -ForegroundColor Yellow
                }
            }
            'u12'
            {
                if ($line -match 'braking distance')
                {
                    Look-AtRobot
                    $skid = Ask-YesNo '  Did the wheels skid or slide when it stopped?'
                    Write-Log "[MEASURED] wheels skidded: $skid  (for: $line)"
                    Write-Host '  Saved. Put the robot back at the start, then press BUTTON 1.' -ForegroundColor Yellow
                }
                elseif ($line -match '^\[S2-FLOOR\] FOLLOW (\d+) mm/s:')
                {
                    Look-AtRobot
                    $smooth = Ask-YesNo "  At $($Matches[1]) mm/s did it move smoothly (no stopping or jerking)?"
                    Write-Log "[MEASURED] smooth at $($Matches[1]) mm/s: $smooth"
                    Write-Host '  Saved. Put the robot back at the start, then press BUTTON 1.' -ForegroundColor Yellow
                }
                elseif ($line -match '^\[S2-FLOOR\] stall ')
                {
                    Look-AtRobot
                    $stopped = Ask-YesNo '  Did both wheels stop by themselves while you held the left tyre?'
                    Write-Log "[MEASURED] both wheels stopped by themselves: $stopped"
                }
            }
            'u06'
            {
                if ($line -match '^\[S2-STEP\] NEXT: S2-U06 (LEFT|RIGHT)')
                {
                    $script:u06Side = $Matches[1]
                }
                elseif ($line -match '^\[S2-U06\]\s+duty (-?\d+):')
                {
                    $duty = [int]$Matches[1]
                    $side = $script:u06Side
                    Look-AtRobot
                    if ($duty -eq 400)
                    {
                        $fwd  = Ask-YesNo "  FORWARD: did the $side wheel turn the way that drives the robot FORWARD?"
                        $only = Ask-YesNo "  Did ONLY the $side wheel turn (the other one stayed still)?"
                        Write-Log "[MEASURED] $side FORWARD drives the robot forward: $fwd; only that wheel turned: $only"
                    }
                    elseif ($duty -lt 0)
                    {
                        $back = Ask-YesNo "  REVERSE: did the $side wheel turn the OPPOSITE way (drives the robot backward)?"
                        Write-Log "[MEASURED] $side REVERSE drives the robot backward: $back"
                    }
                    else
                    {
                        $fast = Ask-YesNo "  FAST: did the $side wheel turn FORWARD again, and faster?"
                        Write-Log "[MEASURED] $side FAST forward and faster: $fast"
                    }
                    Write-Host '  Saved. Press BUTTON 1 for the next run.' -ForegroundColor Yellow
                }
            }
            'u02'
            {
                if ($line -match '^\[TEST\] S2-U02 (PASS|FAIL)')
                {
                    Look-AtRobot
                    $both = Ask-YesNo '  Did BOTH wheels turn the way that drives the robot FORWARD?'
                    Write-Log "[MEASURED] both wheels turned forward: $both"
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
            if ($line -match '^\[S2-FLOOR\] HOLD') { $color = 'Cyan' }
            if ($line -match '\bPASS\b') { $color = 'Green' }
            if ($line -match '\bFAIL\b|ERROR|STALL') { $color = 'Red' }

            if ($line -match '^\[S2-STEP\] NEXT: (.*)$')
            {
                Write-Host ''
                Write-Host ('#' * 70) -ForegroundColor Yellow
                Write-Host "  NEXT: $($Matches[1])" -ForegroundColor Yellow
                if ($Test -eq 'u11')
                {
                    Write-Host '  Place the robot and answer the question below FIRST, then press BUTTON 1.' -ForegroundColor Yellow
                }
                else
                {
                    Write-Host '  Get the robot ready, then press BUTTON 1 on the robot.' -ForegroundColor Yellow
                }
                Write-Host ('#' * 70) -ForegroundColor Yellow
            }
            elseif ($line -match '^\[S2-STEP\] press')
            {
                [Console]::Beep(880, 120)
                [Console]::Beep(880, 120)
            }
            elseif ($line -match '^\[S2-STEP\] ([123])$')
            {
                Write-Host "      $($Matches[1]) ...  WATCH THE ROBOT" -ForegroundColor Magenta
                [Console]::Beep(1000, 150)
            }
            elseif ($line -match '^\[S2-STEP\] GO$')
            {
                Write-Host '      GO' -ForegroundColor Magenta
                [Console]::Beep(1500, 400)
            }
            else
            {
                Write-Host $line -ForegroundColor $color
            }

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
    #  u11: work out the wheel calibration from the measured runs.
    #  Left wheel real travel = D (the left tyre was measured). Heading
    #  change from the sideways drift: theta = 2 y / D (constant curve), so
    #  right wheel real travel = D + theta x track. Scale = real / counted.
    # -----------------------------------------------------------------
    if (($Test -eq 'u11') -and ($script:u11Runs.Count -gt 0))
    {
        $hdr   = [System.IO.File]::ReadAllText((Join-Path $s2Dir 'subsystem2.h'))
        $circL = [double]([regex]::Match($hdr, 'S2_WHEEL_CIRC_UM_L\s+\((\d+)\)').Groups[1].Value)
        $circR = [double]([regex]::Match($hdr, 'S2_WHEEL_CIRC_UM_R\s+\((\d+)\)').Groups[1].Value)
        $track = [double]([regex]::Match($hdr, 'S2_TRACK_MM\s+\((\d+)\)').Groups[1].Value)
        $kL = 0.0
        $kR = 0.0
        $nL = 0
        $nR = 0
        foreach ($r in $script:u11Runs)
        {
            $kL += $r.D / $r.L
            $nL++
            # The right wheel's travel needs the heading change, which only
            # a long run with a measured sideways drift gives accurately.
            if (($null -ne $r.Y) -and ([math]::Abs($r.D) -ge 800))
            {
                $theta = 2.0 * $r.Y / $r.D
                $kR += ($r.D + ($theta * $track)) / $r.R
                $nR++
            }
        }
        $kL /= $nL
        if ($nR -gt 0)
        {
            $kR /= $nR
        }
        else
        {
            $kR = $kL
        }
        $newL = [math]::Round($circL * $kL / 10) * 10
        $newR = [math]::Round($circR * $kR / 10) * 10
        $msg  = ('[CALIBRATION] left went {0:N2} % further than counted ({1} runs), right {2:N2} % ({3} long runs with a sideways gap)' -f
                 (($kL - 1) * 100), $nL, (($kR - 1) * 100), $nR)
        Write-Log $msg
        Write-Log "[CALIBRATION] subsystem2.h now: S2_WHEEL_CIRC_UM_L ($circL), S2_WHEEL_CIRC_UM_R ($circR)"
        Write-Log "[CALIBRATION] suggested:       S2_WHEEL_CIRC_UM_L ($newL), S2_WHEEL_CIRC_UM_R ($newR)"
        Write-Step 'Wheel calibration from your measurements'
        Write-Host $msg.Replace('[CALIBRATION] ', '')
        Write-Host "  now:       S2_WHEEL_CIRC_UM_L ($circL)   S2_WHEEL_CIRC_UM_R ($circR)"
        Write-Host "  suggested: S2_WHEEL_CIRC_UM_L ($newL)   S2_WHEEL_CIRC_UM_R ($newR)"
        if ($nR -eq 0)
        {
            Write-Host '  The right wheel could not be checked: answer the gap question for the 1000 mm runs.' -ForegroundColor Yellow
        }
        if (([math]::Abs($kL - 1) -le 0.01) -and ([math]::Abs($kR - 1) -le 0.01))
        {
            Write-Host '  Both within 1 %: the calibration is good; no change needed.' -ForegroundColor Green
        }
        else
        {
            Write-Host '  More than 1 % off: put the suggested values in app_program\subsystem2\subsystem2.h, commit, run u11 again.' -ForegroundColor Yellow
        }
    }

    # -----------------------------------------------------------------
    #  u03: a turn of theta turns each wheel by theta x pi x track / 360,
    #  so the angle really turned = commanded x track_used / track_real.
    #  Hence track_real = track_used x measured / commanded, per direction
    #  (left = positive, right = negative). Right turns typed without the
    #  minus sign are fine: magnitudes are used.
    # -----------------------------------------------------------------
    if (($Test -eq 'u03') -and ($script:u03Runs.Count -gt 0))
    {
        $hdr = [System.IO.File]::ReadAllText((Join-Path $s2Dir 'subsystem2.h'))
        Write-Step 'Turn calibration from your measurements'
        foreach ($dir in @(@('LEFT', 1), @('RIGHT', -1)))
        {
            $name  = "S2_TURN_TRACK_$($dir[0])_MM"
            $track = [double]([regex]::Match($hdr, $name + '\s+\((\d+)\)').Groups[1].Value)
            $sel   = @($script:u03Runs | Where-Object { [math]::Sign($_.Cmd) -eq $dir[1] })
            if ($sel.Count -eq 0)
            {
                continue
            }
            $ratios = @($sel | ForEach-Object { [math]::Abs($_.Meas) / [math]::Abs($_.Cmd) })
            $ratio  = ($ratios | Measure-Object -Average).Average
            $spread = (($ratios | Measure-Object -Maximum).Maximum - ($ratios | Measure-Object -Minimum).Minimum) * 90
            $newTr  = [math]::Round($track * $ratio)
            $line   = ('{0,-5} turns: {1}, average {2:N1} % of the command (as a 90: {3:N1} deg), spread {4:N1} deg; {5} is {6}, suggested {7}' -f
                       $dir[0], $sel.Count, ($ratio * 100), ($ratio * 90), $spread, $name, $track, $newTr)
            Write-Host "  $line"
            Write-Log "[CALIBRATION] $line"
            if ($spread -gt 6)
            {
                Write-Host "    The $($dir[0]) turns vary by more than 6 deg: that is slip, not calibration. Check the ball caster and the floor first." -ForegroundColor Yellow
            }
            if ([math]::Abs($ratio - 1) -le 0.02)
            {
                Write-Host "    Within 2 %: no change needed." -ForegroundColor Green
            }
            elseif ($sel.Count -lt 3)
            {
                Write-Host "    Fewer than 3 turns: too few to change the calibration." -ForegroundColor Yellow
            }
            else
            {
                Write-Host "    Put $name ($newTr) in app_program\subsystem2\subsystem2.h, commit, run u03 again." -ForegroundColor Yellow
            }
        }
    }
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
