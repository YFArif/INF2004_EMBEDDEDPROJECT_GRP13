<#
    log_serial.ps1 - show the Pico's USB console and save it to a NEW file
    testing/subsystem5/logs/<date>_<time>_<Name>.txt
    e.g. 2026-10-22_1530_unit.txt. Every run gets its own file, so old
    logs are never added to or overwritten.

    Run from the project folder (one line):
        powershell -ExecutionPolicy Bypass -File testing\subsystem5\log_serial.ps1 -Port COM5 -Name unit

    -Name is a short label for what you are testing (unit, monitor, ...).
    Start it with the Pico UNPLUGGED, then plug the Pico in: the script
    waits for the port, so no line is missed. Stop with Ctrl+C.
#>
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [string]$Name = 'unit'
)

$logDir = Join-Path $PSScriptRoot 'logs'
$stamp  = Get-Date -Format 'yyyy-MM-dd_HHmm'
$log    = Join-Path $logDir ('{0}_{1}.txt' -f $stamp, $Name)

# Two runs started in the same minute: add a number instead of reusing.
$count = 2
while (Test-Path $log)
{
    $log    = Join-Path $logDir ('{0}_{1}_{2}.txt' -f $stamp, $Name, $count)
    $count += 1
}

Write-Host "Saving to $log"
Write-Host "Waiting for $Port ... plug the Pico in now."

while (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port))
{
    Start-Sleep -Milliseconds 200
}

$sp = New-Object System.IO.Ports.SerialPort $Port, 115200
$sp.DtrEnable   = $true     # The Pico sends only once the PC says it listens.
$sp.ReadTimeout = 500       # Lets Ctrl+C stop the script promptly.
$sp.Open()
Write-Host "Connected to $Port. Press Ctrl+C to stop."

try
{
    while ($true)
    {
        try
        {
            $line = $sp.ReadLine().TrimEnd("`r")
        }
        catch [System.TimeoutException]
        {
            continue
        }

        Write-Host $line
        Add-Content -Path $log -Value $line
    }
}
catch
{
    Write-Host "Stopped: $($_.Exception.Message)"
}
finally
{
    if ($sp.IsOpen)
    {
        $sp.Close()
    }
    Write-Host "Log saved to $log"
}
