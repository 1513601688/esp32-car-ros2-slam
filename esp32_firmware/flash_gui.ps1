param(
    [switch]$SelfTest,
    [switch]$UiSelfTest
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

$repoDir = $PSScriptRoot
$projectDir = Join-Path $repoDir 'modules\firmware\esp32_car'
$baseDefaults = Join-Path $projectDir 'sdkconfig.defaults'
$generatedDir = Join-Path $repoDir '.flash_gui'
$generatedDefaults = Join-Path $generatedDir 'sdkconfig.defaults'
$generatedSdkconfig = Join-Path $generatedDir 'sdkconfig'
$generatedBuild = Join-Path $generatedDir 'build'
$settingsFile = Join-Path $generatedDir 'settings.json'
$flashLog = Join-Path $generatedDir 'flash.log'
$runnerFile = Join-Path $repoDir 'flash_runner.cmd'
$idfExport = 'D:\ESP-IDF\Espressif\frameworks\esp-idf-v5.3.1\export.bat'
$idfToolsPath = Join-Path $env:USERPROFILE '.espressif'
$idfPythonEnvPath = Join-Path $idfToolsPath 'python_env\idf5.3_py3.11_env'
$idfPythonExe = Join-Path $idfPythonEnvPath 'Scripts\python.exe'

function Get-ConfigValue {
    param([string]$Text, [string]$Name)
    $match = [regex]::Match($Text, '(?m)^' + [regex]::Escape($Name) + '="(.*)"\s*$')
    if ($match.Success) { return $match.Groups[1].Value }
    return ''
}

function ConvertTo-KconfigString {
    param([string]$Value)
    return $Value.Replace('\', '\\').Replace('"', '\"')
}

function New-GeneratedDefaults {
    param([string]$Ssid, [string]$Password)

    if (-not (Test-Path -LiteralPath $generatedDir)) {
        [void][System.IO.Directory]::CreateDirectory($generatedDir)
    }

    $text = [System.IO.File]::ReadAllText($baseDefaults)
    $ssidLine = 'CONFIG_CAR_WIFI_SSID="' + (ConvertTo-KconfigString $Ssid) + '"'
    $passwordLine = 'CONFIG_CAR_WIFI_PASSWORD="' + (ConvertTo-KconfigString $Password) + '"'
    $ssidEvaluator = [System.Text.RegularExpressions.MatchEvaluator]{ param($match) $ssidLine }
    $passwordEvaluator = [System.Text.RegularExpressions.MatchEvaluator]{ param($match) $passwordLine }
    $text = [regex]::Replace($text, '(?m)^CONFIG_CAR_WIFI_SSID=.*$', $ssidEvaluator)
    $text = [regex]::Replace($text, '(?m)^CONFIG_CAR_WIFI_PASSWORD=.*$', $passwordEvaluator)
    [System.IO.File]::WriteAllText($generatedDefaults, $text, [System.Text.UTF8Encoding]::new($false))
}

if ($SelfTest) {
    foreach ($required in @($projectDir, $baseDefaults, $runnerFile, $idfExport, $idfToolsPath, $idfPythonExe)) {
        if (-not (Test-Path -LiteralPath $required)) {
            throw "Missing required path: $required"
        }
    }
    Write-Output 'Self-test passed.'
    exit 0
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::SetUnhandledExceptionMode(
    [System.Windows.Forms.UnhandledExceptionMode]::CatchException)
[System.Windows.Forms.Application]::EnableVisualStyles()
[System.Windows.Forms.Application]::add_ThreadException({
    param($sender, $eventArgs)
    try {
        if (-not (Test-Path -LiteralPath $generatedDir)) {
            [void][System.IO.Directory]::CreateDirectory($generatedDir)
        }
        $crashFile = Join-Path $generatedDir 'crash.log'
        [System.IO.File]::WriteAllText($crashFile, $eventArgs.Exception.ToString(), [System.Text.UTF8Encoding]::new($false))
    } catch { }
    [System.Windows.Forms.MessageBox]::Show(
        "The flasher encountered an error but will remain open.`r`n`r`n$($eventArgs.Exception.Message)`r`n`r`nDetails: .flash_gui\crash.log",
        'ESP32 flasher error', 'OK', 'Error') | Out-Null
})

$baseText = [System.IO.File]::ReadAllText($baseDefaults)
$initialSsid = Get-ConfigValue $baseText 'CONFIG_CAR_WIFI_SSID'
$initialPassword = Get-ConfigValue $baseText 'CONFIG_CAR_WIFI_PASSWORD'
$initialPort = ''

if (Test-Path -LiteralPath $settingsFile) {
    try {
        $saved = [System.IO.File]::ReadAllText($settingsFile) | ConvertFrom-Json
        if ($null -ne $saved.ssid) { $initialSsid = [string]$saved.ssid }
        if ($null -ne $saved.password) { $initialPassword = [string]$saved.password }
        if ($null -ne $saved.port) { $initialPort = [string]$saved.port }
    } catch {
        # Ignore an invalid local settings file and use project defaults.
    }
}

$form = New-Object System.Windows.Forms.Form
$form.Text = 'ESP32-S3 Car Flasher'
$form.StartPosition = 'CenterScreen'
$form.Size = New-Object System.Drawing.Size(900, 680)
$form.MinimumSize = New-Object System.Drawing.Size(760, 580)
$form.Font = New-Object System.Drawing.Font('Segoe UI', 10)

$top = New-Object System.Windows.Forms.TableLayoutPanel
$top.Dock = 'Top'
$top.Height = 178
$top.Padding = New-Object System.Windows.Forms.Padding(14, 12, 14, 4)
$top.ColumnCount = 4
$top.RowCount = 4
[void]$top.ColumnStyles.Add((New-Object System.Windows.Forms.ColumnStyle('Absolute', 120)))
[void]$top.ColumnStyles.Add((New-Object System.Windows.Forms.ColumnStyle('Percent', 55)))
[void]$top.ColumnStyles.Add((New-Object System.Windows.Forms.ColumnStyle('Absolute', 110)))
[void]$top.ColumnStyles.Add((New-Object System.Windows.Forms.ColumnStyle('Percent', 45)))

function New-Label([string]$text) {
    $label = New-Object System.Windows.Forms.Label
    $label.Text = $text
    $label.Dock = 'Fill'
    $label.TextAlign = 'MiddleLeft'
    return $label
}

$ssidBox = New-Object System.Windows.Forms.TextBox
$ssidBox.Dock = 'Fill'
$ssidBox.Text = $initialSsid
$passwordBox = New-Object System.Windows.Forms.TextBox
$passwordBox.Dock = 'Fill'
$passwordBox.UseSystemPasswordChar = $true
$passwordBox.Text = $initialPassword
$showPassword = New-Object System.Windows.Forms.CheckBox
$showPassword.Text = 'Show password'
$showPassword.Dock = 'Fill'
$portBox = New-Object System.Windows.Forms.ComboBox
$portBox.Dock = 'Fill'
$portBox.DropDownStyle = 'DropDown'
$refreshButton = New-Object System.Windows.Forms.Button
$refreshButton.Text = 'Refresh ports'
$refreshButton.Dock = 'Fill'
$startButton = New-Object System.Windows.Forms.Button
$startButton.Text = 'Build and flash'
$startButton.Dock = 'Fill'
$startButton.BackColor = [System.Drawing.Color]::FromArgb(35, 130, 75)
$startButton.ForeColor = [System.Drawing.Color]::White
$startButton.FlatStyle = 'Flat'
$cancelButton = New-Object System.Windows.Forms.Button
$cancelButton.Text = 'Cancel'
$cancelButton.Dock = 'Fill'
$cancelButton.Enabled = $false

$top.Controls.Add((New-Label 'Wi-Fi name (SSID)'), 0, 0)
$top.Controls.Add($ssidBox, 1, 0)
$top.SetColumnSpan($ssidBox, 3)
$top.Controls.Add((New-Label 'Wi-Fi password'), 0, 1)
$top.Controls.Add($passwordBox, 1, 1)
$top.SetColumnSpan($passwordBox, 2)
$top.Controls.Add($showPassword, 3, 1)
$top.Controls.Add((New-Label 'Serial port'), 0, 2)
$top.Controls.Add($portBox, 1, 2)
$top.Controls.Add($refreshButton, 2, 2)
$top.Controls.Add((New-Label 'Settings are stored locally in .flash_gui'), 3, 2)
$top.Controls.Add($startButton, 1, 3)
$top.Controls.Add($cancelButton, 2, 3)

$statusPanel = New-Object System.Windows.Forms.TableLayoutPanel
$statusPanel.Dock = 'Top'
$statusPanel.Height = 103
$statusPanel.Padding = New-Object System.Windows.Forms.Padding(14, 5, 14, 8)
$statusPanel.ColumnCount = 1
$statusPanel.RowCount = 3
[void]$statusPanel.RowStyles.Add((New-Object System.Windows.Forms.RowStyle('Absolute', 28)))
[void]$statusPanel.RowStyles.Add((New-Object System.Windows.Forms.RowStyle('Absolute', 29)))
[void]$statusPanel.RowStyles.Add((New-Object System.Windows.Forms.RowStyle('Absolute', 27)))
$statusLabel = New-Object System.Windows.Forms.Label
$statusLabel.Dock = 'Fill'
$statusLabel.Text = 'Ready'
$statusLabel.TextAlign = 'MiddleLeft'
$addressLink = New-Object System.Windows.Forms.LinkLabel
$addressLink.Dock = 'Fill'
$addressLink.Text = 'Device page: searching for esp32car.local...'
$addressLink.TextAlign = 'MiddleLeft'
$addressLink.Enabled = $false
$progress = New-Object System.Windows.Forms.ProgressBar
$progress.Dock = 'Fill'
$progress.Minimum = 0
$progress.Maximum = 100

$statusPanel.Controls.Add($statusLabel, 0, 0)
$statusPanel.Controls.Add($addressLink, 0, 1)
$statusPanel.Controls.Add($progress, 0, 2)

$logBox = New-Object System.Windows.Forms.RichTextBox
$logBox.Dock = 'Fill'
$logBox.ReadOnly = $true
$logBox.BackColor = [System.Drawing.Color]::FromArgb(24, 26, 28)
$logBox.ForeColor = [System.Drawing.Color]::Gainsboro
$logBox.Font = New-Object System.Drawing.Font('Consolas', 9)
$logBox.WordWrap = $false
$logBox.DetectUrls = $false

$form.Controls.Add($logBox)
$form.Controls.Add($statusPanel)
$form.Controls.Add($top)

$script:flashProcess = $null
$script:cancelRequested = $false
$script:currentPhase = 'ready'
$script:logPosition = 0L
$script:discoveryAttempts = 0

function Set-ProgressState([int]$Value, [string]$Text) {
    $progress.Value = [Math]::Max(0, [Math]::Min(100, $Value))
    $statusLabel.Text = $Text
}

function Add-LogLine([string]$Line) {
    if ($null -eq $Line) { return }
    $logBox.AppendText($Line + [Environment]::NewLine)
    $logBox.SelectionStart = $logBox.TextLength
    $logBox.ScrollToCaret()
}

function Read-NewFlashLog {
    if (-not (Test-Path -LiteralPath $flashLog)) { return }

    $stream = $null
    $reader = $null
    try {
        $stream = [System.IO.FileStream]::new(
            $flashLog,
            [System.IO.FileMode]::Open,
            [System.IO.FileAccess]::Read,
            [System.IO.FileShare]::ReadWrite)
        [void]$stream.Seek($script:logPosition, [System.IO.SeekOrigin]::Begin)
        $reader = [System.IO.StreamReader]::new($stream, [System.Text.Encoding]::UTF8, $true)
        while (($line = $reader.ReadLine()) -ne $null) {
            Add-LogLine $line
            Update-ProgressFromOutput $line
        }
        $script:logPosition = $stream.Position
    } catch {
        # The runner may briefly have the log locked while it starts. Retry next tick.
    } finally {
        if ($null -ne $reader) { $reader.Dispose() }
        elseif ($null -ne $stream) { $stream.Dispose() }
    }
}

function Set-ControlsRunning([bool]$Running) {
    $ssidBox.Enabled = -not $Running
    $passwordBox.Enabled = -not $Running
    $portBox.Enabled = -not $Running
    $refreshButton.Enabled = -not $Running
    $startButton.Enabled = -not $Running
    $cancelButton.Enabled = $Running
}

function Complete-FlashProcess([int]$ExitCode) {
    Set-ControlsRunning $false
    $script:flashProcess = $null

    if ($script:cancelRequested) {
        $statusLabel.Text = 'Cancelled.'
        $progress.Value = 0
    } elseif ($ExitCode -eq 0) {
        $statusLabel.Text = 'Flash completed successfully. Waiting for the device network...'
        $progress.Value = 100
        [System.Media.SystemSounds]::Asterisk.Play()
        Start-DeviceDiscovery
    } else {
        $statusLabel.Text = "Failed (exit code $ExitCode). The window will stay open; check the log below."
        $addressLink.Text = 'Device page: unavailable because flashing failed.'
        $addressLink.Enabled = $false
        [System.Media.SystemSounds]::Hand.Play()
    }
}

function Update-ProgressFromOutput([string]$Line) {
    if ($Line -eq '__PHASE_ENV__') { $script:currentPhase = 'env'; Set-ProgressState 3 'Loading ESP-IDF environment...'; return }
    if ($Line -eq '__PHASE_CONFIG__') { $script:currentPhase = 'config'; Set-ProgressState 8 'Configuring ESP32-S3 project...'; return }
    if ($Line -eq '__PHASE_BUILD__') { $script:currentPhase = 'build'; Set-ProgressState 15 'Building firmware and webpage...'; return }
    if ($Line -eq '__PHASE_FLASH__') { $script:currentPhase = 'flash'; Set-ProgressState 72 'Flashing device...'; return }
    if ($Line -eq '__PHASE_DONE__') { $script:currentPhase = 'done'; Set-ProgressState 100 'Flash completed successfully.'; return }

    $buildMatch = [regex]::Match($Line, '^\[(\d+)/(\d+)\]')
    if ($buildMatch.Success) {
        $current = [double]$buildMatch.Groups[1].Value
        $total = [double]$buildMatch.Groups[2].Value
        if ($total -gt 0) {
            Set-ProgressState ([int](15 + (55 * $current / $total))) "Building firmware... $([int](100 * $current / $total))%"
        }
        return
    }

    $flashMatch = [regex]::Match($Line, '\((\d+)\s*%\)')
    if ($flashMatch.Success -and $script:currentPhase -eq 'flash') {
        $percent = [int]$flashMatch.Groups[1].Value
        Set-ProgressState ([int](72 + (25 * $percent / 100))) "Flashing device... $percent%"
    }
}

function Stop-FlashProcessTree {
    if ($null -eq $script:flashProcess -or $script:flashProcess.HasExited) { return }

    $killInfo = New-Object System.Diagnostics.ProcessStartInfo
    $killInfo.FileName = 'taskkill.exe'
    $killInfo.Arguments = '/PID ' + $script:flashProcess.Id + ' /T /F'
    $killInfo.UseShellExecute = $false
    $killInfo.CreateNoWindow = $true
    try {
        $killer = [System.Diagnostics.Process]::Start($killInfo)
        $killer.WaitForExit(5000) | Out-Null
    } catch {
        try { $script:flashProcess.Kill() } catch { }
    }
}

function Refresh-Ports {
    $selected = [string]$portBox.Text
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object { [int]($_ -replace '\D', '') })
    $portBox.Items.Clear()
    foreach ($port in $ports) { [void]$portBox.Items.Add($port) }
    if ($selected) {
        $portBox.Text = $selected
    } elseif ($initialPort) {
        $portBox.Text = $initialPort
    } elseif ($ports.Count -gt 0) {
        $portBox.SelectedIndex = 0
    }
}

function Test-DeviceHttpPort([System.Net.IPAddress]$Address) {
    $client = $null
    $async = $null
    try {
        $client = [System.Net.Sockets.TcpClient]::new()
        $async = $client.BeginConnect($Address, 80, $null, $null)
        if (-not $async.AsyncWaitHandle.WaitOne(500)) { return $false }
        $client.EndConnect($async)
        return $client.Connected
    } catch {
        return $false
    } finally {
        if ($null -ne $async) { $async.AsyncWaitHandle.Close() }
        if ($null -ne $client) { $client.Close() }
    }
}

function Start-DeviceDiscovery {
    $script:discoveryAttempts = 0
    $addressLink.Text = 'Device page: searching for the current IP...'
    $addressLink.Tag = $null
    $addressLink.Enabled = $false
    $addressTimer.Start()
}

$showPassword.Add_CheckedChanged({ $passwordBox.UseSystemPasswordChar = -not $showPassword.Checked })
$refreshButton.Add_Click({ Refresh-Ports })
$addressLink.Add_LinkClicked({
    $url = [string]$addressLink.Tag
    if (-not [string]::IsNullOrWhiteSpace($url)) {
        $openInfo = New-Object System.Diagnostics.ProcessStartInfo
        $openInfo.FileName = $url
        $openInfo.UseShellExecute = $true
        try { [void][System.Diagnostics.Process]::Start($openInfo) } catch {
            [System.Windows.Forms.MessageBox]::Show($_.Exception.Message, 'Cannot open browser', 'OK', 'Error') | Out-Null
        }
    }
})

$addressTimer = New-Object System.Windows.Forms.Timer
$addressTimer.Interval = 1000
$addressTimer.Add_Tick({
    $script:discoveryAttempts++
    try {
        $addresses = @([System.Net.Dns]::GetHostAddresses('esp32car.local') |
            Where-Object { $_.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork })
        foreach ($address in $addresses) {
            if (Test-DeviceHttpPort $address) {
                $url = 'http://' + $address.IPAddressToString + '/'
                $addressLink.Text = "Device page: $url  (click to open)"
                $addressLink.Tag = $url
                $addressLink.Enabled = $true
                $statusLabel.Text = 'Device is online and the frontend is reachable.'
                $addressTimer.Stop()
                return
            }
        }
    } catch { }

    $addressLink.Text = "Device page: waiting for esp32car.local... ($($script:discoveryAttempts)/30)"
    if ($script:discoveryAttempts -ge 30) {
        $addressTimer.Stop()
        $addressLink.Text = 'Device page: IP not detected. Check that this PC is on the same Wi-Fi.'
        $addressLink.Enabled = $false
    }
})

$pollTimer = New-Object System.Windows.Forms.Timer
$pollTimer.Interval = 200
$pollTimer.Add_Tick({
    Read-NewFlashLog
    if ($null -ne $script:flashProcess -and $script:flashProcess.HasExited) {
        $exitCode = $script:flashProcess.ExitCode
        Read-NewFlashLog
        $pollTimer.Stop()
        Complete-FlashProcess $exitCode
    }
})

$startButton.Add_Click({
    $ssid = $ssidBox.Text.Trim()
    $password = $passwordBox.Text
    $port = $portBox.Text.Trim().ToUpperInvariant()

    if ([string]::IsNullOrWhiteSpace($ssid)) {
        [System.Windows.Forms.MessageBox]::Show('Wi-Fi name cannot be empty.', 'Invalid settings', 'OK', 'Warning') | Out-Null
        return
    }
    if ($ssid.Contains("`r") -or $ssid.Contains("`n") -or $password.Contains("`r") -or $password.Contains("`n")) {
        [System.Windows.Forms.MessageBox]::Show('Wi-Fi values cannot contain line breaks.', 'Invalid settings', 'OK', 'Warning') | Out-Null
        return
    }
    if ($port -notmatch '^COM\d+$') {
        [System.Windows.Forms.MessageBox]::Show('Select or enter a valid COM port, for example COM15.', 'Invalid port', 'OK', 'Warning') | Out-Null
        return
    }
    foreach ($required in @($projectDir, $baseDefaults, $runnerFile, $idfExport, $idfToolsPath, $idfPythonExe)) {
        if (-not (Test-Path -LiteralPath $required)) {
            [System.Windows.Forms.MessageBox]::Show("Required path was not found:`r`n$required", 'Cannot start', 'OK', 'Error') | Out-Null
            return
        }
    }

    try {
        New-GeneratedDefaults $ssid $password
        $settings = [ordered]@{ ssid = $ssid; password = $password; port = $port }
        [System.IO.File]::WriteAllText($settingsFile, ($settings | ConvertTo-Json), [System.Text.UTF8Encoding]::new($false))

        if (Test-Path -LiteralPath $generatedSdkconfig) {
            [System.IO.File]::Delete($generatedSdkconfig)
        }
    } catch {
        [System.Windows.Forms.MessageBox]::Show($_.Exception.Message, 'Configuration error', 'OK', 'Error') | Out-Null
        return
    }

    $logBox.Clear()
    Add-LogLine "Project: $projectDir"
    Add-LogLine "Port: $port"
    Add-LogLine "Wi-Fi: $ssid"
    Add-LogLine 'The password is hidden from this log.'
    Set-ProgressState 1 'Starting...'

    $ssidBox.Enabled = $false
    $passwordBox.Enabled = $false
    $portBox.Enabled = $false
    $refreshButton.Enabled = $false
    $startButton.Enabled = $false
    $cancelButton.Enabled = $true
    $script:cancelRequested = $false
    $script:currentPhase = 'starting'
    $script:logPosition = 0L
    $addressTimer.Stop()
    $addressLink.Text = 'Device page: will be detected after flashing.'
    $addressLink.Tag = $null
    $addressLink.Enabled = $false

    if (Test-Path -LiteralPath $flashLog) {
        try { [System.IO.File]::Delete($flashLog) } catch {
            Add-LogLine "Cannot clear the previous log: $($_.Exception.Message)"
            Set-ControlsRunning $false
            return
        }
    }

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $env:ComSpec
    $psi.Arguments = '/d /s /c ""' + $runnerFile + '" "' + $projectDir + '" "' + $port + '" "' + $generatedDefaults + '" "' + $idfExport + '" "' + $generatedSdkconfig + '" "' + $generatedBuild + '" > "' + $flashLog + '" 2>&1"'
    $psi.WorkingDirectory = $repoDir
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    $psi.EnvironmentVariables['IDF_TOOLS_PATH'] = $idfToolsPath
    $psi.EnvironmentVariables['IDF_PYTHON_ENV_PATH'] = $idfPythonEnvPath

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $psi
    $script:flashProcess = $process

    try {
        [void]$process.Start()
        $pollTimer.Start()
    } catch {
        Set-ControlsRunning $false
        $script:flashProcess = $null
        $statusLabel.Text = 'Could not start the build process.'
        Add-LogLine $_.Exception.Message
    }
})

$cancelButton.Add_Click({
    if ($null -ne $script:flashProcess -and -not $script:flashProcess.HasExited) {
        $script:cancelRequested = $true
        $statusLabel.Text = 'Cancelling...'
        Stop-FlashProcessTree
    }
})

$form.Add_FormClosing({
    if ($null -ne $script:flashProcess -and -not $script:flashProcess.HasExited) {
        $choice = [System.Windows.Forms.MessageBox]::Show(
            'A flash operation is still running. Cancel it and close?',
            'Confirm close', 'YesNo', 'Warning')
        if ($choice -ne 'Yes') {
            $_.Cancel = $true
        } else {
            Stop-FlashProcessTree
        }
    }
})

Refresh-Ports
if ($UiSelfTest) {
    $pollTimer.Start()
    $pollTimer.Stop()
    $form.Dispose()
    Write-Output 'UI self-test passed.'
    exit 0
}
Start-DeviceDiscovery
[void]$form.ShowDialog()
