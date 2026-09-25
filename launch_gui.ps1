# Launcher for pymmcore-gui, invoked (hidden) from the desktop shortcut via launch_gui.vbs.
Set-Location -Path $PSScriptRoot

function Show-ErrorBox($message) {
    Add-Type -AssemblyName System.Windows.Forms | Out-Null
    [System.Windows.Forms.MessageBox]::Show(
        $message, 'pymmcore-gui',
        [System.Windows.Forms.MessageBoxButtons]::OK,
        [System.Windows.Forms.MessageBoxIcon]::Error
    ) | Out-Null
}

# Pick up code changes before launching -- deliberately here, not inside the
# app: this runs before pymmcore_gui's process even starts, so there is no
# acquisition in progress yet to disrupt, unlike an in-app auto-update would
# risk. Never blocks or alarms on failure (no network, a dirty tree, a
# diverged branch, git missing) -- launches with whatever is already checked
# out and just logs why, same as if this step didn't exist. Pulls whatever
# remote/branch is already configured (`git pull --ff-only`, no args), so
# this launcher doesn't need to know whether `origin` points at GitHub or at
# Argus -- see update.log next to this script, and CONTRIBUTING.md's
# "Deploying to the acquisition PC" section for how to point it at either.
$pullLog = Join-Path $PSScriptRoot 'update.log'
try {
    $git = (Get-Command git -ErrorAction Stop).Source
    $before = & $git rev-parse HEAD 2>$null
    $pullOutput = & $git pull --ff-only 2>&1
    $pullExit = $LASTEXITCODE  # capture before the next git call overwrites it
    $after = & $git rev-parse HEAD 2>$null
    $stamp = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
    if ($pullExit -eq 0 -and $before -ne $after) {
        Add-Content -Path $pullLog -Value "$stamp updated $before -> $after"
    }
    elseif ($pullExit -ne 0) {
        Add-Content -Path $pullLog -Value "$stamp pull failed, launching as-is: $pullOutput"
    }
}
catch {
    Add-Content -Path $pullLog -Value "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') git not available, skipping update check: $($_.Exception.Message)"
}

try {
    $ErrorActionPreference = 'Stop'
    & (Join-Path $PSScriptRoot '.venv\Scripts\Activate.ps1')
}
catch {
    Show-ErrorBox "Failed to activate the virtual environment:`n$($_.Exception.Message)"
    exit 1
}

# Run via Start-Process with -RedirectStandardError so the app's stderr goes
# straight to a file at the OS level. PowerShell's own "uv run ... 2> info.log"
# redirection reads stderr through its pipeline and wraps every line as a
# NativeCommandError object -- with $ErrorActionPreference = 'Stop' that
# aborted the script on ordinary DEBUG log lines (reported as a false "launch
# failure" even though the app ran and closed normally), and even under
# 'Continue' it polluted info.log with PowerShell's error-record formatting
# instead of the app's plain log text. Start-Process avoids both.
$uv = (Get-Command uv).Source
$proc = Start-Process -FilePath $uv `
    -ArgumentList @('run', 'python', '-m', 'pymmcore_gui', 'run', '--debug') `
    -WorkingDirectory $PSScriptRoot `
    -RedirectStandardError (Join-Path $PSScriptRoot 'info.log') `
    -NoNewWindow -Wait -PassThru

if ($proc.ExitCode -ne 0) {
    Show-ErrorBox "pymmcore-gui exited with code $($proc.ExitCode).`nSee info.log for details:`n$(Join-Path $PSScriptRoot 'info.log')"
}
