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
