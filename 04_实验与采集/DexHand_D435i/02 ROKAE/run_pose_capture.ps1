param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ScriptArgs
)

$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$PythonExe = Join-Path $ProjectDir ".venv\Scripts\python.exe"
$Script = Join-Path $ProjectDir "eye_in_hand_pose_capture.py"

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Local Python environment not found: $PythonExe"
}

& $PythonExe $Script @ScriptArgs
exit $LASTEXITCODE
