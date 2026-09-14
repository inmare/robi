$repo = $PSScriptRoot
$branch = "main"

while ($true) {
    git -C $repo fetch origin $branch --quiet

    $behind = git -C $repo rev-list --count "HEAD..origin/$branch"

    if ([int]$behind -gt 0) {
        Write-Host "$(Get-Date): 변경 감지, pull 실행"
        git -C $repo pull --ff-only origin $branch
    }

    Start-Sleep -Seconds 15
}