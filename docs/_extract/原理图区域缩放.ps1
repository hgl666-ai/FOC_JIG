param(
  [double]$X0 = 0, [double]$X1 = 1689.84,
  [double]$Y0 = 0, [double]$Y1 = 1195.92,
  [double]$CW = 4.0, [double]$CH = 5.0
)
$ErrorActionPreference = 'Stop'
[xml]$doc = Get-Content 'D:\Project\FOC_JIG\_render\bbox.xml' -Encoding UTF8
$words = @()
foreach ($w in $doc.html.body.doc.page.word) {
  $x = [double]$w.xMin; $y = [double]$w.yMin
  if ($x -ge $X0 -and $x -le $X1 -and $y -ge $Y0 -and $y -le $Y1) {
    $words += [pscustomobject]@{ x = $x; y = $y; t = [string]$w.'#text' }
  }
}
$cols = [int][math]::Ceiling(($X1 - $X0) / $CW)
$rows = [int][math]::Ceiling(($Y1 - $Y0) / $CH)
$grid = New-Object 'string[,]' $rows, $cols
foreach ($w in $words) {
  $r = [int][math]::Floor(($w.y - $Y0) / $CH); if ($r -lt 0) { $r = 0 }; if ($r -ge $rows) { $r = $rows - 1 }
  $c = [int][math]::Round(($w.x - $X0) / $CW); if ($c -lt 0) { $c = 0 }; if ($c -ge $cols) { $c = $cols - 1 }
  for ($i = 0; $i -lt $w.t.Length; $i++) {
    $cc = $c + $i
    if ($cc -ge $cols) { break }
    $grid[$r, $cc] = $w.t.Substring($i, 1)
  }
}
for ($r = 0; $r -lt $rows; $r++) {
  $line = New-Object char[] $cols
  for ($c = 0; $c -lt $cols; $c++) { if ($grid[$r, $c]) { $line[$c] = [char]$grid[$r, $c] } else { $line[$c] = ' ' } }
  $txt = (-join $line).TrimEnd()
  if ($txt.Trim().Length -gt 0) {
    Write-Output ("[y{0,5:N0}] {1}" -f ($Y0 + $r * $CH), $txt)
  }
}
