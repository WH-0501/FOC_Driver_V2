param([string]$QtRoot='C:\Qt\6.12.0\mingw_64',[string]$CompilerRoot='C:\Qt\Tools\mingw1310_64',[string]$NinjaRoot='C:\Qt\Tools\Ninja')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot -Parent
$env:PATH="$CompilerRoot\bin;$QtRoot\bin;$NinjaRoot;$env:PATH"
Push-Location -LiteralPath $projectRoot
try {
    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release "-DCMAKE_PREFIX_PATH=$QtRoot" "-DCMAKE_CXX_COMPILER=$CompilerRoot/bin/g++.exe"
    if($LASTEXITCODE){throw 'CMake configure failed'}
    cmake --build build -j 4
    if($LASTEXITCODE){throw 'Build failed'}
    Copy-Item -LiteralPath compat -Destination build -Recurse -Force
    ctest --test-dir build --output-on-failure
    if($LASTEXITCODE){throw 'Tests failed'}
    cmake --install build --prefix release
    if($LASTEXITCODE){throw 'Qt deployment failed'}
    Copy-Item -LiteralPath library -Destination release -Recurse -Force -ErrorAction SilentlyContinue
    Copy-Item scripts\Install.ps1,scripts\Install.cmd -Destination release -Force
    Write-Host "Release ready: $projectRoot\release"
} finally { Pop-Location }
