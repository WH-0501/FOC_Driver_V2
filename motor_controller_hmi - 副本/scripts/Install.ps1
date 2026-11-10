# Per-user installer; no elevation or machine driver modification.
$ErrorActionPreference='Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()
$form=New-Object System.Windows.Forms.Form
$form.Text='MotorTune 安装向导';$form.Size=New-Object System.Drawing.Size(620,350);$form.StartPosition='CenterScreen'
$label=New-Object System.Windows.Forms.Label;$label.Text='安装目录';$label.SetBounds(20,25,100,25);$form.Controls.Add($label)
$path=New-Object System.Windows.Forms.TextBox;$path.Text=Join-Path $env:LOCALAPPDATA 'Programs\MotorTune';$path.SetBounds(120,25,440,25);$form.Controls.Add($path)
$role=New-Object System.Windows.Forms.ComboBox;$role.DropDownStyle='DropDownList';$role.Items.AddRange(@('研发工作台','产线工作台'));$role.SelectedIndex=0;$role.SetBounds(120,75,250,25);$form.Controls.Add($role)
$lock=New-Object System.Windows.Forms.CheckBox;$lock.Text='锁定为产线工作台';$lock.SetBounds(120,115,300,25);$form.Controls.Add($lock)
$shortcut=New-Object System.Windows.Forms.CheckBox;$shortcut.Text='创建桌面快捷方式';$shortcut.Checked=$true;$shortcut.SetBounds(120,150,300,25);$form.Controls.Add($shortcut)
$status=New-Object System.Windows.Forms.Label;$status.SetBounds(20,190,550,50);$status.Text='包含 Qt 与兼容后端运行时。USB 内核驱动须按厂商说明安装。';$form.Controls.Add($status)
$install=New-Object System.Windows.Forms.Button;$install.Text='安装';$install.SetBounds(440,255,120,35);$form.Controls.Add($install)
$install.Add_Click({
    try {
        $destination=[IO.Path]::GetFullPath($path.Text)
        if($destination.TrimEnd('\') -eq $PSScriptRoot.TrimEnd('\')){throw '请使用不同于解压目录的安装目录'}
        if(-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'MotorTune.exe'))){throw '请从 release 或解压后的发布包运行安装向导'}
        New-Item -ItemType Directory -Path $destination -Force | Out-Null
        Get-ChildItem -LiteralPath $PSScriptRoot | ForEach-Object {Copy-Item -LiteralPath $_.FullName -Destination $destination -Recurse -Force}
        $locked=if($lock.Checked){'true'}else{'false'}
        $factory=if($lock.Checked -or $role.SelectedIndex -eq 1){'true'}else{'false'}
        "[General]`nfactory=$factory`nfactoryLocked=$locked" | Set-Content -LiteralPath (Join-Path $destination 'installation.ini') -Encoding utf8
        if($shortcut.Checked){$shell=New-Object -ComObject WScript.Shell;$link=$shell.CreateShortcut((Join-Path ([Environment]::GetFolderPath('Desktop')) 'MotorTune.lnk'));$link.TargetPath=Join-Path $destination 'MotorTune.exe';$link.WorkingDirectory=$destination;$link.Save()}
        "$(Get-Date -Format o) Installed MotorTune 1.0.0; factory=$factory; locked=$locked" | Add-Content -LiteralPath (Join-Path $destination 'install.log')
        $status.Text="安装完成：$destination";$install.Enabled=$false
    } catch { [System.Windows.Forms.MessageBox]::Show($_.Exception.Message,'安装失败') | Out-Null }
})
[void]$form.ShowDialog()
