@echo off
set rootPath=%cd%

rmdir /s /q "%cd%\Assets\Box2DSharp"

xcopy /s /y /q "%cd%\..\..\src" "%cd%\Assets\Box2DSharp\Box2DSharp\"
xcopy /s /y /q "%cd%\..\Testbed.Abstractions" "%cd%\Assets\Box2DSharp\Testbed.Abstractions\"
xcopy /s /y /q "%cd%\..\Testbed.TestCases" "%cd%\Assets\Box2DSharp\Testbed.TestCases\"

cd "%cd%\Assets\Box2DSharp"
for /d /r . %%d in (bin,obj) do @if exist "%%d" rd /s/q "%%d"
cd %rootPath%
