@echo off
set rootPath=%cd%

rmdir /s /q "%cd%\test\UnityTest\Assets\Box2DSharp"

xcopy /s /y /q "%cd%\src" "%cd%\test\UnityTest\Assets\Box2DSharp\Box2DSharp\"
xcopy /s /y /q "%cd%\test\Testbed.Abstractions" "%cd%\test\UnityTest\Assets\Box2DSharp\Testbed.Abstractions\"
xcopy /s /y /q "%cd%\test\Testbed.TestCases" "%cd%\test\UnityTest\Assets\Box2DSharp\Testbed.TestCases\"

cd "%cd%\test\UnityTest\Assets\Box2DSharp"
for /d /r . %%d in (bin,obj) do @if exist "%%d" rd /s/q "%%d"
cd %rootPath%
