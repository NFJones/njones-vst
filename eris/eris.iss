[Setup]
AppName=Eris
AppVersion=1.1.0
DefaultDirName={cf}\VST3
DefaultGroupName=Eris
OutputBaseFilename=Eris-win64

[Files]
Source: "..\build\VST3\Release\eris.vst3\Contents\x86_64-win\eris.vst3"; DestDir: "{app}\Eris\x86_64-win"
Source: "..\build\VST3\Release\eris.vst3\Contents\Resources\background.png"; DestDir: "{app}\Eris\Resources"
Source: "..\build\VST3\Release\eris.vst3\Contents\Resources\eris.uidesc"; DestDir: "{app}\Eris\Resources"
