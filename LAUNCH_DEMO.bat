@echo off
title SIH 2025 — Autonomous Navigation Judge Demo
color 0A
echo.
echo  ============================================================
echo   SIH 2025 — Autonomous Vehicle: Indian Road Scenarios
echo   Adaptive Path Planning + ML (YOLO / DeepLab / CNN)
echo  ============================================================
echo.
echo  Starting MATLAB R2026a...
echo  Two windows will open per scenario: Chase View + Top View
echo.
"C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -nosplash -nodesktop -r "cd('e:\sih_matlab'); JUDGE_DEMO"
pause
