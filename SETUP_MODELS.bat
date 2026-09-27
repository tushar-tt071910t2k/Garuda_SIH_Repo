@echo off
title SIH 2025 — Setup All ML Models
echo.
echo  ╔═══════════════════════════════════════════════╗
echo  ║   SIH 2025 — Training All 3 ML Models       ║
echo  ║   Run this ONCE before LAUNCH_DEMO.bat       ║
echo  ╚═══════════════════════════════════════════════╝
echo.
echo  This will take ~10 minutes. Do not close MATLAB.
echo.

"C:\Program Files\MATLAB\R2026a\bin\matlab.exe" -nosplash -r "run('e:\sih_matlab\SetupAllModels.m')"
