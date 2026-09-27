% SetupML.m — Master ML Setup Script
% Trains all 3 models: Custom ResNet Detector + DeepLab + CNN
% NO EXTRA SUPPORT PACKAGES REQUIRED
clc; clear;
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║     SIH 2025 — ML Setup (No Extra Packages)          ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n\n');
addpath(genpath('utils')); addpath(genpath('ml')); addpath(genpath('scenarios'));

%% Step 1: Custom ResNet-18 Detector (replaces YOLO)
fprintf('═══ Step 1/3: Training Custom ResNet-18 Detector ═══\n');
fprintf('    Input : 796 real IDD image patches (person/cow/auto/etc)\n');
fprintf('    Model : ResNet-18 fine-tuned for patch classification\n\n');
try
    cd ml; TrainCustomDetector; cd ..;
    fprintf('\n  ✓ Custom detector trained!\n\n');
catch e
    fprintf('\n  [WARN] %s\n  Continuing...\n\n', e.message);
    cd(fileparts(mfilename('fullpath')));
end

%% Step 2: DeepLab Segmentation
fprintf('═══ Step 2/3: Training DeepLab Road Segmentation ═══\n');
fprintf('    Input : IDD images (road vs background mask)\n\n');
try
    cd ml; TrainSegmentation; cd ..;
    fprintf('\n  ✓ DeepLab trained!\n\n');
catch e
    fprintf('\n  [WARN] %s\n  Continuing...\n\n', e.message);
    cd(fileparts(mfilename('fullpath')));
end

%% Step 3: CNN Behavioral Clone
fprintf('═══ Step 3/3: Training CNN Behavioral Clone ═══\n');
fprintf('    Input : Simulation recordings (speed + steer)\n\n');
try
    cd ml; TrainBehavioralClone; cd ..;
    fprintf('\n  ✓ CNN trained!\n\n');
catch e
    fprintf('\n  [WARN] %s\n  Continuing...\n\n', e.message);
    cd(fileparts(mfilename('fullpath')));
end

%% Final
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════╗\n');
fprintf('║  ML Setup Complete — Run: RunAllScenarios            ║\n');
fprintf('╚═══════════════════════════════════════════════════════╝\n');
fprintf('\nTo get YOLOv4 (better accuracy):\n');
fprintf('  MATLAB → Add-Ons → "Computer Vision Toolbox Model for YOLO v4"\n\n');
