clc; close all; clear;

% wheel inputs
mwheel = 1.08862;
rwheel = 0.127;

% speed input (mph)
vmgoal = 20;

% constants
rtr = 60 / (2*pi);
t = 10;

% soccer ball constants
mball = .41;
rball = .11;

% computations
Iball = (2/3) * mball * rball ^ 2 + mball * (rwheel + rball) ^ 2;
Iwheel = .5 * mwheel * rwheel ^ 2;

vgoal = vmgoal / 2.237;
w = vgoal / (rball + rwheel);

w0 = ((2 * Iwheel + Iball) * w) / (2 * Iwheel);

j = Iwheel* (w0 - w) / t;

RPM = w0 * rtr;
RPM1 = w * rtr;

% disp answer
%figure;
%scatter(vmgoal, RPM);

%figure;
%scatter(vmgoal, j);

disp("RPM Requirement: " + RPM);
disp("RPM After Launch: " + RPM1);
disp("Avg torque [kg-cm] for " + t + " second recovery: " + j * 100/9.8);
