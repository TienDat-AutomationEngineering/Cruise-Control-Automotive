%% build_cruise_model.m
%  Project 4 - Cruise Control (PI + back-calculation anti-windup)
%  Dung Simulink model bang API, cau hinh AUTOSAR codegen, chay SIL test.
%
%  Yeu cau: Simulink, Stateflow (cho mode mgr), Embedded Coder,
%           AUTOSAR Blockset (de dat target autosar.tlc).
%  Neu thieu AUTOSAR Blockset: van dung & mo phong duoc (bo qua phan codegen).
%
%  Chay:  >> build_cruise_model
% -------------------------------------------------------------------------

clear; clc;

%% ---- Tham so dieu khien (calibration) ----
Ts  = 0.01;     % 100 Hz
Kp  = 5.0;
Ki  = 1.5;
Kb  = 1.0;      % back-calculation gain
umin = 0.0; umax = 100.0;

%% ---- Tham so plant (longitudinal vehicle bac 1) ----
m  = 1500;      % kg
b  = 50;        % N.s/m
Kf = 60;        % N per % torque

mdl = 'CruiseControl';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

%% ---- Cau hinh solver & code ----
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts), ...
    'StopTime','40');
set_param(mdl,'SystemTargetFile','autosar.tlc');   % AUTOSAR target
% Neu khong co AUTOSAR Blockset, dung thay:
% set_param(mdl,'SystemTargetFile','ert.tlc');

%% ---- Inport: VehSpdLgt, VehSpdSetPt, CcEna ----
add_block('simulink/Sources/In1',[mdl '/VehSpdLgt'],   'Position',[20 40 50 54]);
add_block('simulink/Sources/In1',[mdl '/VehSpdSetPt'], 'Position',[20 100 50 114],'Port','2');
add_block('simulink/Sources/In1',[mdl '/CcEna'],       'Position',[20 160 50 174],'Port','3');
set_param([mdl '/CcEna'],'OutDataTypeStr','boolean');

%% ---- PI + anti-windup (dat trong Enabled Subsystem 'Cruise Active') ----
sub = [mdl '/Cruise Active'];
add_block('built-in/Subsystem',sub,'Position',[140 30 360 200]);
% (Chi tiet block trong subsystem - dung add_block/add_line tuong tu;
%  rut gon o day. Xem MODEL_SPEC.md muc 2 cho so do day du.)
%
%  e        = VehSpdSetPt - VehSpdLgt
%  presat   = Kp*e + I
%  EngTqReq = sat(presat,[0,100])
%  I        += (Ki*e + Kb*(EngTqReq-presat))*Ts     % back-calculation
%
% Goi y dung khoi:
%   Sum, Gain(Kp), Discrete-Time Integrator (Forward Euler, Ts),
%   Gain(Ki), Gain(Kb), Saturation[0 100].
% Enable port -> reset states khi CcEna=0 (StatesWhenEnabling='reset').

%% ---- Outport: EngTqReq ----
add_block('simulink/Sinks/Out1',[mdl '/EngTqReq'],'Position',[420 100 450 114]);

%% ---- Map AUTOSAR ports (neu co AUTOSAR Blockset) ----
try
    arProps = autosar.api.getAUTOSARProperties(mdl); %#ok<NASGU>
    slMap   = autosar.api.getSimulinkMapping(mdl);
    mapInport(slMap, 'VehSpdLgt',   'VehSpdLgt',   'VehSpdLgt');
    mapInport(slMap, 'VehSpdSetPt', 'VehSpdSetPt', 'VehSpdSetPt');
    mapInport(slMap, 'CcEna',       'CcEna',       'CcEna');
    mapOutport(slMap,'EngTqReq',    'EngTqReq',    'EngTqReq');
    disp('AUTOSAR port mapping done.');
catch ME
    warning('Bo qua AUTOSAR mapping (thieu AUTOSAR Blockset): %s', ME.message);
end

save_system(mdl);

%% =========================================================================
%% SIL-style verification (mo phong roi rac, doc lap voi Simulink build)
%% =========================================================================
fprintf('\n=== SIL verification (discrete PI + back-calc anti-windup) ===\n');
scen = { '1) Phang  -> 100 km/h', 0,    false;
         '2) Doc (tai 2500N)   ', 2500, false;
         '3) Windup (tai 6000N->0 @20s)', 6000, true };

for i = 1:size(scen,1)
    [ov, ess, uf, pk] = simLoop(100, scen{i,2}, scen{i,3}, ...
                                Kp,Ki,Kb,Ts,umin,umax,m,b,Kf);
    fprintf('%-30s overshoot=%5.1f%%  ess=%6.3f  u_final=%5.1f%%  peak=%6.1f\n', ...
            scen{i,1}, ov, ess, uf, pk);
end

% So sanh co/khong anti-windup cho kich ban windup
[ovAW,~,~,~]   = simLoop(100,6000,true, Kp,Ki,Kb,Ts,umin,umax,m,b,Kf);
[ovNo,~,~,~]   = simLoop(100,6000,true, Kp,Ki,0 ,Ts,umin,umax,m,b,Kf);
fprintf('\nAnti-windup effect: overshoot WITH=%.1f%%  vs  NO-AW=%.1f%%\n', ovAW, ovNo);

%% ---- (Tuy chon) sinh code AUTOSAR ----
% slbuild(mdl);   % -> tao thu muc CruiseControl_autosar_rtw/

%% ========================== local functions =============================
function [overshoot, ess, u_final, peak] = simLoop(sp,load_N,windup, ...
        Kp,Ki,Kb,Ts,umin,umax,m,b,Kf)
    T = 40; n = round(T/Ts);
    v = 0; I = 0; V = zeros(1,n); U = zeros(1,n);
    for k = 1:n
        tk  = (k-1)*Ts;
        spd = v*3.6;
        e   = sp - spd;
        pre = Kp*e + I;
        u   = min(max(pre,umin),umax);
        I   = I + (Ki*e + Kb*(u-pre))*Ts;
        if windup, ld = (tk<20)*load_N; else, ld = load_N; end
        F = Kf*u - b*v - ld;
        v = v + Ts*F/m;
        V(k) = spd; U(k) = u;
    end
    if windup
        idx = round(20/Ts):n; peak = max(V(idx));
    else
        peak = max(V);
    end
    overshoot = max(0,(peak-sp)/sp*100);
    ess = sp - V(end);
    u_final = U(end);
end

function mapInport(slMap,blk,port,elem)
    mapDataAccess(slMap,'Inport',blk,port,elem);
end
function mapOutport(slMap,blk,port,elem)
    mapDataAccess(slMap,'Outport',blk,port,elem);
end
function mapDataAccess(slMap,kind,blk,port,elem)
    if strcmp(kind,'Inport')
        slMap.mapInport(blk,port,elem,'ImplicitReceive');
    else
        slMap.mapOutport(blk,port,elem,'ImplicitSend');
    end
end
