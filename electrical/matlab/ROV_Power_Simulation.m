%% ROV_Power_Simulation.m
% MATLAB simulation of the "ROV Electrical Power Post-Mortem" (Rev 1.0)
%   6S LiPo (22.2 V, 8 Ah) | 4- and 6-thruster configurations
%
% What it does
%   1. Static case study C1-C8 (4T and 6T): power, battery current with I^2R
%      sag, bus voltage, losses, fuse check, runtime.
%   2. Thruster model  P_full(V)=645*(V/20)^2.24 , P(u)=Pmax*u^3.
%   3. Mixed-mission endurance (weighted average, as in the report).
%   4. Time-domain mission simulation with coulomb counting, an open-circuit
%      voltage vs SoC curve, and the firmware power manager from Section 13
%      (voltage-compensated 400 W cap, 1700 W global budget, derate, cutoff).
%   5. Contactor pre-charge transient (Section 11).
%
% Requires MATLAB R2016b or newer (local functions in scripts). No toolboxes.
% Run:  >> ROV_Power_Simulation

clear; close all; clc;

P = rov_params();

%% ------------------------------------------------------------------------
%  1. OPERATING-CASE TABLE (C1-C8)
% -------------------------------------------------------------------------
cfgs = struct('name',{'4T','6T'}, 'nH',{2,4}, 'nV',{2,2});
ids  = {'C1','C2','C3','C4','C5','C6','C7','C8'};

R = cell(1,2);            % results, per config
for c = 1:2
    for k = 1:8
        R{c}(k) = case_eval(P, cfgs(c), k, false);
    end
end
Rb = case_eval(P, cfgs(2), 6, true);        % 6T C6 with 1700 W budget

for c = 1:2
    fprintf('\n=== %s configuration (Qusable = %.1f Ah) ===\n', cfgs(c).name, P.Qusable);
    fprintf('%-4s %9s %9s %8s %8s %9s %9s %8s  %s\n', ...
        'Case','Thr [W]','Total [W]','I [A]','Vbus [V]','I2R [W]','Run [min]','Hotel[W]','Fuse');
    for k = 1:8
        r = R{c}(k);
        flag = ''; if r.I > P.Ifuse, flag = '>100A FUSE'; end
        fprintf('%-4s %9.0f %9.1f %8.1f %8.2f %9.1f %9.1f %8.1f  %s\n', ...
            ids{k}, r.Pthr, r.Ptot, r.I, r.Vbus, r.Ploss, r.runtime_min, r.Photel, flag);
    end
end
fprintf('\n6T C6 with %.0f W global budget: %.0f W, %.1f A, Vbus %.2f V\n', ...
    P.Pbudget, Rb.Ptot, Rb.I, Rb.Vbus);

%% ------------------------------------------------------------------------
%  2. MIXED-MISSION AVERAGE (report Section 9)
% -------------------------------------------------------------------------
mix_cases = [2 3 4 5 6 7];
mix_w     = [0.05 0.25 0.30 0.30 0.07 0.03];
fprintf('\n=== Weighted mission (static) ===\n');
for c = 1:2
    Ic = zeros(1,6);
    for j = 1:6
        % 6T C6 uses the global budget in the real vehicle
        r = case_eval(P, cfgs(c), mix_cases(j), c==2);
        Ic(j) = r.I;
    end
    Iavg = sum(mix_w .* Ic);
    fprintf('%s: average %.2f A (%.1f W) -> endurance %.1f min; 15%% reserve plan %.1f min\n', ...
        cfgs(c).name, Iavg, Iavg*P.Vnom, P.Qusable/Iavg*60, 0.85*P.Qusable/Iavg*60);
end
fprintf('(Report: 4T 12.30 A / 31 min, 6T 17.97 A / 21 min; the report mixes the\n');
fprintf(' un-budgeted 124.1 A for 6T C6, so 6T static here is slightly longer.)\n');

%% ------------------------------------------------------------------------
%  3. TIME-DOMAIN MISSION SIMULATION WITH FIRMWARE POWER MANAGER
% -------------------------------------------------------------------------
S = cell(2,2);   % S{config, mode}  mode 1: constant 22.2 V, mode 2: Voc(SoC)
for c = 1:2
    for m = 1:2
        S{c,m} = simulate_mission(P, cfgs(c), m);
    end
end
fprintf('\n=== Time-domain mission (80%% DoD, firmware limiter active) ===\n');
for c = 1:2
    fprintf('%s: const 22.2 V -> %.1f min | Voc(SoC) curve -> %.1f min (%s)\n', ...
        cfgs(c).name, S{c,1}.t_end/60, S{c,2}.t_end/60, S{c,2}.reason);
end

%% ------------------------------------------------------------------------
%  4. PRE-CHARGE
% -------------------------------------------------------------------------
Rsrc = P.Rtot;                         % battery + distribution
tau_pre = P.Rpre * P.Cbus;
fprintf('\n=== Contactor pre-charge ===\n');
fprintf('Cbus = %.2f mF | direct close peak = %.0f A | tau_pre = %.1f ms | peak with %g ohm = %.2f A | E_R = %.2f J\n', ...
    P.Cbus*1e3, P.Voc_full/Rsrc, tau_pre*1e3, P.Rpre, P.Voc_full/P.Rpre, 0.5*P.Cbus*P.Voc_full^2);

%% ------------------------------------------------------------------------
%  FIGURES
% -------------------------------------------------------------------------
col4 = [0.00 0.45 0.74]; col6 = [0.85 0.33 0.10]; gry = [0.45 0.45 0.45];

% --- Fig 1: thruster model --------------------------------------------------
figure('Name','Thruster model','Color','w','Position',[60 60 1050 420]);
V = linspace(12, 26, 300);
subplot(1,2,1); hold on; grid on;
patch([P.Vcut P.Voc_full P.Voc_full P.Vcut],[0 0 1300 1300],[0.9 0.95 1],'EdgeColor','none','FaceAlpha',0.6);
plot(V, thr_pfull(P,V), 'LineWidth',2, 'Color',col6);
plot(V, P.Iesc_lim*V, '--', 'LineWidth',1.5, 'Color',gry);
yline(P.Pcap, 'k-', '400 W cap', 'LineWidth',1.5);
xline(20, ':', '20 V rating');
plot([12 16 20],[205 390 645],'ko','MarkerFaceColor','k');
xlabel('Bus voltage [V]'); ylabel('Full-throttle power [W]');
title('Thruster P_{full}(V) vs. 30 A ESC limit'); legend({'6S window','P_{full}=645(V/20)^{2.24}','30 A ESC limit','Cap','','Published'},'Location','northwest');
ylim([0 1300]); xlim([12 26]);
subplot(1,2,2); hold on; grid on;
Vw = linspace(P.Vcut, P.Voc_full, 100);
plot(Vw, 100*min(1,(P.Pcap./thr_pfull(P,Vw)).^(1/3)), 'LineWidth',2, 'Color',col4);
xlabel('Bus voltage [V]'); ylabel('Max throttle for 400 W cap [%]');
title('Voltage-compensated throttle limit u_{max}'); ylim([60 100]);

% --- Fig 2: battery sag -------------------------------------------------------
figure('Name','Battery sag','Color','w','Position',[80 80 1050 420]);
subplot(1,2,1); hold on; grid on;
Ia = linspace(0,130,200);
vocs = [P.Voc_full P.Vnom P.Vcut]; lbl = {'Full 25.2 V','Nominal 22.2 V','Cutoff 19.8 V'};
cl = lines(3);
for j = 1:3
    plot(Ia, vocs(j) - Ia*P.Rtot, 'LineWidth',1.8, 'Color',cl(j,:));
end
yline(P.Vfloor,'r--','18 V absolute floor'); xline(P.Ifuse,'k:','100 A fuse');
xlabel('Battery current [A]'); ylabel('ESC bus voltage [V]');
title('Bus voltage vs. current (R_{tot} = 18.84 m\Omega)'); legend(lbl,'Location','southwest');
subplot(1,2,2); hold on; grid on;
Pl = linspace(0,3000,300);
for j = 1:3
    Isol = solve_current(vocs(j), P.Rtot, Pl);
    plot(Pl, Isol, 'LineWidth',1.8, 'Color',cl(j,:));
end
xline(1663,':','4T cap'); yline(P.Ifuse,'k:','100 A');
xlabel('Load power [W]'); ylabel('Battery current [A]'); title('Constant-power load current'); legend(lbl,'Location','northwest');

% --- Fig 3: battery current per case -------------------------------------------
figure('Name','Case currents & runtime','Color','w','Position',[100 100 1100 440]);
I4 = arrayfun(@(r) r.I, R{1}); I6 = arrayfun(@(r) r.I, R{2});
subplot(1,2,1); hold on; grid on;
bar([I4(1:7); I6(1:7)]', 'grouped');
yline(P.Ifuse, 'r--', '100 A fuse / XT90', 'LineWidth',1.5);
set(gca,'XTickLabel',ids(1:7)); ylabel('Battery current [A]');
title('Battery current per case (C8 omitted: 206 A / 355 A)'); legend({'4T','6T'},'Location','northwest');
subplot(1,2,2); hold on; grid on;
rt4 = arrayfun(@(r) r.runtime_min, R{1}); rt6 = arrayfun(@(r) r.runtime_min, R{2});
bar([rt4(1:7); rt6(1:7)]', 'grouped'); set(gca,'YScale','log','XTickLabel',ids(1:7));
ylabel('Continuous runtime [min] (log)'); title('Runtime per case on 6.4 Ah usable'); legend({'4T','6T'});

% --- Fig 4: mission time-domain --------------------------------------------------
figure('Name','Mission simulation','Color','w','Position',[120 120 1150 650]);
cc = {col4, col6};
for m = 1:2
    subplot(3,2,m);   hold on; grid on;
    for c = 1:2, plot(S{c,m}.t/60, S{c,m}.I, 'Color',cc{c}); end
    ylabel('I_{batt} [A]'); yline(P.Ifuse,'r--');
    title(ternary(m==1,'Constant 22.2 V pack','Pack with Voc(SoC) curve'));
    if m==1, legend({'4T','6T'}); end
    subplot(3,2,2+m); hold on; grid on;
    for c = 1:2, plot(S{c,m}.t/60, S{c,m}.Vbus, 'Color',cc{c}); end
    ylabel('Bus V [V]'); yline(P.Vcut,'k:'); yline(P.Vfloor,'r--');
    subplot(3,2,4+m); hold on; grid on;
    for c = 1:2, plot(S{c,m}.t/60, 100*S{c,m}.soc, 'Color',cc{c}); end
    ylabel('SoC [%]'); xlabel('Time [min]'); yline(20,'k:','80% DoD');
end

% --- Fig 5: pre-charge -------------------------------------------------------------
figure('Name','Pre-charge','Color','w','Position',[140 140 1000 400]);
t = linspace(0, 0.2, 2000);
tau_dir = Rsrc * P.Cbus;
subplot(1,2,1); hold on; grid on;
plot(t*1e3, P.Voc_full*(1-exp(-t/tau_pre)), 'LineWidth',2, 'Color',col4);
plot(t*1e3, P.Voc_full*(1-exp(-t/tau_dir)), 'LineWidth',2, 'Color',col6);
xlabel('Time [ms]'); ylabel('ESC bus voltage [V]'); title('Bus charge-up');
legend({sprintf('10 \\Omega pre-charge (\\tau=%.0f ms)',tau_pre*1e3), 'Direct close'},'Location','southeast');
subplot(1,2,2); hold on; grid on;
semilogy(t*1e3, P.Voc_full/P.Rpre*exp(-t/tau_pre), 'LineWidth',2, 'Color',col4);
semilogy(t*1e3, P.Voc_full/Rsrc*exp(-t/tau_dir), 'LineWidth',2, 'Color',col6);
set(gca,'YScale','log'); ylim([0.01 3000]); xlim([0 50]);
xlabel('Time [ms]'); ylabel('Inrush current [A]'); title('Contactor inrush (log)');
legend({sprintf('Pre-charge peak %.1f A',P.Voc_full/P.Rpre), sprintf('Direct peak %.0f A',P.Voc_full/Rsrc)});

%% ------------------------------------------------------------------------
%  EXPORT ALL FIGURES (one click: PNGs in results/figures + results/ROV_graphs.zip)
% -------------------------------------------------------------------------
outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
figs = findobj('Type', 'figure');
[~, order] = sort([figs.Number]);  figs = figs(order);
files = cell(1, numel(figs));
for f = 1:numel(figs)
    nm = regexprep(figs(f).Name, '[^\w]+', '_');
    files{f} = fullfile(outDir, sprintf('%02d_%s.png', f, nm));
    try
        exportgraphics(figs(f), files{f}, 'Resolution', 200);   % R2020a+
    catch
        saveas(figs(f), files{f});
    end
end
zipFile = fullfile(fileparts(outDir), 'ROV_graphs.zip');
zip(zipFile, outDir);
fprintf('\nSaved %d graphs to %s\nand zipped as %s\n', numel(figs), outDir, zipFile);
fprintf('MATLAB Online: right-click ROV_graphs.zip in the Current Folder panel -> Download.\n');

%% ========================================================================
%  LOCAL FUNCTIONS
% ========================================================================
function P = rov_params()
    P.Ns = 6;  P.Cap_Ah = 8;  P.DoD = 0.8;  P.Qusable = P.Cap_Ah*P.DoD;
    P.Vnom = 22.2; P.Voc_full = 25.2; P.Vcut = 19.8; P.Vfloor = 18.0; P.Vwarn = 21.0;
    P.Rcell = 2.5e-3; P.Rbatt = P.Ns*P.Rcell; P.Rdist = 3.84e-3;
    P.Rtot  = P.Rbatt + P.Rdist;                 % 18.84 mOhm
    % battery open-circuit voltage vs SoC (per cell, typical LiPo)
    P.soc_tab  = [0   0.05 0.10 0.20 0.30 0.50 0.70 0.90 1.00];
    P.vcell_tab= [3.00 3.45 3.55 3.70 3.74 3.80 3.95 4.10 4.20];
    % thruster / ESC
    P.Pcap = 400; P.Pbudget = 1700; P.Iesc = 0.05; P.Iesc_lim = 30;
    P.Ifuse = 100;
    % converters / loads
    P.eta5 = 0.90; P.eta12 = 0.93;
    P.P5typ = 5.67; P.P5pk = 11.25;              % 5 V rail output [W]
    P.Plamp_out = 2*20 + 2*0.4;                  % 2 lamps + driver loss at 100 %
    P.Rcoil = 192; P.Ipdb = 0.003;
    % pre-charge
    P.Cbus = 6*470e-6 + 2*100e-6;  P.Rpre = 10;
    % firmware
    P.Rbatt_fw = 0.015;
    % per-case definitions (C1..C8)
    P.uH      = [0 0 0.15 0.20 0.60 1 0 1];
    P.uV      = [0 0 0.30 0.35 0.30 1 1 1];
    P.lights  = [0 0 0.50 1.00 1.00 1 1 1];
    P.armed   = [0 1 1 1 1 1 1 1];
    P.capped  = [1 1 1 1 1 1 1 0];               % C8 = firmware cap bypassed
    P.p5pk    = [0 0 0 0 0 1 1 1];               % 5 V rail at peak in C6-C8
    P.Voc_case= [22.2*ones(1,7) 25.2];           % C8 at full charge
end

function p = thr_pfull(P, V)
    % full-throttle electrical power vs. bus voltage (fit to T200 data)
    p = 645 * (V/20).^2.24;
end

function I = solve_current(Voc, R, Pw)
    % battery current for a constant-power load including I^2R sag
    disc = Voc.^2 - 4*R*Pw;
    disc(disc < 0) = NaN;                        % load exceeds max power transfer
    I = (Voc - sqrt(disc)) ./ (2*R);
end

function r = case_eval(P, cfg, k, useBudget)
    Voc = P.Voc_case(k);
    nH = cfg.nH; nV = cfg.nV; nT = nH + nV;
    if P.capped(k), Pf = P.Pcap; else, Pf = thr_pfull(P, Voc); end
    Pthr = Pf * (nH*P.uH(k)^3 + nV*P.uV(k)^3);
    if useBudget, Pthr = min(Pthr, P.Pbudget); end
    Pesc  = P.armed(k) * nT * P.Iesc * Voc;
    Pcoil = P.armed(k) * Voc^2 / P.Rcoil;
    P5in  = (P.p5pk(k)*P.P5pk + (1-P.p5pk(k))*P.P5typ) / P.eta5;
    Plt   = P.lights(k) * P.Plamp_out / P.eta12;
    Ppdb  = P.Ipdb * Voc;
    r.Pthr   = Pthr;
    r.Photel = Pesc + Pcoil + P5in + Plt + Ppdb;
    r.Ptot   = Pthr + r.Photel;
    r.I      = solve_current(Voc, P.Rtot, r.Ptot);
    r.Vbus   = Voc - r.I*P.Rtot;
    r.Ploss  = r.I^2 * P.Rtot;
    r.runtime_min = P.Qusable / r.I * 60;
end

function voc = voc_of_soc(P, soc)
    voc = P.Ns * interp1(P.soc_tab, P.vcell_tab, min(max(soc,0),1), 'pchip');
end

function S = simulate_mission(P, cfg, mode)
    % Repeating 10-minute mission with the report's weights; the firmware
    % power manager (Section 13) limits thruster commands every step.
    dt = 0.2;  Tcyc = 600;
    seq = [2 3 4 5 6 7];  wts = [0.05 0.25 0.30 0.30 0.07 0.03];
    segEnd = cumsum(wts) * Tcyc;
    Nmax = round(14400/dt);
    t = zeros(1,Nmax); I = t; Vb = t; soc = t; Pb = t;
    Q = 0;
    Voc = ternary(mode==1, P.Vnom, voc_of_soc(P,1));
    vbus = Voc; ibus = 0; n = 0; reason = 'usable capacity reached (80% DoD)';
    nH = cfg.nH; nV = cfg.nV;
    while n < Nmax
        n = n + 1;
        tn = (n-1)*dt;
        tc = mod(tn, Tcyc);
        c  = seq(find(tc < segEnd, 1));
        cmd = [P.uH(c)*ones(nH,1); P.uV(c)*ones(nV,1)];

        soc_now = 1 - Q/P.Cap_Ah;
        if mode==1, Voc = P.Vnom; else, Voc = voc_of_soc(P, soc_now); end

        % ---- firmware power manager (uses previous-step measurements) ----
        v_oc_est = vbus + ibus*P.Rbatt_fw;
        pf    = thr_pfull(P, vbus);
        umax  = min(1, (P.Pcap/pf)^(1/3));
        preq  = sum(P.Pcap * abs(cmd).^3);
        s     = 1;
        if preq > P.Pbudget, s = (P.Pbudget/preq)^(1/3); end
        if v_oc_est < P.Vwarn, s = s*0.6; end
        if v_oc_est < P.Vcut
            reason = 'firmware low-voltage cutoff'; n = n - 1; break;
        end
        % ---- plant ----
        Pthr = sum(pf * abs(cmd*s*umax).^3);
        Pesc  = (nH+nV) * P.Iesc * Voc;
        Pcoil = Voc^2 / P.Rcoil;
        p5pk  = Pthr > 800;
        P5in  = (p5pk*P.P5pk + ~p5pk*P.P5typ) / P.eta5;
        Plt   = P.lights(c) * P.Plamp_out / P.eta12;
        Ptot  = Pthr + Pesc + Pcoil + P5in + Plt + P.Ipdb*Voc;
        ibus  = solve_current(Voc, P.Rtot, Ptot);
        if isnan(ibus), reason = 'load exceeds pack power capability'; n = n - 1; break; end
        vbus  = Voc - ibus*P.Rtot;
        Q     = Q + ibus*dt/3600;

        t(n) = tn; I(n) = ibus; Vb(n) = vbus; soc(n) = 1 - Q/P.Cap_Ah; Pb(n) = Ptot;
        if Q >= P.Qusable, break; end
    end
    S.t = t(1:n); S.I = I(1:n); S.Vbus = Vb(1:n); S.soc = soc(1:n); S.Pbatt = Pb(1:n);
    S.t_end = S.t(end) + dt; S.reason = reason; S.Iavg = mean(S.I);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
