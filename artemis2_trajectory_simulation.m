%% ARTEMIS-II-STYLE FIGURE-8 DEMO + DIRECT RETURN TO ORIGINAL EARTH ORBIT
% Earth-centered 2D simulation with:
%   - Earth fixed at origin
%   - Moon on circular orbit
%   - spacecraft in circular Earth parking orbit
%   - ONE Earth orbit before TLI
%   - one major TLI burn
%   - one small trajectory correction burn
%   - targeted far-side lunar flyby + return to Earth
%   - direct Earth return:
%       1) coast back toward Earth
%       2) at first inbound crossing of original orbit radius (or tangent touch),
%          apply one burn
%       3) circularize immediately into the orbit at that position

clear; close all; clc;

%% ----------------------- Physical constants ----------------------------
p.muE = 3.986004418e14;      % Earth GM [m^3/s^2]
p.muM = 4.9048695e12;        % Moon GM  [m^3/s^2]
p.Re  = 6378e3;              % Earth radius [m]
p.Rm  = 1737e3;              % Moon radius [m]
p.Dem = 384400e3;            % Earth-Moon distance [m]

p.Tmoon = 27.321661 * 24 * 3600;
p.omegaMoon = 2*pi / p.Tmoon;

%% ----------------------- Parking orbit ---------------------------------
p.altitude0 = 300e3;
p.r0 = p.Re + p.altitude0;
p.vcirc = sqrt(p.muE/p.r0);
p.Torbit = 2*pi*sqrt(p.r0^3/p.muE);

% Only ONE Earth orbit before TLI
p.nParkingOrbits = 1;
p.tTLI = p.nParkingOrbits * p.Torbit;

%% ----------------------- Mission timeline ------------------------------
p.tfinal = 10.5 * 24 * 3600;
% p.tfinal = 194 * 3600;
p.nominalCorrDays = 1.8;

% After final capture, show some extra time so the final orbit is visible
p.nPostCaptureOrbits = 1.5;
p.tPostCapture = p.nPostCaptureOrbits * p.Torbit;

%% ----------------------- Targeting goals -------------------------------
p.targetFlybyAlt   = 9000e3;
p.targetReturnRad  = p.r0;
p.maxCorrDV        = 30;

%% ----------------------- Bounds for refinement -------------------------
p.lb = [deg2rad(0);    deg2rad(0);    2950; deg2rad(-18); 0.5;  -40; -40];
p.ub = [deg2rad(360);  deg2rad(360);  3350; deg2rad(18);  3.5;   40;  40];

%% ----------------------- GIF controls ----------------------------------
makeGIF       = true;
gifName       = 'artemis2_return_to_original_orbit.gif';
gifDelay      = 0.05;
gifSkip       = 6;
showFullTrail = true;
trailLength   = 300;

%% ----------------------- Coarse seed search ----------------------------
fprintf('Coarse seed search...\n');

thetaGridDeg = 0:45:315;
moonGridDeg  = 0:30:330;
dvGrid       = 3000:100:3300;
gammaGridDeg = -8:4:8;

bestSeed = [];
bestJ = inf;

for thetaDeg = thetaGridDeg
    for moonDeg = moonGridDeg
        for dvTLI = dvGrid
            for gammaDeg = gammaGridDeg

                x0 = [deg2rad(thetaDeg); ...
                      deg2rad(moonDeg);  ...
                      dvTLI; ...
                      deg2rad(gammaDeg); ...
                      p.nominalCorrDays; ...
                      0; ...
                      0];

                sol = simulate_free_return(x0, p);
                if isempty(sol)
                    continue
                end

                J = objective_from_solution(x0, sol, p);

                if sol.flybyAlt < 50000e3 && sol.returnRadius < 500000e3
                    if J < bestJ
                        bestJ = J;
                        bestSeed = x0;
                        fprintf(['  seed update: theta0=%5.1f deg, moon@TLI=%5.1f deg, ' ...
                                 'dv=%6.1f m/s, gamma=%5.1f deg, J=%8.3f\n'], ...
                                 thetaDeg, moonDeg, dvTLI, gammaDeg, J);
                    end
                end
            end
        end
    end
end

if isempty(bestSeed)
    error('No coarse seed found. Widen thetaGridDeg / moonGridDeg / dvGrid / gammaGridDeg.');
end

%% ----------------------- Refinement ------------------------------------
fprintf('\nRefining with shooting method (fminsearch)...\n');

z0 = bestSeed;
opts = optimset('Display','iter', 'MaxIter', 220, 'MaxFunEvals', 800, ...
                'TolX',1e-5, 'TolFun',1e-5);

zOpt = fminsearch(@(z) bounded_objective(z, p), z0, opts);
xOpt = clamp_to_bounds(zOpt, p.lb, p.ub);

%% ----------------------- Final free-return simulation ------------------
sol = simulate_free_return(xOpt, p);
if isempty(sol)
    error('Final simulation failed.');
end

%% ----------------------- Earth return to original orbit ----------------
sol = add_earth_recapture(sol, p);

% For plotting: break trajectory lines across impulsive burns
sol.plotBreakIdx = unique([sol.iTLI, sol.iCorr, sol.iCapture2]);
sol.rPlot = build_plot_trajectory(sol.r, sol.plotBreakIdx);

fprintf('\nFinal targeted solution:\n');
fprintf('  theta0                = %8.3f deg\n', rad2deg(xOpt(1)));
fprintf('  Moon phase at TLI     = %8.3f deg\n', rad2deg(xOpt(2)));
fprintf('  dvTLI                 = %8.3f m/s\n', xOpt(3));
fprintf('  gammaTLI              = %8.3f deg\n', rad2deg(xOpt(4)));
fprintf('  tCorr after TLI       = %8.3f days\n', xOpt(5));
fprintf('  correction burn [T,R] = [%8.3f, %8.3f] m/s\n', xOpt(6), xOpt(7));
fprintf('  |correction burn|     = %8.3f m/s\n', hypot(xOpt(6), xOpt(7)));
fprintf('  lunar flyby altitude  = %8.3f km\n', sol.flybyAlt/1e3);
fprintf('  Earth return radius   = %8.3f km\n', sol.returnRadius/1e3);
fprintf('  Earth return vr       = %8.3f m/s\n', sol.returnVr);
fprintf('  far-side metric       = %8.3f km (positive is good)\n', sol.farSideMetric/1e3);
fprintf('  circularization burn  = %8.3f m/s\n', sol.dvCapture2);
fprintf('  total Earth burns     = %8.3f m/s\n', sol.dvCapture);
fprintf('  circularized radius   = %8.3f km\n', sol.rFinalCirc/1e3);

%% ----------------------- Static plots ----------------------------------
theta = linspace(0, 2*pi, 400);

figure('Color','w','Position',[80 80 1280 950]); hold on; axis equal; grid on;

plot(sol.rPlot(:,1)/1e6, sol.rPlot(:,2)/1e6, 'b-', 'LineWidth', 1.8);
plot(p.Dem*cos(theta)/1e6, p.Dem*sin(theta)/1e6, 'k--', 'LineWidth', 0.8);

plot(0,0,'bo','MarkerSize',14,'MarkerFaceColor','b');
plot((p.Re*cos(theta))/1e6, (p.Re*sin(theta))/1e6, 'b:', 'LineWidth', 1.0);
plot((p.r0*cos(theta))/1e6, (p.r0*sin(theta))/1e6, 'g--', 'LineWidth', 1.0);

moonFly = moon_position(sol.t(sol.iMoon), p, sol.moonPhase0);
plot(moonFly(1)/1e6, moonFly(2)/1e6, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.75 0.75 0.75]);
plot((moonFly(1)+p.Rm*cos(theta))/1e6, (moonFly(2)+p.Rm*sin(theta))/1e6, 'k:', 'LineWidth', 1.0);

plot(sol.r(1,1)/1e6,             sol.r(1,2)/1e6,             'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot(sol.r(sol.iTLI,1)/1e6,      sol.r(sol.iTLI,2)/1e6,      'mo', 'MarkerFaceColor','m', 'MarkerSize',8);
plot(sol.r(sol.iCorr,1)/1e6,     sol.r(sol.iCorr,2)/1e6,     'ks', 'MarkerFaceColor','y', 'MarkerSize',8);
plot(sol.r(sol.iMoon,1)/1e6,     sol.r(sol.iMoon,2)/1e6,     'co', 'MarkerFaceColor','c', 'MarkerSize',8);
plot(sol.r(sol.iReturn,1)/1e6,   sol.r(sol.iReturn,2)/1e6,   'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
plot(sol.r(sol.iCapture2,1)/1e6, sol.r(sol.iCapture2,2)/1e6, 'bd', 'MarkerFaceColor','b', 'MarkerSize',9);

xlabel('x [10^6 m]');
ylabel('y [10^6 m]');
title('Return to Earth orbit with one circularization burn');
legend('Spacecraft trajectory','Moon orbit','Earth','Earth outline', ...
       'Original circular orbit','Moon at flyby','Moon outline', ...
       'Start','TLI burn','Correction burn','Closest Moon approach', ...
       'Earth return','Circularization burn', ...
       'Location','bestoutside');

figure('Color','w','Position',[100 100 1150 760]);

subplot(2,1,1); hold on; grid on;
plot(sol.t/3600, sol.rEarth/1e3, 'b-', 'LineWidth', 1.5);
yline(p.Re/1e3, 'b:', 'Earth radius');
yline(p.r0/1e3, 'g--', 'Original orbit radius');
yline(sol.rFinalCirc/1e3, 'b-.', 'Final circularized radius');
xline(sol.t(sol.iTLI)/3600, 'm--', 'TLI');
xline(sol.t(sol.iCorr)/3600, 'k--', 'TCM');
xline(sol.t(sol.iReturn)/3600, 'r--', 'Return');
xline(sol.tCapture2/3600, 'b--', 'Circularization');
xlabel('Time [hr]');
ylabel('Distance to Earth center [km]');
title('Distance to Earth');

subplot(2,1,2); hold on; grid on;
plot(sol.t/3600, sol.rMoon/1e3, 'k-', 'LineWidth', 1.5);
yline(p.Rm/1e3, 'k:', 'Moon radius');
xline(sol.t(sol.iMoon)/3600, 'c--', 'Flyby');
xlabel('Time [hr]');
ylabel('Distance to Moon center [km]');
title('Distance to Moon');

%% ----------------------- GIF export -----------------------------------
if makeGIF
    fprintf('\nWriting GIF: %s\n', gifName);

    % % Portrait / mobile-friendly figure
    % figGif = figure('Color','w','Position',[100 60 720 1200]);
    % % Main axes occupy upper portion, leaving room for bottom legend
    % ax = axes('Parent', figGif, 'Position', [0.10 0.23 0.84 0.67]);

    % figGif = figure('Color','w','Position',[100 60 760 980]);
    % ax = axes('Parent', figGif, 'Position', [0.10 0.18 0.84 0.72]);

    figGif = figure('Color','w','Position',[100 60 760 930]);
    ax = axes('Parent', figGif, 'Position', [0.10 0.13 0.84 0.73]);

    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');

    earthMinHalfWidth    = 12;
    earthMaxHalfWidth    = 450;
    earthPadFrac         = 0.35;
    earthMinPad          = 4;

    % Make final Earth zoom match the initial Earth close-up for better looping
    moonFlybyHalfWidth   = 100;
    earthReturnHalfWidth = earthMinHalfWidth;

    moonZoomStartDist    = 140;
    moonZoomFullDist     = 70;

    % Make return zoom mirror the initial Earth zoom-in behavior
    earthZoomStartDist   = 50;
    earthZoomFullDist    = 5;

    % Only draw GIF frames through 194 hours
    tGifMax = 194 * 3600;

    passedMoon = false;
    frameIdx = 1:gifSkip:length(sol.t);

    hMoonOrbit = plot(ax, nan, nan, 'k--', 'LineWidth', 0.8, 'DisplayName', 'Moon orbit');
    hEarth = plot(ax, nan, nan, 'bo', 'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'Earth');
    hEarthOutline = plot(ax, nan, nan, 'b:', 'LineWidth', 1.0, 'DisplayName', 'Earth outline');
    hCircOrbit = plot(ax, nan, nan, 'g--', 'LineWidth', 1.0, 'DisplayName', 'Circular orbit');

    hMoon = plot(ax, nan, nan, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.75 0.75 0.75], 'DisplayName', 'Moon');

    hTraj = plot(ax, nan, nan, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Spacecraft trajectory');
    hCraft = plot(ax, nan, nan, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Spacecraft');
    hStart = plot(ax, nan, nan, 'go', 'MarkerSize', 7, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');

    hTLI = plot(ax, nan, nan, 'mo', 'MarkerSize', 7, 'MarkerFaceColor', 'm', 'DisplayName', 'Main departure burn');
    hCorr = plot(ax, nan, nan, 'ks', 'MarkerSize', 7, 'MarkerFaceColor', 'y', 'DisplayName', 'Course correction burn');
    hFlyby = plot(ax, nan, nan, 'co', 'MarkerSize', 7, 'MarkerFaceColor', 'c', 'DisplayName', 'Lunar flyby');
    hCap2 = plot(ax, nan, nan, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', [1 0.5 0], 'DisplayName', 'Circularization burn');

    set([hTLI hCorr hFlyby hCap2], 'Visible', 'off');

    lgd = legend(ax, [hTraj, hCraft, hEarth, hEarthOutline, hCircOrbit, ...
                      hMoon, hMoonOrbit, hStart, ...
                      hTLI, hCorr, hFlyby, hCap2], ...
                 'Location', 'southoutside', ...
                 'Orientation', 'horizontal');
    lgd.AutoUpdate = 'off';
    lgd.NumColumns = 3;
    set(lgd, 'FontSize', 10.5);

    hText = text(ax, 0, 0, '', 'FontSize', 13, 'BackgroundColor', 'w', 'Margin', 4);

    earthOutlineX = (p.Re*cos(theta))/1e6;
    earthOutlineY = (p.Re*sin(theta))/1e6;
    circOrbitX = (sol.rFinalCirc*cos(theta))/1e6;
    circOrbitY = (sol.rFinalCirc*sin(theta))/1e6;
    moonOrbitX = (p.Dem*cos(theta))/1e6;
    moonOrbitY = (p.Dem*sin(theta))/1e6;

    wroteFrame = false;

    for kk = 1:length(frameIdx)
        k = frameIdx(kk);

        % Stop GIF once time exceeds 194 hours
        if sol.t(k) > tGifMax
            break
        end

        moonNow = moon_position(sol.t(k), p, sol.moonPhase0);

        set(hMoonOrbit, 'XData', moonOrbitX, 'YData', moonOrbitY);
        set(hEarth, 'XData', 0, 'YData', 0);
        set(hEarthOutline, 'XData', earthOutlineX, 'YData', earthOutlineY);
        set(hCircOrbit, 'XData', circOrbitX, 'YData', circOrbitY);

        set(hMoon, 'XData', moonNow(1)/1e6, 'YData', moonNow(2)/1e6);

        if showFullTrail
            i1 = 1;
        else
            i1 = max(1, k-trailLength);
        end

        localBreaks = sol.plotBreakIdx(sol.plotBreakIdx >= i1 & sol.plotBreakIdx <= k) - i1 + 1;
        rTrail = build_plot_trajectory(sol.r(i1:k,:), localBreaks);
        set(hTraj, 'XData', rTrail(:,1)/1e6, 'YData', rTrail(:,2)/1e6);

        set(hCraft, 'XData', sol.r(k,1)/1e6, 'YData', sol.r(k,2)/1e6);
        set(hStart, 'XData', sol.r(1,1)/1e6, 'YData', sol.r(1,2)/1e6);

        if k >= sol.iTLI
            set(hTLI, 'XData', sol.r(sol.iTLI,1)/1e6, 'YData', sol.r(sol.iTLI,2)/1e6, 'Visible', 'on');
        else
            set(hTLI, 'Visible', 'off');
        end

        if k >= sol.iCorr
            set(hCorr, 'XData', sol.r(sol.iCorr,1)/1e6, 'YData', sol.r(sol.iCorr,2)/1e6, 'Visible', 'on');
        else
            set(hCorr, 'Visible', 'off');
        end

        if k >= sol.iMoon
            set(hFlyby, 'XData', sol.r(sol.iMoon,1)/1e6, 'YData', sol.r(sol.iMoon,2)/1e6, 'Visible', 'on');
            passedMoon = true;
        else
            set(hFlyby, 'Visible', 'off');
        end

        if k >= sol.iCapture2
            set(hCap2, 'XData', sol.r(sol.iCapture2,1)/1e6, 'YData', sol.r(sol.iCapture2,2)/1e6, 'Visible', 'on');
        else
            set(hCap2, 'Visible', 'off');
        end

        xlabel(ax, 'x [10^6 m]', 'FontSize', 13);
        ylabel(ax, 'y [10^6 m]', 'FontSize', 13);
        set(ax, 'FontSize', 13);
        title(ax, sprintf('Artemis II Trajectory Simulation, t = %.1f hr', sol.t(k)/3600), 'FontSize', 14);

        if sol.t(k) < sol.t(sol.iTLI)
            subt = 'Parking orbit around Earth';
        elseif sol.t(k) < sol.t(sol.iMoon)
            subt = 'Outbound path after the main departure burn';
        elseif sol.t(k) < sol.tCapture2
            subt = 'Lunar flyby bends the spacecraft back toward Earth';
        else
            subt = 'One burn circularizes the spacecraft into Earth orbit';
        end

        craftPos = sol.r(k,:) / 1e6;
        moonPosNow = moonNow(:).' / 1e6;

        distEarth = norm(craftPos);
        distMoon  = norm(craftPos - moonPosNow);

        earthHalfWidth = distEarth * (1 + earthPadFrac) + earthMinPad;
        earthHalfWidth = max(earthMinHalfWidth, min(earthMaxHalfWidth, earthHalfWidth));

        if distMoon >= moonZoomStartDist
            moonBlend = 0;
        elseif distMoon <= moonZoomFullDist
            moonBlend = 1;
        else
            u = (moonZoomStartDist - distMoon) / (moonZoomStartDist - moonZoomFullDist);
            moonBlend = 3*u^2 - 2*u^3;
        end

        if passedMoon
            if distEarth >= earthZoomStartDist
                earthReturnBlend = 0;
            elseif distEarth <= earthZoomFullDist
                earthReturnBlend = 1;
            else
                u = (earthZoomStartDist - distEarth) / (earthZoomStartDist - earthZoomFullDist);
                earthReturnBlend = 3*u^2 - 2*u^3;
            end
        else
            earthReturnBlend = 0;
        end

        viewCenter = [0, 0];
        halfWidth = earthHalfWidth;

        if moonBlend > 0
            viewCenter = (1 - moonBlend) * [0, 0] + moonBlend * moonPosNow;
            halfWidth  = (1 - moonBlend) * earthHalfWidth + moonBlend * moonFlybyHalfWidth;
        end

        if earthReturnBlend > 0
            viewCenter = (1 - earthReturnBlend) * viewCenter + earthReturnBlend * [0, 0];
            halfWidth  = (1 - earthReturnBlend) * halfWidth + earthReturnBlend * earthReturnHalfWidth;
        end

        xlim(ax, viewCenter(1) + [-halfWidth, halfWidth]);
        ylim(ax, viewCenter(2) + [-halfWidth, halfWidth]);

        set(hText, ...
            'Position', [viewCenter(1) - 0.92*halfWidth, viewCenter(2) + 0.88*halfWidth, 0], ...
            'String', subt);

        drawnow;

        frame = getframe(figGif);
        im = frame2im(frame);
        [A,map] = rgb2ind(im,256);

        if ~wroteFrame
            imwrite(A, map, gifName, 'gif', 'LoopCount', inf, 'DelayTime', gifDelay);
            wroteFrame = true;
        else
            imwrite(A, map, gifName, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
        end
    end

    fprintf('GIF saved: %s\n', gifName);
end

%% ----------------------- GIF export -----------------------------------
if makeGIF
    fprintf('\nWriting GIF: %s\n', gifName);

    figGif = figure('Color','w','Position',[100 100 1260 900]);
    ax = axes('Parent', figGif);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');

    earthMinHalfWidth    = 12;
    earthMaxHalfWidth    = 450;
    earthPadFrac         = 0.35;
    earthMinPad          = 4;

    % Make final Earth zoom match the initial Earth close-up for better looping
    moonFlybyHalfWidth   = 100;
    earthReturnHalfWidth = earthMinHalfWidth;

    moonZoomStartDist    = 140;
    moonZoomFullDist     = 70;

    % Make return zoom mirror the initial Earth zoom-in behavior
    earthZoomStartDist   = 50;
    earthZoomFullDist    = 5;

    passedMoon = false;
    frameIdx = 1:gifSkip:length(sol.t);

    hMoonOrbit = plot(ax, nan, nan, 'k--', 'LineWidth', 0.8, 'DisplayName', 'Moon orbit');
    hEarth = plot(ax, nan, nan, 'bo', 'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'Earth');
    hEarthOutline = plot(ax, nan, nan, 'b:', 'LineWidth', 1.0, 'DisplayName', 'Earth outline');
    hCircOrbit = plot(ax, nan, nan, 'g--', 'LineWidth', 1.0, 'DisplayName', 'Circular orbit');

    hMoon = plot(ax, nan, nan, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.75 0.75 0.75], 'DisplayName', 'Moon');

    hTraj = plot(ax, nan, nan, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Spacecraft trajectory');
    hCraft = plot(ax, nan, nan, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Spacecraft');
    hStart = plot(ax, nan, nan, 'go', 'MarkerSize', 7, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');

    hTLI = plot(ax, nan, nan, 'mo', 'MarkerSize', 7, 'MarkerFaceColor', 'm', 'DisplayName', 'Main departure burn');
    hCorr = plot(ax, nan, nan, 'ks', 'MarkerSize', 7, 'MarkerFaceColor', 'y', 'DisplayName', 'Course correction burn');
    hFlyby = plot(ax, nan, nan, 'co', 'MarkerSize', 7, 'MarkerFaceColor', 'c', 'DisplayName', 'Lunar flyby');
    hCap2 = plot(ax, nan, nan, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', [1 .5 0], 'DisplayName', 'Circularization burn');

    set([hTLI hCorr hFlyby hCap2], 'Visible', 'off');

    lgd = legend(ax, [hTraj, hCraft, hEarth, hEarthOutline, hCircOrbit, ...
                      hMoon, hMoonOrbit, hStart, ...
                      hTLI, hCorr, hFlyby, hCap2], ...
                 'Location', 'eastoutside');
    lgd.AutoUpdate = 'off';
    set(lgd, 'FontSize', 14);

    hText = text(ax, 0, 0, '', 'FontSize', 14, 'BackgroundColor', 'w', 'Margin', 4);

    earthOutlineX = (p.Re*cos(theta))/1e6;
    earthOutlineY = (p.Re*sin(theta))/1e6;
    circOrbitX = (sol.rFinalCirc*cos(theta))/1e6;
    circOrbitY = (sol.rFinalCirc*sin(theta))/1e6;
    moonOrbitX = (p.Dem*cos(theta))/1e6;
    moonOrbitY = (p.Dem*sin(theta))/1e6;

    for kk = 1:length(frameIdx)
        k = frameIdx(kk);

        moonNow = moon_position(sol.t(k), p, sol.moonPhase0);

        set(hMoonOrbit, 'XData', moonOrbitX, 'YData', moonOrbitY);
        set(hEarth, 'XData', 0, 'YData', 0);
        set(hEarthOutline, 'XData', earthOutlineX, 'YData', earthOutlineY);
        set(hCircOrbit, 'XData', circOrbitX, 'YData', circOrbitY);

        set(hMoon, 'XData', moonNow(1)/1e6, 'YData', moonNow(2)/1e6);

        if showFullTrail
            i1 = 1;
        else
            i1 = max(1, k-trailLength);
        end

        localBreaks = sol.plotBreakIdx(sol.plotBreakIdx >= i1 & sol.plotBreakIdx <= k) - i1 + 1;
        rTrail = build_plot_trajectory(sol.r(i1:k,:), localBreaks);
        set(hTraj, 'XData', rTrail(:,1)/1e6, 'YData', rTrail(:,2)/1e6);

        set(hCraft, 'XData', sol.r(k,1)/1e6, 'YData', sol.r(k,2)/1e6);
        set(hStart, 'XData', sol.r(1,1)/1e6, 'YData', sol.r(1,2)/1e6);

        if k >= sol.iTLI
            set(hTLI, 'XData', sol.r(sol.iTLI,1)/1e6, 'YData', sol.r(sol.iTLI,2)/1e6, 'Visible', 'on');
        else
            set(hTLI, 'Visible', 'off');
        end

        if k >= sol.iCorr
            set(hCorr, 'XData', sol.r(sol.iCorr,1)/1e6, 'YData', sol.r(sol.iCorr,2)/1e6, 'Visible', 'on');
        else
            set(hCorr, 'Visible', 'off');
        end

        if k >= sol.iMoon
            set(hFlyby, 'XData', sol.r(sol.iMoon,1)/1e6, 'YData', sol.r(sol.iMoon,2)/1e6, 'Visible', 'on');
            passedMoon = true;
        else
            set(hFlyby, 'Visible', 'off');
        end

        if k >= sol.iCapture2
            set(hCap2, 'XData', sol.r(sol.iCapture2,1)/1e6, 'YData', sol.r(sol.iCapture2,2)/1e6, 'Visible', 'on');
        else
            set(hCap2, 'Visible', 'off');
        end

        xlabel(ax, 'x [10^6 m]', 'FontSize', 14);
        ylabel(ax, 'y [10^6 m]', 'FontSize', 14);
        set(ax, 'FontSize', 14);
        title(ax, sprintf('Artemis II Trajectory Simulation, t = %.1f hr', sol.t(k)/3600), 'FontSize', 14);

        if sol.t(k) < sol.t(sol.iTLI)
            subt = 'Parking orbit around Earth';
        elseif sol.t(k) < sol.t(sol.iMoon)
            subt = 'Outbound path after the main departure burn';
        elseif sol.t(k) < sol.tCapture2
            subt = 'Lunar flyby bends the spacecraft back toward Earth';
        else
            subt = 'One burn circularizes into Earth orbit';
        end

        craftPos = sol.r(k,:) / 1e6;
        moonPosNow = moonNow(:).' / 1e6;

        distEarth = norm(craftPos);
        distMoon  = norm(craftPos - moonPosNow);

        earthHalfWidth = distEarth * (1 + earthPadFrac) + earthMinPad;
        earthHalfWidth = max(earthMinHalfWidth, min(earthMaxHalfWidth, earthHalfWidth));

        if distMoon >= moonZoomStartDist
            moonBlend = 0;
        elseif distMoon <= moonZoomFullDist
            moonBlend = 1;
        else
            u = (moonZoomStartDist - distMoon) / (moonZoomStartDist - moonZoomFullDist);
            moonBlend = 3*u^2 - 2*u^3;
        end

        if passedMoon
            if distEarth >= earthZoomStartDist
                earthReturnBlend = 0;
            elseif distEarth <= earthZoomFullDist
                earthReturnBlend = 1;
            else
                u = (earthZoomStartDist - distEarth) / (earthZoomStartDist - earthZoomFullDist);
                earthReturnBlend = 3*u^2 - 2*u^3;
            end
        else
            earthReturnBlend = 0;
        end

        viewCenter = [0, 0];
        halfWidth = earthHalfWidth;

        if moonBlend > 0
            viewCenter = (1 - moonBlend) * [0, 0] + moonBlend * moonPosNow;
            halfWidth  = (1 - moonBlend) * earthHalfWidth + moonBlend * moonFlybyHalfWidth;
        end

        if earthReturnBlend > 0
            viewCenter = (1 - earthReturnBlend) * viewCenter + earthReturnBlend * [0, 0];
            halfWidth  = (1 - earthReturnBlend) * halfWidth + earthReturnBlend * earthReturnHalfWidth;
        end

        xlim(ax, viewCenter(1) + [-halfWidth, halfWidth]);
        ylim(ax, viewCenter(2) + [-halfWidth, halfWidth]);

        set(hText, ...
            'Position', [viewCenter(1) - 0.95*halfWidth, viewCenter(2) + 0.90*halfWidth, 0], ...
            'String', subt);

        drawnow;

        frame = getframe(figGif);
        im = frame2im(frame);
        [A,map] = rgb2ind(im,256);

        if kk == 1
            imwrite(A, map, gifName, 'gif', 'LoopCount', inf, 'DelayTime', gifDelay);
        else
            imwrite(A, map, gifName, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelay);
        end
    end

    fprintf('GIF saved: %s\n', gifName);
end

%% ========================= Local functions =============================

function J = bounded_objective(z, p)
    x = clamp_to_bounds(z, p.lb, p.ub);
    sol = simulate_free_return(x, p);

    if isempty(sol)
        J = 1e9;
        return
    end

    J = objective_from_solution(x, sol, p);
end

function x = clamp_to_bounds(z, lb, ub)
    x = min(max(z, lb), ub);
end

function J = objective_from_solution(x, sol, p)
    dvCorr = hypot(x(6), x(7));

    t1 = ((sol.flybyAlt   - p.targetFlybyAlt) / 2500e3)^2;
    t2 = ((sol.returnRadius - p.targetReturnRad) / 500e3)^2;
    t3 = (max(sol.returnVr, 0) / 1200)^2;
    t4 = (max(-sol.farSideMetric, 0) / 3000e3)^2;
    t5 = (dvCorr / max(p.maxCorrDV,1))^2;
    t6 = (max(sol.flybyAlt - 30000e3, 0) / 15000e3)^2;
    t7 = (max(sol.returnRadius - 300000e3, 0) / 100000e3)^2;

    J = 4*t1 + 5*t2 + 2*t3 + 5*t4 + 1.5*t5 + 3*t6 + 2*t7;
end

function sol = simulate_free_return(x, p)
    try
        theta0    = x(1);
        moonAtTLI = x(2);
        dvTLI     = x(3);
        gammaTLI  = x(4);
        tCorrDays = x(5);
        dvCorrT   = x(6);
        dvCorrR   = x(7);

        moonPhase0 = moonAtTLI - p.omegaMoon * p.tTLI;
        tCorr = p.tTLI + tCorrDays * 24 * 3600;
        tCorr = min(max(tCorr, p.tTLI + 0.2*24*3600), p.tfinal - 0.5*24*3600);

        r_init = p.r0 * [cos(theta0); sin(theta0)];
        v_init = p.vcirc * [-sin(theta0); cos(theta0)];
        y0 = [r_init; v_init];

        n1 = 500;
        tspan1 = linspace(0, p.tTLI, n1);
        [t1,y1] = ode45(@(t,y) rhs(t,y,p,moonPhase0), tspan1, y0, ...
                        odeset('RelTol',1e-9,'AbsTol',1e-9));

        yTLIminus = y1(end,:)';
        rNow = yTLIminus(1:2);
        vNow = yTLIminus(3:4);

        that = vNow / norm(vNow);
        rhat = rNow / norm(rNow);
        burnDir = cos(gammaTLI)*that + sin(gammaTLI)*rhat;
        burnDir = burnDir / norm(burnDir);

        yTLIplus = yTLIminus;
        yTLIplus(3:4) = yTLIplus(3:4) + dvTLI * burnDir;

        n2 = 700;
        tspan2 = linspace(p.tTLI, tCorr, n2);
        [t2,y2] = ode45(@(t,y) rhs(t,y,p,moonPhase0), tspan2, yTLIplus, ...
                        odeset('RelTol',1e-9,'AbsTol',1e-9));

        yCminus = y2(end,:)';
        rNow = yCminus(1:2);
        vNow = yCminus(3:4);

        that = vNow / norm(vNow);
        rhat = rNow / norm(rNow);

        yCplus = yCminus;
        yCplus(3:4) = yCplus(3:4) + dvCorrT * that + dvCorrR * rhat;

        n3 = 1600;
        tspan3 = linspace(tCorr, p.tfinal, n3);
        [t3,y3] = ode45(@(t,y) rhs(t,y,p,moonPhase0), tspan3, yCplus, ...
                        odeset('RelTol',1e-9,'AbsTol',1e-9));

        t = [t1; t2(2:end); t3(2:end)];
        y = [y1; y2(2:end,:); y3(2:end,:)];

        r = y(:,1:2);
        v = y(:,3:4);

        moonPos = zeros(size(r));
        moonVel = zeros(size(r));
        for i = 1:numel(t)
            [moonPos(i,:), moonVel(i,:)] = moon_state(t(i), p, moonPhase0);
        end

        rEarth = vecnorm(r,2,2);
        rMoon  = vecnorm(r - moonPos,2,2);

        iTLI  = size(t1,1);
        iCorr = size(t1,1) + size(t2,1) - 1;

        [minMoonDist, idxMoonLocal] = min(rMoon(iTLI:end));
        iMoon = idxMoonLocal + iTLI - 1;
        flybyAlt = minMoonDist - p.Rm;

        moonHat = moonPos(iMoon,:).' / norm(moonPos(iMoon,:));
        farSideMetric = dot(r(iMoon,:).' - moonPos(iMoon,:).', moonHat);

        [returnRadius, idxRetLocal] = min(rEarth(iMoon:end));
        iReturn = idxRetLocal + iMoon - 1;

        rRet = r(iReturn,:).';
        vRet = v(iReturn,:).';
        returnVr = dot(rRet, vRet) / norm(rRet);

        rRel = r(iMoon,:).' - moonPos(iMoon,:).';
        vRel = v(iMoon,:).' - moonVel(iMoon,:).';
        hRel = rRel(1)*vRel(2) - rRel(2)*vRel(1);

        sol = struct();
        sol.t = t;
        sol.y = y;
        sol.r = r;
        sol.v = v;
        sol.rEarth = rEarth;
        sol.rMoon = rMoon;
        sol.moonPos = moonPos;
        sol.moonVel = moonVel;
        sol.moonPhase0 = moonPhase0;

        sol.iTLI = iTLI;
        sol.iCorr = iCorr;
        sol.iMoon = iMoon;
        sol.iReturn = iReturn;

        sol.minMoonDist = minMoonDist;
        sol.flybyAlt = flybyAlt;
        sol.returnRadius = returnRadius;
        sol.returnVr = returnVr;
        sol.farSideMetric = farSideMetric;
        sol.flybySign = sign(hRel);
        sol.corrDV = hypot(dvCorrT, dvCorrR);

    catch
        sol = [];
    end
end

function sol = add_earth_recapture(sol, p)
    % One-burn circularization:
    %   1) prefer first true inbound crossing of p.r0
    %   2) if not found, use first post-flyby local minimum radius
    %      (tangent touch / nearest approach)
    %   3) at that instant, keep position fixed and set velocity to local
    %      circular-orbit velocity
    %   4) coast in the circular orbit

    nArc2 = 900;

    rEarth = sol.rEarth;
    iMoon  = sol.iMoon;

    found = false;
    useLocalMin = false;

    % ---- Try true inbound crossing first ----
    for i = iMoon:(numel(sol.t)-1)
        rr1 = rEarth(i);
        rr2 = rEarth(i+1);

        if rr1 >= p.r0 && rr2 <= p.r0
            a = (rr1 - p.r0) / (rr1 - rr2 + eps);
            yEvent = (1-a)*sol.y(i,:).' + a*sol.y(i+1,:).';
            rEvent = yEvent(1:2);
            vEvent = yEvent(3:4);
            vr = dot(rEvent, vEvent) / norm(rEvent);

            if vr < 0
                tEvent = sol.t(i) + a*(sol.t(i+1) - sol.t(i));
                iEventBase = i;
                found = true;
                break
            end
        end
    end

    % ---- If no crossing, use first local minimum after flyby ----
    if ~found
        for i = max(iMoon+1,2):(numel(sol.t)-1)
            if rEarth(i-1) > rEarth(i) && rEarth(i+1) >= rEarth(i)
                tEvent = sol.t(i);
                yEvent = sol.y(i,:).';
                iEventBase = i;
                found = true;
                useLocalMin = true;
                break
            end
        end
    end

    if ~found
        error('Could not find a usable Earth-return circularization point.');
    end

    r2 = yEvent(1:2);
    v2 = yEvent(3:4);

    r2mag = norm(r2);
    rhat2 = r2 / r2mag;

    % Match direction of motion using angular momentum sign
    hNow = r2(1)*v2(2) - r2(2)*v2(1);
    if hNow >= 0
        that2 = [-rhat2(2); rhat2(1)];
    else
        that2 = [ rhat2(2); -rhat2(1)];
    end

    vCircFinal = sqrt(p.muE / r2mag);
    vTarget2 = vCircFinal * that2;

    dv2Vec = vTarget2 - v2;
    dv2Mag = norm(dv2Vec);

    y2plus = [r2; vTarget2];

    % Final coast in circular orbit at the event radius
    tEnd = tEvent + p.tPostCapture;
    tspan2 = linspace(tEvent, tEnd, nArc2);
    [tB, yB] = ode45(@(t,y) rhs(t,y,p,sol.moonPhase0), tspan2, y2plus, ...
                     odeset('RelTol',1e-9,'AbsTol',1e-9));

    % Stitch:
    % keep pre-burn coast through event base index, then insert exact event
    tPre = sol.t(1:iEventBase);
    yPre = sol.y(1:iEventBase,:);

    tNew = [
        tPre;
        tEvent;
        tB(2:end)
    ];

    yNew = [
        yPre;
        yEvent.';
        yB(2:end,:)
    ];

    keep = true(size(tNew));
    for k = 2:numel(tNew)
        if abs(tNew(k)-tNew(k-1)) < 1e-12 && ...
           norm(yNew(k,1:2)-yNew(k-1,1:2)) < 1e-9 && ...
           norm(yNew(k,3:4)-yNew(k-1,3:4)) < 1e-12
            keep(k) = false;
        end
    end
    tNew = tNew(keep);
    yNew = yNew(keep,:);

    rNew = yNew(:,1:2);
    vNew = yNew(:,3:4);

    moonPosNew = zeros(size(rNew));
    moonVelNew = zeros(size(rNew));
    for i = 1:numel(tNew)
        [moonPosNew(i,:), moonVelNew(i,:)] = moon_state(tNew(i), p, sol.moonPhase0);
    end

    sol.t = tNew;
    sol.y = yNew;
    sol.r = rNew;
    sol.v = vNew;
    sol.moonPos = moonPosNew;
    sol.moonVel = moonVelNew;
    sol.rEarth = vecnorm(sol.r,2,2);
    sol.rMoon  = vecnorm(sol.r - sol.moonPos,2,2);

    sol.iCapture1 = NaN;
    [~, sol.iCapture2] = min(abs(sol.t - tEvent));
    sol.iCapture = sol.iCapture2;

    sol.dvCapture1 = 0;
    sol.dvCapture2 = dv2Mag;
    sol.dvCapture3 = 0;
    sol.dvCapture  = dv2Mag;

    sol.tCapture1 = NaN;
    sol.tCapture2 = tEvent;
    sol.tCapture3 = tEvent;
    sol.tCapture  = tEvent;

    sol.tBlendStart = tEvent;
    sol.tBlendEnd   = tEvent;

    sol.rReturnCirc = r2mag;
    sol.rFinalCirc  = r2mag;
    sol.usedTangentCapture = useLocalMin;
end

function dydt = rhs(t, y, p, moonPhase0)
    r = y(1:2);
    v = y(3:4);

    moonPos = moon_position(t, p, moonPhase0);

    aE = -p.muE * r / norm(r)^3;
    dM = r - moonPos;
    aM = -p.muM * dM / norm(dM)^3;

    dydt = [v; aE + aM];
end

function rPlot = build_plot_trajectory(r, breakIdx)
    if isempty(r)
        rPlot = r;
        return
    end

    breakIdx = unique(breakIdx(:)');
    breakIdx = breakIdx(breakIdx >= 1 & breakIdx < size(r,1));

    nBreak = numel(breakIdx);
    rPlot = nan(size(r,1) + nBreak, 2);

    src = 1;
    dst = 1;
    for b = 1:nBreak
        iBreak = breakIdx(b);
        nCopy = iBreak - src + 1;
        rPlot(dst:dst+nCopy-1,:) = r(src:iBreak,:);
        dst = dst + nCopy;
        rPlot(dst,:) = [NaN NaN];
        dst = dst + 1;
        src = iBreak + 1;
    end
    rPlot(dst:dst + (size(r,1)-src), :) = r(src:end,:);
end

function yq = interp_state(t, y, tq)
    yq = zeros(4,1);
    for j = 1:4
        yq(j) = interp1(t, y(:,j), tq, 'linear');
    end
end

function [r, v] = hermite_bridge(t, t0, t1, r0, v0, r1, v1)
    T = t1 - t0;
    s = (t - t0) / T;

    h00 = 2*s.^3 - 3*s.^2 + 1;
    h10 = s.^3 - 2*s.^2 + s;
    h01 = -2*s.^3 + 3*s.^2;
    h11 = s.^3 - s.^2;

    dh00 = (6*s.^2 - 6*s) / T;
    dh10 = (3*s.^2 - 4*s + 1);
    dh01 = (-6*s.^2 + 6*s) / T;
    dh11 = (3*s.^2 - 2*s);

    r = h00.*r0.' + h10.*(T*v0.') + h01.*r1.' + h11.*(T*v1.');
    v = dh00.*r0.' + dh10.*v0.' + dh01.*r1.' + dh11.*v1.';
end

function rMoon = moon_position(t, p, moonPhase0)
    th = moonPhase0 + p.omegaMoon*t;
    rMoon = p.Dem * [cos(th); sin(th)];
end

function [rMoon, vMoon] = moon_state(t, p, moonPhase0)
    th = moonPhase0 + p.omegaMoon*t;
    rMoon = p.Dem * [cos(th), sin(th)];
    vMoon = p.Dem * p.omegaMoon * [-sin(th), cos(th)];
end
