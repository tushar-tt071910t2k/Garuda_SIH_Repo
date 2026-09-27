function PlotResults(allMetrics)
% PlotResults - Summary dashboard for all 5 scenarios (R2026a compatible)

    nScen  = numel(allMetrics);
    names  = cellfun(@(m) strrep(m.ScenarioName,'_',' '), allMetrics,'UniformOutput',false);
    avgSpd = cellfun(@(m) mean(m.Speeds),           allMetrics);
    smooth = cellfun(@(m) rms(diff(m.SteerAngles)), allMetrics);
    minD   = cellfun(@(m) minFinite(m.MinDistances),allMetrics);
    brkCt  = cellfun(@(m) m.BrakingCount,           allMetrics);
    goal   = cellfun(@(m) m.GoalReached,             allMetrics);
    collis = cellfun(@(m) m.CollisionOccurred,       allMetrics);

    fig = figure('Name','SIH — Performance Dashboard','NumberTitle','off',...
                 'Color',[0.10 0.10 0.16],'Position',[60 60 1300 720]);
    tl  = tiledlayout(2,3,'Padding','compact','TileSpacing','compact');
    title(tl,'Adaptive Path Planning — All 5 Indian Road Scenarios',...
          'Color','w','FontSize',14,'FontWeight','bold');

    barColor = @(c) repmat(c, nScen, 1);

    % 1 — Average Speed
    nexttile; barChart(avgSpd, names, barColor([0.20 0.60 1.00]), 'Avg Speed (m/s)', 'm/s');

    % 2 — Path Smoothness
    nexttile; barChart(smooth, names, barColor([0.20 0.85 0.50]), 'Path Smoothness (RMS Δsteer)', 'rad');

    % 3 — Min Clearance (traffic-light coloring)
    nexttile;
    ax3 = gca; cols3 = zeros(nScen,3);
    for i=1:nScen
        if minD(i)<2; cols3(i,:)=[0.9 0.2 0.2];
        elseif minD(i)<5; cols3(i,:)=[0.95 0.7 0.1];
        else; cols3(i,:)=[0.2 0.8 0.4]; end
    end
    b3=bar(minD,'FaceColor','flat'); b3.CData=cols3;
    stylePlotAx(ax3, names, 'Min Clearance Distance (m)', 'm');

    % 4 — Braking Events
    nexttile; barChart(brkCt, names, barColor([0.95 0.40 0.20]), 'Braking Events', 'count');

    % 5 — Goal Reached
    nexttile;
    ax5 = gca; cols5=zeros(nScen,3);
    for i=1:nScen; if goal(i); cols5(i,:)=[0.2 0.8 0.4]; else; cols5(i,:)=[0.9 0.2 0.2]; end; end
    b5=bar(double(goal),'FaceColor','flat'); b5.CData=cols5;
    stylePlotAx(ax5, names, 'Goal Reached', '');
    set(ax5,'YTick',[0 1],'YTickLabel',{'No','Yes'});

    % 6 — Pass/Fail Text Table
    nexttile; ax6=gca; axis(ax6,'off');
    set(ax6,'Color',[0.10 0.10 0.16]);
    txt = sprintf('%-20s  %4s  %7s\n','Scenario','Goal','Collide');
    txt = [txt repmat('-',1,36) newline];
    for i=1:nScen
        g=ternary(goal(i),'PASS','FAIL');
        c=ternary(collis(i),'YES ','NO  ');
        txt=[txt sprintf('%-20s  %4s  %7s\n', names{i}(1:min(20,end)), g, c)];
    end
    text(0.05,0.92,txt,'Color','w','FontSize',9,'FontName','Courier',...
         'VerticalAlignment','top','Units','normalized','Parent',ax6);
    title(ax6,'Pass / Fail Summary','Color','w','FontWeight','bold');
end

function barChart(data, names, col, ttl, ylbl)
    b=bar(data,'FaceColor','flat'); b.CData=col;
    stylePlotAx(gca, names, ttl, ylbl);
end

function stylePlotAx(ax, names, ttl, ylbl)
    set(ax,'Color',[0.16 0.16 0.22],'XColor','w','YColor','w',...
           'XTick',1:numel(names),'XTickLabel',names,'FontSize',8);
    title(ax, ttl, 'Color','w','FontSize',10);
    if ~isempty(ylbl); ylabel(ax, ylbl, 'Color','w'); end
    xtickangle(ax, 18);
end

function v = minFinite(arr)
    f = arr(isfinite(arr));
    if isempty(f); v=Inf; else; v=min(f); end
end

function r = ternary(cond, a, b)
    if cond; r=a; else; r=b; end
end
