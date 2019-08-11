limitsY = ylim;
line([time(initSS_left(1)), time(initSS_left(1))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
line([time(initSS_right(1)), time(initSS_right(1))], limitsY,'Color',[0.6, 0.4, 0.08],'LineStyle', '-.', 'linewidth', 3.5);
line([time(initDS(1)), time(initDS(1))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);

for indexPhase = 2:length(initSS_left)
    line([time(initSS_left(indexPhase)), time(initSS_left(indexPhase))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
end

for indexPhase = 2:length(initSS_right)
    line([time(initSS_right(indexPhase)), time(initSS_right(indexPhase))], limitsY,'Color',[0.6, 0.4, 0.08],'LineStyle', '-.', 'linewidth', 3.5);
end

for indexPhase = 2:length(initDS)
    line([time(initDS(indexPhase)), time(initDS(indexPhase))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);
end