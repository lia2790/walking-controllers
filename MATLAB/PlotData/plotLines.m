limitsY = ylim;
line([time(initSS(1)), time(initSS(1))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
line([time(initDS(1)), time(initDS(1))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);


for indexPhase = 2:length(initSS)
    line([time(initSS(indexPhase)), time(initSS(indexPhase))], limitsY,'Color',[0.4660, 0.6740, 0.1880],'LineStyle', '-.', 'linewidth', 3.5);
end
for indexPhase = 2:length(initDS)
    line([time(initDS(indexPhase)), time(initDS(indexPhase))], limitsY,'Color',[0.4940, 0.1840, 0.5560],'LineStyle', '-.', 'linewidth', 3.5);
end