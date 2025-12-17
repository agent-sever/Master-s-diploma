function [x_fig,y_fig] = ds2nfu(ax, x, y)
    axpos = ax.Position;
    xl = ax.XLim;
    yl = ax.YLim;

    x_fig = axpos(1) + (x - xl(1)) / (xl(2) - xl(1)) * axpos(3);
    y_fig = axpos(2) + (y - yl(1)) / (yl(2) - yl(1)) * axpos(4);
end