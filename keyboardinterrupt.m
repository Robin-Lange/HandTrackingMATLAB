test

function test
fig = uifigure();
ax = uiaxes('Parent', fig);
h = plot(ax, NaN);
stop = false;
    function setstop(varargin)
        stop = true;
    end
b = uibutton(fig, 'text', 'stop', 'ButtonPushedFcn', @setstop);
for i = 1:Inf
    set(h,'YData', sin(linspace(0,2*pi)-i/10));
    pause(0.03);
    if stop
        break
    end
end
end