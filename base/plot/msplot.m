function msplot(mnp, x, y, xstr, ystr)
% my sub plot
    if mod(mnp,10)==1, figure; end   % 如果是第一幅小图，则新建一个figure
    subplot(mnp); plot(x, y, 'linewidth', 2); grid on;
    if nargin==4, ystr = xstr; xstr = '\itt\rm / s'; end  % 如果只输入一个字符串，则默认xlabel为时间
    if exist('ystr', 'var'), xlabel(xstr); ylabel(ystr); end