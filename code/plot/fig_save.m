function fig_save(h, filename, str)
% FIG_SAVE saves the figure with handle h according to specific format in 
% string str under the current directory.

% Written by Qiong Wang at University of Pennsylvania
% Nov. 9th, 2013

% Set units to inches
set(h, 'Units', 'inches');
set(h, 'PaperUnits', 'inches');

% Set paper position and size as on the screenx
figPos    = get(h, 'Position');
figWidth  = figPos(3);
figHeight = figPos(4);
set(h, 'PaperPosition', [0 0 figWidth figHeight]);
set(h, 'PaperSize',     [figWidth figHeight]);

% Save as pdf
print(h, ['-d', str], filename);
end