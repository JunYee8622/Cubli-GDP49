function fig = new_fig(args, w, h)
% NEW_FIG  Figure of W x H pixels; hidden unless the Show option is true.
vis = 'off';
if args.Show, vis = 'on'; end
fig = figure('Visible', vis, 'Color', 'w', 'Position', [100 100 w h]);
try
    theme(fig, 'light');  % R2025a+: keep plots readable when MATLAB is in dark mode
catch
end
end
