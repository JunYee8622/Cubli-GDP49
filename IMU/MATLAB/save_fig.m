function save_fig(fig, args, name)
% SAVE_FIG  Save FIG as a PNG in the Out folder, then close it (unless Show is true).
if ~isfolder(args.Out), mkdir(args.Out); end
path = fullfile(args.Out, name);
exportgraphics(fig, path, 'Resolution', 130);
fprintf('plot saved: %s\n', path);
if ~args.Show, close(fig); end
end
