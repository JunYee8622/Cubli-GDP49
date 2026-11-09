function log_serial(port, varargin)
% LOG_SERIAL  Record the ESP32 stream to a CSV file.
%
%   log_serial('COM5', 'Label', 'still', 'Seconds', 300)
%   log_serial('/dev/ttyUSB0', 'Label', 'six_pose')
%
% A small window opens while logging. Press Enter / Space in that window, or click the
% Marker button, to insert a marker (the 'marker' column increments). The tests use markers
% to split a recording into poses / segments. Close the window (or Ctrl+C) to stop early.
% Opening the port resets the ESP32, which is expected.
%
% Options: Baud (921600), Label ('log', used in the file name), Seconds (0 = until stopped),
% Dir ('data').
%
% Needs MATLAB R2019b+ (serialport). NOTE: unlike the analysis scripts this one has not been
% tested against a real board - check the first recording with visualise_graph.
p = inputParser;
p.addRequired('port', @(x) ischar(x) || isstring(x));
p.addParameter('Baud', 921600, @isscalar);
p.addParameter('Label', 'log', @(x) ischar(x) || isstring(x));
p.addParameter('Seconds', 0, @isscalar);
p.addParameter('Dir', 'data', @(x) ischar(x) || isstring(x));
p.parse(port, varargin{:});
args = p.Results;

c = imu_const();
HEADER = c.HEADER;

if ~isfolder(args.Dir), mkdir(args.Dir); end
path = fullfile(args.Dir, sprintf('%s_%s.csv', args.Label, char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));

s = serialport(args.port, args.Baud, 'Timeout', 1);
cleanup_s = onCleanup(@() delete(s));
pause(1.5);  % board reboots when the port opens
flush(s);

markers = 0;
fig = figure('Name', 'log_serial', 'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
             'Position', [100 100 360 120], 'KeyPressFcn', @on_key);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Marker (Enter / Space)', ...
          'Units', 'normalized', 'Position', [0.1 0.35 0.8 0.5], 'FontSize', 12, 'Callback', @(~, ~) add_marker());
status = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.05 0.02 0.9 0.25], ...
                   'String', 'starting...', 'HorizontalAlignment', 'left');

fid = fopen(path, 'w');
cleanup_f = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', strjoin(HEADER, ','));
fprintf('logging to %s  (marker: Enter/Space in the window, stop: close the window or Ctrl+C)\n', path);

n = 0;
t0 = tic;
while isgraphics(fig) && (args.Seconds == 0 || toc(t0) < args.Seconds)
    if s.NumBytesAvailable == 0
        pause(0.002);
        drawnow limitrate;
        continue
    end
    line = strtrim(char(readline(s)));
    if isempty(line), continue, end
    if line(1) == '#'
        fprintf('%s\n', line);  % status / overrun messages from the board
        continue
    end
    if sum(line == ',') ~= numel(HEADER) - 1 || strncmp(line, 't_us', 4)
        continue  % header or a partial line from the reset
    end
    fprintf(fid, '%s\n', line);
    n = n + 1;
    if mod(n, 500) == 0
        set(status, 'String', sprintf('%d samples, %.0f s, marker %d', n, toc(t0), markers));
        drawnow limitrate;
    end
    if mod(n, 2500) == 0
        fprintf('  %d samples, %.0f s\n', n, toc(t0));
    end
end
if isgraphics(fig), close(fig); end
fprintf('done: %d samples -> %s\n', n, path);

    function add_marker()
        write(s, uint8('m'), 'uint8');
        markers = markers + 1;
        fprintf('  marker -> %d\n', markers);
    end

    function on_key(~, evt)
        if any(strcmp(evt.Key, {'return', 'space'}))
            add_marker();
        end
    end
end
