function cal = load_cal(path)
% LOAD_CAL  Read calibration.json (same file format as the Python scripts), or defaults.
%
%   Fields: acc_offset, acc_scale, gyro_bias (1x3 rows), plus anything else saved by the tests.
cal.acc_offset = [0.0, 0.0, 0.0];
cal.acc_scale = [1.0, 1.0, 1.0];
cal.gyro_bias = [0.0, 0.0, 0.0];
if nargin > 0 && ~isempty(path) && isfile(path)
    loaded = jsondecode(fileread(path));
    for f = fieldnames(loaded)'
        v = loaded.(f{1});
        if isnumeric(v) && numel(v) > 1, v = v(:).'; end  % jsondecode gives columns
        cal.(f{1}) = v;
    end
end
end
