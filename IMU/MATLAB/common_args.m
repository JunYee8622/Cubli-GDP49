function p = common_args()
% COMMON_ARGS  inputParser holding the options shared by every test script.
%
%   p = common_args();  p.addParameter('Extra', 1);  p.parse(file, varargin{:});  args = p.Results;
%
% Options (name-value pairs, same meaning as the Python flags):
%   Axis   tilt axis: 0 = about IMU X, 1 = about IMU Y (match TILT_AXIS in config.h)
%   Cal    calibration file (read/updated), default 'calibration.json'
%   Skip   seconds to drop from the start of the log
%   Out    folder for plots, default 'results'
%   Alpha  complementary filter alpha
%   QAngle, QBias, R   Kalman tuning (R in deg^2)
%   Show   true = leave the figure open on screen as well as saving the PNG
p = inputParser;
p.addRequired('file', @(x) ischar(x) || isstring(x));
p.addParameter('Axis', 0, @(x) isscalar(x) && any(x == [0 1]));
p.addParameter('Cal', 'calibration.json', @(x) ischar(x) || isstring(x));
p.addParameter('Skip', 0.0, @isscalar);
p.addParameter('Out', 'results', @(x) ischar(x) || isstring(x));
p.addParameter('Alpha', 0.98, @isscalar);
p.addParameter('QAngle', 0.001, @isscalar);
p.addParameter('QBias', 0.003, @isscalar);
p.addParameter('R', 0.03, @isscalar);
p.addParameter('Show', false, @(x) islogical(x) || isnumeric(x));
end
