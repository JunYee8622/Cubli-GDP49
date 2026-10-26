function y = wrap180(x)
% WRAP180  Wrap an angle difference into [-180, 180) deg.
y = mod(x + 180.0, 360.0) - 180.0;
end
