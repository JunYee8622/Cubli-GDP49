function [a, b] = quietest_window(signal, i0, i1, n)
% QUIETEST_WINDOW  Window of N samples inside i0:i1 (inclusive) where SIGNAL has the lowest
% variance. Returns the first and last index of the window (inclusive).
x = signal(i0:i1);
x = x(:);
if numel(x) <= n
    a = i0;
    b = i1;
    return
end
c1 = cumsum([0; x]);
c2 = cumsum([0; x .* x]);
s = c1(n + 1:end) - c1(1:end - n);
v = (c2(n + 1:end) - c2(1:end - n)) / n - (s / n) .^ 2;
[~, k] = min(v);
a = i0 + k - 1;
b = a + n - 1;
end
