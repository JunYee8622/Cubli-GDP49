function out = complementary_filter(rate, acc_ang, dt, alpha)
% COMPLEMENTARY_FILTER  Same maths as ComplementaryFilter in firmware/filters.h.
out = zeros(size(rate));
a = acc_ang(1);
for i = 1:numel(rate)
    pred = a + rate(i) * dt(i);
    a = pred + (1 - alpha) * wrap180(acc_ang(i) - pred);  % = alpha*pred + (1-alpha)*acc, wrap-safe
    out(i) = a;
end
end
