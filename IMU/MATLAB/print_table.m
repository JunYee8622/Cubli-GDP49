function print_table(headers, rows, width)
% PRINT_TABLE  Right-aligned table. HEADERS is a cell array of strings, ROWS a cell array
% (or numeric matrix). Numbers print with 3 decimals; anything else is printed as text.
if nargin < 3, width = 10; end
if isnumeric(rows), rows = num2cell(rows); end
fmt = sprintf('%%%ds', width);
cells = cellfun(@(h) sprintf(fmt, char(h)), headers, 'UniformOutput', false);
fprintf('%s\n', strjoin(cells, ' '));
for r = 1:size(rows, 1)
    cells = cell(1, size(rows, 2));
    for c = 1:size(rows, 2)
        v = rows{r, c};
        if isnumeric(v)
            txt = sprintf('%.3f', v);
        else
            txt = char(string(v));
        end
        cells{c} = sprintf(fmt, txt);
    end
    fprintf('%s\n', strjoin(cells, ' '));
end
end
