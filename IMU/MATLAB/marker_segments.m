function segs = marker_segments(marker)
% MARKER_SEGMENTS  [first last] sample indices (inclusive, 1-based) of each run of equal marker values.
marker = marker(:);
edges = find(diff(marker) ~= 0) + 1;
segs = [[1; edges], [edges - 1; numel(marker)]];
end
