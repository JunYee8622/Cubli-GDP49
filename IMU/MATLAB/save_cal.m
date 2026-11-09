function cal = save_cal(path, updates)
% SAVE_CAL  Merge the fields of the struct UPDATES into calibration.json and write it back.
cal = load_cal(path);
for f = fieldnames(updates)'
    cal.(f{1}) = updates.(f{1});
end
fid = fopen(path, 'w');
fwrite(fid, jsonencode(cal, 'PrettyPrint', true));
fclose(fid);
end
