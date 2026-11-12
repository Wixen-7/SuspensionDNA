function imp = compare_metrics(mPassive, mOther)
%COMPARE_METRICS  Percentage improvement vs passive (positive = better, lower-is-better metrics).
f = fieldnames(mPassive);
for i = 1:numel(f)
    a = mPassive.(f{i});  b = mOther.(f{i});
    if isnan(a) || a == 0, imp.(f{i}) = NaN; else, imp.(f{i}) = (a - b) / a * 100; end
end
end
