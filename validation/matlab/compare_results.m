function results = compare_results(varargin)
%COMPARE_RESULTS Compare C# validation outputs against MATLAB golden data.
%   compare_results()                       all scenarios, default folders
%   compare_results('Scenarios', {'S01'})   subset
%   compare_results('CsharpDir', d, 'PlotDir', p, 'OutDir', o)  alternative folders
%
%   Reference layer: golden/<id>/{tables_gear1..6,resist,r1_history,r2_history}.csv
%   and summary.json vs out/csharp/<id>/ (same names), tolerance |e| <= abs + rel*|g|.
%   Interactive layer (separate section): out/csharp/<id>/interactive_r1.csv vs
%   golden/<id>/r1_history.csv, interpolated on time and on distance.
%   Tolerances: validation/matlab/compare_config.json.
%   Writes <OutDir>/comparison.json, <OutDir>/comparison.md and PNGs to <PlotDir>.

here   = fileparts(mfilename('fullpath'));
valDir = fileparts(here);

p = inputParser;
p.addParameter('Scenarios', {});
p.addParameter('GoldenDir', fullfile(valDir, 'golden'));
p.addParameter('CsharpDir', fullfile(valDir, 'out', 'csharp'));
p.addParameter('PlotDir',   fullfile(valDir, 'plots'));
p.addParameter('OutDir',    fullfile(valDir, 'out'));
p.addParameter('Config',    fullfile(here, 'compare_config.json'));
p.parse(varargin{:});
o = p.Results;

cfg = jsondecode(fileread(o.Config));
scen = jsondecode(fileread(fullfile(valDir, 'scenarios.json')));
list = scen.scenarios; if ~iscell(list), list = num2cell(list); end
ids = cellfun(@(e) e.id, list, 'UniformOutput', false);
cmpInter = cellfun(@(e) ~isfield(e, 'compare_interactive') || e.compare_interactive, list);
if ~isempty(o.Scenarios)
    sel = ismember(ids, cellstr(o.Scenarios));
    ids = ids(sel); cmpInter = cmpInter(sel);
end
doPlots = ~isfield(cfg, 'plots') || cfg.plots;
if doPlots && ~exist(o.PlotDir, 'dir'), mkdir(o.PlotDir); end
if ~exist(o.OutDir, 'dir'), mkdir(o.OutDir); end
oldVis = get(groot, 'DefaultFigureVisible');
set(groot, 'DefaultFigureVisible', 'off');
c1 = onCleanup(@() set(groot, 'DefaultFigureVisible', oldVis));

rt = cfg.reference;
results = struct('generated', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    'golden_dir', o.GoldenDir, 'csharp_dir', o.CsharpDir, 'config', cfg, ...
    'reference', struct([]), 'interactive', struct([]));
refRes = {}; intRes = {};

tableFiles = [arrayfun(@(g) sprintf('tables_gear%d.csv', g), 1:6, 'UniformOutput', false), ...
    {'resist.csv', 'engine_torque.csv', 'pchip_probe.csv'}];
optionalFiles = {'engine_torque.csv', 'pchip_probe.csv'};

for k = 1:numel(ids)
    id = ids{k};
    gDir = fullfile(o.GoldenDir, id);
    cDir = fullfile(o.CsharpDir, id);
    R = struct('id', id, 'status', 'MISSING', 'notes', {{}}, 'files', struct([]), ...
        'histories', struct([]), 'summary', struct([]), 'worst_abs', NaN, 'worst_rel', NaN);
    if ~exist(gDir, 'dir')
        R.notes{end+1} = 'golden folder missing (run run_reference)';
        refRes{end+1} = R; continue; %#ok<AGROW>
    end
    if ~exist(cDir, 'dir')
        R.notes{end+1} = ['C# folder missing: ' cDir];
        refRes{end+1} = R; continue; %#ok<AGROW>
    end
    allPass = true;
    files = {};
    for f = 1:numel(tableFiles)
        fn = tableFiles{f};
        gp = fullfile(gDir, fn); cp = fullfile(cDir, fn);
        if ~isfile(cp)
            if ismember(fn, optionalFiles), continue; end
            files{end+1} = struct('file', fn, 'present', false, 'columns', struct([])); %#ok<AGROW>
            allPass = false; R.notes{end+1} = ['missing C# file ' fn]; continue;
        end
        G = read_csv(gp); C = read_csv(cp);
        [cols, ok, note] = compare_columns(G, C, rt);
        if ~isempty(note), R.notes{end+1} = [fn ': ' note]; end
        allPass = allPass && ok;
        files{end+1} = struct('file', fn, 'present', true, 'columns', cols); %#ok<AGROW>
        if doPlots && strcmp(fn, 'tables_gear1.csv'), plot_tables(gDir, cDir, id, o.PlotDir); end
    end
    R.files = [files{:}];

    hist = {};
    for r = {'r1', 'r2'}
        fn = [r{1} '_history.csv'];
        cp = fullfile(cDir, fn);
        if ~isfile(cp)
            allPass = false; R.notes{end+1} = ['missing C# file ' fn]; continue;
        end
        G = read_csv(fullfile(gDir, fn)); C = read_csv(cp);
        H = compare_history(G, C, rt);
        H.file = fn;
        if ~H.same_length
            R.notes{end+1} = sprintf('%s: length differs (golden %d, C# %d, common %d)', ...
                fn, H.n_golden, H.n_csharp, H.n_common);
        end
        allPass = allPass && H.pass;
        hist{end+1} = H; %#ok<AGROW>
        if doPlots, plot_history(G, C, id, r{1}, o.PlotDir); end
    end
    R.histories = [hist{:}];

    gs = jsondecode(fileread(fullfile(gDir, 'summary.json')));
    cs = struct();
    if isfile(fullfile(cDir, 'summary.json'))
        cs = jsondecode(fileread(fullfile(cDir, 'summary.json')));
    else
        R.notes{end+1} = 'missing C# summary.json'; allPass = false;
    end
    keys = fieldnames(gs);
    sm = {};
    for j = 1:numel(keys)
        key = keys{j}; gv = gs.(key);
        if ~isnumeric(gv), continue; end
        e = struct('key', key, 'golden', gv, 'csharp', NaN, 'abs_err', NaN, 'rel_err', NaN, 'pass', false);
        if isfield(cs, key) && isnumeric(cs.(key))
            e.csharp = cs.(key);
            e.abs_err = abs(e.csharp - gv);
            e.rel_err = e.abs_err / max(abs(gv), realmin);
            e.pass = e.abs_err <= rt.abs_tol + rt.rel_tol * abs(gv);
        else
            R.notes{end+1} = ['summary key missing in C#: ' key];
        end
        allPass = allPass && e.pass;
        sm{end+1} = e; %#ok<AGROW>
    end
    R.summary = [sm{:}];

    [R.worst_abs, R.worst_rel] = worst_errors(R);
    R.status = ternary(allPass, 'PASS', 'FAIL');
    refRes{end+1} = R; %#ok<AGROW>
    fprintf('[reference] %s: %s  worst abs %.3g  worst rel %.3g\n', id, R.status, R.worst_abs, R.worst_rel);
end

it = cfg.interactive;
for k = 1:numel(ids)
    id = ids{k};
    cp = fullfile(o.CsharpDir, id, 'interactive_r1.csv');
    gp = fullfile(o.GoldenDir, id, 'r1_history.csv');
    if ~cmpInter(k), continue; end
    I = struct('id', id, 'status', 'MISSING', 'notes', {{}}, 'by_time', struct([]), ...
        'by_distance', struct([]), 't_reach', struct([]));
    if ~isfile(cp) || ~isfile(gp)
        I.notes{end+1} = 'interactive_r1.csv or golden r1_history.csv missing';
        intRes{end+1} = I; continue; %#ok<AGROW>
    end
    G = read_csv(gp); C = read_csv(cp);
    dcol = '';
    if isfield(C.data, 'track_s_m'), dcol = 'track_s_m'; elseif isfield(C.data, 'd_m'), dcol = 'd_m'; end
    if isempty(dcol) || ~isfield(C.data, 't_s') || ~isfield(C.data, 'v_kmh')
        I.notes{end+1} = 'interactive_r1.csv lacks t_s / v_kmh / track_s_m';
        intRes{end+1} = I; continue; %#ok<AGROW>
    end
    I = compare_interactive(I, G, C, dcol, it);
    intRes{end+1} = I; %#ok<AGROW>
    if doPlots, plot_interactive(G, C, dcol, id, o.PlotDir); end
    fprintf('[interactive] %s: %s  v err by time max %.3g km/h, by distance max %.3g km/h\n', ...
        id, I.status, I.by_time.max_abs, I.by_distance.max_abs);
end

results.reference = refRes;
results.interactive = intRes;
fid = fopen(fullfile(o.OutDir, 'comparison.json'), 'w');
fwrite(fid, jsonencode(results, 'PrettyPrint', true)); fclose(fid);
write_markdown(fullfile(o.OutDir, 'comparison.md'), results, cfg);
fprintf('Wrote %s and comparison.md\n', fullfile(o.OutDir, 'comparison.json'));
end

% -------------------------------------------------------------------------
function T = read_csv(path)
txt = splitlines(string(fileread(path)));
txt = txt(strlength(strtrim(txt)) > 0 & ~startsWith(strtrim(txt), '#'));
hdr = strtrim(split(txt(1), ','));
body = txt(2:end);
T.header = cellstr(hdr.');
T.n = numel(body);
T.data = struct();
if T.n == 0
    for c = 1:numel(hdr), T.data.(char(hdr(c))) = zeros(0, 1); end
    return;
end
cells = split(body, ',', 2);
if size(cells, 2) ~= numel(hdr) && T.n == 1, cells = reshape(cells, 1, []); end
if size(cells, 2) ~= numel(hdr)
    error('compare_results:csv', '%s: row width %d != header width %d', path, size(cells, 2), numel(hdr));
end
for c = 1:numel(hdr)
    T.data.(matlab.lang.makeValidName(char(hdr(c)))) = str2double(cells(:, c));
end
end

function m = col_metrics(g, c, tol)
e = c - g;
m.n = numel(g);
m.max_abs = max([abs(e); 0]);
m.rmse = sqrt(mean(e.^2));
nz = g ~= 0;
if any(nz), m.max_rel = max(abs(e(nz)) ./ abs(g(nz))); else, m.max_rel = NaN; end
bad = ~(abs(e) <= tol.abs_tol + tol.rel_tol * abs(g)) & ~(isnan(g) & isnan(c));
m.n_fail = nnz(bad);
m.pass = m.n_fail == 0;
[~, m.worst_index] = max(abs(e));
if isempty(m.worst_index), m.worst_index = NaN; end
end

function [cols, ok, note] = compare_columns(G, C, tol)
note = ''; ok = true; cols = {};
if G.n ~= C.n
    note = sprintf('row count differs (golden %d, C# %d); compared first %d', G.n, C.n, min(G.n, C.n));
    ok = false;
end
n = min(G.n, C.n);
for j = 1:numel(G.header)
    h = matlab.lang.makeValidName(G.header{j});
    if ~isfield(C.data, h)
        cols{end+1} = struct('column', G.header{j}, 'n', 0, 'max_abs', NaN, 'rmse', NaN, ...
            'max_rel', NaN, 'n_fail', NaN, 'pass', false, 'worst_index', NaN); %#ok<AGROW>
        ok = false; note = strtrim([note ' missing column ' G.header{j}]);
        continue;
    end
    m = col_metrics(G.data.(h)(1:n), C.data.(h)(1:n), tol);
    cols{end+1} = cell2struct([{G.header{j}}; struct2cell(m)], [{'column'}; fieldnames(m)], 1); %#ok<AGROW>
    ok = ok && m.pass;
end
cols = [cols{:}];
end

function H = compare_history(G, C, tol)
H.n_golden = G.n; H.n_csharp = C.n;
H.same_length = G.n == C.n;
[tf, loc] = ismembertol(G.data.t_s, C.data.t_s, 1e-9, 'DataScale', 1);
H.n_common = nnz(tf);
cols = {};
for name = {'t_s', 'd_m', 'v_kmh'}
    g = G.data.(name{1})(tf); c = C.data.(name{1})(loc(tf));
    m = col_metrics(g, c, tol);
    cols{end+1} = cell2struct([{name{1}}; struct2cell(m)], [{'column'}; fieldnames(m)], 1); %#ok<AGROW>
end
H.columns = [cols{:}];
H.pass = H.same_length && H.n_common == G.n && all([H.columns.pass]);
end

function I = compare_interactive(I, G, C, dcol, it)
tol = struct('abs_tol', it.v_kmh_abs_tol, 'rel_tol', it.v_kmh_rel_tol);
tg = G.data.t_s; dg = G.data.d_m; vg = G.data.v_kmh;
tc = C.data.t_s; dc = C.data.(dcol); vc = C.data.v_kmh;
I.n_golden = G.n; I.n_interactive = C.n;

in = tg >= min(tc) & tg <= max(tc);
vi = interp1(tc, vc, tg(in), 'linear');
I.by_time = col_metrics(vg(in), vi, tol);
I.by_time.t_range = [min(tg(in)), max(tg(in))];

[dcu, iu] = unique(dc, 'stable');
mono = [true; diff(dcu) > 0];
dcu = dcu(mono); vcu = vc(iu(mono)); tcu = tc(iu(mono));
in = dg >= min(dcu) & dg <= max(dcu);
vi = interp1(dcu, vcu, dg(in), 'linear');
I.by_distance = col_metrics(vg(in), vi, tol);
I.by_distance.d_range = [min(dg(in)), max(dg(in))];

dRef = min(max(dg), max(dcu));
[dgu, ig] = unique(dg, 'stable');
tR.d_ref_m = dRef;
tR.t_golden = interp1(dgu, tg(ig), dRef, 'linear');
tR.t_interactive = interp1(dcu, tcu, dRef, 'linear');
tR.abs_err = abs(tR.t_interactive - tR.t_golden);
tR.pass = tR.abs_err <= it.t_reach_abs_tol_s;
I.t_reach = tR;
I.status = ternary(I.by_time.pass && I.by_distance.pass && tR.pass, 'PASS', 'FAIL');
end

function [wa, wr] = worst_errors(R)
a = []; r = [];
for f = R.files, if f.present && ~isempty(f.columns), a = [a, f.columns.max_abs]; r = [r, f.columns.max_rel]; end, end %#ok<AGROW>
for h = R.histories, a = [a, h.columns.max_abs]; r = [r, h.columns.max_rel]; end %#ok<AGROW>
if ~isempty(R.summary), a = [a, R.summary.abs_err]; r = [r, R.summary.rel_err]; end
wa = max([a, -Inf]); wr = max([r, -Inf]);
if isinf(wa), wa = NaN; end
if isinf(wr), wr = NaN; end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

% -------------------------------------------------------------------------
function plot_history(G, C, id, tag, plotDir)
[tf, loc] = ismembertol(G.data.t_s, C.data.t_s, 1e-9, 'DataScale', 1);
t = G.data.t_s(tf);
f = newfig([0 0 1000 700]);
subplot(2, 1, 1);
semilogy_safe(t, abs(C.data.v_kmh(loc(tf)) - G.data.v_kmh(tf)));
ylabel('|v_{C#} - v_{MATLAB}| [km/h]'); grid on;
title(sprintf('%s %s: error vs time (%d common samples)', id, upper(tag), nnz(tf)));
subplot(2, 1, 2);
semilogy_safe(t, abs(C.data.d_m(loc(tf)) - G.data.d_m(tf)));
ylabel('|d_{C#} - d_{MATLAB}| [m]'); xlabel('t [s]'); grid on;
exportgraphics(f, fullfile(plotDir, sprintf('%s_%s_error.png', id, tag)), 'Resolution', 110);
close(f);

f = newfig([0 0 1000 500]);
plot(G.data.t_s, G.data.v_kmh, 'k-', 'LineWidth', 2.5); hold on;
plot(C.data.t_s, C.data.v_kmh, 'r--', 'LineWidth', 1.2);
legend('MATLAB golden', 'C# reference layer', 'Location', 'southeast');
xlabel('t [s]'); ylabel('v [km/h]'); grid on;
title(sprintf('%s %s: speed overlay', id, upper(tag)));
exportgraphics(f, fullfile(plotDir, sprintf('%s_%s_overlay.png', id, tag)), 'Resolution', 110);
close(f);
end

function plot_tables(gDir, cDir, id, plotDir)
f = newfig([0 0 1000 700]);
cols = lines(6);
for pnl = 1:2
    subplot(2, 1, pnl); hold on; grid on;
    for g = 1:6
        fn = sprintf('tables_gear%d.csv', g);
        if ~isfile(fullfile(cDir, fn)), continue; end
        G = read_csv(fullfile(gDir, fn)); C = read_csv(fullfile(cDir, fn));
        if G.n ~= C.n, continue; end
        if pnl == 1, col = 'a_x_real'; else, col = 'f_tractive_eff_n'; end
        semilogy_safe(G.data.rpm, abs(C.data.(col) - G.data.(col)), cols(g, :));
    end
    if pnl == 1, ylabel('|err a\_x\_real| [m/s^2]'); else, ylabel('|err f\_tractive\_eff| [N]'); xlabel('rpm'); end
    legend(arrayfun(@(g) sprintf('gear %d', g), 1:6, 'UniformOutput', false), 'Location', 'eastoutside');
end
sgtitle(sprintf('%s: per-gear table errors (zero errors not drawn)', id));
exportgraphics(f, fullfile(plotDir, sprintf('%s_tables_error.png', id)), 'Resolution', 110);
close(f);
end

function plot_interactive(G, C, dcol, id, plotDir)
f = newfig([0 0 1200 800]);
subplot(2, 2, 1);
plot(G.data.t_s, G.data.v_kmh, 'k-', 'LineWidth', 2.5); hold on;
plot(C.data.t_s, C.data.v_kmh, 'r--', 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('v [km/h]'); legend('MATLAB R1', 'C# interactive', 'Location', 'southeast');
title('speed vs time');
subplot(2, 2, 2);
plot(G.data.d_m, G.data.v_kmh, 'k-', 'LineWidth', 2.5); hold on;
plot(C.data.(dcol), C.data.v_kmh, 'r--', 'LineWidth', 1.2); grid on;
xlabel('d [m]'); ylabel('v [km/h]'); title('speed vs distance');
subplot(2, 2, 3);
in = G.data.t_s >= min(C.data.t_s) & G.data.t_s <= max(C.data.t_s);
plot(G.data.t_s(in), interp1(C.data.t_s, C.data.v_kmh, G.data.t_s(in)) - G.data.v_kmh(in), 'b-'); grid on;
xlabel('t [s]'); ylabel('v_{int} - v_{MATLAB} [km/h]'); title('error vs time');
subplot(2, 2, 4);
[dcu, iu] = unique(C.data.(dcol), 'stable'); vcu = C.data.v_kmh(iu);
mono = [true; diff(dcu) > 0]; dcu = dcu(mono); vcu = vcu(mono);
in = G.data.d_m >= min(dcu) & G.data.d_m <= max(dcu);
plot(G.data.d_m(in), interp1(dcu, vcu, G.data.d_m(in)) - G.data.v_kmh(in), 'b-'); grid on;
xlabel('d [m]'); ylabel('v_{int} - v_{MATLAB} [km/h]'); title('error vs distance');
sgtitle(sprintf('%s: interactive layer vs MATLAB R1', id));
exportgraphics(f, fullfile(plotDir, sprintf('%s_interactive_r1.png', id)), 'Resolution', 110);
close(f);
end

function semilogy_safe(x, y, col)
if nargin < 3, col = [0 0.45 0.74]; end
y(y == 0) = NaN;
if all(isnan(y))
    plot(x, zeros(size(x)), '-', 'Color', col);
    text(0.02, 0.9, 'all errors exactly 0', 'Units', 'normalized');
else
    semilogy(x, y, '-', 'Color', col);
end
end

% -------------------------------------------------------------------------
function write_markdown(path, res, cfg)
L = {};
L{end+1} = '# Validation comparison: C# vs MATLAB golden';
L{end+1} = '';
L{end+1} = sprintf('Generated %s. Golden: `%s`. C#: `%s`.', res.generated, res.golden_dir, res.csharp_dir);
L{end+1} = sprintf('Reference-layer tolerance: |e| <= %g + %g*|golden| on every element.', ...
    cfg.reference.abs_tol, cfg.reference.rel_tol);
L{end+1} = '';
L{end+1} = '## Reference layer';
L{end+1} = '';
L{end+1} = '| scenario | status | worst abs err | worst rel err | notes |';
L{end+1} = '|---|---|---|---|---|';
for k = 1:numel(res.reference)
    R = res.reference{k};
    L{end+1} = sprintf('| %s | %s | %.3g | %.3g | %s |', R.id, R.status, R.worst_abs, R.worst_rel, ...
        strjoin(R.notes, '; ')); %#ok<AGROW>
end
for k = 1:numel(res.reference)
    R = res.reference{k};
    if strcmp(R.status, 'MISSING'), continue; end
    L{end+1} = ''; L{end+1} = sprintf('### %s', R.id); L{end+1} = ''; %#ok<AGROW>
    L{end+1} = '| file | column | n | max abs | RMSE | max rel | fails | pass |'; %#ok<AGROW>
    L{end+1} = '|---|---|---|---|---|---|---|---|'; %#ok<AGROW>
    for f = R.files
        for c = f.columns
            L{end+1} = sprintf('| %s | %s | %d | %.3g | %.3g | %.3g | %d | %d |', f.file, c.column, c.n, ...
                c.max_abs, c.rmse, c.max_rel, c.n_fail, c.pass); %#ok<AGROW>
        end
    end
    for h = R.histories
        for c = h.columns
            L{end+1} = sprintf('| %s (common %d of %d/%d) | %s | %d | %.3g | %.3g | %.3g | %d | %d |', h.file, ...
                h.n_common, h.n_golden, h.n_csharp, c.column, c.n, c.max_abs, c.rmse, c.max_rel, c.n_fail, c.pass); %#ok<AGROW>
        end
    end
    L{end+1} = ''; %#ok<AGROW>
    L{end+1} = '| summary key | golden | C# | abs err | rel err | pass |'; %#ok<AGROW>
    L{end+1} = '|---|---|---|---|---|---|'; %#ok<AGROW>
    for s = R.summary
        L{end+1} = sprintf('| %s | %.17g | %.17g | %.3g | %.3g | %d |', s.key, s.golden, s.csharp, ...
            s.abs_err, s.rel_err, s.pass); %#ok<AGROW>
    end
end
L{end+1} = '';
L{end+1} = '## Interactive layer (separate; differs by design)';
L{end+1} = '';
L{end+1} = sprintf(['Speed tolerance |e| <= %g km/h + %g*|v|; time-to-reach tolerance %g s ' ...
    '(compare_config.json; justified in docs/validation_report.md).'], cfg.interactive.v_kmh_abs_tol, ...
    cfg.interactive.v_kmh_rel_tol, cfg.interactive.t_reach_abs_tol_s);
L{end+1} = '';
L{end+1} = '| scenario | status | v err by time: max / RMSE [km/h] | v err by distance: max / RMSE [km/h] | t to reach d_ref: MATLAB / C# [s] | notes |';
L{end+1} = '|---|---|---|---|---|---|';
for k = 1:numel(res.interactive)
    I = res.interactive{k};
    if strcmp(I.status, 'MISSING')
        L{end+1} = sprintf('| %s | MISSING | | | | %s |', I.id, strjoin(I.notes, '; ')); %#ok<AGROW>
    else
        L{end+1} = sprintf('| %s | %s | %.4g / %.4g | %.4g / %.4g | %.4f / %.4f (d_ref %.1f m) | %s |', I.id, I.status, ...
            I.by_time.max_abs, I.by_time.rmse, I.by_distance.max_abs, I.by_distance.rmse, ...
            I.t_reach.t_golden, I.t_reach.t_interactive, I.t_reach.d_ref_m, strjoin(I.notes, '; ')); %#ok<AGROW>
    end
end
fid = fopen(path, 'w');
fprintf(fid, '%s\n', L{:});
fclose(fid);
end

function f = newfig(pos)
f = figure('Position', pos, 'Color', 'w');
try
    f.Theme = 'light';
catch
end
end
