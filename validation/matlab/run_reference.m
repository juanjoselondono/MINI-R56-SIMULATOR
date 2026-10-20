function run_reference(scenarioIds)
%RUN_REFERENCE Generate golden data from the unmodified reference/matlab/model.m.
%   run_reference()            runs every scenario in validation/scenarios.json
%   run_reference({'S01','S07'}) runs a subset
%
%   model.m is read as text; the "clear; clc; close all;" line is removed, each
%   override replaces the right-hand side of exactly one assignment line, and the
%   result is executed as a temporary function (clean workspace) written to
%   validation/out/tmp/. Outputs follow validation/FILE_CONTRACT.md and go to
%   validation/golden/<id>/.

here     = fileparts(mfilename('fullpath'));
valDir   = fileparts(here);
rootDir  = fileparts(valDir);
modelPth = fullfile(rootDir, 'reference', 'matlab', 'model.m');
scenPth  = fullfile(valDir, 'scenarios.json');
tmpDir   = fullfile(valDir, 'out', 'tmp');
goldDir  = fullfile(valDir, 'golden');

scen = load_scenarios(scenPth, modelPth);
if nargin < 1 || isempty(scenarioIds)
    scenarioIds = {scen.id};
end
scenarioIds = cellstr(scenarioIds);

modelText = fileread(modelPth);
modelSha  = sha256_file(modelPth);
fprintf('model.m sha256 = %s\n', modelSha);

if ~exist(tmpDir, 'dir'), mkdir(tmpDir); end
oldVis = get(groot, 'DefaultFigureVisible');
set(groot, 'DefaultFigureVisible', 'off');
cleanupVis = onCleanup(@() set(groot, 'DefaultFigureVisible', oldVis));
addpath(tmpDir);
cleanupPath = onCleanup(@() rmpath(tmpDir));

for k = 1:numel(scenarioIds)
    id = scenarioIds{k};
    s  = scen(strcmp({scen.id}, id));
    if isempty(s), error('run_reference:unknownScenario', 'Unknown scenario %s', id); end
    fprintf('--- %s: %s\n', id, s.description);

    txt = strip_clear_line(modelText);
    for j = 1:numel(s.overrideNames)
        txt = apply_override(txt, s.overrideNames{j}, s.overrideValues{j});
    end

    fname = ['model_ref_' regexprep(id, '\W', '_')];
    fpath = fullfile(tmpDir, [fname '.m']);
    capture = sprintf(['\n\n%% ---- appended by run_reference.m ----\n' ...
        'ws__ = struct();\nvn__ = who;\n' ...
        'for kk__ = 1:numel(vn__)\n' ...
        '    if ~strcmp(vn__{kk__}, ''ws__''), ws__.(vn__{kk__}) = eval(vn__{kk__}); end\n' ...
        'end\n']);
    write_text(fpath, sprintf('function ws__ = %s()\n%s%s', fname, txt, capture));
    clear(fname);
    rehash;
    ws = feval(fname);
    close all force;

    outDir = fullfile(goldDir, id);
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    export_scenario(ws, outDir, id, modelSha, s);
    fprintf('    F_x_max=%.6f  v_curva_final=%.6f  lap=%.6f s  -> %s\n', ...
        ws.car.F_x_max, ws.v_curva_final_kmh, ws.t_vuelta_completa, outDir);
end
end

% -------------------------------------------------------------------------
function scen = load_scenarios(scenPth, modelPth)
raw = jsondecode(fileread(scenPth));
list = raw.scenarios;
if ~iscell(list), list = num2cell(list); end
lhs = assignment_lhs(fileread(modelPth));
scen = struct('id', {}, 'description', {}, 'overrideNames', {}, 'overrideValues', {});
for k = 1:numel(list)
    e = list{k};
    names = {}; vals = {};
    if isfield(e, 'overrides') && isstruct(e.overrides)
        f = fieldnames(e.overrides);
        for j = 1:numel(f)
            % jsondecode mangles keys such as "car.M" -> "car_M"; map back to the
            % model.m left-hand side that mangles to the same field name.
            cand = lhs(strcmp(cellfun(@matlab.lang.makeValidName, lhs, 'UniformOutput', false), f{j}));
            cand = unique(cand);
            if numel(cand) ~= 1
                error('run_reference:override', 'Scenario %s: override key "%s" maps to %d model.m variables', ...
                    e.id, f{j}, numel(cand));
            end
            names{end+1} = cand{1}; %#ok<AGROW>
            vals{end+1}  = e.overrides.(f{j}); %#ok<AGROW>
        end
    end
    desc = '';
    if isfield(e, 'description'), desc = e.description; end
    scen(end+1) = struct('id', e.id, 'description', desc, ...
        'overrideNames', {names}, 'overrideValues', {vals}); %#ok<AGROW>
end
end

function lhs = assignment_lhs(txt)
tok = regexp(txt, '^\s*([A-Za-z]\w*(?:\.\w+)*)\s*=[^=]', 'tokens', 'lineanchors');
lhs = cellfun(@(c) c{1}, tok, 'UniformOutput', false);
end

function txt = strip_clear_line(txt)
pat = '^[ \t]*clear;[ \t]*clc;[ \t]*close all;[ \t]*$';
n = numel(regexp(txt, pat, 'lineanchors'));
if n ~= 1
    error('run_reference:strip', 'Expected exactly one "clear; clc; close all;" line, found %d', n);
end
txt = regexprep(txt, pat, '% (clear; clc; close all; removed by run_reference.m)', 'lineanchors');
end

function txt = apply_override(txt, name, value)
pat = ['^([ \t]*' regexptranslate('escape', name) '[ \t]*=[ \t]*)([^;\n%]*?)([ \t]*;[^\n]*)$'];
ext = regexp(txt, pat, 'tokenExtents', 'lineanchors');
if numel(ext) ~= 1
    error('run_reference:override', 'Override "%s" matched %d assignment lines (need exactly 1)', name, numel(ext));
end
valStr = matlab_literal(value);
e = ext{1};
txt = [txt(1:e(2,1)-1) valStr txt(e(2,2)+1:end)];
fprintf('    override %s = %s\n', name, valStr);
end

function s = matlab_literal(v)
if ischar(v) || isstring(v)
    s = ['''' strrep(char(v), '''', '''''') ''''];
elseif isscalar(v)
    s = sprintf('%.17g', v);
else
    s = ['[' strjoin(arrayfun(@(x) sprintf('%.17g', x), v(:).', 'UniformOutput', false), ', ') ']'];
end
end

% -------------------------------------------------------------------------
function export_scenario(ws, outDir, id, modelSha, s)
car = ws.car;
nG = numel(car.gear_ratios);
for g = 1:nG
    M = [car.rpm_axis(:), ws.V_gear_kmh(g,:).', car.T_engine(:), ws.T_wheel_Nm(g,:).', ...
         ws.F_tractive_N(g,:).', ws.a_x_ms2(g,:).', ws.F_tractive_effective(g,:).', ws.a_x_real(g,:).'];
    write_csv(fullfile(outDir, sprintf('tables_gear%d.csv', g)), ...
        {'rpm','v_kmh','t_engine_nm','t_wheel_nm','f_tractive_n','a_x_ms2','f_tractive_eff_n','a_x_real'}, M);
end
write_csv(fullfile(outDir, 'resist.csv'), ...
    {'v_kmh','f_drag_n','f_rolling_n','f_pend_n','f_resistive_n','p_demanded_kw','a_x_envelope'}, ...
    [ws.v_kmh(:), ws.F_drag(:), ws.F_rolling(:), ws.F_pend(:), ws.F_resistive(:), ...
     ws.P_demanded_kW(:), ws.a_x_envelope(:)]);
write_csv(fullfile(outDir, 'r1_history.csv'), {'t_s','d_m','v_kmh'}, ...
    [ws.history_t_r1(:), ws.history_d_r1(:), ws.history_v_r1(:)]);
write_csv(fullfile(outDir, 'r2_history.csv'), {'t_s','d_m','v_kmh'}, ...
    [ws.history_t_r2(:), ws.history_d_r2(:), ws.history_v_r2(:)]);

% Extra MATLAB-only helper files for the C# PCHIP unit test.
write_csv(fullfile(outDir, 'engine_torque.csv'), {'rpm','t_engine_nm'}, ...
    [car.rpm_axis(:), car.T_engine(:)]);
rpmFine = unique([1000:7:6500, 6500]);
write_csv(fullfile(outDir, 'pchip_probe.csv'), {'rpm','t_engine_nm'}, ...
    [rpmFine(:), reshape(interp1(ws.rpm_raw, ws.torque_raw, rpmFine, 'pchip'), [], 1)]);

keys = {'F_x_max','v_curva_adherencia_kmh','optimal_gear_curva','v_max_gear_motriz_kmh', ...
    'v_curva_final_kmh','d_frenado_r1','t_frenado_r1','d_acc_efectiva_r1','t_acc_r1', ...
    'v_max_recta1','t_recta1_total','d_frenado_r2','t_frenado_r2','d_acc_efectiva_r2', ...
    't_acc_r2','v_max_recta2','t_recta2_total','t_curva1_total','t_curva2_total', ...
    't_vuelta_completa','distancia_lap'};
lines = {sprintf('  "scenario_id": "%s"', id)};
for k = 1:numel(keys)
    if strcmp(keys{k}, 'F_x_max'), v = car.F_x_max; else, v = ws.(keys{k}); end
    lines{end+1} = sprintf('  "%s": %s', keys{k}, json_num(v)); %#ok<AGROW>
end
ovr = cell(1, numel(s.overrideNames));
for j = 1:numel(s.overrideNames)
    ovr{j} = sprintf('%s=%s', s.overrideNames{j}, matlab_literal(s.overrideValues{j}));
end
lines{end+1} = sprintf('  "overrides": "%s"', strrep(strjoin(ovr, ';'), '"', '\"'));
lines{end+1} = sprintf('  "model_sha256": "%s"', modelSha);
write_text(fullfile(outDir, 'summary.json'), sprintf('{\n%s\n}\n', strjoin(lines, sprintf(',\n'))));
end

function s = json_num(v)
if ~isscalar(v) || ~isfinite(v)
    error('run_reference:json', 'Non-finite or non-scalar summary value');
end
s = sprintf('%.17g', v);
end

function write_csv(path, header, M)
fid = fopen(path, 'w');
if fid < 0, error('run_reference:io', 'Cannot write %s', path); end
c = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', strjoin(header, ','));
fmt = [strjoin(repmat({'%.17g'}, 1, size(M, 2)), ',') '\n'];
fprintf(fid, fmt, M.');
end

function write_text(path, txt)
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0, error('run_reference:io', 'Cannot write %s', path); end
c = onCleanup(@() fclose(fid));
fwrite(fid, unicode2native(txt, 'UTF-8'));
end

function h = sha256_file(path)
fid = fopen(path, 'r');
bytes = fread(fid, Inf, '*uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
d = typecast(md.digest(bytes), 'uint8');
h = lower(reshape(dec2hex(d, 2).', 1, []));
end
