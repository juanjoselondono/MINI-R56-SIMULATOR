% Random colon-operator ranges a:d:b evaluated by MATLAB, every element written with %.17g.
% Output: ml_colon_random.txt, one range per line: a d b n v(1) ... v(n)
rng(42, 'twister');
fid = fopen('ml_colon_random.txt', 'w');
steps = [0.1, -0.05, 0.05, 1, 0.3, -0.1, 7, 0.01];
for i = 1:1200
    d = steps(mod(i-1, numel(steps)) + 1);
    span = (5 + 75*rand) * abs(d) * (1 + 0.37*rand);   % 5..~100 elements
    a = round(100*rand * 1e4) / 1e4;
    if mod(i, 3) == 0, a = 100*rand; end              % arbitrary double start
    if mod(i, 5) == 0, b = a + sign(d)*round(span/abs(d))*abs(d);   % endpoint (nearly) hit
    else, b = a + sign(d)*span; end
    v = a:d:b;
    fprintf(fid, '%.17g %.17g %.17g %d', a, d, b, numel(v));
    fprintf(fid, ' %.17g', v);
    fprintf(fid, '\n');
end
fclose(fid);
