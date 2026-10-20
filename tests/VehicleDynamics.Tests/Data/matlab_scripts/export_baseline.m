% Generates MATLAB baseline reference data for the C# unit tests (FILE_CONTRACT.md format).
runmodel;
od = 'matlab_S01'; if ~exist(od,'dir'), mkdir(od); end
for g = 1:6
  M = [car.rpm_axis; V_gear_kmh(g,:); car.T_engine; T_wheel_Nm(g,:); F_tractive_N(g,:); a_x_ms2(g,:); F_tractive_effective(g,:); a_x_real(g,:)]';
  w(fullfile(od,sprintf('tables_gear%d.csv',g)), 'rpm,v_kmh,t_engine_nm,t_wheel_nm,f_tractive_n,a_x_ms2,f_tractive_eff_n,a_x_real', M);
end
w(fullfile(od,'resist.csv'),'v_kmh,f_drag_n,f_rolling_n,f_pend_n,f_resistive_n,p_demanded_kw,a_x_envelope',[v_kmh; F_drag; F_rolling; F_pend; F_resistive; P_demanded_kW; a_x_envelope]');
w(fullfile(od,'r1_history.csv'),'t_s,d_m,v_kmh',[history_t_r1; history_d_r1; history_v_r1]');
w(fullfile(od,'r2_history.csv'),'t_s,d_m,v_kmh',[history_t_r2; history_d_r2; history_v_r2]');
names = {'F_x_max','v_curva_adherencia_kmh','optimal_gear_curva','v_max_gear_motriz_kmh','v_curva_final_kmh','d_frenado_r1','t_frenado_r1','d_acc_efectiva_r1','t_acc_r1','v_max_recta1','t_recta1_total','d_frenado_r2','t_frenado_r2','d_acc_efectiva_r2','t_acc_r2','v_max_recta2','t_recta2_total','t_curva1_total','t_curva2_total','t_vuelta_completa','distancia_lap'};
fid = fopen(fullfile(od,'summary.json'),'w'); fprintf(fid,'{\n  "scenario_id": "S01"');
for i=1:numel(names)
  if strcmp(names{i},'F_x_max'), val = car.F_x_max; else, val = eval(names{i}); end
  fprintf(fid,',\n  "%s": %.17g', names{i}, val);
end
fprintf(fid,'\n}\n'); fclose(fid);
function w(fn, hdr, M)
  fid = fopen(fn,'w'); fprintf(fid,'%s\n',hdr);
  fmt = [repmat('%.17g,',1,size(M,2)-1) '%.17g\n'];
  fprintf(fid, fmt, M'); fclose(fid);
end
