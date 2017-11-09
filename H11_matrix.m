function H11_matrix_eq = H11_matrix(LAnklePitch,LHipPitch,LKneePitch)
%H11_MATRIX
%    H11_MATRIX_EQ = H11_MATRIX(LANKLEPITCH,LHIPPITCH,LKNEEPITCH)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    08-Nov-2017 23:58:19

t2 = conj(LHipPitch);
t3 = cos(t2);
t4 = conj(LAnklePitch);
t5 = sin(t2);
t6 = conj(LKneePitch);
t7 = cos(t4);
t8 = t3.*3.999932400016125e-1;
t9 = cos(t6);
t10 = t9.*3.999932400016125e-1;
t11 = sin(t4);
t12 = t11.*1.999966200008062e-1;
t13 = t5.*6.553546186543979e-4;
t14 = sin(t6);
t15 = t14.*6.553546186543979e-4;
t17 = t7.*3.27677309327199e-4;
t16 = t8+t10+t12+t13+t15-t17+4.617962917440799e-1;
t18 = 1.0./t16;
t19 = 1.0./t16.^2;
t20 = t7.*2.151417339578776;
t21 = t3.*2.563374024964379e2;
t22 = t9.*2.563374024964379e2;
t23 = t11.*1.28168701248219e2;
t25 = t5.*4.302834679157552;
t26 = t14.*4.302834679157552;
t24 = t20+t21+t22+t23-t25-t26-3.007350315754342e4;
t27 = t3.*6.553546186543979e-4;
t28 = t5.*3.999932400016125e-1;
t29 = t27-t28;
t30 = t9.*6.553546186543979e-4;
t31 = t14.*3.999932400016125e-1;
t32 = t30-t31;
t33 = t7.*1.428209142567586e2;
t34 = t3.*1.428028299746757e2;
t35 = t9.*1.428028299746757e2;
t36 = t11.*7.140141498733786e1;
t41 = t5.*2.856418285135171e2;
t42 = t14.*2.856418285135171e2;
t37 = t33+t34+t35+t36-t41-t42+5.325369876374279e2;
t38 = t7.*1.999966200008062e-1;
t39 = t11.*3.27677309327199e-4;
t40 = t38+t39;
H11_matrix_eq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t18.*(t3.*4.302834679157552+t5.*2.563374024964379e2)-t19.*t24.*t29,-t18.*(t3.*2.856418285135171e2+t5.*1.428028299746757e2)-t19.*t29.*t37,0.0,0.0,-t18.*(t9.*4.302834679157552+t14.*2.563374024964379e2)-t19.*t24.*t32,-t18.*(t9.*2.856418285135171e2+t14.*1.428028299746757e2)-t19.*t32.*t37,t18.*(t7.*1.28168701248219e2-t11.*2.151417339578776)-t19.*t24.*t40,t18.*(t7.*7.140141498733786e1-t11.*1.428209142567586e2)-t19.*t37.*t40,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[2, 42]);
