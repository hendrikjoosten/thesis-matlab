function H14_matrix_eq = H14_matrix(RHipPitch,RKneePitch)
%H14_MATRIX
%    H14_MATRIX_EQ = H14_MATRIX(RHIPPITCH,RKNEEPITCH)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    08-Nov-2017 16:04:35

t2 = conj(RHipPitch);
t3 = sin(t2);
t4 = cos(t2);
t5 = t4.*(2.0./5.0);
t6 = conj(RKneePitch);
t7 = cos(t6);
t8 = t7.*(8.3e1./2.0e2);
t9 = t5+t8+1.0./1.0e1;
t10 = 1.0./t9;
t11 = sin(t6);
t12 = 1.0./t9.^2;
t13 = t4.*2.587302772907413e2;
t14 = t7.*2.684326626891441e2;
t15 = t13+t14-6.15308728319153;
t16 = t4.*1.471180898652705e2;
t17 = t7.*1.526350182352181e2;
t18 = t3.*(-2.841287376523865e2)-t11.*2.94783565314351e2+t16+t17+2.967630402500796e1;
H14_matrix_eq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t3.*t10.*(-2.587302772907413e2)+t3.*t12.*t15.*(2.0./5.0),-t10.*(t3.*1.471180898652705e2+t4.*2.841287376523865e2)+t3.*t12.*t18.*(2.0./5.0),0.0,0.0,t10.*t11.*(-2.684326626891441e2)+t11.*t12.*t15.*(8.3e1./2.0e2),-t10.*(t7.*2.94783565314351e2+t11.*1.526350182352181e2)+t11.*t12.*t18.*(8.3e1./2.0e2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[2, 42]);
