function H19_matrix_eq = H19_matrix(RHipYaw,RHipPitch,RKneePitch)
%H19_MATRIX
%    H19_MATRIX_EQ = H19_MATRIX(RHIPYAW,RHIPPITCH,RKNEEPITCH)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    01-Nov-2017 22:05:21

t2 = cos(RHipPitch);
t3 = sin(RHipYaw);
t4 = cos(RHipYaw);
t5 = sin(RHipPitch);
t6 = cos(RKneePitch);
H19_matrix_eq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2.*t4.*2.772533284635101e5,t2.*t3.*2.780138479353805e5,t5.*-4.0e2+t2.*t3.*1.259458156345963e5+t2.*t4.*2.607126706737241e5,0.0,t3.*t5.*(-2.772533284635101e5),t4.*t5.*2.780138479353805e5,t3.*t5.*(-2.607126706737241e5)+t4.*t5.*1.259458156345963e5,0.0,t6.*2.876503282808917e5,0.0,t6.*2.704893958239888e5-sin(RKneePitch).*4.15e2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[4, 42]);