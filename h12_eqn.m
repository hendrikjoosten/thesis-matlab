function h12_eqn = h12_eqn(RHipPitch,RKneePitch)
%H12_EQN
%    H12_EQN = H12_EQN(RHIPPITCH,RKNEEPITCH)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    07-Nov-2017 23:37:39

t2 = conj(RHipPitch);
t3 = cos(t2);
t4 = conj(RKneePitch);
t5 = cos(t4);
t6 = t3.*(2.0./5.0);
t7 = t5.*(1.0./5.0);
t8 = t6+t7+1.0./1.0e1;
t9 = 1.0./t8;
h12_eqn = [t9.*(t3.*2.587302772907413e2+t5.*1.293651386453706e2-6.15308728319153);t9.*(t3.*1.471180898652705e2+t5.*7.355904493263524e1-sin(t2).*2.841287376523865e2-sin(t4).*1.420643688261932e2+2.967630402500796e1)];
