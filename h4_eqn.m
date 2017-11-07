function h4_eqn = h4_eqn(RHipPitch)
%H4_EQN
%    H4_EQN = H4_EQN(RHIPPITCH)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    07-Nov-2017 23:37:35

t2 = conj(RHipPitch);
t3 = cos(t2);
t4 = t3.*(2.0./5.0);
t5 = t4+1.0./1.0e1;
t6 = 1.0./t5;
h4_eqn = [t6.*(t3.*2.587302772907413e2-6.15308728319153);t6.*(t3.*1.471180898652705e2-sin(t2).*2.841287376523865e2+2.967630402500796e1)];
