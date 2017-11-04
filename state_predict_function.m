function predict_eq = state_predict_function(LAnklePitch,LHipYaw,LHipPitch,LKneePitch,RAnklePitch,RHipYaw,RHipPitch,RKneePitch,bodyX,bodyY,bodyZ,bodyYaw,bodyRoll,bodyPitch,d_LHipYaw,d_LHipPitch,d_LKneePitch,d_LAnklePitch,d_RHipYaw,d_RHipPitch,d_RKneePitch,d_RAnklePitch,d_bodyX,d_bodyY,d_bodyZ,d_bodyYaw,d_bodyRoll,d_bodyPitch,dd_bodyX,dd_bodyY,dd_bodyZ,dd_LHipYaw,dd_RHipYaw,dd_bodyYaw,dd_bodyRoll,dd_LHipPitch,dd_RHipPitch,dd_bodyPitch,dd_LKneePitch,dd_RKneePitch,dd_LAnklePitch,dd_RAnklePitch)
%STATE_PREDICT_FUNCTION
%    PREDICT_EQ = STATE_PREDICT_FUNCTION(LANKLEPITCH,LHIPYAW,LHIPPITCH,LKNEEPITCH,RANKLEPITCH,RHIPYAW,RHIPPITCH,RKNEEPITCH,BODYX,BODYY,BODYZ,BODYYAW,BODYROLL,BODYPITCH,D_LHIPYAW,D_LHIPPITCH,D_LKNEEPITCH,D_LANKLEPITCH,D_RHIPYAW,D_RHIPPITCH,D_RKNEEPITCH,D_RANKLEPITCH,D_BODYX,D_BODYY,D_BODYZ,D_BODYYAW,D_BODYROLL,D_BODYPITCH,DD_BODYX,DD_BODYY,DD_BODYZ,DD_LHIPYAW,DD_RHIPYAW,DD_BODYYAW,DD_BODYROLL,DD_LHIPPITCH,DD_RHIPPITCH,DD_BODYPITCH,DD_LKNEEPITCH,DD_RKNEEPITCH,DD_LANKLEPITCH,DD_RANKLEPITCH)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    02-Nov-2017 22:08:45

predict_eq = [bodyX.*(1.01e2./1.0e2);bodyY.*(1.01e2./1.0e2);bodyZ.*(1.01e2./1.0e2);bodyRoll+d_bodyRoll.*(1.0./1.0e2);bodyPitch+d_bodyPitch.*(1.0./1.0e2);bodyYaw+d_bodyYaw.*(1.0./1.0e2);LHipPitch+d_LHipPitch.*(1.0./1.0e2);LHipYaw+d_LHipYaw.*(1.0./1.0e2);LKneePitch+d_LKneePitch.*(1.0./1.0e2);LAnklePitch+d_LAnklePitch.*(1.0./1.0e2);RHipPitch+d_RHipPitch.*(1.0./1.0e2);RHipYaw+d_RHipYaw.*(1.0./1.0e2);RKneePitch+d_RKneePitch.*(1.0./1.0e2);RAnklePitch+d_RAnklePitch.*(1.0./1.0e2);d_bodyX+dd_bodyX.*(1.0./1.0e2);d_bodyY+dd_bodyY.*(1.0./1.0e2);d_bodyZ+dd_bodyZ.*(1.0./1.0e2);d_bodyRoll+dd_bodyRoll.*(1.0./1.0e2);d_bodyPitch+dd_bodyPitch.*(1.0./1.0e2);d_bodyYaw+dd_bodyYaw.*(1.0./1.0e2);d_LHipPitch+dd_LHipPitch.*(1.0./1.0e2);d_LHipYaw+dd_LHipYaw.*(1.0./1.0e2);d_LKneePitch+dd_LKneePitch.*(1.0./1.0e2);d_LAnklePitch+dd_LAnklePitch.*(1.0./1.0e2);d_RHipPitch+dd_RHipPitch.*(1.0./1.0e2);d_RHipYaw+dd_RHipYaw.*(1.0./1.0e2);d_RKneePitch+dd_RKneePitch.*(1.0./1.0e2);d_RAnklePitch+dd_RAnklePitch.*(1.0./1.0e2);dd_bodyX;dd_bodyY;dd_bodyZ;dd_bodyRoll;dd_bodyPitch;dd_bodyYaw;dd_LHipPitch;dd_LHipYaw;dd_LKneePitch;dd_LAnklePitch;dd_RHipPitch;dd_RHipYaw;dd_RKneePitch;dd_RAnklePitch];
