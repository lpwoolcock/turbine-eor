% these parameters are required for the generator to respond correctly
% to torque commands. don't really understand why

GenEff   =  100.0;          % - Generator efficiency [ignored by the Thevenin and user-defined generator models] (%)
GBRatio  =   22.5;          % - Gearbox ratio (-)
SIG_SlPc =    1.5125;       % - Rated generator slip percentage [>0] (%)              Now HSS side!
SIG_SySp = 2400.0;          % - Synchronous (zero-torque) generator speed [>0] (rpm)  Now HSS side!
SIG_RtTq = 1367.9;          % - Rated torque [>0] (N-m)                               Now HSS side!
SIG_PORt =    2.0;          % - Pull-out ratio (Tpullout/Trated) [>1] (-)% 
SIG_SySp = SIG_SySp*pi/30;  % convert to rad/s
SIG_RtSp = SIG_SySp*(1.0+0.01*SIG_SlPc);
SIG_POS1=SIG_PORt*(SIG_RtSp-SIG_SySp);
SIG_POTq=SIG_RtTq*SIG_PORt;
SIG_Slop=SIG_RtTq/(SIG_RtSp - SIG_SySp);
