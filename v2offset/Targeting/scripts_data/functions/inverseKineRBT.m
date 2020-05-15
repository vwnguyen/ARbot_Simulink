function [th] = inverseKineRBT(px,py,pz,phi)
% Solves IK problem and returns joint angle vector
% th=[th1 th2 th3 th4]
% Link Lengths

% ARBOT V1
%L1=2;L2=5;L3=4;L4=3;

% ARBOT V2
%L1=0.2271;
%L2=0.39807;
%L3=0.3792;
%L4=0.264972;

% pre allocate 
th = zeros(1,4);

% ARBOT V2_Beta
% L1=.227;
% L2=.317;
% L3=.343;
% L4=.150;
%pz=-pz+L3;

% ARBOT V2_offset
L1=.224;
L2=.317;
L3=.343;
L4=.150;

th1=atan2(py,px);
A=px-L4*cos(th1)*cos(phi);
B=py-L4*sin(th1)*cos(phi);
C=pz-L1-L4*sin(phi);
th3=real(acos((A^2 + B^2 + C^2 - L2^2 - L3^2)/(2*L2*L3)));

% restricting domain of th3 for simulink to not be confused by complex
% values of cos(th3)

% A = max(min(A,1),-1);
% B = max(min(B,1),-1);
% C = max(min(C,1),-1);
% th3 = max(min(th3,1),-1);

a=L3*sin(th3);
b=L2+L3*cos(th3);
c=pz-L1-L4*sin(phi);
r=sqrt(a^2 + b^2);

% a = max(min(a,1),-1);
% b = max(min(b,1),-1);
% c = max(min(c,1),-1);

th2= ( atan2(c,real(sqrt(r^2-c^2)))-atan2(a,b) );

th4=phi-th2-th3;
th=[th1 th2 th3 th4];
%{
gamma=atan2(pz,px);
xq=px-L4*cos(phi);
yq=py;
zq=pz--L4*sin(phi)
%}
end

