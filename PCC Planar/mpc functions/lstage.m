function l = lstage(in1,in2)
%LSTAGE
%    L = LSTAGE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    28-Nov-2021 12:58:56

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
l = u1.^2./1.0e+2+u2.^2./1.0e+2+u3.^2./1.0e+2+x1.^2.*1.0e+1+x2.^2.*1.0e+1+x3.^2.*1.0e+1+x4.^2./1.0e+1+x5.^2./1.0e+1+x6.^2./1.0e+1;
