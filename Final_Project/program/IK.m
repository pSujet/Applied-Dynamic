function [theta1,theta2,theta3] = IK(x,y,z,params)
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;
r1 = sqrt(x.^2+y.^2);
r2 = z-l3;
r3 = sqrt(r1.^2+r2.^2);
theta3 = atan2(y,x);
theta3 = unwrap(theta3);
theta1 = (atan2(r2,r1)-acos((l2^2-l1^2-r3.^2)./(-2*l1.*r3)));
theta1 = unwrap(theta1);
theta2 = pi-acos((r3.^2-l1^2-l2^2)/(-2*l1*l2));
theta2 = unwrap(theta2);
end