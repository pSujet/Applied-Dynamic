function [x,y,z] = FK(theta1,theta2,theta3,params)
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;
x = (l1*cos(theta1)+l2.*cos(theta1+theta2)).*cos(theta3);
y = (l1*cos(theta1)+l2.*cos(theta1+theta2)).*sin(theta3);
z = l3+l1.*sin(theta1)+l2.*sin(theta1+theta2);
end