clear all
%   MATLAB Animation Program for Falling Box
%===Define the vertices of the box
a=0.1;
b=0.2;
x=[0 a a 0 0];
y=[0 0 b b 0];
%===Define a matrix whose column vectors are the box vertices
r=[x; y];
%===Draw the box in the initial position
figure(1), clf
axis([-0.3 0.3 -0.3 0.3])
line(x, y,'linestyle','--');
grid on
%===Define parameters 
m=2;   
g=9.81;
C=0.05;
L=0.5*sqrt(a^2+b^2);
I=m*(a^2+b^2)/12;
e=m*g*L/I;
c=C/I;
%===Define initial conditions
theta = 0;
omega = -100;
phi_0 = atan(a/b);
%===steps
dt = 0.00001;		% time step for simulation
n = 500;			% # of frames for animation 
M=moviein(n);	% define a matrix M for movie in
%========= Finish data input ============================
%===Integrate the equations of motion
for j = 1:n;	% Do loop for getting new box graphic

for ii =1:500;	% Do loop for elapsed time integration
    omega = omega+dt*e*sin(theta+phi_0)-dt*c*omega;
    theta=theta+dt*omega;
end
%===Rotate box graphic using finite rotation matrix
A=[cos(theta) sin(theta); -sin(theta) cos(theta)];
r1=A*r;
x1=r1(1,:);
y1=r1(2,:);
patch(x1,y1, 'g');
axis('square')
M(:,j)=getframe;
end
%===Show movie
% figure(2), clf
% axis('square')
% axis([-0.3 0.3 -0.3 0.3])
% movie(M,1,2);
