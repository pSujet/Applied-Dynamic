%theta d a alpha
%syms a1 a2 a3 a4 d1 d3 q1 q2
l0 = 0;
l1 = 1;
l2 = 1;
d2 = 0;
d3 = 0;
t0 = 0;
t1 = 0;
t2 = 0;

L(1) = Link([t0 l0 0 pi/2]) 
L(2) = Link([t1 0 l1 0]) 
L(3) = Link([t2 0 l2 0])


R = SerialLink(L)
R.plot([0 0 0]) %q1 q2 q3 q4
R.teach
