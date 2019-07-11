% syms a1 a2 s1 c1 s12 c12 l1 l2 kr1 kr2
% %link
% JP_l1 = [-l1*s1 0;l2*c1 0;0 0];
% JP_l2 = [-a1*s1-l2*s12 -l2*s12;a1*c1+l2*c12 l2*c12];
% JO_l1 = [0 0;0 0;1 0];
% JO_l2 = [0 0;0 0;1 1];
% %motor
% JP_m1 = [0 0;0 0;0 0];
% JP_m2 = [-a1*s1 0;a1*c1 0;0 0];
% JO_m1 = [0 0;0 0;kr1 0];
% JO_m2 = [0 0;0 0;1 kr2];
% %Cal
B = 5