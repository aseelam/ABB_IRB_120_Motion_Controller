function coeffs = coeffs_for_straight_line(ps,pe,ts,te) 
global d0 d1 d2 d3
syms d0 d1 d2 d3

eq1 = cubic_eq(ts);
eq2 = cubic_eq(te);
eq3 = diff_cubic_eq(ts);
eq4 = diff_cubic_eq(te);

A = [vpa(diff(eq1,d0)),vpa(diff(eq1,d1)),vpa(diff(eq1,d2)),vpa(diff(eq1,d3));
    vpa(diff(eq2,d0)),vpa(diff(eq2,d1)),vpa(diff(eq2,d2)),vpa(diff(eq2,d3));
    vpa(diff(eq3,d0)),vpa(diff(eq3,d1)),vpa(diff(eq3,d2)),vpa(diff(eq3,d3));
    vpa(diff(eq4,d0)),vpa(diff(eq4,d1)),vpa(diff(eq4,d2)),vpa(diff(eq4,d3))];
B = [ps;pe;0;0];

coeffs = linsolve(A,B);

function eq = cubic_eq(t)
eq = d0 + d1*t + d2*(t^2) + d3*(t^3) ;
end
function eq = diff_cubic_eq(t)
eq = d1 + 2*d2*t + 3*d3*(t^2) ;
end
end

% To force a straight line, we pick three points in between the start and
% end point. They are p_mid(midpoint of ps and pe), p_s_mid(midpoint of ps
% and p_mid) and p_mid_e(midpoint of p_mid and pe).

% p_mid = (ps+pe)/2;
% t_mid = (ts+te)/2;
% p_s_mid = (ps+p_mid)/2;
% t_s_mid = (ts+t_mid)/2;
% p_mid_e = (p_mid+pe)/2;
% t_mid_e = (t_mid+te)/2;
% 
% eq2 = cubic_eq(t_s_mid);
% eq3 = cubic_eq(t_mid);
% eq4 = cubic_eq(t_mid_e);
% 
% vpa(diff(eq2,d0)),vpa(diff(eq2,d1)),vpa(diff(eq2,d2)),vpa(diff(eq2,d3));
% vpa(diff(eq3,d0)),vpa(diff(eq3,d1)),vpa(diff(eq3,d2)),vpa(diff(eq3,d3));
% vpa(diff(eq4,d0)),vpa(diff(eq4,d1)),vpa(diff(eq4,d2)),vpa(diff(eq4,d3));
% 
% p_s_mid;
% p_mid;
% p_mid_e;