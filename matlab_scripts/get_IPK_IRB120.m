function q = get_IPK_IRB120(x,y,z,Ax,Ay,Az)
T07 = Transformation_Wrist(x,y,z,Ax,Ay,Az);
q = IPK_IRB120(T07);
end