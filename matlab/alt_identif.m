%plot(VarName2+VarName3+VarName4+VarName5-3000*4)
%hold on
%plot(VarName9)

g=9.8;
m=0.375;

Kmass=0.0009;
Tmass=0.1;

Ts=1/5;

A=[0 1;0 0];
B=[0;1/m];
C=[1 0]; %m -> cm
D=[0];

[Am Bm Cm Dm]=tf2ss(Kmass,[Tmass 1]);
Bm = Bm;

sys1 = ss(Am,Bm,Cm,Dm);
sys2 = ss(A,B,C,D);

sys_s = series(sys1,sys2);

Ad = (eye(length(sys_s.A)) + sys_s.A*Ts);
Bd = sys_s.B*Ts;
Cd = sys_s.C;


t = length(VarName1);
prev_states = [VarName6(1);0.1;(VarName2(1)+VarName3(1)+VarName4(1)+VarName5(1)-3000*4)*Tmass];
current_states = prev_states;
out = zeros(t,3);

for i = 1:t
    out(i,:) = current_states';
    
    in = VarName2(i)+VarName3(i)+VarName4(i)+VarName5(i)-3000*4-10;
    current_states = Ad*prev_states + Bd*in - [0;9.8;0]*Ts;
    prev_states = current_states;
end

plot(out(:,1:2))
hold on
plot(VarName7)
hold on
plot(VarName6)
hold on
plot((VarName2+VarName3+VarName4+VarName5-3000*4-4000-40)/100)