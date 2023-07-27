close all

Ts = 0.2;

t = 0:Ts:length(VarName1)*Ts-Ts;
mass = VarName2.*9.8/1000;
voltage = VarName1/10000;

hold on
%plot (t,VarName1)
plot (t,mass)
%plot (t,VarName3)

p1_a = 1;
p1_b = 149;

p2_a = 153;
p2_b = 295;

p3_a = 303;
p3_b = 447;

p4_a = 451;
p4_b = 593;

p5_a = 601;
p5_b = 743;

p6_a = 750;
p6_b = 894;

points = [[p1_a p1_b]; [p2_a p2_b]; [p3_a p3_b]; [p4_a p4_b]; [p5_a p5_b]; [p6_a p6_b]];
regRes = zeros(length(points),3);
for i=1:length(points)
    x1 = points(i,1);
    x2 = points(i,2);
    
    X=[voltage(x1:x2) VarName3(x1:x2)];
    params = fitglm(X,mass(x1:x2),'linear');
    regRes(i,:) = params.Coefficients.Estimate';

    Ydec = zeros (1,length(voltage));
    for j=x1:x2
       Ydec(j) = regRes(i,1) + voltage(j)*regRes(i,2) + VarName3(j)*regRes(i,3);
    end
    
    plot(t,Ydec);
end

X=[voltage VarName3];
params = fitglm(X,mass,'linear');
regRess = params.Coefficients.Estimate';

Ydec = zeros (1,length(voltage));
for j=1:length(voltage)
   Ydec(j) = regRess(1) + voltage(j)*regRess(2) + VarName3(j)*regRess(3);
end
    
figure
plot(t,Ydec);
hold on
plot (t,mass)

sys = ss (ss16.A,ss16.B,ss16.C,ss16.D);
close all

Yp=lsim(sys,[[voltage;voltage] [VarName3;VarName3]]',[t t+178.8000],[1 1 1]');
figure
plot([t t+178.8000],Yp);
hold on
plot([t t+178.8000],[mass ;mass])
