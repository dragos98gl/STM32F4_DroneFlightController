t1 = VarName1 - VarName1(1);
t2 = VarName3 - VarName3(1);

t=0:length(t1)-1;
t=t*0.0069;

y1 = VarName2;
y2 = VarName4;

scale_0a = 7174;
scale_0b = 7219;
zero_mean = mean(y2(scale_0a:scale_0b));
y2 = y2 - zero_mean-3.1820;

plot (t,y1/1000)
hold on
plot (t2,(y2/1000*9.8))

kMASS=0.001;
TMASS=0.12;
hMASS=tf(kMASS,[TMASS 1])

Y=lsim(hMASS,y1,t);
plot(t,Y*4);