pos = zeros(1,length(VarName1));
for i=11:length(VarName1)
    cpi = (VarName2(i)/1000/11.914)*2.54;
    pos(i) = VarName1(i)*cpi - VarName3(i-2)*cpi*10;
end

plot(pos)