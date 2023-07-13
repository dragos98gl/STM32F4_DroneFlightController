g=9.8; 
m=0.375;

Kmass=0.001;
Tmass=0.12;

Ts=1/5;

A=[0 1;0 0];
B=[0;1/m];
C=[1 0]; %m -> cm
D=[0];

[Am Bm Cm Dm]=tf2ss(Kmass,[Tmass 1]);
Bm = Bm * 4;

sys1 = ss(Am,Bm,Cm,Dm);
sys2 = ss(A,B,C,D);

sys_s = series(sys1,sys2);

Ad = (eye(length(sys_s.A)) + sys_s.A*Ts);
Bd = sys_s.B*Ts;
Cd = sys_s.C;

Ad_aug = [Ad Bd; zeros(1,length(Ad)) 1];
Bd_aug = [Bd;1];
Cd_aug = [Cd 0];


Q=1000;
R=1;
S=1000;
hz=100;

CQC = Cd_aug'*Q*Cd_aug;
CSC = Cd_aug'*S*Cd_aug;
QC = Q*Cd_aug;
SC = S*Cd_aug;

Qdb = zeros(length(CQC)*hz,length(CQC)*hz);
Rdb = zeros(hz,hz);
Tdb = zeros(hz,length(CQC)*hz);
Cdb = zeros(length(Bd_aug)*hz,hz);
Adc = zeros(hz*length(Ad_aug),length(Ad_aug));

for i=1:hz
    i_begin = (i-1)*length(CQC)+1;
    i_end = i*length(CQC);
    
    if i<hz
        Qdb(i_begin:i_end,i_begin:i_end) = CQC;
        Tdb(i:i,i_begin:i_end) = QC;
    else
        Qdb(i_begin:i_end,i_begin:i_end) = CSC;
        Tdb(i:i,i_begin:i_end) = SC;
    end
    
    Rdb(i,i) = R;
    Adc((i-1)*length(Ad_aug)+1:i*length(Ad_aug),1:length(Ad_aug)) = Ad_aug^i;
    
    for j=1:hz
        if j<=i
            Cdb((i-1)*length(Bd_aug)+1:i*length(Bd_aug),j) = Ad_aug^(i-j)*Bd_aug;
            
            if i == j
                Cdb((i-1)*length(Bd_aug)+1:i*length(Bd_aug),j) = Cdb((i-1)*length(Bd_aug)+1:i*length(Bd_aug),j);
            end
        end
    end
end

Hdb = Cdb'*Qdb*Cdb + Rdb;
Fdbt = [Adc'*Qdb*Cdb;-Tdb*Cdb];

x = [0; 0; 0; 0];
r = ones(1,hz)*0.1;

ft = [x' r]*Fdbt;
du=-inv(Hdb)*ft';

close all

%compMCU(Fdbt,Hdb,hz)
%simSTM32(Ad,Bd,Cd,hz,Hdb,Fdbt,20)
sim1(Ad,Bd,hz,Hdb,Fdbt,x,r)

%dlmwrite('filename.txt',reshape(Fdbt',1,[]),'precision',10)
%dlmwrite('filename.txt',reshape(-inv(Hdb)',1,[]),'precision',10)

function none = compMCU(Fdbt,Hdb,hz)
    mcuBinFile = fopen('C:\Users\Dragos\STM32F4_DroneFlightController\matlab\mcu_export.txt');
    mcuData = fread(mcuBinFile,'float')

    x = [0.0340000018; 0;-2.20458317; -4.72786903];
    r = ones(1,hz)*-0.155669793;
    ft = [x' r]*Fdbt;
    du=-inv(Hdb)*ft';

    plot (du)
    hold on
    plot (mcuData)
end
    

function none = sim1(Ad_aug,Bd_aug,hz,Hdb,Fdbt,x,r)
    t = 2000;

    y_pred = zeros(t,4);
    prev_states = [0;0;0;0];
    currentStates = [0;0;0;0];

    y_pred2 = zeros(t,4);
    
    err = 150;
    
    x1_dt = 0;
    x1 = 0;
    prev_x1 = 0;
    x3 = 0;
    prev_x3 = 0;
    x_aug = 0;
    for i=1:t
        y = compute_mpc(Hdb,Fdbt,currentStates,r);
        x_aug = x_aug + y(1);

        err = err + (20 - currentStates(1))*0.01;

        currentStatesTemp = Ad_aug*prev_states(1:3) + Bd_aug*x_aug;
        currentStates = [currentStatesTemp ; x_aug] - [0;9.8;0;0];
        
        x3 = x_aug*0.8 + prev_x3*-0.666666666666667;
        x1 = currentStates(1);
        x1_dt = x1 - prev_x1;
        prev_x3 = x3;
        prev_x1 = x1;
        
        y_pred2(i,:) = [x1 x1_dt*5 x3 x_aug];
         if currentStates(1)<0
             currentStates(1) = 0;
         end
         if currentStates(2)<0
             currentStates(2) = 0;
         end

        r = ones(1,hz)*err;

        prev_states = currentStates;
        y_pred(i,:) = currentStates';
    end

    plot(y_pred(:,1))
    hold on
    plot(y_pred2(:,1))

end

function y = compute_mpc(Hdb,Fdbt,x,r)
    ft = [x' r]*Fdbt;
    du=-inv(Hdb)*ft';
    
    y = du;
end

function states = simAltModel(Ad,Bd,prev_states,input)
    states = Ad*prev_states + Bd*input;
end

function none = simSTM32(Ad,Bd,Cd,hz,Hdb,Fdbt,ref)
    t = 3;
    x=[0;0;0;0];
    r = ones(1,hz)*0.1;
    
    input_array = zeros(t,4);
    
    err = 0;
    Ki = 0.01;
    
    x1 = 0;
    x1_dt = 0;
    x3 = 0;
    
    prev_x1 = 0;
    prev_x3 = 0;
    prev_states = [0;0;0];
    current_states = [0;0;0];
    
    x_aug = 0;
    
    
    for i = 1:t
        du = compute_mpc(Hdb,Fdbt,x,r);
        du0 = du(1);
        x_aug = x_aug + du0;
        
        err = err + (ref - x1)*Ki;
        
        current_states = Ad*prev_states + Bd*x_aug;
        
        x3 = x_aug*0.8 + prev_x3*-0.666666666666667;
        x1 = current_states(1);
        x1_dt = x1 - prev_x1;
        
        prev_x1 = x1;
        prev_x3 = x3;
        
        r = ones(1,100)*err;
        x = [x1; x1_dt*5; x3; x_aug]
        
        prev_states = x(1:3);
        input_array(i,:) = x';
    end
    
    %plot(input_array)
end