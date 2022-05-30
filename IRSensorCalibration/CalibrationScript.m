%close all;
T = readtable('Thermopile Characteristic Table(ZTP-148SR).xlsx');
T_amb = table2array(T(:,1));
T_amb = T_amb(2:end);
T_obj = table2array(T(1,:));
T_obj = T_obj(6:end);
V = table2array(T(2:end, 6:end));
[T_OBJ,T_AMB] = meshgrid(T_obj,T_amb);
figure()
h1 = surf(T_OBJ,T_AMB, V);
xlabel('Object Temperature (C)')
ylabel('Ambient Temperature (C)')
zlabel('Voltage output (mV)')
title('Calibration Data')
%set(h1,'LineStyle','none');
colorbar


T_amb_model = [0:2:100]; %Celsius units
T_amb_model = [T_amb_model(1:13) 25 T_amb_model(14:end)]; %Insert 25C
T_obj_model = [0:5:100]; %Celsius units
[T_OBJ_MODEL, T_AMB_MODEL] = meshgrid(T_obj_model, T_amb_model); %Create meshgrid for 3D mapping
SE = 1e-2; %Seebeck coeff * emissivity factor = 1e-2
x = 2.50; %Calibration factor = 2.50
offset = 0; %Calibration factor
V_model = SE*((T_OBJ_MODEL.^(4-x))-(T_AMB_MODEL.^(4-x)))+offset;
figure()
h2 = surf(T_OBJ_MODEL, T_AMB_MODEL, V_model);
surf(T_OBJ,T_AMB, V);
xlabel('Object Temperature (C)')
ylabel('Ambient Temperature (C)')
zlabel('Voltage output (mV)')
title('Theoretical Model')
%set(h2,'LineStyle','none');
colorbar

figure()
surf(T_OBJ,T_AMB,V)
hold on
surf(T_OBJ_MODEL,T_AMB_MODEL,V_model)
hold off
grid on
xlabel('Object Temperature (C)')
ylabel('Ambient Temperature (C)')
zlabel('Voltage output (mV)')
title('Overlaid Models')
colorbar

min_value = abs(mean(mean(V_model-V)))
% SE_min = 0;
% x_min = 0;

% for x = -4:0.1:4
%     for SE = 1e-6:1e-3:1
%         V_model = SE*((T_OBJ_MODEL.^(4-x))-(T_AMB_MODEL.^(4-x)))+offset;
%         value = abs(mean(mean(V_model-V)));
%         if(value < min_value)
%             SE_min = SE;
%             x_min = x;
%         end
%     end
% end
% 
% disp(SE_min)
% disp(x_min)