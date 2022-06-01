close all;
T = readtable('ThermistorCalibration.xlsx');
T_amb = table2array(T(:,3));
T_amb = T_amb(1:end);
voltage = table2array(T(:,2));
voltage = voltage(1:end);
f = fit(voltage,T_amb,"rat22")
plot(f,voltage, T_amb)
xlabel("Voltage Reading (mV)")
ylabel("Ambient Temperature (C)")
