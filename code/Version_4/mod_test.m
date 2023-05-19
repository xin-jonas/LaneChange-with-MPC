% for showing the problem of function 'mod'
close all
clc
clear all
temp = 0;
t = zeros(1,125);
x = zeros(1,125);
x_new = zeros(1,125);
for i = 1:125
    t(1,i) = temp;
    x(1,i) = mod(temp,10*0.2);
    x_new(1,i) = mod(int32(temp/0.2),10);
    temp = temp + 0.2;
    if i  == 11
        disp(t(1,i)/0.2);
        disp(x(1,i));

        disp(mod(t(1,i),5*0.2)); % i have used this way, the calculate result is wrong
        disp(mod(t(1,i)/0.2,5)); % this way is also wrong
        disp(mod(int32(t(1,i)/0.2),5)); % this way is correct
    end
end