syms t;
theta = 15+20*t-28.8*t^2+4.24*t^3;
t=[0:0.01:5];
plot(t,subs(theta,t))
xlabel('Time (sec)')
ylabel('Joint position (deg)')
grid on

syms t;
theta = 20 - 57.6*t + 12.72*t^2;
t=[0:0.01:5];
plot(t,subs(theta,t))
xlabel('Time (sec)')
ylabel('Joint velocity (deg/s)')
grid on

syms t;
theta = -57.6 + 25.44*t;
t=[0:0.01:5];
plot(t,subs(theta,t))
xlabel('Time (sec)')
ylabel('Joint accelaration (deg/s^2)')
grid on