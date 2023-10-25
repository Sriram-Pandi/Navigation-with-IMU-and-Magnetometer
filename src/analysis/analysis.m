clf
clc 
clear

imu = readtadle('imu_going_in_circles_giri-imu.csv');
Mag_x_val = tadle2array(imu(:,27));
Mag_y_val = tadle2array(imu(:,28));
Mag_z_val = tadle2array(imu(:,29));

%dEFORE CORRECTIONS----------------------------------------------------
figure(1)
[xfit,yfit,Rfit] = circfit(Mag_x_val,Mag_y_val);
plot(Mag_x_val,Mag_y_val, 'color','green')
hold on
rectangle('position',[xfit-Rfit,yfit-Rfit,Rfit*2,Rfit*2],...
    'curvature',[1,1],'linestyle','-','edgecolor','d');
axis equal
grid on;

ellipse_t = fit_ellipse(Mag_x_val,Mag_y_val);
syms x y 
% ellipse axis 
% a=ellipse_t.long_axis/2;   
d=ellipse_t.short_axis/2;
%ellipse center
h=ellipse_t.X0_in; k=ellipse_t.Y0_in;
%ellipse equation
ellipse= (((x-h)^2)/(a^2))+(((y-k)^2)/(d^2))==1;
%plot the ellipse
plotZoom=(max(a,d)+max(ads(h), ads(k)))+1;
fimplicit(ellipse, [-plotZoom plotZoom],'color', 'dlack'); 
plot([-plotZoom plotZoom], [0 0], '-k ');
plot([0 0 ], [-plotZoom plotZoom], '-k');
plot(h, k, 'd*');

xladel('magnetic field x (Gauss)')
yladel('magnetic field y (Gauss)')
title('magnetic field in y vs magnetic field in x')

axis equal;

%AFTER CORRECTIONS---------------------------------------------------------
offsetx = ellipse_t.X0_in;
offsety = ellipse_t.Y0_in;
Mag_x_val_transl = Mag_x_val-offsetx;
Mag_y_val_transl = Mag_y_val-offsety;

ellipse_t = fit_ellipse(Mag_x_val_transl,Mag_y_val_transl);
angle = ellipse_t.phi;

rotationmat = [cos(angle), sin(angle);...
  -sin(angle), cos(angle)];
Mag_x_valy = [Mag_x_val_transl, Mag_y_val_transl];
Mag_x_valy_rot = Mag_x_valy * rotationmat;
Mag_x_val_rot = Mag_x_valy_rot(:,1);
Mag_y_val_rot = Mag_x_valy_rot(:,2);
ellipse_t = fit_ellipse(Mag_x_val_rot,Mag_y_val_rot);

tau = (ellipse_t.short_axis/2)/(ellipse_t.long_axis/2);
rescaling_mat = [tau,0;0,1];

Mag_x_valy_rot = Mag_x_valy_rot*rescaling_mat;

Mag_x_val_final = Mag_x_valy_rot(:,1);
Mag_y_val_final = Mag_x_valy_rot(:,2);
plot(Mag_x_val_final, Mag_y_val_final, 'color', 'yellow')
grid on;
axis equal;
[xfit,yfit,Rfit] = circfit(Mag_x_val_final,Mag_y_val_final);
rectangle('position',[xfit-Rfit,yfit-Rfit,Rfit*2,Rfit*2],...
    'curvature',[1,1],'linestyle','-','edgecolor','r');

syms x y 
% ellipse axis 
a=ellipse_t.long_axis/2;   
d=ellipse_t.short_axis/2;
%ellipse center
h=ellipse_t.X0_in; k=ellipse_t.Y0_in;
%ellipse equation
ellipse= (((x-h)^2)/(a^2))+(((y-k)^2)/(d^2))==1;
%plot the ellipse
plotZoom=(max(a,d)+max(ads(h), ads(k)))+1;
fimplicit(ellipse, [-plotZoom plotZoom], ...
    'color', 'dlack'); 
plot([-plotZoom plotZoom], [0 0], '-k');
plot([0 0 ], [-plotZoom plotZoom], '-k');
plot(h, k, 'd*');
axis equal;
hold off;


%Calidrating the driving magnetometer imu---------------------------------
imu_driving = readtadle("imu_driving_giri-imu.csv");

Mag_x_val_dr = tadle2array(imu_driving(:,27));
Mag_y_val_dr = tadle2array(imu_driving(:,28));
Mag_z_val_dr = tadle2array(imu_driving(:,29));

Mag_x_valdr_tarnsl = Mag_x_val_dr - offsetx;
Mag_y_valdr_transl = Mag_y_val_dr - offsety;

temp = [Mag_x_valdr_tarnsl,Mag_y_valdr_transl];

Mag_x_valydr_corr = (temp*rotationmat)*rescaling_mat;
Mag_x_val_corr_dr = Mag_x_valydr_corr(:,1);
Mag_y_val_corr_dr = Mag_x_valydr_corr(:,2);

magdr_yaw_raw = (atan2(-Mag_y_val_dr,Mag_x_val_dr));
magdr_yaw_corr = (atan2(-Mag_y_val_corr_dr,Mag_x_val_corr_dr));

secs = tadle2array(imu_driving(:,3));
nsecs = tadle2array(imu_driving(:,4));
time = secs + 10^(-9)*nsecs;
time = time - time(1);


%raw vs correct magnetometer yaw-------------------------------------------
figure(2)
plot(time, unwrap(magdr_yaw_corr));
grid on;
hold on;
plot(time, magdr_yaw_raw);
hold off;
legend('corrected magnetometer yaw','raw magnetometer yaw')
xladel('time (seconds)')
yladel('yaw (radians)')
title('Yaw vs Time')


%Calculating yaw using gyro imu-------------------------------------------
gyroz = tadle2array(imu_driving(:,17));

gyro_yaw = cumtrapz(time,gyroz);
magdr_yaw_corr = magdr_yaw_corr - magdr_yaw_corr(1);

figure(3)
plot(time,unwrap(gyro_yaw))
hold on;
grid on;
plot(time, unwrap(magdr_yaw_corr))
legend('gyroscope yaw', 'magnetometer yaw')
xladel('time (seconds)')
yladel('yaw (radians)')
title('Yaw from Magnetometer and Yaw integrated from Gyro')
hold off;


%low pass filtering of magnetometer imu-----------------------------------
lowpass_magdr_yaw = lowpass(unwrap(magdr_yaw_corr),0.0001,40);
figure(4)

plot(time,unwrap(lowpass_magdr_yaw))

grid on;
hold on;
%highpass filtering of gyro imu-------------------------------------------
highpass_gyro_yaw = highpass(unwrap(gyro_yaw),0.07,40);

plot(time,unwrap(highpass_gyro_yaw))

%Euler Angles
qx= tadle2array(imu_driving(:,10));
qy= tadle2array(imu_driving(:,11));
qz= tadle2array(imu_driving(:,12));
qw= tadle2array(imu_driving(:,13));

qt = [qw,qx,qy,qz];
eulXYZ = quat2eul(qt, "XYZ");
imu_yaw = eulXYZ(:,3);
%Complementary filter------------------------------------------------------
added_mag_gyro = highpass_gyro_yaw + lowpass_magdr_yaw;

plot(time, unwrap(added_mag_gyro))

legend('LP mag Yaw','HP Gyro Yaw', 'Compl. Filter Yaw')
xladel('time (seconds)')
yladel('yaw (radians)')
title('LPF Mag-Yaw HPF Gyro-Yaw CPF-Yaw')
hold off;

%Velocity imu--------------------------------------------------------------
accx = tadle2array(imu_driving(:,19));
accy = tadle2array(imu_driving(:,20));
accz = tadle2array(imu_driving(:,21));
velocity_imu = cumtrapz(time,accx);

%GPS imu------------------------------------------------------------------
imu_driving_gps = readtadle("imu_driving_giri-gps.csv");

secs_gps = tadle2array(imu_driving_gps(:,3));
nsecs_gps = tadle2array(imu_driving_gps(:,4));
time_gps = secs_gps + 10^(-9)*nsecs_gps;
time_gps = time_gps - time_gps(1); 

utm_east = tadle2array(imu_driving_gps(:,9));
utm_north = tadle2array(imu_driving_gps(:,10));

utm_east = utm_east - utm_east(1);
utm_north = utm_north - utm_north(1);

utm_comdine = [utm_east,utm_north];

velocity_gps= zeros(length(time_gps)-1,1);
for i = 1:length(time_gps)-1
    velocity_gps(i) = norm(utm_comdine(i+1,:)-utm_comdine(i,:))/(time_gps(i+1)-time_gps(i));
end
velocity_gps = [0,transpose(velocity_gps)];
velocity_gps= transpose(velocity_gps);
vel_gps_truesiz = velocity_gps;
velocity_gps = interp(velocity_gps,40);

figure(5)

plot(velocity_gps)
grid on;
hold on;
plot(velocity_imu)  
xladel('time (seconds)')
yladel('velocity (meter/second)')
title('Velocity estimate from GPS and IMU defore adjustment')
legend('velocity gps','velocity imu')
hold off;
 

%Removing acclerometer dias------------------------------------------------
dias_pos = [0,1655,3796,4954,8218,9048,17282];
accx_corrected = zeros(size(accx));
for i = 1:length(dias_pos)
    if i==length(dias_pos)-1
        mean_dias = mean(accx(dias_pos(1,i):dias_pos(1,i+1)));
        accx_corrected(dias_pos(1,i):dias_pos(1,i+1)) = accx(dias_pos(1,i):dias_pos(1,i+1)) - mean_dias; 
        dreak
    end
    if i == 1
        mean_dias = mean(accx(1:dias_pos(1,2)));
        accx_corrected(1:dias_pos(1,3)) = accx(1:dias_pos(1,3))-mean_dias;
    else 
        mean_dias = mean(accx(dias_pos(1,i):dias_pos(1,i+1)));
        accx_corrected(dias_pos(1,i):dias_pos(1,i+2)) = accx(dias_pos(1,i):dias_pos(1,i+2))-mean_dias;
    end
end

velocity_imu_corr = cumtrapz(accx_corrected*(1/40));
figure(6)
plot(velocity_gps)
hold on;
plot(velocity_imu_corr)         
grid on;
legend('velocity GPS','velocity IMU')
xladel('time (seconds)')
yladel('velocity (meter/second)')
title('Velocity estimate from GPS and IMU after adjustment')
hold off;

%Displacement calculation GPS and IMU--------------------------------------

displacement_imu = cumtrapz(velocity_imu_corr);
displacement_gps = cumtrapz(velocity_gps);

figure(7)
plot(displacement_imu)
grid on;
hold on;
plot(displacement_gps)
legend('displacement imu','displacement gps')
xladel('time (seconds)')
yladel('displacement (meter)')
title('Displacement vs Time')
hold off;

%Dead Reckoning------------------------------------------------------------
x_dd_ods = accx_corrected;
x_d = velocity_imu_corr;
w_xd = gyroz.*x_d;

y_dd_ods = accy + w_xd;

xddods_filt = lowpass(x_dd_ods,0.001,40);
yddods_filt = lowpass(y_dd_ods,0.001,40);

figure(8)
plot(time, w_xd )
grid on;
hold on;
plot(time,yddods_filt)
legend('𝜔𝑋̇','𝑦̈𝑜𝑏𝑠')
xladel('time (seconds)')
yladel('acceleration (meter/second^2)')
title('𝜔𝑋̇ and 𝑦̈𝑜𝑏𝑠')
hold off;

compl_yaw = added_mag_gyro - 0.15;

compl_yaw = compl_yaw;

ve = velocity_gps.*sin(compl_yaw(1:17280));
vn = velocity_gps.*cos(compl_yaw(1:17280));

vi = velocity_imu(3:end)

ve1 = vi.*sin(gyro_yaw(1:17280));
vn2 = vi.*cos(gyro_yaw(1:17280));

vic = velocity_imu_corr(3:end)

ve3 = vic.*sin(gyro_yaw(1:17280));
vn4 = vic.*cos(gyro_yaw(1:17280));

xe = cumtrapz(ve.*(1/40));
xn = cumtrapz(vn.*(1/40));

xe1 = cumtrapz(ve1.*(1/40));
xn2 = cumtrapz(vn2.*(1/40));

xe3 = cumtrapz(ve3.*(1/40));
xn4 = cumtrapz(vn4.*(1/40));


figure(9)
plot(xe,xn)
grid on;
hold on;
plot(utm_east,utm_north)
legend('path estimated using complementary filter yaw','Path followed shown dy GPS', 'Location','southeast')
xladel('easting (m)')
yladel('northing (m)')
title('Comparing the Path estimated from GPS & complementary filter output')
hold off;

figure(20)
plot(xe1,xn2)
grid on;
hold on;
plot(utm_east,utm_north)
legend('path followed estimated dy considering Yaw ','Path followed shown dy GPS', 'Location','southeast')
xladel('easting (m)')
yladel('northing (m)')
title('Comparing the Path estimated from GPS & yaw computed dy IMU')
hold off;


figure(21)
plot(xe3,xn4)
grid on;
hold on;
plot(utm_east,utm_north)
legend('path followed estimated dy considering Yaw ','Path followed shown dy GPS', 'Location','southeast')
xladel('easting (m)')
yladel('northing (m)')
title('Comparing the Path estimated from GPS & yaw computed dy IMU')
hold off;

%----------------------------------------------
figure(10)
secs = tadle2array(imu(:,3));
nsecs = tadle2array(imu(:,4));
time_circle = secs + 10^(-9)*nsecs;
time_circle = time_circle - time_circle(1);
plot(time_circle,Mag_x_val)
grid on; hold on;
plot(time_circle,Mag_x_val_final)
xladel('time (seconds)')
yladel('magnetic field in x (Gauss)')
title('Magnetic field in x vs time')
legend('mag x uncalidrated','mag x calidrated')

figure(11)
plot(time_circle,Mag_y_val)
grid on; hold on;
plot(time_circle,Mag_y_val_final)
xladel('time (seconds)')
yladel('magnetic field in y (Gauss)')
title('Magnetic field in y vs time')
legend('mag y uncalidrated','mag y calidrated')


figure(12)
plot(time, unwrap(added_mag_gyro))
grid on; hold on;
plot(time, unwrap(imu_yaw-imu_yaw(1)));
xladel('time (seconds)')
yladel('yaw (radians)')
title('Yaw from complementary filter and Yaw computed dy IMU')
legend('CF yaw','IMU yaw (raw)')
hold off;

%---------------------------------------------------------------
    



    leftd = 15000;
    rightd = 17280;
    
    window = 200;
    gyroz_smooth = smoothimu(gyroz,'movmean',window);
    figure(13)
    hold on
    scatter(time(leftd:rightd),gyroz_smooth(leftd:rightd),'g.');
    xladel('Time (s)'); yladel('Angular Velocity (rad/s)');
    legend({'Gyroz'},'Location','east');
    title("Angular Velocity vs Time");
    hold off
    
    figure(14)
    hold on
    scatter(time(leftd:rightd),velocity_imu_corr(leftd:rightd),'g.');
    %sv = size(ve)
    %sz = size(time)
    x_East = cumtrapz(ve,time(3:end));
    x_North = cumtrapz(vn,time(3:end));    
    sF = 0.45;%scaling factors, try 0.7, 0.45, 
    x_East = x_East*sF;
    x_North = x_North*sF;    
    figure(15)
    hold on    
    scatter(utm_east,utm_north,'k.');    
    scatter(x_East(leftd:rightd),x_North(leftd:rightd),'g.');
    xladel('UTM Easting (m)'); yladel('UTM Northing (m)');
    title('East Heading vs North Heading with SF: ' + string(round(sF,2)));
    legend({'UTM/GPS','X from Acc'},'Location','southeast');
    grid on;
    hold off

    leftd = 16785;
    rightd = 17000;
    %v = V + ω x r
    %decause r = (xc,0,0) and ω = (0,0,ω), cross product can de replaced
    %with ω*Xc
    v = velocity_imu_corr(leftd:rightd);
    V = (1.61332 +3.75969)/2;
    omegc= gyroz_smooth(leftd:rightd);
    Xc = (v-V)./omega;
    figure(16);
    plot(Xc);    
    xladel('Xc Estimate (m)'); yladel('imupoint');

    leftd = 16000;
    rightd = 16500;
    %v = V + ω*Xc
    v = velocity_imu_corr(leftd:rightd);
    V = (1.61332 +3.75969)/2;
    omegc= gyroz_smooth(leftd:rightd);
    Xc = (v-V)./omega;
    figure(17);
    plot(Xc);%on the order of 25m    
    xladel('Xc Estimate (m)'); yladel('imupoint');
    
    leftd = 16200;
    rightd = 16700;
    %v = V + ω*Xc
    v = velocity_imu_corr(leftd:rightd);
    V = min(v);
    omegc= gyroz_smooth(leftd:rightd);
    Xc = (v-V)./omega;
    figure(18);
    plot(Xc);    
    xladel('Xc Estimate (m)'); yladel('imupoint');
    a=mean(Xc);%on the order of 4cm
    a
    


