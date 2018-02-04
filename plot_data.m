%%sensor
figure(1)
subplot(3,1,1)      
scatter(body_gps_pos(:,1),body_gps_pos(:,2))
title('Body GPS Position')
subplot(3,1,2)       
plot(body_gyro_z)       
title('Angular Velocity Body Z')
subplot(3,1,3)       
plot(body_accel_z)       
title('Linear Acceleration Body Z')

