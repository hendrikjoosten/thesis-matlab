%%sensor
figure 
subplot(3,1,1)       
plot(z01Avail)
title('IMU availability')
subplot(3,1,2)       
plot(z02Avail)       
title('barometer availability')
subplot(3,1,3)       
plot(z03Avail)       
title('GPS availability')

%%camera 1
figure 
subplot(4,1,1)      
plot(z04Avail)
title('Front Left Point 1 availability')
subplot(4,1,2)       
plot(z05Avail)       
title('Front Left Point 2 availability')
subplot(4,1,3)       
plot(z06Avail)       
title('Front Left Point 3 availability')
subplot(4,1,4)       
plot(z07Avail)       
title('Front Left Point 4 availability')

%%camera 2
figure 
subplot(4,1,1)      
plot(z08Avail)
title('Front Right Point 1 availability')
subplot(4,1,2)       
plot(z09Avail)       
title('Front Right Point 2 availability')
subplot(4,1,3)       
plot(z10Avail)       
title('Front Right Point 3 availability')
subplot(4,1,4)       
plot(z11Avail)       
title('Front Right Point 4 availability')

%%camera 3
figure 
subplot(4,1,1)      
plot(z12Avail)
title('Back Left Point 1 availability')
subplot(4,1,2)       
plot(z13Avail)       
title('Back Left Point 2 availability')
subplot(4,1,3)       
plot(z14Avail)       
title('Back Left Point 3 availability')
subplot(4,1,4)       
plot(z15Avail)       
title('Back Left Point 4 availability')

%%camera 4
figure 
subplot(4,1,1)      
plot(z16Avail)
title('Back Right Point 1 availability')
subplot(4,1,2)       
plot(z17Avail)       
title('Back Right Point 2 availability')
subplot(4,1,3)       
plot(z18Avail)       
title('Back Right Point 3 availability')
subplot(4,1,4)       
plot(z19Avail)       
title('Back Right Point 4 availability')


