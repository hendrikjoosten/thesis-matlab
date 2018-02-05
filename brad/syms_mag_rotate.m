syms th1 ph1 rh1 Mx My Mz

R = [[ cos(ph1)*cos(th1) - sin(ph1)*sin(rh1)*sin(th1), -cos(rh1)*sin(ph1), cos(ph1)*sin(th1) + cos(th1)*sin(ph1)*sin(rh1)];
    [ cos(th1)*sin(ph1) + cos(ph1)*sin(rh1)*sin(th1),  cos(ph1)*cos(rh1), sin(ph1)*sin(th1) - cos(ph1)*cos(th1)*sin(rh1)];
[-cos(rh1)*sin(th1), sin(rh1), cos(rh1)*cos(th1)]];

tot = (R*[Mx;My;Mz]).';

matlabFunction(tot, 'File', 'Thesis/generated_mag_rot');