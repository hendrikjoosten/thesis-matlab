syms th1 ph1 rh1 Ax Ay Az

R = [[ cos(ph1)*cos(th1) - sin(ph1)*sin(rh1)*sin(th1), -cos(rh1)*sin(ph1), cos(ph1)*sin(th1) + cos(th1)*sin(ph1)*sin(rh1)];
    [ cos(th1)*sin(ph1) + cos(ph1)*sin(rh1)*sin(th1),  cos(ph1)*cos(rh1), sin(ph1)*sin(th1) - cos(ph1)*cos(th1)*sin(rh1)];
[-cos(rh1)*sin(th1), sin(rh1), cos(rh1)*cos(th1)]];

tot = R*[Ax;Ay;Az].';

matlabFunction(tot, 'File', 'generated_acc_rot');
                      
               