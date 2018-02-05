syms th1 th2 th3 ph1 ph2 ph3 l1 l2 l3

hx = [   l1*cos(ph1)*cos(th1) + l2*cos(ph2)*cos(th2) + l3*cos(ph3)*cos(th3);
                l1*cos(th1)*sin(ph1) + l2*cos(th2)*sin(ph2) + l3*cos(th3)*sin(ph3);
                          - l1*sin(th1) - l2*sin(th2) - l3*sin(th3)];

x_vec = [th1 th2 th3 ph1 ph2 ph3].';                      

H = jacobian(hx, x_vec);

matlabFunction(H, hx, 'File', 'generated_H3');
                      
               