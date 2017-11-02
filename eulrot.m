function f = eulrot(roll,pitch,yaw)
    f = RotateZ(yaw)*RotateY(pitch)*RotateX(roll);
end