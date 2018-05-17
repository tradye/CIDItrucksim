function [x,y] = funcLocalToGlobal(Lx2,Ly2,Gx1,Gy1,angle)
    x = Lx2*cos(angle) - Ly2*sin(angle) + Gx1;
    y = Lx2*sin(angle) + Ly2*cos(angle) + Gy1;
end % funcLocalToGlobal