function [Lx,Ly] = funcGlobalToLocal(Gdx,Gdy,angle)
    Lx = Gdx*cos(angle) + Gdy*sin(angle);
    Ly = -Gdx*sin(angle) + Gdy*cos(angle);
end % globalToLocal