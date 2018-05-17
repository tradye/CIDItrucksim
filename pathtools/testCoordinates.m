clear;

Gdx=5;
Gdy=0;
angle=0.5;
[Lx,Ly] = funcGlobalToLocal(Gdx,Gdy,angle);
disp(Lx + ", " + Ly);

Gdx=5;
Gdy=10;
angle=1;
[Lx,Ly] = funcGlobalToLocal(Gdx,Gdy,angle);
disp(Lx + ", " + Ly);

Gdx=5;
Gdy=10;
angle=1;
[Lx,Ly] = funcGlobalToLocal(Gdx,Gdy,angle);
disp(Lx + ", " + Ly);

angle = 0.5;
Gdx = -10*sin(angle);
Gdy = 10*cos(angle);
[Lx,Ly] = funcGlobalToLocal(Gdx,Gdy,angle);
disp(Lx + ", " + Ly);