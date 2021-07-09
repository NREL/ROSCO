function a = Af_sigma(x0,x1);

a3 = 2/(x0-x1)^3;
a2 = -3*(x0+x1)/(x0-x1)^3;
a1 = 6*x1*x0/(x0-x1)^3;
a0 = (x0-3*x1)*x0^2/(x0-x1)^3;

a = [a3,a2,a1,a0];