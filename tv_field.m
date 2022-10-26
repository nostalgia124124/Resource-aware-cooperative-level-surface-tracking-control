function P = tv_field(X,Y,Z,t)

%   a time-varying field
    source = [0.5 0.5 1]; 
    t = t/6;
    beta = 35 - 27*(t)/7000;
    alpha = 160000/(30000-(t));
    P = alpha.* exp(beta.*(-1.*(X-source(1)).^2-1.*(Y-source(2)).^2-1.*(Z-source(3)).^2));

end