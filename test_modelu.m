function dydt = test_modelu(t,y,wr,wl,r,d)
    V=r/2*(wr+wl);
    w=r/d*(wr-wl);
    dydt = zeros(3,1);
    dydt(1)=w;
    dydt(2)=V*cos(y(1));
    dydt(3)=V*sin(y(1));
end