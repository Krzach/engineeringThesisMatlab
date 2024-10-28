function y = model_zdyskretyzowany(y,T,wr,wl,r,d)
    V=r/2*(wr+wl);
    w=r/d*(wr-wl);
    y(3) = T*w + y(3);
    y(1) = T*V*cos(y(3)) + y(1);
    y(2) = T*V*sin(y(3)) + y(2);
end