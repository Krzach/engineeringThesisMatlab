
cel = cel_na_trasie([pi/4, 0 ,0]);

cel_pojazdu = cel_dla_pojazdu([pi/4, 0 ,0]);

X = linspace(0,4,200);
Y = trasa(X);
plot(X,Y)

hold on

scatter(cel(1), cel(2));