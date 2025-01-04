figure
%plot(xHistory(:,2), xHistory(:,3))
%hold on
syms x
    eq2 = 0.5*sin(x*pi*0.5)^5 ;
fplot(eq2, [0,4]);
hold on

%for k = 2:20:T
%quiver(xHistory(k,2),xHistory(k,3),dxHistory(k,1)-xHistory(k,2),dxHistory(k,2)-xHistory(k,3),0,"Color","red",  'MaxHeadSize', 0.5, 'LineWidth', 1.5)
%end 
%plot(dxHistory(:,1), dxHistory(:,2))

xlabel("X")
ylabel("Y")
title("Funkcja trasy")
%legend("Trasa","Pojazd")
