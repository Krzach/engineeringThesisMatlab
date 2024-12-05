function [A,B] = linearizeModel(X,prevU, T)

    a=0.02/0.18*T;
    b=0.01*T;

    A=zeros(3,3);
    B=zeros(3,2);

    A(1,1)=1;
    A(2,2)=1;
    A(3,3)=1;
    A(2,1) = -sin(X(1))*b*(prevU(1)+prevU(2));
    A(3,1) = cos(X(1))*b*(prevU(1)+prevU(2));

    B(1,1) = a;
    B(1,2) = -a;
    B(2,1) = cos(X(1))*b;
    B(2,2) = cos(X(1))*b;
    B(3,1) = sin(X(1))*b;
    B(3,2) = sin(X(1))*b;

end