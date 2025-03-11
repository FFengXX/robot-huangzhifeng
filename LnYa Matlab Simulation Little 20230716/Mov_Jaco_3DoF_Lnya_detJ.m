close all;
clear;

figure;

th1=0;
th2=-10;
th3=-90;
th4=0;
th5=0;
th6=0;

DHfk_3DoF2_Lnya(th1,th2,th3,0);
pause;
num=1;
for i=1:20
    figure(1);
    DHfk_3DoF2_Lnya(th1,th2,th3,0);
    J=Jacobian6DoF_Ln(th1,th2,th3,th4,th5,th6)
    
    x=det(J) ;fprintf('x= %2.4f ',x);
    xout(num)=x;
    t(num)=i;
    num=num+1;
 
    dD=[0 0 4 0 0 0]';
    dth=inv(J)*dD
    th1=th1+dth(1)/pi*180;
    th2=th2+dth(2)/pi*180;
    th3=th3+dth(3)/pi*180;
    th4=th4+dth(4)/pi*180;
    th5=th5+dth(5)/pi*180;
    th6=th6+dth(6)/pi*180;
    
    figure(3);
    axis([0,num,-5000000,5000000]);
    plot(t,xout,'r-O');hold on;
end



