function J=Jacobian7DoF_Ln(th1,th2,th3,th4,d5,th6,th7)
% close all
global Link

jsize=7;
J=zeros(6,jsize);

Link(2).th=th1*pi/180;
Link(3).th=th2*pi/180;
Link(4).th=th3*pi/180;
Link(5).th=th4*pi/180;
Link(6).dz=d5; % �� 5 ���ؽڵ�λ��
Link(7).th=th6*pi/180;
Link(8).th=th7*pi/180; % �� 7 ���ؽڵĽǶ�
%�����Ǽ�������
for i=1:8
    Matrix_DH_Ln(i);
end

Link(1).p=Link(1).p(1:3);
for i=2:8
    Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(1:3,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end
%�����ſɱȾ���  
% for n=1:jsize
%     a=Link(n).R*Link(n).az;
%     J(:,n)=[cross(a,Link(8).p-Link(n).p); a];
% end
%%%%�����ſɱȾ���  
for n=1:jsize
    a = Link(n).R * Link(n).az; % % ��ȡ�ؽڵ� Z �᷽����ת���ƽ���ᣩ
    
    if n == 5 % �� 5 ���ؽ��������ؽ�
        % �����ؽڵ��ſɱȾ�����ֻ����ƽ�Ʋ��֣����ٶȲ���Ϊ 0
        J(:, n) = [a; 0; 0; 0];
    else % �����ؽ�����ת�ؽ�?
        % ��ת�ؽڵ��ſɱȾ�����ʹ��������
        J(:, n) = [cross(a, Link(8).p - Link(n).p); a];
    end
end