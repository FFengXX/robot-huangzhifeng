close all;
clear;

figure; 

th1=0;
th2=90;
th3=0;
th4=0;
d5=0;
th6=0;
th7 =0;
stp=15;
dtime=0.02;
DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7,0); 

pause;hold off;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint1
for i=-180:stp:180
   DHfk7Dof_Lnya(th1+i,th2,th3,th4,d5,th6,th7,1); 

end
for i=180:-stp:-180
    if i==-180
      DHfk7Dof_Lnya(th1+i,th2,th3,th4,d5,th6,th7,0); 
    else
      DHfk7Dof_Lnya(th1+i,th2,th3,th4,d5,th6,th7,1); 
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint2
for i=-90:stp:140
   DHfk7Dof_Lnya(th1,th2+i,th3,th4,d5,th6,th7,1); 

end
for i=140:-stp:-90
    if i==-90
      DHfk7Dof_Lnya(th1,th2+i,th3,th4,d5,th6,th7,0); 
    else
      DHfk7Dof_Lnya(th1,th2+i,th3,th4,d5,th6,th7,1); 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint3
for i=-100:stp:130
   DHfk7Dof_Lnya(th1,th2,th3+i,th4,d5,th6,th7,1); 

end
for i=130:-stp:-100
    if i==-100
      DHfk7Dof_Lnya(th1,th2,th3+i,th4,d5,th6,th7,0); 
    else
      DHfk7Dof_Lnya(th1,th2,th3+i,th4,d5,th6,th7,1); 
    end
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint4
for i=-150:stp:170
   DHfk7Dof_Lnya(th1,th2,th3,th4+i,d5,th6,th7,1); 

end
for i=170:-stp:-150
    if i==-150
      DHfk7Dof_Lnya(th1,th2,th3,th4+i,d5,th6,th7,0); 
    else
      DHfk7Dof_Lnya(th1,th2,th3,th4+i,d5,th6,th7,1); 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint5ÉìËõ¹Ø½Ú
for i=0:stp:500
   DHfk7Dof_Lnya(th1,th2,th3,th4,d5+i,th6,th7,1); 

end
for i=500:-stp:0
    if i==0
      DHfk7Dof_Lnya(th1,th2,th3,th4,d5+i,th6,th7,0); 
    else
     DHfk7Dof_Lnya(th1,th2,th3,th4,d5+i,th6,th7,1); 
    end
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint6
for i=-170:stp:170
   DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6+i,th7,1); 

end
for i=170:-stp:-170
    if i==-170
      DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6+i,th7,0); 
    else
      DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6+i,th7,1); 
    end
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Joint7
for i=-180:stp:180
   DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7+i,1); 

end
for i=180:-stp:-180
    if i==-180
      DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7+i,0); 
    else
      DHfk7Dof_Lnya(th1,th2,th3,th4,d5,th6,th7+i,1); 
    end
end
