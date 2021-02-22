qsq1=[0.46088 0.37699 0 1.31 0 0];
qsq2=[.81681 0.56549 0 1.0681 0 0 ];
qsq3=[2.36 0.69115 0 0.848 0 0];
qsq4=[2.66 0.37699 0 1.31 0 0];
% qsq5=[pi/2 0.62831 0 1.5708 0 0.94249 0];
% qsq6=[0 0.62831 0 1.5708 0 0.94249 0];

t=0:.04:2;
sqtraj1=jtraj(q0,qsq1,t); 
sqtraj2=jtraj(qsq1,qsq2,t); 
sqtraj3=jtraj(qsq2,qsq3,t); 
sqtraj4=jtraj(qsq3,qsq4,t);
sqtraj5=jtraj(qsq4,qsq1,t);
sqtraj6=jtraj(qsq1,q0,t);
% sqtraj7=jtraj(qsq6,q0,t);

hold on
atj=zeros(4,4);
view(-35,40)
xlim([-40,40])
ylim([-40,40])
zlim([0,60])
for i=1:1:51
    atj=AT03.fkine(sqtraj1(i,:));
%     jta=transpose(atj);
    JTA(i,:)=jta(4,1:3);
    jta=JTA;
    plot2(jta(i,:),'r.')
    AT03.plot(sqtraj1(i,:))
    plot2(JTA,'b')
end
for i=1:1:51
    atj2=AT03.fkine(sqtraj2(i,:));
    jta2=transpose(atj2);
    JTA2(i,:)=jta2(4,1:3);
    jta2=JTA2;
    plot2(jta2(i,:),'r.')
    AT03.plot(sqtraj2(i,:))
    plot2(JTA2,'b')
end
for i=1:1:51
    atj3=AT03.fkine(sqtraj3(i,:));
    jta3=transpose(atj3);
    JTA3(i,:)=jta3(4,1:3);
    jta3=JTA3;
    plot2(jta3(i,:),'r.')
    AT03plot(sqtraj3(i,:))
    plot2(JTA3,'b')
end
for i=1:1:51
    atj4=Rbt.fkine(sqtraj4(i,:));
    jta4=transpose(atj4);
    JTA4(i,:)=jta4(4,1:3);
    jta4=JTA4;
    plot2(jta4(i,:),'r.')
    Rbt.plot(sqtraj4(i,:))
    plot2(JTA4,'b')
end
for i=1:1:51
    atj5=Rbt.fkine(sqtraj5(i,:));
    jta5=transpose(atj5);
    JTA5(i,:)=jta5(4,1:3);
    jta5=JTA5;
    plot2(jta5(i,:),'r.')
    Rbt.plot(sqtraj5(i,:))
    plot2(JTA5,'b')
end
for i=1:1:51
    atj6=Rbt.fkine(sqtraj6(i,:));
    jta6=transpose(atj6);
    JTA6(i,:)=jta6(4,1:3);
    jta6=JTA6;
    plot2(jta6(i,:),'r.')
    Rbt.plot(sqtraj6(i,:))
    plot2(JTA6,'b')
end
