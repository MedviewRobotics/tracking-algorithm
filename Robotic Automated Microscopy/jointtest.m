function [Joint1,Joint2,Joint3,Joint4,Joint5,Joint6] = jointtest(Q,count)

    Joint1(count*10 - 9:10*count,1) = Q(:,1);
    Joint2(count*10 - 9:10*count,1) = Q(:,2);
    Joint3(count*10 - 9:10*count,1) = Q(:,3);
    Joint4(count*10 - 9:10*count,1) = Q(:,4);
    Joint5(count*10 - 9:10*count,1) = Q(:,5);
    Joint6(count*10 - 9:10*count,1) = Q(:,6);
end

