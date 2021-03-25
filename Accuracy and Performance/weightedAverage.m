function surgicalTip_3D_norm = weightedAverage(surgicalTip_3D, k)

W = [0.05 0.05  0.15  0.2  0.25  0.3];
recent_values = [surgicalTip_3D(:, k-5)  surgicalTip_3D(:, k-4)  surgicalTip_3D(:, k-3)  surgicalTip_3D(:, k-2)  surgicalTip_3D(:, k-1)  surgicalTip_3D(:, k)];
temp = recent_values.*W;

surgicalTip_3D_norm(1) = sum(temp(1,:));
surgicalTip_3D_norm(2) = sum(temp(2,:));
surgicalTip_3D_norm(3) = sum(temp(3,:));

end

