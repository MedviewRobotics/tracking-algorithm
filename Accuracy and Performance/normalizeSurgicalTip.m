function [surgicalTip_out, deviation] = normalizeSurgicalTip(surgicalTip_in, k)

surgicalTip_out(1, k) = mean(surgicalTip_in(1, k-5:k));
surgicalTip_out(2, k) = mean(surgicalTip_in(2, k-5:k));
surgicalTip_out(3, k) = mean(surgicalTip_in(3, k-5:k));

end

