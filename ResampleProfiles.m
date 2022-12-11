function [ss, ll] = ResampleProfiles(s, l, nfe)
ss = [];
ll = [];

for ii = 1 : (length(s) - 1)
    ss = [ss, linspace(s(ii), s(ii+1), nfe)];
    ll = [ll, linspace(l(ii), l(ii+1), nfe)];
end

NFE = length(ss);
id_list = round(linspace(1, NFE, nfe));
ss = ss(id_list);
ll = ll(id_list);
end