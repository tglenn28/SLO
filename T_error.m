function d = T_error(T1,T2)
    blah = logm(T1*inv(T2));
    d = norm(blah(:));
end
