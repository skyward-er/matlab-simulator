function flag = check_gps_correction(nas, measures)
    
    flag = ( norm(measures.acc)< 34 ) && ...
            nas.flag.flag_gps_correction && ...
            measures.gps.lastindex ~= measures.gps.index;

end

