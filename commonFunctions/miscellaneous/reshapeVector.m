function vRef = reshapeVector(xRef, xV, yV)
    
    ratio = xRef(end)/xV(end); 
    xV = ratio .* xV;               % vector scaled to have the required ending point
    
    xV(1) = xRef(1); 
    xV(end) = xRef(end); 
    
    vRef = interp1(xV, yV, xRef);  % correct number of points     
end