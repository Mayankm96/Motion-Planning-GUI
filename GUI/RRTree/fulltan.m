function v = fulltan(y,x)
%Computes atan from range 0 to 2pi
%   Usage: similar to atan(Y,X) in MATLAB
    v=nan;
    if x>0
        v=atan(y/x);
    end
    if y>=0 && x<0
        v=pi+atan(y/x);
    end
    if y<0 && x<0
        v=-pi+atan(y/x);
    end
    if y>0 && x==0
        v=pi/2;
    end
    if y<0 && x==0
        v=-pi/2;
    end
    if v<0
        v=v+2*pi;
    end
end
