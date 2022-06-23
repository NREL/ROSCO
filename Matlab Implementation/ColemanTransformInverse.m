function [PitComIPC] = ColemanTransformInverse(axTIn, axYIn, aziAngle, nHarmonic, aziOffset)
% The inverse Coleman or d-q axis transformation transforms the direct axis and quadrature axis
% back to root out of plane bending moments of each turbine blade

    
    phi2 = 2*pi/3; % Phase difference from first to second blade
    phi3 = 4*pi/3; % Phase difference from first to third blade

    % Body
    PitComIPC(1) = cos(nHarmonic*(aziAngle+aziOffset))*axTIn + sin(nHarmonic*(aziAngle+aziOffset))*axYIn;
    PitComIPC(2) = cos(nHarmonic*(aziAngle+aziOffset+phi2))*axTIn + sin(nHarmonic*(aziAngle+aziOffset+phi2))*axYIn;
    PitComIPC(3) = cos(nHarmonic*(aziAngle+aziOffset+phi3))*axTIn + sin(nHarmonic*(aziAngle+aziOffset+phi3))*axYIn;

end
