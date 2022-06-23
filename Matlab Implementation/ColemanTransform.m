function [axTOut,axYOut] = ColemanTransform(rootMOOP, aziAngle, nHarmonic)
% The Coleman or d-q axis transformation transforms the root out of plane bending moments of each turbine blade
% to a direct axis and a quadrature axis
    
    phi2 = 2*pi/3; % Phase difference from first to second blade
    phi3 = 4*pi/3; % Phase difference from first to third blade
    % Body
    axTOut  = 2.0/3.0 * (cos(nHarmonic*(aziAngle))*rootMOOP(1) + cos(nHarmonic*(aziAngle+phi2))*rootMOOP(2) + cos(nHarmonic*(aziAngle+phi3))*rootMOOP(3));
    axYOut  = 2.0/3.0 * (sin(nHarmonic*(aziAngle))*rootMOOP(1) + sin(nHarmonic*(aziAngle+phi2))*rootMOOP(2) + sin(nHarmonic*(aziAngle+phi3))*rootMOOP(3));

end