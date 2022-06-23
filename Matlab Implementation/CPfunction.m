function [CPfunction_out] = CPfunction(CP, lambda)
% Paremeterized Cp(lambda) function for a fixed pitch angle. Circumvents the need of importing a look-up table
  
    % Inputs
%     REAL(8), INTENT(IN) :: CP(4)    % Parameters defining the parameterizable Cp(lambda) function
%     REAL(8), INTENT(IN) :: lambda    % Estimated or measured tip-speed ratio input

    % Lookup
    CPfunction_out = exp(-CP(1)/lambda)*(CP(2)/lambda-CP(3))+CP(4)*lambda;
    CPfunction_out = saturate(CPfunction_out, 0.001, 1.0);

end