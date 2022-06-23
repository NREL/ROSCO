function [interp_value] = interp1d(xData, yData, xq)
    % interp1d 1-D interpolation (table lookup), xData should be monotonically increasing

        % Inputs
%         REAL(8), DIMENSION(:), INTENT(IN)       :: xData        % Provided x data (vector), to be interpolated
%         REAL(8), DIMENSION(:), INTENT(IN)       :: yData        % Provided y data (vector), to be interpolated
%         REAL(8), INTENT(IN)                     :: xq           % x-value for which the y value has to be interpolated
%         INTEGER(4)                              :: I            % Iteration index
        

%         % ROSCO-V2.4 Patch
%         
%         % Catch Errors
%         % Are xData and yData the same size?
%         if (size(xData) ~= size(yData)) 
%             fprintf('interp1d: xData and yData are not the same size')
%         end

        % Is xData non decreasing
        for I_DIFF = 1 : size(xData) - 1
            if (xData(I_DIFF + 1) - xData(I_DIFF) <= 0) 
                fprintf('interp1d: xData is not strictly increasing')
%                 return
            end
        end

        % Interpolate
        if (xq <= min(xData))
            interp_value = yData(1);
        elseif (xq >= max(xData))
            interp_value = yData(end);
        else
            for I = 1:length(xData)
                if (xq <= xData(I))
                    interp_value = yData(I-1) + (yData(I) - yData(I-1))/(xData(I) - xData(I-1))*(xq - xData(I-1));
                    return
                else
                    continue
                end
            end
        end
        
end