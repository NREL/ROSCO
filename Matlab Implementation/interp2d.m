function [interp_value] = interp2d(xData, yData, zData, xq, yq)
%     % interp2d 2-D interpolation (table lookup). Query done using bilinear interpolation. 
%     % Note that the interpolated matrix with associated query vectors may be different than "standard", - zData should be formatted accordingly
%     % - xData follows the matrix from left to right
%     % - yData follows the matrix from top to bottom
%     % A simple case: xData = [1 2 3], yData = [4 5 6]
%     %        | 1    2   3
%     %       -------------
%     %       4| a    b   c
%     %       5| d    e   f
%     %       6| g    H   i
% 
%         IMPLICIT NONE
%         % Inputs
%         REAL(8), DIMENSION(:),   INTENT(IN)     :: xData        % Provided x data (vector), to find query point (should be monotonically increasing)
%         REAL(8), DIMENSION(:),   INTENT(IN)     :: yData        % Provided y data (vector), to find query point (should be monotonically increasing)
%         REAL(8), DIMENSION(:,:), INTENT(IN)     :: zData        % Provided z data (vector), to be interpolated
%         REAL(8),                 INTENT(IN)     :: xq           % x-value for which the z value has to be interpolated
%         REAL(8),                 INTENT(IN)     :: yq           % y-value for which the z value has to be interpolated
%         % Allocate variables
%         INTEGER(4)                              :: i            % Iteration index & query index, x-direction
%         INTEGER(4)                              :: ii           % Iteration index & second que .  ry index, x-direction
%         INTEGER(4)                              :: j            % Iteration index & query index, y-direction
%         INTEGER(4)                              :: jj           % Iteration index & second query index, y-direction
%         REAL(8), DIMENSION(2,2)                 :: fQ           % zData value at query points for bilinear interpolation            
%         REAL(8), DIMENSION(1)                   :: fxy           % Interpolated z-data point to be returned
%         REAL(8)                                 :: fxy1          % zData value at query point for bilinear interpolation            
%         REAL(8)                                 :: fxy2          % zData value at query point for bilinear interpolation            
%         
        % ---- Find corner indices surrounding desired interpolation point -----
            % x-direction
        if (xq <= min(xData))        % On lower x-bound, just need to find zData(yq)
            j = 1;
            jj = 1;
            interp_value = interp1d(yData,zData(:,j),yq);
            return
        elseif (xq >= max(xData))    % On upper x-bound, just need to find zData(yq)
            j = length(xData);
            jj = length(xData);
            interp_value = interp1d(yData,zData(:,j),yq);
            return
        else
            for j = 1:length(xData)            
                if (xq == xData(j))  % On axis, just need 1d interpolation
                    jj = j;
                    interp_value = interp1d(yData,zData(:,j),yq);
                    return
                elseif (xq < xData(j)) 
                    jj = j;
                    break
                end
            end
        end
        j = j-1; % Move j back one
            % y-direction
        if (yq <= min(yData))        % On lower y-bound, just need to find zData(xq)
            i = 1;
            ii = 1;
            interp_value = interp1d(xData,zData(i,:),xq);
            return
        elseif (yq >= max(yData))    % On upper y-bound, just need to find zData(xq)
            i = length(yData);
            ii = length(yData);
            interp_value = interp1d(xData,zData(i,:),xq);
            return
        else
            for i = 1:length(yData)
                if (yq == yData(i))     % On axis, just need 1d interpolation
                    ii = i;
                    interp_value = interp1d(xData,zData(i,:),xq);
                    return
                elseif (yq < yData(i)) 
                    ii = i;
                    break
                end
            end
        end
        i = i-1; % move i back one
        
        % ---- Do bilinear interpolation ----
        % Find values at corners 
        fQ(1,1) = zData(i,j);
        fQ(2,1) = zData(ii,j);
        fQ(1,2) = zData(i,jj);
        fQ(2,2) = zData(ii,jj);
        % Interpolate
        fxy1 = (xData(jj) - xq)/(xData(jj) - xData(j))*fQ(1,1) + (xq - xData(j))/(xData(jj) - xData(j))*fQ(1,2);
        fxy2 = (xData(jj) - xq)/(xData(jj) - xData(j))*fQ(2,1) + (xq - xData(j))/(xData(jj) - xData(j))*fQ(2,2);
        fxy = (yData(ii) - yq)/(yData(ii) - yData(i))*fxy1 + (yq - yData(i))/(yData(ii) - yData(i))*fxy2;

        interp_value = fxy;

end