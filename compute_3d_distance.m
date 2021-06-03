function distance3d = compute_3d_distance(A,B)

% FUNCTION TO COMPUTE THE DISTANCE BETWEEN TWO POINTS IN 3D. BOTH X AND Y
% ARE Nx3 MATRICES, WHERE N IS THE NUMBER OF POINT PAIRS, AND DIMENSIONS
% ARE X,Y,Z. 

if (size(A,2)~=3 || size(B,2)~=3)
    error('Both matrices should have three columns, specifying the locations on x, y, and z coordinates.')
elseif (size(A,1) > 1 && size(B,1) == 1) % replicate B and compute 3D distances
    B_temp = repmat(B,size(A,1),1);
    distance3d = sqrt(sum((A-B_temp).^2,2));
elseif(size(B,1) > 1 && size(A,1) == 1) % replicate A and compute 3D distances
    A_temp = repmat(A,size(B,1),1);
    distance3d = sqrt(sum((A_temp-B).^2,2));
elseif(size(B,1) ~= size(A,1))
    error('Both matrices should have either one or the same number of rows.')
else
    distance3d = sqrt(sum((A-B).^2,2));
end
    