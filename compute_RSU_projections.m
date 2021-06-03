function locRSUprojected = compute_RSU_projections(locRSU,cvLane,locCv)

% RSU projections, from lane 1 to lane ii-1
jj = 1:cvLane-1;
locRSUprojected = (locRSU' * (cvLane-jj) + locCv(1)* (jj-0.5))./(cvLane-0.5);
locRSUprojected = locRSUprojected.';
end