function v2v_projection = compute_V2V_projections(locCV, locRelay, cvLane, relayLane, config, vehicleProperties)

% FUNCTION TO COMPUTE V2V LINK PROJECTION POINTS ON EACH LANE IN BETWEEN
% THE CV AND THE RELAY. RETURNS A VECTOR OF SIZE (cvLane-relayLane+1).

if cvLane == relayLane
    error('CV lane index should be larger than the relay lane index.')
end

laneWidth = config.laneWidth;
vehicleWidth = vehicleProperties.width(1);
indexVector = (relayLane+1):(cvLane-1);
v2v_projection_centers = locRelay + (locCV - locRelay) .* (indexVector - relayLane) ./ (cvLane - relayLane);

y = locCV-locRelay;
x = y*vehicleWidth/(2*laneWidth*(cvLane - relayLane));

v2v_projection_centers = [locRelay v2v_projection_centers locCV];
v2v_projection = nan(length(v2v_projection_centers),2); % start and end points of blockage zones
v2v_projection(1,:) = sort([locRelay locRelay+x]);
v2v_projection(end,:) = sort([locCV-x, locCV]);
for lIdx = 2:(length(v2v_projection_centers)-1)
    v2v_projection(lIdx,:) = sort([v2v_projection_centers(lIdx)-x, v2v_projection_centers(lIdx)+x]);
end