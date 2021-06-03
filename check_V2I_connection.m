function isConnected = check_V2I_connection(vehicles, config, locRSU, locCV, cvLane)

% FUNCTION TO CHECK IF A GIVEN COMMUNICATING VEHICLE IS CONNECTED TO AT
% LEAST ONE RSU.

isConnected = 0;
locRSUprojected = compute_RSU_projections(locRSU,cvLane,locCV);

blockingLanes = 1:cvLane-1;
criticalHeightVec = (config.heightRSU - config.heightAntenna) .* (cvLane-blockingLanes) ./ (cvLane - 0.5) + config.heightAntenna; %triangle similarity

RSU_loc_3d = [locRSU', zeros(length(locRSU),1), config.heightRSU*ones(length(locRSU),1)];
CV_loc_3d = [locCV, (cvLane-1/2)*config.laneWidth, config.heightAntenna];
coveredRSUindices = find(compute_3d_distance(RSU_loc_3d, CV_loc_3d) <= config.LOSrange);
if length(coveredRSUindices) > config.numRSU % if more than numRSU in the range (due to numerical reasons), eliminate the furthest away one
    coveredRSUindices = coveredRSUindices(2:end);
end

for rsuIdx = coveredRSUindices' % go over all RSUs in the coverage range
    isRSUblocked = 0;
    for laneIdx = blockingLanes % go over each blocking lane
        maxHeight = max(vehicles{laneIdx}.height); % maximum height in this lane
        if maxHeight >= criticalHeightVec(laneIdx) % check only if any vehicle can block the LOS link
            blockingVehicleIndex = find(vehicles{laneIdx}.startPositions < locRSUprojected(laneIdx,rsuIdx), 1, 'last'); % find the closest vehicle
            if ((vehicles{laneIdx}.endPositions(blockingVehicleIndex) > locRSUprojected(laneIdx,rsuIdx)) && (vehicles{laneIdx}.height(blockingVehicleIndex) >= criticalHeightVec(laneIdx)))
                isRSUblocked = 1;
                break;
            end
        end
    end
    if ~isRSUblocked % if RSU is not blocked, store coverage state and exit
        isConnected = 1;
        break;
    end
end