function [isConnected, isV2Vfound] = check_VehicularRelay_connection(vehicles, config, vehicleProperties, locCV, locRSU)

% FUNCTION TO CHECK IF A VEHICULAR RELAY CONNECTED TO AN RSU (V2V+V2I LINK)
% CAN BE FOUND.

numLanes = config.numLane;
Rlos = config.LOSrange;
cvLane = config.CVlane;

isConnected = 0;
isV2Vfound = 0;

CV_loc_3d = [locCV, (cvLane-1/2)*config.laneWidth, config.heightAntenna];

for laneIdx = numLanes:-1:1 % go over each lane for connectivity
    switch laneIdx
        case cvLane % checking the connections in the same lane
            vehicleToLeft = find(vehicles{laneIdx}.antennaPositions<locCV,1,'last'); % find the closest vehicle on the left
            for vehicleIdx = vehicleToLeft:-1:1 % move to the left
                antennaPos = vehicles{laneIdx}.antennaPositions(vehicleIdx);
                distance = locCV - antennaPos;
                if(distance > Rlos) % out of coverage, stop checking
                    break;
                elseif(vehicles{laneIdx}.height(vehicleIdx) > config.heightAntenna) % if the vehicle is blocking, stop checking
                    break;
                elseif(vehicles{laneIdx}.isCommunicating(vehicleIdx) == 1) % if the vehicle is CV, check for V2I connection
                    isV2Vfound = 1;
                    isRelayV2Iconnected = check_V2I_connection(vehicles, config, locRSU, antennaPos, cvLane);
                    if isRelayV2Iconnected % if relay is V2I connected, break and save connectivity
                        isConnected = 1;
                        break;
                    end
                end
            end
            if ~isConnected % check the other direction only if not connected
                vehicleToRight = find(vehicles{laneIdx}.antennaPositions>locCV,1,'first'); % find the closest vehicle on the right
                for vehicleIdx = vehicleToRight:length(vehicles{laneIdx}.antennaPositions) % move to the right
                    antennaPos = vehicles{laneIdx}.antennaPositions(vehicleIdx);
                    distance = antennaPos - locCV;
                    if(distance > Rlos) % out of coverage, stop checking
                        break;
                    elseif(vehicles{laneIdx}.height(vehicleIdx) > config.heightAntenna) % if the vehicle is blocking, stop checking
                        break;
                    elseif(vehicles{laneIdx}.isCommunicating(vehicleIdx) == 1) % if the vehicle is CV, check for V2I connection
                        isV2Vfound = 1;
                        isRelayV2Iconnected = check_V2I_connection(vehicles, config, locRSU, antennaPos, cvLane);
                        if isRelayV2Iconnected % if relay is V2I connected, break and save connectivity
                            isConnected = 1;
                            break;
                        end
                    end
                end
            end
        otherwise % check the potential relays in other lanes
            antennaPositionVec = vehicles{laneIdx}.antennaPositions;
            antenna_loc_3d = [antennaPositionVec', (laneIdx-1/2)*config.laneWidth*ones(length(antennaPositionVec),1), config.heightAntenna*ones(length(antennaPositionVec),1)];
            distances_3d = compute_3d_distance(antenna_loc_3d,CV_loc_3d);
            relayVehicleIndices = find((distances_3d' < config.LOSrange) & (vehicles{laneIdx}.isCommunicating == 1)); % find the relay indices within LOS coverage range
            for vehicleIdx = relayVehicleIndices % go over each potential relay
                antennaPos = antennaPositionVec(vehicleIdx);
                v2v_projection = compute_V2V_projections(locCV, antennaPos, cvLane, laneIdx, config, vehicleProperties);
                isRelayConnected = 1;
                blockingLaneIndices = laneIdx:cvLane;
                for bIdx = 1:length(blockingLaneIndices) % go over all lanes potentially blocking the V2V link
                    blockingLane = blockingLaneIndices(bIdx);
                    v2v_projections_this_lane = v2v_projection(bIdx,:); % locations of the V2V link projection on this lane
                    
                    firstVehicleToLeft = find(vehicles{blockingLane}.endPositions < v2v_projections_this_lane(1), 1, 'last') + 1;
                    firstVehicleToRight = find(vehicles{blockingLane}.startPositions > v2v_projections_this_lane(end), 1, 'first') - 1;
                    allBlockingVehicles = firstVehicleToLeft:firstVehicleToRight;
                    if any(vehicles{blockingLane}.height(allBlockingVehicles) > config.heightAntenna)
                        isRelayConnected = 0;
                        break;
                    end
                    
                end
                if isRelayConnected % if relay is connected via V2V, check V2I connection
                    isV2Vfound = 1;
                    isRelayConnected = check_V2I_connection(vehicles, config, locRSU, antennaPos, laneIdx);
                    if isRelayConnected % if the relay is not blocked (both V2V and V2I), store coverage and exit
                        isConnected = 1;
                        break; % no need to check other relays
                    end
                end
            end
    end
    if isConnected % if vehicle is connected, break and save connectivity
        break;
    end
end