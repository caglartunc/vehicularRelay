function vehicles = generate_vehicles(config, vehicleProperties, numVehiclesPerLane)
numLane = config.numLane;

vehicles = cell(1,numLane);
distanceVec = exprnd(1/config.s_average,numLane,numVehiclesPerLane);

vehicleProbVec = cumsum(vehicleProperties.probability);
H = rand(numLane, numVehiclesPerLane);
vehicleHeights = zeros(numLane,numVehiclesPerLane);
vehicleLengths = zeros(numLane,numVehiclesPerLane);
vehicleWidths = zeros(numLane,numVehiclesPerLane);
vehicleSpeeds = zeros(numLane,numVehiclesPerLane);
isVehicleCV = zeros(numLane,numVehiclesPerLane);
modifiedProbVec = [0 vehicleProbVec];
for vInd = 1:length(vehicleProbVec)
    H_temp = (H <= modifiedProbVec(vInd+1)) & (H >= modifiedProbVec(vInd));
    vehicleHeights = vehicleHeights + vehicleProperties.height(vInd)*H_temp;
    vehicleLengths = vehicleLengths + H_temp.*exprnd(vehicleProperties.length(vInd),numLane, numVehiclesPerLane);
    vehicleWidths = vehicleWidths + vehicleProperties.width(vInd)*(H_temp.*ones(numLane, numVehiclesPerLane));
    vehicleSpeeds = vehicleSpeeds + vehicleProperties.speed_ms(vInd)*H_temp;
    commProb = vehicleProperties.commProbability(vInd); % probability of communication
    H_temp = (H <= vehicleProbVec(vInd)*commProb);
    isVehicleCV = isVehicleCV + H_temp;
end

shiftedSum = distanceVec + [zeros(numLane,1) vehicleLengths(:,1:end-1)];
vehicleStartPositions = cumsum(shiftedSum,2); % starting positions of the vehicles on each lane
vehicleEndPositions = vehicleStartPositions + vehicleLengths; % starting positions of the vehicles on each lane
vehicleAntennaPositions = (vehicleStartPositions + vehicleEndPositions)./2;

for n = 1:numLane
    vehicles{n}.startPositions = vehicleStartPositions(n,:);
    vehicles{n}.endPositions = vehicleEndPositions(n,:);
    vehicles{n}.antennaPositions = vehicleAntennaPositions(n,:);
    vehicles{n}.isCommunicating = isVehicleCV(n,:);
    vehicles{n}.height = vehicleHeights(n,:);
    vehicles{n}.length = vehicleLengths(n,:);
    vehicles{n}.width = vehicleWidths(n,:);
    vehicles{n}.speed = vehicleSpeeds(n,:);
end



