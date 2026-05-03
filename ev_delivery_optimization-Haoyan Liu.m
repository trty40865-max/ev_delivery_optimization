function ev_delivery_optimization()

rng('shuffle');

csvfile = '3.zip.csv';
expected_N = 300;

vehicle_speed = 10;
B_max = 42;
B_init_ratio = 0.8;
B_init = B_max * B_init_ratio;
charge_rate_kw = 40;
battery_consume_base = 0.2;
load_coeff = 0.05;
min_charge_threshold = 0.2;

alpha_base = 1.0; beta_base = 3.0;
self_pick_penalty_coeff = 0.5;
base_dispersion_weight = 0.5;
redundancy_penalty = 0.2;
max_dispersion_penalty = 30;

c_d = 3.0; c_t = 1.8; alpha = 1.0; beta = 3.0;
F_s = 200; U_v = 250;

time_windows = {
    [9,0,11,0,8,0,12,0];
    [9,0,11,0,8,0,12,0; 12,0,14,0,11,0,15,0];
    [9,0,11,0,8,0,12,0; 12,0,14,0,11,0,15,0; 14,0,16,0,13,0,17,0];
    [9,0,11,0,8,0,12,0; 12,0,14,0,11,0,15,0; 14,0,16,0,13,0,17,0; 16,0,18,0,15,0,19,0];
    [9,0,11,0,8,0,12,0; 12,0,14,0,11,0,15,0; 14,0,16,0,13,0,17,0; 16,0,18,0,15,0,19,0; 18,0,20,0,17,0,21,0];
};

tolerance_penalty_coeff = 1.5;
intolerance_penalty_coeff = 5.0;

useCSV = false;
if isfile(csvfile)
    T = readtable(csvfile);
    vars = T.Properties.VariableNames;
    if any(strcmpi(vars,'X')) && any(strcmpi(vars,'Y'))
        lon_all = double(T.('X')); lat_all = double(T.('Y')); useCSV = true;
    elseif any(strcmpi(vars,'lon')) && any(strcmpi(vars,'lat'))
        lon_all = double(T.('lon')); lat_all = double(T.('lat')); useCSV = true;
    else
        warning('CSV 不含 X/Y 或 lon/lat 列，回退使用网格坐标（km）。');
        useCSV = false;
    end
end

if useCSV
    mask = isfinite(lon_all) & isfinite(lat_all);
    lon_all = lon_all(mask); lat_all = lat_all(mask);
    npts = numel(lon_all);
    if npts == 0, error('CSV 无有效经纬度点'); end
    N_demand = max(1, min(300, min(expected_N, npts)));
    lon_use = lon_all(1:N_demand); lat_use = lat_all(1:N_demand);
    lon0 = mean(lon_use); lat0 = mean(lat_use);
    lat0_rad = deg2rad(lat0);
    R = 6371000;
    x_m = (lon_use - lon0) * (pi/180) * R * cos(lat0_rad);
    y_m = (lat_use - lat0) * (pi/180) * R;
    demandXY_km = [x_m, y_m] / 1000;
    demandLonLat = [lon_use, lat_use];
    
outTbl = table(demandLonLat(:,1), demandLonLat(:,2), 'VariableNames', {'lon','lat'});
writetable(outTbl, 'map.csv');
fprintf('Saved %d lon/lat points to map.csv\n', size(demandLonLat,1));

if exist('newXY_km','var')
    x_m_new = newXY_km(:,1) * 1000;
    y_m_new = newXY_km(:,2) * 1000;
    lon_from_km = lon0 + (x_m_new) ./ (R * cos(lat0_rad)) * (180/pi);
    lat_from_km = lat0 + (y_m_new) ./ R * (180/pi);
    writetable(table(lon_from_km, lat_from_km, 'VariableNames', {'lon','lat'}), 'map.csv');
    fprintf('Saved %d lon/lat points converted from newXY_km to map.csv\n', numel(lon_from_km));
end
    depot_xy_km = [-2,-1.5];
    lon_min = min(lon_use); lon_max = max(lon_use);
    lat_min = min(lat_use); lat_max = max(lat_use);
    lon_buffer = (lon_max - lon_min) * 0.1;
    lat_buffer = (lat_max - lat_min) * 0.1;
    cs_lonlat = [
        lon_min - lon_buffer, (lat_min + lat_max)/2;
        (lon_min + lon_max)/2, lat_max + lat_buffer;
        lon_max + lon_buffer, (lat_min + lat_max)/2;
        (lon_min + lon_max)/2, lat_min - lat_buffer;
    ];
    cs_x = (cs_lonlat(:,1)-lon0)*(pi/180)*R*cos(lat0_rad)/1000;
    cs_y = (cs_lonlat(:,2)-lat0)*(pi/180)*R/1000;
    chargeStations = [cs_x, cs_y];

tryRouteNames = {'optimalRoute','bestRoute','routes','rhRoute','routeCoords','route_xy_km','finalRoute'};

if ~exist('lon0','var') && exist('lon_use','var')
    lon0 = mean(lon_use);
    lat0 = mean(lat_use);
    lat0_rad = deg2rad(lat0);
    R = 6371000;
end

if exist('demandLonLat','var') && ~isempty(demandLonLat)
    demand_lon = demandLonLat(:,1); demand_lat = demandLonLat(:,2);
elseif exist('lon_use','var') && exist('lat_use','var')
    demand_lon = lon_use; demand_lat = lat_use;
elseif exist('demandXY_km','var') && exist('lon0','var')
    x_m = demandXY_km(:,1)*1000; y_m = demandXY_km(:,2)*1000;
    demand_lon = lon0 + (x_m) ./ (R * cos(lat0_rad)) * (180/pi);
    demand_lat = lat0 + (y_m_new) ./ R * (180/pi);
else
    error('无法找到需求点的坐标，请确保存在 demandLonLat 或 lon_use/lat_use 或 demandXY_km');
end

cs_lonlat_exist = exist('cs_lonlat','var') && ~isempty(cs_lonlat);
cs_km_exist = exist('chargeStations','var') && ~isempty(chargeStations);
if cs_lonlat_exist
    cs_lon = cs_lonlat(:,1); cs_lat = cs_lonlat(:,2);
elseif cs_km_exist && exist('lon0','var')
    x_m = chargeStations(:,1)*1000; y_m = chargeStations(:,2)*1000;
    cs_lon = lon0 + (x_m) ./ (R * cos(lat0_rad)) * (180/pi);
    cs_lat = lat0 + (y_m) ./ R * (180/pi);
else
    cs_lon = []; cs_lat = [];
end

depot_lon = []; depot_lat = [];
if exist('depot_lon','var') && exist('depot_lat','var')
elseif exist('depot_xy_km','var') && exist('lon0','var')
    x_m = depot_xy_km(1)*1000; y_m = depot_xy_km(2)*1000;
    depot_lon = lon0 + (x_m) ./ (R * cos(lat0_rad)) * (180/pi);
    depot_lat = lat0 + (y_m) ./ R * (180/pi);
end

routeStructs = {};
for i=1:numel(tryRouteNames)
    name = tryRouteNames{i};
    if evalin('base', sprintf('exist(''%s'',''var'')', name))
        routeVar = evalin('base', name);
        routeStructs{end+1} = struct('name', name, 'data', routeVar);
    end
end
w = evalin('base','whos');
for i=1:numel(w)
    wn = w(i).name;
    if any(strcmp(wn, tryRouteNames)), continue; end
    if contains(lower(wn), 'route') || contains(lower(wn),'path')
        routeVar = evalin('base', wn);
        routeStructs{end+1} = struct('name', wn, 'data', routeVar);
    end
end

hasGeo = exist('geoscatter','file')==2;

if hasGeo
    figure;
    geoscatter(demand_lat, demand_lon, 36, 'filled');
    hold on;
    title('Demand / Charging Stations / Depot / Routes (geographic)');
    try
        geobasemap('streets');
    catch
    end
    if ~isempty(cs_lon)
        geoscatter(cs_lat, cs_lon, 80, 's', 'MarkerFaceColor','blue');
    end
    if exist('depot_lon','var') && ~isempty(depot_lon)
        geoscatter(depot_lat, depot_lon, 120, 'p', 'MarkerFaceColor','red');
    end
    for k=1:numel(routeStructs)
        r = routeStructs{k}.data;
        if isempty(r), continue; end
        if isvector(r) && all(r==floor(r)) && all(r>=1) && max(r) <= numel(demand_lon)
            lon_r = demand_lon(r); lat_r = demand_lat(r);
            geoplot(lat_r, lon_r, '-','LineWidth',1.8);
            continue;
        end
        if ismatrix(r) && size(r,2)==2
            if all(abs(r(:,1))<=180) && all(abs(r(:,2))<=90)
                lon_r = r(:,1); lat_r = r(:,2);
                geoplot(lat_r, lon_r, '-','LineWidth',1.8);
            else
                x_m = r(:,1)*1000; y_m = r(:,2)*1000;
                lon_r = lon0 + (x_m) ./ (R * cos(lat0_rad)) * (180/pi);
                lat_r = lat0 + (y_m) ./ R * (180/pi);
                geoplot(lat_r, lon_r, '-','LineWidth',1.8);
            end
            continue;
        end
        if iscell(r)
            for c=1:numel(r)
                rc = r{c};
                if isvector(rc) && all(rc==floor(rc)) && max(rc) <= numel(demand_lon)
                    lon_r = demand_lon(rc); lat_r = demand_lat(rc);
                    geoplot(lat_r, lon_r, '-','LineWidth',1.2);
                elseif ismatrix(rc) && size(rc,2)==2
                    if all(abs(rc(:,1))<=180) && all(abs(rc(:,2))<=90)
                        lon_r = rc(:,1); lat_r = rc(:,2);
                        geoplot(lat_r, lon_r, '-','LineWidth',1.2);
                    else
                        x_m = rc(:,1)*1000; y_m = rc(:,2)*1000;
                        lon_r = lon0 + (x_m) ./ (R * cos(lat0_rad)) * (180/pi);
                        lat_r = lat0 + (y_m) ./ R * (180/pi);
                        geoplot(lat_r, lon_r, '-','LineWidth',1.2);
                    end
                end
            end
            continue;
        end
    end
    legendEntries = {'Demand'};
    if ~isempty(cs_lon), legendEntries{end+1} = 'Charging Station'; end
    if exist('depot_lon','var') && ~isempty(depot_lon), legendEntries{end+1} = 'Depot'; end
    legend(legendEntries,'Location','best');
    hold off;
else
    figure;
    x_m = (demand_lon - lon0) * (pi/180) * R * cos(lat0_rad);
    y_m = (demand_lat - lat0) * (pi/180) * R;
    demand_km = [x_m, y_m]/1000;
    scatter(demand_km(:,1), demand_km(:,2), 36, 'filled'); hold on;
    title('Demand / Charging Stations / Depot / Routes (local km projection)');
    xlabel('x (km)'); ylabel('y (km)'); axis equal;
    if cs_km_exist
        scatter(chargeStations(:,1), chargeStations(:,2), 80, 's', 'filled');
    elseif ~isempty(cs_lon)
        x_m = (cs_lon - lon0) * (pi/180) * R * cos(lat0_rad);
        y_m = (cs_lat - lat0) * (pi/180) * R;
        scatter(x_m/1000, y_m/1000, 80, 's', 'filled');
    end
    if exist('depot_xy_km','var')
        scatter(depot_xy_km(1), depot_xy_km(2), 120, 'p', 'filled');
    elseif exist('depot_lon','var') && ~isempty(depot_lon)
        x_m = (depot_lon - lon0) * (pi/180) * R * cos(lat0_rad);
        y_m = (depot_lat - lat0) * (pi/180) * R;
        scatter(x_m/1000, y_m/1000, 120, 'p', 'filled');
    end
    for k=1:numel(routeStructs)
        r = routeStructs{k}.data;
        if isempty(r), continue; end
        if isvector(r) && all(r==floor(r)) && max(r) <= size(demand_km,1)
            plot(demand_km(r,1), demand_km(r,2), '-','LineWidth',1.8);
            continue;
        end
        if ismatrix(r) && size(r,2)==2
            if all(abs(r(:,1))<=180) && all(abs(r(:,2))<=90)
                x_m = (r(:,1) - lon0) * (pi/180) * R * cos(lat0_rad);
                y_m = (r(:,2) - lat0) * (pi/180) * R;
                rk = [x_m,y_m]/1000;
                plot(rk(:,1), rk(:,2), '-','LineWidth',1.8);
            else
                plot(r(:,1), r(:,2), '-','LineWidth',1.8);
            end
            continue;
        end
        if iscell(r)
            for c=1:numel(r)
                rc = r{c};
                if isvector(rc) && all(rc==floor(rc)) && max(rc) <= size(demand_km,1)
                    plot(demand_km(rc,1), demand_km(rc,2), '-','LineWidth',1.2);
                elseif ismatrix(rc) && size(rc,2)==2
                    if all(abs(rc(:,1))<=180) && all(abs(rc(:,2))<=90)
                        x_m = (rc(:,1) - lon0) * (pi/180) * R * cos(lat0_rad);
                        y_m = (rc(:,2) - lat0) * (pi/180) * R;
                        plot(x_m/1000, y_m/1000, '-','LineWidth',1.2);
                    else
                        plot(rc(:,1), rc(:,2), '-','LineWidth',1.2);
                    end
                end
            end
            continue;
        end
    end
    hold off;
end

fprintf('Visualization complete. Mapped %d demand points, %d charging stations, %d route candidates.\n', ...
    numel(demand_lon), numel(cs_lon), numel(routeStructs));
end

numPickup = 3;
numPickup = max(1, numPickup);

twSet = time_windows{3};
pickup_time_windows = cell(1,numPickup);
for i=1:numPickup
    tw = twSet(i,:);
    pickup_time_windows{i} = struct('early_h',tw(1),'early_m',tw(2),'late_h',tw(3),'late_m',tw(4),...
        'elastic_early_h',tw(5),'elastic_early_m',tw(6),'elastic_late_h',tw(7),'elastic_late_m',tw(8),...
        'tolerance_pen',tolerance_penalty_coeff,'intolerance_pen',intolerance_penalty_coeff);
end

qvals = struct('normal',1,'frozen',1,'double',2);
num_normal = max(1, randi([1, max(1,N_demand-2)]));
remaining = N_demand - num_normal;
num_frozen = max(1, randi([1, max(1, remaining-1)]));
num_double = N_demand - num_normal - num_frozen;
temp_idx_list = [ones(num_normal,1); 2*ones(num_frozen,1); 3*ones(num_double,1)];
temp_idx_list = temp_idx_list(randperm(N_demand));
demand_q = zeros(N_demand,1);
demand_q(temp_idx_list==1) = qvals.normal;
demand_q(temp_idx_list==2) = qvals.frozen;
demand_q(temp_idx_list==3) = qvals.double;

normalCoords = demandXY_km(temp_idx_list==1,:);
frozenCoords = demandXY_km(temp_idx_list==2,:);
doubleCoords = demandXY_km(temp_idx_list==3,:);

fprintf('读取/生成 %d 个客户 (km 单位)。普通=%d 冷冻=%d 双=%d\n', N_demand, sum(temp_idx_list==1), sum(temp_idx_list==2), sum(temp_idx_list==3));

if numPickup == 1
    map_center = mean(demandXY_km,1);
    [~, bestPickupIdx] = min(vecnorm(demandXY_km - map_center, 2, 2));
    bestPickupIdx = bestPickupIdx(:);
else
    [candIdx, ~] = bacoSelectPickups(demandXY_km, numPickup, struct('alpha',alpha_base,'beta',beta_base), ...
        self_pick_penalty_coeff, 0.5, base_dispersion_weight, max_dispersion_penalty);
    candIdx = candIdx(:);

    min_sep_km = 0.8;
    if numel(candIdx) < numPickup
        more = setdiff(1:size(demandXY_km,1), candIdx);
        randAdd = more(randperm(numel(more), numPickup - numel(candIdx)));
        candIdx = [candIdx; randAdd(:)];
    end

    bestPickupIdx = enforce_dispersion_indices(demandXY_km, candIdx, numPickup, min_sep_km);
end

initPickupPoints = demandXY_km(bestPickupIdx,:);
initPickupPoints = adjustPickupPoints(initPickupPoints, demandXY_km, 0.1);

if size(initPickupPoints,1) ~= numPickup
    if size(initPickupPoints,1) > numPickup
        initPickupPoints = initPickupPoints(1:numPickup,:);
    else
        addN = numPickup - size(initPickupPoints,1);
        addIdx = randperm(N_demand, addN);
        initPickupPoints = [initPickupPoints; demandXY_km(addIdx,:)];
    end
end

[assignment, ~] = bacoAssignCustomers(demandXY_km, initPickupPoints, temp_idx_list, struct('alpha',alpha_base,'beta',beta_base), self_pick_penalty_coeff);
assignment = clampAssignment(assignment, numPickup);

vehicles_struct = struct();
vehicles_struct(1).battery_init = B_init;
vehicles_struct(1).battery_cap = B_max;
vehicles_struct(1).speed = vehicle_speed;
vehicles_struct(1).consume_base = battery_consume_base;
vehicles_struct(1).load_coeff = load_coeff;
vehicles_struct(1).cap_normal = 100; vehicles_struct(1).cap_frozen = 50;
vehicles_struct(1).min_charge_threshold = min_charge_threshold;

allNodes = [depot_xy_km; initPickupPoints];
[giantTour, ~] = osmmasGenerateGiantTour(allNodes, alpha_base*1.2, beta_base*0.8);
distMat_nodes = pdist2(allNodes, allNodes);
giantTour = threeOptPerm(giantTour, distMat_nodes, 60, redundancy_penalty);

pickupDemand = zeros(numPickup,1);
pickupDemand_normal = zeros(numPickup,1);
pickupDemand_frozen = zeros(numPickup,1);
for i=1:numPickup
    idxs = find(assignment == i);
    if ~isempty(idxs)
        pickupDemand_frozen(i) = sum(demand_q(idxs(temp_idx_list(idxs)==2)));
        pickupDemand_normal(i) = sum(demand_q(idxs(temp_idx_list(idxs)==1 | temp_idx_list(idxs)==3)));
    end
    pickupDemand(i) = pickupDemand_normal(i) + pickupDemand_frozen(i);
end

max_capacity_normal = vehicles_struct(1).cap_normal;
max_capacity_frozen = vehicles_struct(1).cap_frozen;
max_capacity = min(sum(pickupDemand)/1, (max_capacity_normal + max_capacity_frozen)/1);

[splitRoutes, pickupSeq] = splitGiantTour(giantTour, allNodes, pickupDemand, max_capacity, redundancy_penalty);
pickupSeq = clampAssignment(pickupSeq, numPickup);

if ~isempty(pickupSeq)
    pickupPoints = initPickupPoints(pickupSeq,:);
else
    pickupPoints = initPickupPoints;
end

baseRoute = [depot_xy_km; pickupPoints; depot_xy_km];
chargeStationUsage = zeros(size(chargeStations,1),1);
[rhRoute, chargeInfo] = removalHeuristic(baseRoute, chargeStations, B_max, battery_consume_base, ...
    max_range_base_km(B_max,battery_consume_base), redundancy_penalty, chargeStationUsage, ...
    B_init, min_charge_threshold);

n_rh = size(rhRoute,1);
nonDominatedStations = cell(n_rh-1,1);
for i=1:n_rh-1
    nonDominatedStations{i} = findNonDominatedStations(rhRoute(i,:), rhRoute(i+1,:), chargeStations);
end
ub = chargeInfo{2};

[optimalRoute, ~] = restrictedEnumeration(rhRoute, nonDominatedStations, ub, B_max, ...
    battery_consume_base, B_init, min_charge_threshold);

smoothedRoute = smoothPath(optimalRoute, 0.4, redundancy_penalty);

total_normal_load = sum(pickupDemand_normal);
total_frozen_load = sum(pickupDemand_frozen);
load_rate_normal = total_normal_load / vehicles_struct(1).cap_normal;
load_rate_frozen = total_frozen_load / vehicles_struct(1).cap_frozen;
avg_load_rate = mean([load_rate_normal, load_rate_frozen]);
battery_consume_dynamic = battery_consume_base + load_coeff * avg_load_rate;

[totalDist_km, totalElapsed_h, totalChargeTime_h, battery_trace, seq_log, currentChargeCount, current_vehicle_tw_penalty, frozen_charge_ratio] = ...
    simulateDelivery(smoothedRoute, chargeStations, pickupPoints, pickup_time_windows, vehicles_struct, 9, 11-9, 12-9, 0.25, battery_consume_dynamic, demand_q, temp_idx_list, assignment);

normal_cust = find(temp_idx_list==1 | temp_idx_list==3);
frozen_cust = find(temp_idx_list==2);

normal_dist_cost = 0;
if ~isempty(normal_cust)
    valid_assignments = clampAssignment(assignment(normal_cust), size(pickupPoints,1));
    normal_dist_cost = c_d * sum(vecnorm(demandXY_km(normal_cust,:) - pickupPoints(valid_assignments,:),2,2));
end

frozen_dist_cost = 0;
if ~isempty(frozen_cust)
    valid_assignments = clampAssignment(assignment(frozen_cust), size(pickupPoints,1));
    frozen_dist_cost = c_d * sum(vecnorm(demandXY_km(frozen_cust,:) - pickupPoints(valid_assignments,:),2,2));
end

time_cost = c_t * totalElapsed_h;
tw_penalty_cost = c_t * current_vehicle_tw_penalty;

redundancy_ratio = totalDist_km / (sum(vecnorm(diff(optimalRoute),2,2))+eps);
redundancy_cost = max(0, redundancy_ratio - 1.1) * redundancy_penalty * totalDist_km;

C_vr = normal_dist_cost + frozen_dist_cost + time_cost + tw_penalty_cost + redundancy_cost;

tp = temp_params_default();
normal_emis_dist = tp.Normal.e_e * normal_dist_cost * 0.1;
frozen_emis_dist = tp.Frozen.e_e * frozen_dist_cost * 0.1;
emis_charge_cost_normal = tp.Normal.e_c * totalChargeTime_h * (1 - frozen_charge_ratio);
emis_charge_cost_frozen = tp.Frozen.e_c * totalChargeTime_h * frozen_charge_ratio;
E_vr = normal_emis_dist + frozen_emis_dist + emis_charge_cost_normal + emis_charge_cost_frozen;

total_facility_cost = F_s * numPickup;
total_vehicle_fixed = U_v * 1;

total_global_cost = total_facility_cost + total_vehicle_fixed + alpha*C_vr + beta*E_vr;

fprintf('\n=== 结果摘要 ===\n');
fprintf('总里程: %.2f km | 总耗时: %.2f h | 充电次数: %d | 总充电时长: %.2f h\n', ...
    totalDist_km, totalElapsed_h, currentChargeCount, totalChargeTime_h);
fprintf('动态耗电率: %.3f kWh/km | 全局总成本: %.2f\n', battery_consume_dynamic, total_global_cost);

fprintf('\n=== 成本明细 ===\n');
fprintf('1. 配送距离成本\n');
fprintf('   - 普通客户: %.2f\n', normal_dist_cost);
fprintf('   - 冷冻客户: %.2f\n', frozen_dist_cost);
fprintf('   - 小计: %.2f\n', normal_dist_cost + frozen_dist_cost);

fprintf('\n2. 时间相关成本\n');
fprintf('   - 总耗时成本: %.2f\n', time_cost);
fprintf('   - 时间窗惩罚成本: %.2f\n', tw_penalty_cost);
fprintf('   - 小计: %.2f\n', time_cost + tw_penalty_cost);

fprintf('\n3. 冗余路径成本: %.2f\n', redundancy_cost);
fprintf('\n4. 路径优化总成本（C_vr）: %.2f\n', C_vr);

fprintf('\n5. 碳排放相关成本\n');
fprintf('   - 普通客户配送碳排放: %.2f\n', normal_emis_dist);
fprintf('   - 冷冻客户配送碳排放: %.2f\n', frozen_emis_dist);
fprintf('   - 普通客户充电碳排放: %.2f\n', emis_charge_cost_normal);
fprintf('   - 冷冻客户充电碳排放: %.2f\n', emis_charge_cost_frozen);
fprintf('   - 小计（E_vr）: %.2f\n', E_vr);

fprintf('\n6. 固定成本\n');
fprintf('   - 自提点设施成本: %.2f\n', total_facility_cost);
fprintf('   - 车辆固定成本: %.2f\n', total_vehicle_fixed);
fprintf('   - 小计: %.2f\n', total_facility_cost + total_vehicle_fixed);

fprintf('\n7. 全局总成本（含权重）: %.2f\n', total_global_cost);

figure('Position',[100 100 1200 800]); hold on; grid off; axis equal;
title(sprintf('配送路径（投影单位 km） 总里程=%.2f km', totalDist_km));

path_h = plot(smoothedRoute(:,1), smoothedRoute(:,2), '-r', 'LineWidth',2, 'DisplayName', '路径');
scatter(smoothedRoute(:,1), smoothedRoute(:,2), 30, 'r', 'filled', 'HandleVisibility','off');

normal_h = scatter(normalCoords(:,1), normalCoords(:,2), 70, 'o', 'filled', ...
    'MarkerFaceColor',[1 0.6 0.2], 'DisplayName', '普通客户');
frozen_h = scatter(frozenCoords(:,1), frozenCoords(:,2), 70, 'd', 'filled', ...
    'MarkerFaceColor',[0.2 0.6 1], 'DisplayName', '冷冻客户');
double_h = scatter(doubleCoords(:,1), doubleCoords(:,2), 70, '^', 'filled', ...
    'MarkerFaceColor',[0.7 0.3 0.8], 'DisplayName', '双需求客户');

depot_h = scatter(depot_xy_km(1), depot_xy_km(2), 180, 's', 'filled', ...
    'MarkerFaceColor',[0.3 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', '仓库');
text(depot_xy_km(1)+0.1, depot_xy_km(2)+0.1, '仓库', 'FontSize', 10);

pickup_h = scatter(pickupPoints(1,1), pickupPoints(1,2), 140, '^', 'filled', ...
    'MarkerFaceColor',[0 0.45 0.74], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', '自提点');
for i=2:size(pickupPoints,1)
    scatter(pickupPoints(i,1), pickupPoints(i,2), 140, '^', 'filled', ...
        'MarkerFaceColor',[0 0.45 0.74], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'HandleVisibility','off');
end
text(pickupPoints(i,1)+0.1, pickupPoints(i,2)+0.1, sprintf('P%d', i), 'FontSize', 10);

cs_h = scatter(chargeStations(1,1), chargeStations(1,2), 150, 'd', 'filled', ...
    'MarkerFaceColor',[0.2 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
    'DisplayName', '充电站');
for i=2:size(chargeStations,1)
    scatter(chargeStations(i,1), chargeStations(i,2), 150, 'd', 'filled', ...
        'MarkerFaceColor',[0.2 0.8 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'HandleVisibility','off');
end
text(chargeStations(i,1)+0.15, chargeStations(i,2)+0.15, sprintf('CS%d', i), 'FontSize', 10, 'FontWeight', 'bold');

xlabel('X (km)'); ylabel('Y (km)');
legend('show', 'Location','eastoutside');
hold off;

end

function p = temp_params_default()
p.Normal.c_d = 3.0; p.Normal.e_e = 0.4; p.Normal.e_c = 2.5; p.Normal.c_t = 1.8;
p.Frozen.c_d = 4.5; p.Frozen.e_e = 0.6; p.Frozen.e_c = 3.8; p.Frozen.c_t = 2.7;
end

function mr = max_range_base_km(Bmax, cons_base)
mr = Bmax / (cons_base + 1e-9);
end

function idx = findNearestPoint(query, points, tol)
if nargin<3, tol = 1e-6; end
if isempty(points), idx = []; return; end
d = vecnorm(points - query, 2, 2);
[~, mi] = min(d);
idx = mi;
end

function clamped = clampAssignment(assignment, maxIndex)
clamped = assignment;
if isempty(clamped), return; end
clamped(clamped < 1) = 1;
clamped(clamped > maxIndex) = maxIndex;
end

function adjustedPoints = adjustPickupPoints(pickupPoints, gridPoints, minDistance)
adjustedPoints = pickupPoints;
if isempty(pickupPoints) || isempty(gridPoints), return; end
n = size(pickupPoints,1);
for i=1:n
    dists = vecnorm(gridPoints - adjustedPoints(i,:), 2, 2);
    if any(dists < minDistance)
        theta = rand * 2 * pi;
        adjustedPoints(i,:) = adjustedPoints(i,:) + (minDistance + 0.01) * [cos(theta), sin(theta)];
    end
end
[C, ia] = unique(round(adjustedPoints,6),'rows');
if size(C,1) < size(adjustedPoints,1)
    for j=1:size(adjustedPoints,1)
        if ~ismember(j, ia)
            adjustedPoints(j,:) = adjustedPoints(j,:) + 0.005*randn(1,2);
        end
    end
end
end

function selectedIdx = enforce_dispersion_indices(allPoints, seedIdx, numNeeded, min_sep_km)
N = size(allPoints,1);
if isempty(seedIdx)
    seedIdx = randi(N,1,1);
end

seedIdx = unique(seedIdx(:))';
seedIdx(seedIdx < 1 | seedIdx > N) = [];
if isempty(seedIdx)
    seedIdx = randi(N,1,1);
end

selected = seedIdx(1);
for s = seedIdx(2:end)
    if numel(selected) >= numNeeded, break; end
    if ~ismember(s, selected)
        selected(end+1) = s;
    end
end

while numel(selected) < numNeeded
    unpicked = setdiff(1:N, selected);
    dmin = inf(numel(unpicked),1);
    for k=1:numel(unpicked)
        dists = vecnorm(allPoints(unpicked(k),:) - allPoints(selected,:), 2, 2);
        dmin(k) = min(dists);
    end
    [~, idm] = max(dmin);
    selected(end+1) = unpicked(idm);
end

pairwise = squareform(pdist(allPoints(selected,:)));
pairwise(pairwise==0) = inf;
minpair = min(pairwise(:));
if minpair < min_sep_km
    bestSelected = selected;
    bestMinPair = minpair;
    trials = 12;
    for t=1:trials
        s0 = randi(N,1,1);
        selT = s0;
        while numel(selT) < numNeeded
            unpicked = setdiff(1:N, selT);
            dmin = arrayfun(@(u) min(vecnorm(allPoints(u,:) - allPoints(selT,:),2,2)), unpicked);
            [~, imax] = max(dmin);
            selT(end+1) = unpicked(imax);
        end
        pw = squareform(pdist(allPoints(selT,:)));
        pw(pw==0) = inf;
        mp = min(pw(:));
        if mp > bestMinPair
            bestMinPair = mp; bestSelected = selT;
        end
    end
    selected = bestSelected;
    minpair = bestMinPair;
end

selectedIdx = selected(:);
if minpair < min_sep_km
    fprintf('警告：无法达到要求的最小间距 %.3f km，当前最大可达最小间距为 %.3f km（请减小 min_sep_km 或减少 numPickup）。\n', min_sep_km, minpair);
else
    fprintf('自提点已分散：达成最小间距 >= %.3f km（实际 %.3f km）。\n', min_sep_km, minpair);
end
end

function [bestIdx, bestCost] = bacoSelectPickups(demandCoords, numPickup, params, self_pick_penalty_coeff, min_dist, dispersion_weight, max_dispersion_penalty)
Nd = size(demandCoords,1);
if Nd <= numPickup
    bestIdx = 1:Nd; bestCost = 0; return;
end
maxIter = 30; numAnts = 18;
distMat = pdist2(demandCoords, demandCoords);
tau = ones(Nd,1);
demandDensity = arrayfun(@(x) sum(vecnorm(demandCoords - demandCoords(x,:),2,2) < 4), 1:Nd);
avgSelfPickDist = zeros(Nd,1);
for x=1:Nd
    custDists = vecnorm(demandCoords - demandCoords(x,:),2,2);
    validDists = custDists(custDists < 12);
    if isempty(validDists), avgSelfPickDist(x)=0; else avgSelfPickDist(x)=mean(validDists); end
end
eta = (1./(mean(distMat,2)+eps)) .* (demandDensity + 1) .* (1./(avgSelfPickDist + eps)).^self_pick_penalty_coeff;
bestCost = inf; bestIdx = [];

for iter=1:maxIter
    for ant=1:numAnts
        available = 1:Nd; chosen = []; dispersion_penalty = 0;
        while numel(chosen) < numPickup
            dynamicEta = eta(available);
            if ~isempty(chosen)
                for c=1:length(chosen)
                    dists = distMat(available, chosen(c));
                    dynamicEta(dists < min_dist) = dynamicEta(dists < min_dist)*0.3;
                    dynamicEta(dists >= min_dist & dists < min_dist*1.5) = dynamicEta(dists >= min_dist & dists < min_dist*1.5)*1.2;
                end
            end
            prob = (tau(available).^params.alpha) .* (dynamicEta.^params.beta);
            if sum(prob)==0, prob = ones(size(prob))/numel(prob); else prob = prob/sum(prob); end
            [~, idx] = max(prob);
            new_point = available(idx);
            chosen(end+1) = new_point;
            if length(chosen) > 1
                distsC = distMat(new_point, chosen(1:end-1));
                too_close = distsC < min_dist;
                if any(too_close)
                    raw_pen = sum(min_dist - distsC(too_close)) * 1.5;
                    dispersion_penalty = dispersion_penalty + min(raw_pen, max_dispersion_penalty/numPickup);
                end
            end
            available(idx) = [];
        end
        dists = min(distMat(:,chosen),[],2);
        selfPickDist = min(pdist2(demandCoords, demandCoords(chosen,:)),[],2);
        distance_cost = sum(dists) + self_pick_penalty_coeff * sum(selfPickDist);
        total_cost = (1-dispersion_weight)*distance_cost + dispersion_weight*dispersion_penalty;
        if total_cost < bestCost
            bestCost = total_cost; bestIdx = chosen;
        end
        tau(chosen) = tau(chosen) + 1/(total_cost + 1e-9);
    end
    tau = (1 - 0.12)*tau;
end
end

function [assignment, assignCost] = bacoAssignCustomers(customers, pickups, temp_idx, params, self_pick_penalty_coeff)
N = size(customers,1); M = size(pickups,1);
if N==0 || M==0, assignment=[]; assignCost = []; return; end
distMat = pdist2(customers, pickups);
tau = ones(N,M);
eta = 1./( (distMat + eps) .* (distMat + eps).^self_pick_penalty_coeff );
bestCost = inf; bestAssignment = ones(N,1);
maxIter = 20; numAnts = 12;
for iter=1:maxIter
    for ant=1:numAnts
        assign = zeros(N,1);
        for i=1:N
            prob = (tau(i,:).^params.alpha) .* (eta(i,:).^params.beta);
            if sum(prob)==0, prob = ones(1,M)/M; else prob = prob / sum(prob); end
            [~, j] = max(prob);
            assign(i) = j;
        end
        cost = 0;
        for j=1:M
            idx = find(assign==j);
            if ~isempty(idx)
                deliveryCost = sum(distMat(idx,j));
                selfPickCost = self_pick_penalty_coeff * sum(distMat(idx,j));
                cost = cost + deliveryCost + selfPickCost;
            end
        end
        if cost < bestCost
            bestCost = cost; bestAssignment = assign;
        end
        for i=1:N
            tau(i,bestAssignment(i)) = tau(i,bestAssignment(i)) + 1/(cost + 1e-9);
        end
    end
    tau = (1 - 0.15)*tau;
end
assignment = bestAssignment;
assignCost = zeros(M,1);
for j=1:M
    idx = find(assignment==j);
    if ~isempty(idx), assignCost(j) = sum(distMat(idx,j)); end
end
end

function [giantTour, tau] = osmmasGenerateGiantTour(nodes, alpha, beta)
n = size(nodes,1);
if n<=1, giantTour = 1:n; tau = []; return; end
rho = 0.15; tau0 = 1.0; maxIter = 40; numAnts = 15;
tau = tau0 * ones(n,n);
distMat = pdist2(nodes, nodes);
eta = 1./(distMat + 1e-6); eta(1:n+1:end)=0;
bestCost = inf; bestTour = 1:n;
for iter=1:maxIter
    allTours = cell(numAnts,1); allCosts = inf(numAnts,1);
    for ant=1:numAnts
        visited = false(1,n); current = 1; visited(current)=true; tour = current;
        while sum(visited) < n
            unvisited = find(~visited);
            prob = (tau(current, unvisited).^alpha) .* (eta(current, unvisited).^beta);
            if sum(prob)==0, prob = ones(size(prob))/numel(prob); end
            prob = prob / sum(prob);
            if rand < 0.8
                [~, idx] = max(prob);
            else
                cum = cumsum(prob); idx = find(cum>=rand,1);
            end
            next = unvisited(idx);
            tour(end+1) = next; visited(next) = true; current = next;
        end
        cost = 0; for k=1:n-1, cost = cost + distMat(tour(k), tour(k+1)); end
        cost = cost + distMat(tour(end), tour(1));
        allTours{ant} = tour; allCosts(ant) = cost;
    end
    [iterBestCost, idxBest] = min(allCosts); iterBestTour = allTours{idxBest};
    if iterBestCost < bestCost, bestCost = iterBestCost; bestTour = iterBestTour; end
    tau = (1 - rho)*tau;
    delta = 1/(iterBestCost + 1e-9);
    for k=1:n
        u = iterBestTour(k); v = iterBestTour(mod(k,n)+1);
        tau(u,v) = tau(u,v) + delta; tau(v,u) = tau(u,v);
    end
end
giantTour = bestTour;
end

function perm2 = threeOptPerm(perm, distMat, maxIterations, redundancy_penalty)
n = numel(perm); if n<=3, perm2=perm; return; end
improved = true; perm2 = perm;
bestCost = tourCost(perm2, distMat, redundancy_penalty);
iterCount = 0;
while improved && iterCount < maxIterations
    improved = false; iterCount = iterCount + 1;
    for i=1:n-2
        for j=i+1:n-1
            for k=j+1:n
                newPerms = {
                    [perm2(1:i), perm2(j+1:k), perm2(i+1:j), perm2(k+1:end)];
                    [perm2(1:i), perm2(j:k), perm2(i+1:j-1), perm2(k+1:end)];
                    [perm2(1:i), perm2(k:-1:j+1), perm2(i+1:j), perm2(k+1:end)];
                    [perm2(1:i), perm2(k:-1:j), perm2(i+1:j-1), perm2(k+1:end)];
                };
                for p=1:4
                    newCost = tourCost(newPerms{p}, distMat, redundancy_penalty);
                    if newCost < bestCost - 1e-9
                        perm2 = newPerms{p}; bestCost = newCost; improved = true; break;
                    end
                end
                if improved, break; end
            end
            if improved, break; end
        end
        if improved, break; end
    end
end
end

function cost = tourCost(perm, distMat, redundancy_penalty)
n = numel(perm); if n<=1, cost=0; return; end
base_cost = 0; for k=1:n-1, base_cost = base_cost + distMat(perm(k), perm(k+1)); end
base_cost = base_cost + distMat(perm(end), perm(1));
redundancy_cost = 0; m = mean(distMat(:));
for k=2:n-1
    prev = perm(k-1); curr = perm(k); next = perm(k+1);
    if distMat(prev,curr) < m && distMat(curr,next) < m
        redundancy_cost = redundancy_cost + redundancy_penalty * (m - (distMat(prev,curr)+distMat(curr,next))/2);
    end
end
cost = base_cost + redundancy_cost;
end

function [rhRoute, chargeInfo] = removalHeuristic(baseRoute, chargeStations, B_max, battery_consume_dynamic, max_range, redundancy_penalty, chargeStationUsage, initial_battery, min_charge_threshold)
chargeInfo = {0,0,0,0};
rhRoute = baseRoute; insertedStations = []; totalExtra = 0;
n_base = size(baseRoute,1);

current_battery = initial_battery;

for i=1:n_base-1
    node1 = rhRoute(i,:); node2 = rhRoute(i+1,:);
    distDirect = norm(node2 - node1);
    bestScore = inf; bestStation = [];
    chosenIdx = -1;
    
    direct_batt_needed = distDirect * battery_consume_dynamic;
    
    will_need_charge = (current_battery - direct_batt_needed) < (B_max * min_charge_threshold);
    
    if will_need_charge
        for s=1:size(chargeStations,1)
            sPos = chargeStations(s,:);
            d1 = norm(sPos - node1); d2 = norm(node2 - sPos);
            totalDist_km = d1 + d2;
            battNeeded = totalDist_km * battery_consume_dynamic;
            
            remaining_after = current_battery - battNeeded + B_max;
            if remaining_after >= B_max * min_charge_threshold
                extra = (d1 + d2) - distDirect;
                score = extra + 0.1*norm(sPos - (node1+node2)/2);
                if score < bestScore
                    bestScore = score; bestStation = sPos; chosenIdx = s;
                end
            end
        end
    end
    
    if ~isempty(bestStation)
        rhRoute = [rhRoute(1:i,:); bestStation; rhRoute(i+1:end,:)];
        insertedStations = [insertedStations; bestStation];
        totalExtra = totalExtra + bestScore;
        chargeStationUsage(chosenIdx) = chargeStationUsage(chosenIdx) + 1;
        
        current_battery = B_max;
    else
        current_battery = current_battery - direct_batt_needed;
        current_battery = max(current_battery, 0);
    end
end

chargeInfo{1} = size(insertedStations,1);
chargeInfo{3} = totalExtra;
chargeInfo{2} = size(insertedStations,1);
chargeInfo{4} = totalExtra;
end

function nonDominated = findNonDominatedStations(node1, node2, chargeStations)
m = size(chargeStations,1);
if m<=1, nonDominated = chargeStations; return; end
dist1 = vecnorm(chargeStations - node1,2,2); dist2 = vecnorm(chargeStations - node2,2,2);
dominated = false(m,1);
for i=1:m
    for j=1:m
        if i==j, continue; end
        if (dist1(i)<=dist1(j) && dist2(i)<dist2(j)) || (dist1(i)<dist1(j) && dist2(i)<=dist2(j))
            dominated(j) = true;
        end
    end
end
nonDominated = chargeStations(~dominated,:);
end

function [optimalRoute, optimalInfo] = restrictedEnumeration(baseRoute, nonDominatedStations, ub, B_max, battery_consume, initial_battery, min_charge_threshold)
route = baseRoute;
chargeCount = 0;
n = size(baseRoute,1);

current_battery = initial_battery;

for pos = 1:n-1
    if chargeCount >= ub, break; end
    
    node1 = route(pos,:); node2 = route(pos+1,:);
    distDirect = norm(node2 - node1);
    direct_batt_needed = distDirect * battery_consume;
    
    will_need_charge = (current_battery - direct_batt_needed) < (B_max * min_charge_threshold);
    
    if will_need_charge
        cand = nonDominatedStations{pos};
        if isempty(cand), continue; end
        
        bestExtra = inf; bestS = [];
        for s=1:size(cand,1)
            station = cand(s,:);
            d1 = norm(station - node1);
            d2 = norm(node2 - station);
            totalDist_km = d1 + d2;
            battNeeded = totalDist_km * battery_consume;
            
            remaining_after = current_battery - battNeeded + B_max;
            if remaining_after >= B_max * min_charge_threshold
                extra = totalDist_km - distDirect;
                if extra < bestExtra
                    bestExtra = extra; bestS = station;
                end
            end
        end
        
        if ~isempty(bestS)
            route = [route(1:pos,:); bestS; route(pos+1:end,:)];
            chargeCount = chargeCount + 1;
            pos = pos + 1;
            
            current_battery = B_max;
        else
            current_battery = current_battery - direct_batt_needed;
            current_battery = max(current_battery, 0);
        end
    else
        current_battery = current_battery - direct_batt_needed;
        current_battery = max(current_battery, 0);
    end
end

optimalRoute = route;
optimalInfo = {chargeCount, 0};
end

function smoothed = smoothPath(route, factor, redundancy_penalty)
n = size(route,1);
if n<=2, smoothed = route; return; end
path_complexity = sum(vecnorm(diff(diff(route)),2,2));
interpPoints = max(5, round(n*(2 + min(1, redundancy_penalty*path_complexity))));
t = linspace(0,1,n); ts = linspace(0,1,interpPoints);
x = interp1(t, route(:,1), ts, 'spline');
y = interp1(t, route(:,2), ts, 'spline');
smoothed = [x', y'];

keyPoints = route;
for i=1:size(keyPoints,1)
    [~, idx] = min(vecnorm(smoothed - keyPoints(i,:),2,2));
    smoothed(idx,:) = keyPoints(i,:);
    if idx > 1
        smoothed(idx-1,:) = (smoothed(idx-1,:) + keyPoints(i,:)) / 2;
    end
    if idx < size(smoothed,1)
        smoothed(idx+1,:) = (smoothed(idx+1,:) + keyPoints(i,:)) / 2;
    end
end
end

function [splitRoutes, pickupSeq] = splitGiantTour(giantTour, allNodes, pickupDemand, max_capacity, redundancy_penalty)
n = length(giantTour);
if n<=2
    splitRoutes = {giantTour};
    pickupSeq = unique(giantTour(giantTour~=1)) - 1;
    return;
end
V = inf(n,1); P = zeros(n,1); V(1)=0;
for i=2:n
    for j=i-1:-1:1
        nodesSegment = giantTour(j+1:i);
        demandSum = 0;
        for k=1:numel(nodesSegment)
            nd = nodesSegment(k);
            if nd ~= 1
                idxP = nd - 1;
                if idxP >=1 && idxP <= numel(pickupDemand)
                    demandSum = demandSum + pickupDemand(idxP);
                end
            end
        end
        if demandSum > max_capacity, break; end
        cost = 0;
        for k=j:i-1
            cost = cost + norm(allNodes(giantTour(k+1),:) - allNodes(giantTour(k),:));
            if k > j
                vec1 = allNodes(giantTour(k),:) - allNodes(giantTour(k-1),:);
                vec2 = allNodes(giantTour(k+1),:) - allNodes(giantTour(k),:);
                ang = acos(dot(vec1,vec2)/(norm(vec1)*norm(vec2)+1e-9));
                cost = cost + redundancy_penalty * ang;
            end
        end
        if V(j) + cost < V(i)
            V(i) = V(j) + cost; P(i) = j;
        end
    end
end
splitRoutes = {}; cur = n;
while cur > 1
    prev = P(cur); if prev < 1, prev = 1; end
    splitRoutes{end+1} = giantTour(prev:cur);
    cur = prev;
end
splitRoutes = flip(splitRoutes);
allPickupNodes = giantTour(giantTour ~= 1);
pickupSeq = unique(allPickupNodes) - 1;
end

function [totalDist, totalElapsed, totalChargeTime, battery_trace, sequence, currentChargeCount, current_vehicle_tw_penalty, frozen_charge_ratio] = ...
    simulateDelivery(routeNodes, chargeStations, pickupPoints, pickup_time_windows, vehicles, start_hour, rest_start_elapsed, rest_end_elapsed, pickup_wait_time, battery_consume_dynamic, demand_q, temp_idx_list, assignment)

n_route = size(routeNodes,1);
tol = 1e-6;
chargeNodeMask = false(n_route,1);
pickupNodeMask = false(n_route,1);
for i=1:n_route
    if ~isempty(chargeStations)
        dcs = vecnorm(chargeStations - routeNodes(i,:), 2, 2); chargeNodeMask(i) = any(dcs <= tol); end
    if ~isempty(pickupPoints)
        dps = vecnorm(pickupPoints - routeNodes(i,:), 2, 2); pickupNodeMask(i) = any(dps <= tol); end
end

totalDist = 0; totalElapsed = 0; totalChargeTime = 0;
battery = vehicles(1).battery_init;
battery_trace = battery;
sequence = {sprintf('仓库（%02d:%02d出发，电量%.1fkWh）', start_hour, 0, battery)};
currentChargeCount = 0; current_vehicle_tw_penalty = 0;

frozen_demand_total = sum(demand_q(temp_idx_list==2));
total_demand = sum(demand_q);
if total_demand>0, frozen_charge_ratio = frozen_demand_total / total_demand; else frozen_charge_ratio = 0; end

min_charge_threshold = vehicles(1).min_charge_threshold;
B_max = vehicles(1).battery_cap;

for i=1:n_route-1
    pos = routeNodes(i,:); target = routeNodes(i+1,:);
    isCharge = chargeNodeMask(i+1); isPickup = pickupNodeMask(i+1);
    dist_km = norm(target - pos);
    time_needed_h = dist_km / vehicles(1).speed;
    batt_needed = dist_km * battery_consume_dynamic;
    totalDist = totalDist + dist_km;
    totalElapsed = totalElapsed + time_needed_h;
    [totalElapsed, ~] = checkRestTimeElapsed(totalElapsed, rest_start_elapsed, rest_end_elapsed);
    [arr_h, arr_m] = elapsedToClock(totalElapsed, start_hour);
    battery = battery - batt_needed;
    battery_trace(end+1) = battery;
    
    if isCharge
        if battery < B_max * min_charge_threshold
            cs_idx = findNearestPoint(target, chargeStations);
            chargeNeeded = B_max - battery;
            if chargeNeeded < 0, chargeNeeded = 0; end
            chargeTime_h = chargeNeeded / 40;
            totalChargeTime = totalChargeTime + chargeTime_h;
            totalElapsed = totalElapsed + chargeTime_h;
            [totalElapsed, ~] = checkRestTimeElapsed(totalElapsed, rest_start_elapsed, rest_end_elapsed);
            [end_h, end_m] = elapsedToClock(totalElapsed, start_hour);
            battery = B_max;
            currentChargeCount = currentChargeCount + 1;
            battery_trace(end+1) = battery;
            sequence{end+1} = sprintf('充电站%d（%02d:%02d充电，%02d:%02d完成）', cs_idx, arr_h, arr_m, end_h, end_m);
            fprintf('  充电站%d：%02d:%02d到达，充电%.0f分钟\n', cs_idx, arr_h, arr_m, chargeTime_h*60);
        else
            sequence{end+1} = sprintf('经过充电站（%02d:%02d，电量充足不充电）', arr_h, arr_m);
            fprintf('  经过充电站：%02d:%02d到达，电量%.1fkWh，充足不充电\n', arr_h, arr_m, battery);
        end
    elseif isPickup
        pidx = findNearestPoint(target, pickupPoints);
        pidx = clampAssignment(pidx, length(pickup_time_windows));
        tw = pickup_time_windows{pidx};
        arrival_elapsed = totalElapsed;
        early_elapsed = clockToElapsed(tw.early_h, tw.early_m, start_hour);
        late_elapsed = clockToElapsed(tw.late_h, tw.late_m, start_hour);
        elastic_early = clockToElapsed(tw.elastic_early_h, tw.elastic_early_m, start_hour);
        elastic_late = clockToElapsed(tw.elastic_late_h, tw.elastic_late_m, start_hour);
        if arrival_elapsed >= early_elapsed && arrival_elapsed <= late_elapsed
            tw_penalty = 0; status = '准时';
        elseif arrival_elapsed >= elastic_early && arrival_elapsed < early_elapsed
            tw_penalty = (early_elapsed - arrival_elapsed)*tw.tolerance_pen;
            status = sprintf('弹性提前%.0f分钟', (early_elapsed - arrival_elapsed)*60);
        elseif arrival_elapsed > late_elapsed && arrival_elapsed <= elastic_late
            tw_penalty = (arrival_elapsed - late_elapsed)*tw.tolerance_pen;
            status = sprintf('弹性延后%.0f分钟', (arrival_elapsed - late_elapsed)*60);
        else
            tw_penalty = abs(arrival_elapsed - (early_elapsed+late_elapsed)/2)*tw.intolerance_pen;
            status = sprintf('超弹性区间%.0f分钟', abs(arrival_elapsed - (early_elapsed+late_elapsed)/2)*60);
        end
        current_vehicle_tw_penalty = current_vehicle_tw_penalty + tw_penalty;
        totalElapsed = totalElapsed + pickup_wait_time;
        [wait_h, wait_m] = elapsedToClock(totalElapsed, start_hour);
        [totalElapsed, ~] = checkRestTimeElapsed(totalElapsed, rest_start_elapsed, rest_end_elapsed);
        sequence{end+1} = sprintf('自提点%d（%s，%02d:%02d完成）', pidx, status, wait_h, wait_m);
        fprintf('  自提点%d：%02d:%02d到达（%s），等待至%02d:%02d\n', pidx, arr_h, arr_m, status, wait_h, wait_m);
    else
        sequence{end+1} = sprintf('仓库（%02d:%02d到达）', arr_h, arr_m);
        fprintf('  仓库：%02d:%02d到达，电量%.1fkWh\n', arr_h, arr_m, battery);
    end
end

current_vehicle_tw_penalty = current_vehicle_tw_penalty;
end

function [totalElapsed, rest_time] = checkRestTimeElapsed(totalElapsed, rest_start_elapsed, rest_end_elapsed)
rest_time = 0;
if totalElapsed >= rest_start_elapsed && totalElapsed < rest_end_elapsed
    rest_time = rest_end_elapsed - totalElapsed;
    totalElapsed = rest_end_elapsed;
end
end

function [h,m] = elapsedToClock(elapsed_h, start_hour)
base_min = start_hour * 60;
elapsed_min = round(elapsed_h * 60);
total_min = base_min + elapsed_min;
h = floor(total_min/60); m = mod(total_min,60);
end

function elapsed_h = clockToElapsed(h,m,start_hour)
total_min_current = h*60 + m;
total_min_base = start_hour*60;
elapsed_min = total_min_current - total_min_base;
if elapsed_min >= 0, elapsed_h = elapsed_min / 60; else elapsed_h = 0; end
end

function addDirectionArrows(route, arrowDensity, color)
n = size(route,1); if n<=1, return; end
totalLength = 0;
for i=1:n-1, totalLength = totalLength + norm(route(i+1,:)-route(i,:)); end
numArrows = max(2, round(totalLength * arrowDensity));
if numArrows > n-1, numArrows = n-1; end
segmentLengths = zeros(n-1,1);
for i=1:n-1, segmentLengths(i)=norm(route(i+1,:)-route(i,:)); end
cumLengths = cumsum(segmentLengths);
arrowPositions = linspace(0, totalLength, numArrows+2); arrowPositions = arrowPositions(2:end-1);
for pos = arrowPositions
    segIdx = find(cumLengths >= pos, 1);
    if isempty(segIdx), segIdx = n-1; end
    if segIdx==1, prevLength = 0; else prevLength = cumLengths(segIdx-1); end
    ratio = (pos - prevLength)/segmentLengths(segIdx);
    arrowPoint = route(segIdx,:) + ratio*(route(segIdx+1,:)-route(segIdx,:));
    if segIdx < n-1, nextPoint = route(segIdx+1,:); else nextPoint = route(segIdx,:) + (route(segIdx,:) - route(segIdx-1,:)); end
    direction = nextPoint - arrowPoint; direction = direction / (norm(direction)+1e-9);
    quiver(arrowPoint(1), arrowPoint(2), direction(1)*0.08, direction(2)*0.08, 'Color', color, 'LineWidth', 1.5, 'MaxHeadSize', 2, 'AutoScale', 'off');
end
end