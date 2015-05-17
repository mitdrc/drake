function [pelvis_motion_data, qs] = planWalkingPelvisMotion(obj, plan_settings, foot_motions, q0, qstar, pelvis_height_above_sole, min_knee_angle)
% Given the results of the ZMP tracker, find a pelvis trajectory for the robot to execute
% its walking plan.
% TODO: make this be a method of QPLocomotionPlanSettings instead of Biped?

comtraj = plan_settings.comtraj;
zmptraj = plan_settings.zmptraj;
supports = plan_settings.supports;
support_times = plan_settings.support_times;
contact_groups = plan_settings.contact_groups;
qtraj = plan_settings.qtraj;
constrained_indices = plan_settings.constrained_dofs;

% time spacing of samples for IK
if ~isa(comtraj, 'Trajectory')
  comtraj = ExpPlusPPTrajectory(comtraj.breaks,...
    comtraj.K,...
    comtraj.A,...
    comtraj.alpha,...
    comtraj.gamma);
end

robot = obj.getManipulator();

breaks_per_support = 3;
ts = upsampleLinearly(support_times, breaks_per_support);
% ts = support_times(1) : 0.1 : support_times(length(support_times));

% unwrap rpy
rpy0_unwrapped = unwrap([zeros(3, 1) q0(4:6)]);
q0(4:6) = rpy0_unwrapped(:, 2);

sides = {'l', 'r'};

state_frame = obj.getStateFrame;
cost = Point(state_frame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 500;
cost.base_roll = 500;
cost.base_pitch = 500;
cost.base_yaw = 500;
cost.back_bkz = 10;
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
cost = cost(1 : robot.getNumPositions());

ikoptions = IKoptions(obj);

forwardkin_options.rotation_type = 2;
pelvis_id = robot.findLinkId('pelvis');
pelvis_height_idx = state_frame.findCoordinateIndex('base_z');
pelvis_yaw_idx = state_frame.findCoordinateIndex('base_yaw');

ankle_joint_constraints = containers.Map;
foot_ids = containers.Map;
foot_sides = containers.Map('KeyType', 'double', 'ValueType', 'char');
side_to_akx_state_frame_id = containers.Map;
side_to_aky_state_frame_id = containers.Map;
for i = 1 : length(sides)
  side = sides{i};
  ankle_joint_constraints(side) = AtlasAnkleXYJointLimitConstraint(obj, side);
  body_id = robot.findLinkId([side '_foot']);
  foot_ids(side) = body_id;
  foot_sides(body_id) = side;
  side_to_akx_state_frame_id(side) = state_frame.findCoordinateIndex([side '_leg_akx']);
  side_to_aky_state_frame_id(side) = state_frame.findCoordinateIndex([side '_leg_aky']);
end

is_qtraj_constant = ~isa(qtraj, 'Trajectory');
if is_qtraj_constant
  constrained_dofs_constraint = PostureConstraint(obj).setJointLimits(constrained_indices, qtraj(constrained_indices), qtraj(constrained_indices));
end

knee_constraints = containers.Map;
for i = 1 : length(sides)
  side = sides{i};
  knee_ind = state_frame.findCoordinateIndex([side '_leg_kny']);
  knee_constraints(side) =  PostureConstraint(obj).setJointLimits(knee_ind, min_knee_angle, inf);
end

foot_motion_body_ids = zeros(size(foot_motions));
for i = 1 : length(foot_motion_body_ids)
  foot_motion_body_ids(i) = getBodyId(robot, foot_motions(i).body_id);
end
body_id_to_foot_motion_id = containers.Map(foot_motion_body_ids, 1 : length(foot_motions), 'uniformValues', true);

toe_off_possible_body_ids = isToeOffPossible(robot, zmptraj, supports, support_times, contact_groups, foot_motions, ts, body_id_to_foot_motion_id);

qs = zeros(obj.getNumPositions(), length(ts));
pelvis_xyzquat = zeros(7, length(ts));
base_heights = zeros(1, length(ts));
average_foot_yaws = zeros(1, length(ts));
in_single_support = false(1, length(ts));
standard_constraints = ankle_joint_constraints.values;
standard_constraints{end + 1} = constrained_dofs_constraint;
knee_constraints_values = knee_constraints.values;
standard_constraints = {standard_constraints{:}, knee_constraints_values{:}};
full_IK_calls = 0;
alpha_nominal_pelvis_height = 0.3;

for i = 1 : length(ts)
  t = ts(i);  
  base_heights(i) = computeBaseHeight(robot, support_times, t, contact_groups, foot_motions, body_id_to_foot_motion_id);
  
  if (i > 1)
    nominal_pelvis_height = base_heights(i) + pelvis_height_above_sole;
    % smoothen:
    nominal_pelvis_height = alpha_nominal_pelvis_height * qs(pelvis_height_idx, i - 1) + (1 - alpha_nominal_pelvis_height) * nominal_pelvis_height;
    qstar(pelvis_height_idx) = nominal_pelvis_height;
    
    average_foot_yaws(i) = computeAverageFootYaw(support_times, t, foot_motions, body_id_to_foot_motion_id);
    qstar(pelvis_yaw_idx) = average_foot_yaws(i);

    constraints = standard_constraints;
    for j = 1:length(foot_motions)
      frame_or_body_id = foot_motions(j).body_id;
      body_id = getBodyId(robot, frame_or_body_id);
      support_idx = findSegmentIndex(support_times, t);
      in_single_support(i) = length(supports(support_idx).bodies) == 1;
      is_swing_foot = ~any(supports(support_idx).bodies == body_id);
      side = foot_sides(body_id);
      
      if is_swing_foot
        cost(side_to_akx_state_frame_id(side)) = 0;
        cost(side_to_aky_state_frame_id(side)) = 0;
      else
        cost(side_to_akx_state_frame_id(side)) = 10;
        cost(side_to_aky_state_frame_id(side)) = 10;
      end
      
      ikoptions = ikoptions.setQ(diag(cost));
      
      xyz_exp = foot_motions(j).eval(t);
      xyz = xyz_exp(1:3);
      expmap = xyz_exp(4:6);
      quat = expmap2quat(expmap);
      xyz(foot_motions(j).weight_multiplier(4:6) == 0) = nan;

      if toe_off_possible_body_ids(i) == body_id
        trailing_toe_points_body = contact_groups{body_id}.toe;
        trailing_toe_points_world = transformPointsFromBodyToWorld(robot, foot_motions(j), t, trailing_toe_points_body);

        constraints{end + 1} = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,...
          obj,body_id, trailing_toe_points_body, trailing_toe_points_world, trailing_toe_points_world);
      else
        constraints{end + 1} = constructRigidBodyConstraint(RigidBodyConstraint.WorldPositionConstraintType,true,...
          obj,frame_or_body_id, [0;0;0],xyz, xyz);
        if ~is_swing_foot % be robust to strange swing trajectories
          constraints{end + 1} = constructRigidBodyConstraint(RigidBodyConstraint.WorldQuatConstraintType,true,obj,frame_or_body_id,quat,0.01);
        end
      end
    end
    kc_com = constructRigidBodyConstraint(RigidBodyConstraint.WorldCoMConstraintType,true,obj.getMexModelPtr,[comtraj.eval(t);nan],[comtraj.eval(t);nan]);
    constraints{end + 1} = kc_com;
    q_seed = qs(:,i-1);
    q_seed(pelvis_yaw_idx) = qstar(pelvis_yaw_idx); % fix wraparound issues
    
    if ~is_qtraj_constant
      qtraj_val = qtraj.eval(t);
      constraints{end + 1} = PostureConstraint(obj).setJointLimits(constrained_indices, qtraj_val(constrained_indices), qtraj_val(constrained_indices));
    end
    
    [qs(:,i), info, infeasible] = inverseKin(obj,q_seed,qstar,constraints{:},ikoptions);
    if info ~= 1
      fprintf('no solution found at t = %0.2f. Using the solution from the previous time step.\n', t);
      qs(:, i) = qs(:, i - 1); % desparation move
    end
  else
    qs(:,i) = q0;
  end

  kinsol = doKinematics(robot, qs(:,i));
  pelvis_xyzquat(:, i) = forwardKin(robot, kinsol, pelvis_id, zeros(3, 1), forwardkin_options);
end

if full_IK_calls > 0
  fprintf(1, 'Called inverseKin due to failure of approximateIK %d times.\n', full_IK_calls);
end

pelvis_motion_data = BodyMotionData.from_body_xyzquat(pelvis_id, ts, pelvis_xyzquat);
pelvis_motion_data.weight_multiplier = [1;1;1;0;0;1];

debug = false;
if debug  
  n = length(ts);
  rpy = zeros(3, n);
  for i = 1 : length(ts)
    rpy(:, i) = quat2rpy(pelvis_xyzquat(4:7, i));
  end
  leg_position_indices = robot.findPositionIndices('leg');

  ankle_constraint_active = false(length(sides), n);
  knee_constraint_active = false(length(sides), n);
  joint_limit_constraint_active = false(n, 1);
  active_tol = 1e-5;
  knee_inds = zeros(2, 1);
  for i = 1 : length(ts)
    for j = 1 : length(sides)
      side = sides{j};
      knee_ind = robot.findPositionIndices([side '_leg_kny']);
      knee_inds(j) = knee_ind;
      knee_constraint = knee_constraints(side);
      knee_constraint_active(j, i) = any(abs(qs(knee_ind, i) - knee_constraint.lb(knee_ind)) < active_tol) || any(abs(qs(knee_ind, i) - knee_constraint.ub(knee_ind)) < active_tol);
      
      ankle_constraint = ankle_joint_constraints(side);
      ankle_val = ankle_constraint.eval(ts(i), qs(:, i));
      [ankle_lb, ankle_ub] = ankle_constraint.bounds(ts(i));
      ankle_constraint_active(j, i) = any(abs(ankle_val - ankle_lb) < active_tol) || any(abs(ankle_val - ankle_ub) < active_tol);
    end
    at_min_limit = abs(qs(leg_position_indices, i) - robot.joint_limit_min(leg_position_indices)) < active_tol;
    at_max_limit = abs(qs(leg_position_indices, i) - robot.joint_limit_max(leg_position_indices)) < active_tol;
    joint_limit_constraint_active(i) = any(at_min_limit) || any(at_max_limit);
  end
  

  figure(152);
  clf();
  subplot(3, 2, 1);
  plot(ts, pelvis_xyzquat(3, :), 'r');
  xlabel('time');
  ylabel('pelvis height');
  
  subplot(3, 2, 2);
  pelvis_height_modification = pelvis_xyzquat(3, :) - base_heights - pelvis_height_above_sole;
  hold on;
  plot(ts, pelvis_height_modification, 'b');
  legend_strings = {'pelvis height modification'};
  toe_off_inds = ~isnan(toe_off_possible_body_ids);
  if ~isempty(plot(ts(toe_off_inds), pelvis_height_modification(toe_off_inds), 'kx'))
    legend_strings{end + 1} = 'toe off allowed';
  end
  if ~isempty(plot(ts(any(ankle_constraint_active, 1)), pelvis_height_modification(any(ankle_constraint_active, 1)), 'rx'))
    legend_strings{end + 1} = 'ankle cstr active';
  end
  if ~isempty(plot(ts(any(knee_constraint_active, 1)), pelvis_height_modification(any(knee_constraint_active, 1)), 'ro'))
    legend_strings{end + 1} = 'knee cstr active';
  end
  if ~isempty(plot(ts(joint_limit_constraint_active), pelvis_height_modification(joint_limit_constraint_active), 'rs'))
    legend_strings{end + 1} = 'joint lim. cstr active';
  end
%   plot(ts(in_single_support), pelvis_height_modification(in_single_support), 'gx');
  xlabel('time');
  legend(legend_strings);
  hold off;
 
  subplot(3, 2, 3)
  plot(ts, base_heights);
  xlabel('time');
  ylabel('base height');
  
  subplot(3, 2, 4)
  plot(ts, rpy);
  xlabel('time');
  ylabel('angles');
  legend({'roll', 'pitch', 'yaw'});

  subplot(3, 2, 5)
  plot(ts, qs(knee_inds, :));
  xlabel('time');
  ylabel('knee angles');
  legend(sides);
end

end

function breaks = upsampleLinearly(ts, rate)
n = length(ts) - 1;
breaks = interp1(0 : rate : rate * n, ts, 0 : rate * n);
end

function ret = isToeOffPossible(robot, zmptraj, supports, support_times, contact_groups, foot_motions, ts, body_id_to_foot_motion_id)
ret = nan(size(ts));
for support_idx = 1 : length(supports)
  support = supports(support_idx);
  in_double_support = length(support.bodies) == 2;
  if support_idx > 1 && in_double_support
    previous_in_single_support = length(supports(support_idx - 1).bodies) == 1;
    if previous_in_single_support
      trailing_leg_body_id = supports(support_idx - 1).bodies;
      trailing_leg_foot_motion = foot_motions(body_id_to_foot_motion_id(trailing_leg_body_id));
      
      next_support_idx = min(support_idx + 1, length(supports));
      support_tspan = support_times([support_idx, next_support_idx]);
      
      ts_for_support_indices = find(ts >= support_tspan(1) & ts <= support_tspan(2));
      
      for j = ts_for_support_indices
        t = ts(j);
        trailing_leg_motion_t_ind = trailing_leg_foot_motion.findTInd(t);
        if trailing_leg_foot_motion.toe_off_allowed(trailing_leg_motion_t_ind)
          trailing_leg_mask = support.bodies == trailing_leg_body_id;
          trailing_leg_contact_group = contact_groups{trailing_leg_body_id};
          if isfield(trailing_leg_contact_group, 'toe')
            trailing_leg_points = trailing_leg_contact_group.toe;
          else
            trailing_leg_points = support.contact_pts{trailing_leg_mask};
          end
          trailing_leg_points_world = transformPointsFromBodyToWorld(robot, trailing_leg_foot_motion, t, trailing_leg_points);
          
          leading_leg_mask = ~trailing_leg_mask;
          leading_leg_body_id = support.bodies(leading_leg_mask);
          leading_leg_points = support.contact_pts{leading_leg_mask};
          leading_leg_foot_motion = foot_motions(body_id_to_foot_motion_id(leading_leg_body_id));
          leading_leg_points_world = transformPointsFromBodyToWorld(robot, leading_leg_foot_motion, t, leading_leg_points);
          
          contact_points_xy_world = [leading_leg_points_world(1:2, :), trailing_leg_points_world(1:2, :)];
          K = convhull(contact_points_xy_world');
          toe_off_base_of_support = contact_points_xy_world(:, K);
          zmp = zmptraj.eval(t);
          if inpolygon(zmp(1), zmp(2), toe_off_base_of_support(1, :), toe_off_base_of_support(2, :))
            ret(j) = trailing_leg_body_id;
          end
          
          debug = false;
          if debug
            figure(153);
            clf();
            title(['toe off possible: ' num2str(ret(j))])
            hold on;
            patch(toe_off_base_of_support(1, :), toe_off_base_of_support(2, :), 'r');
            plot(zmp(1), zmp(2), 'k*');
            axis equal;
            hold off;
          end
        end
      end
    end
  end
end
end

function ret = transformPointsFromBodyToWorld(robot, body_motion, t, points)
T_frame_to_world = xyzExpMapToTransform(body_motion.eval(t));
T_body_to_frame = homogTransInv(robot.getFrame(body_motion.body_id).T);
T_body_to_world = T_frame_to_world * T_body_to_frame;
ret = homogTransMult(T_body_to_world, points);
end

function ret = xyzExpMapToTransform(xyz_expmap)
ret = [quat2rotmat(expmap2quat(xyz_expmap(4 : 6))), xyz_expmap(1:3); zeros(1, 3), 1];
end

function base_height = computeBaseHeight(robot, support_times, t, contact_groups, foot_motions, body_id_to_foot_motion_id)
% use weighted average of toe point heights
all_support_bodies = body_id_to_foot_motion_id.keys;
all_support_bodies = [all_support_bodies{:}];
[weights, support_times_span] = computeSupportWeights(support_times, t);

base_height = 0;
for body_id = all_support_bodies
  toe_points_body = contact_groups{body_id}.toe;
  foot_motion = foot_motions(body_id_to_foot_motion_id(body_id));
  for j = 1 : length(support_times_span)
    % compute toe points in world for each body, both in current support and
    % next support
    support_time = support_times_span(j);
    toe_points_world = transformPointsFromBodyToWorld(robot, foot_motion, support_time, toe_points_body);
    
    % do a weighted average over support times
    base_height = base_height + weights(j) * mean(toe_points_world(3, :));
  end
end
% average over bodies
base_height = base_height / length(all_support_bodies);
end

function ret = computeAverageFootYaw(support_times, t, foot_motions, body_id_to_foot_motion_id)
% angle weighted average of support foot yaws
all_support_bodies = body_id_to_foot_motion_id.keys;
all_support_bodies = [all_support_bodies{:}];
[weights, support_times_span] = computeSupportWeights(support_times, t);

n = length(all_support_bodies);
average_yaws = zeros(length(support_times), 1);
for j = 1 : length(support_times_span)
  support_time = support_times_span(j);
  foot_yaws = zeros(n, 1);
  for i = 1 : n
    body_id = all_support_bodies(i);
    foot_motion = foot_motions(body_id_to_foot_motion_id(body_id));
    xyz_expmap = foot_motion.eval(support_time);
    rpy = quat2rpy(expmap2quat(xyz_expmap(4 : 6)));
    foot_yaws(i) = rpy(3);
  end
  average_yaws(j) = angleAverage(foot_yaws(1), foot_yaws(2)); % average over the feet at a support time
end
ret = angleWeightedAverage(average_yaws(1), weights(1), average_yaws(2), weights(2)); % weighted average over support times

end

function ret = findSegmentIndex(breaks, t)
ret = 1;
while (ret < length(breaks) && t >= breaks(ret + 1))
  ret = ret + 1;
end
end

function body_id = getBodyId(robot, frame_or_body_id)
if frame_or_body_id < 0
  body_id = robot.getFrame(frame_or_body_id).body_ind;
else
  body_id = frame_or_body_id;
end
end

function [weights, support_times_span] = computeSupportWeights(support_times, t)
support_idx = findSegmentIndex(support_times, t);
next_support_idx = min(support_idx + 1, length(support_times));
support_times_span = support_times([support_idx, next_support_idx]);

delta = diff(support_times_span);
if delta < eps
  support_fraction = 0;
else
  x = (t - support_times_span(1)) / delta;
%   support_fraction = x; % linear
  support_fraction = 3 * x^2 - 2 * x^3; % cubic polynomial that goes from 0 to 1 on the interval [0 1]
%   support_fraction = 10 * x^3 -15 * x^4 + 6 * x^5; % quintic
end
weights(1) = 1 - support_fraction;
weights(2) = support_fraction;
end
