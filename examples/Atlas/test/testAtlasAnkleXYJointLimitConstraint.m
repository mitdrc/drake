function testAtlasAnkleXYJointLimitConstraint

options.floating = true;
sides = {'l', 'r'};
robot = RigidBodyManipulator('../urdf/atlas_minimal_contact.urdf',options);

vertices_x = [-0.641118124436429;-0.00405770964833141;0.641118124436429;0.641118124436429;0.351668169522092;-0.00405770964833141;-0.351668169522092;-0.641118124436429;-0.641118124436429];
vertices_y = [0.366546438232642;0.692515779981966;0.366546438232642;-0.812894499549143;-0.981965734896303;-1.01983769161407;-0.981965734896303;-0.812894499549143;0.366546438232642];

for i = 1 : length(sides)
  side = sides{i};
  constraint = AtlasAnkleXYJointLimitConstraint(robot, side);
  akx_idx = robot.findPositionIndices([side '_leg_akx']);
  aky_idx = robot.findPositionIndices([side '_leg_aky']);
  
  for j = 1 : 1000
    q = rand(robot.getNumPositions(), 1);
    t = randn;
    q(akx_idx) = (rand - 0.5) * 3;
    q(aky_idx) = (rand - 0.5) * 3;
    
    val = constraint.eval(t, q);
    [lb, ub] = constraint.bounds(t);
    
    inpoly = inpolygon(q(akx_idx), q(aky_idx), vertices_x, vertices_y);
    constraint_satisfied = all(val >= lb) && all(val <= ub);
    valuecheck(inpoly, constraint_satisfied);
  end
end

end
