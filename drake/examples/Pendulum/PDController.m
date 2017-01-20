classdef PDController < DrakeSystem  
  properties 
    p
  end
  
  methods
    function obj = PDController(plant)
      obj = obj@DrakeSystem(0,0,plant.getNumOutputs,plant.getNumInputs,true,true);
      obj.p = plant;
      obj = obj.setInputFrame(plant.getStateFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
    
    function u = output(obj,t,junk,x)        
        % PD Control
        x_ref = [-pi;0];
        state_error = x - x_ref;
        %u_pd = pd_gain_matrix * state_error;
        K_p = 1;
        K_d = .01;
        u_pd = K_p * state_error(1) - K_d * state_error(2);
        
        u_ff = 0;
        
        u = u_ff + u_pd;
    end
  end
end 