classdef RigidBodyFlatTerrain < RigidBodyTerrain
%  This class provides an implementation of RigidBodyTerrain with z=0
%  everywhere
  
  methods 
    function obj = RigidBodyFlatTerrain(color)
      if nargin < 1
        color = hex2dec({'ee','cb','ad'})'/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      end
      obj.geom = constructRigidBodyGeometry(obj, color);
    end
    
    function [z,normal] = getHeight(obj,xy)
      [m n] = size(xy);
      z = repmat(0,1,n);
      normal = repmat([0;0;1],1,n);
    end

    function geom = getCollisionGeometry(obj)
      geom = obj.geom;
    end

    function geom = getVisualGeometry(obj)
      geom = obj.geom;
    end
    
    function geom = constructRigidBodyGeometry(obj, color)
      if nargin < 2
        color = hex2dec({'ee','cb','ad'})'/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
      else
        color = reshape(color, 3, 1);
      end
      box_width = 1000;
      box_depth = 10;
      geom = RigidBodyBox([box_width;box_width;box_depth]);
      geom.T(3,4) = -box_depth/2;
      geom.c = color;
      geom.name = 'terrain';
%      geom.c = hex2dec({'cd','af','95'})'/256;
    end
  end
  
  properties
    geom;
  end
end
