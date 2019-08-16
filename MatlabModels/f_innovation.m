% Filename: f_innovation.m
% Author: Dino Spiller
% Date: 22/06/2014
%
% Description: function for calculating the innovation  of a process
function[ e ]= f_innovation( y, fi, theta )
    
pred = fi'*theta
e = y(i)-pred(:,1)

