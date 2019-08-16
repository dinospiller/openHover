% Filename: f_quantizer.m
% Author: Dino Spiller
% Date: 22/06/2014
%
% Description: function for calculating the quantized version of a continous
% input. It returns the quantized value of the input (xq). The arguments are:
% x: the input value
% Nb: number of bits (the number of levels will result in NL = 2^Nb)
% Tq: the saturation value (if the input signal exceeds it, it will be
% "clamped" to the saturation value.

function [xq]=f_quantizer(x,Nb,Tq)
    if(x > Tq)
        xq = Tq
    elseif(x < -Tq)
        xq = -Tq
    else
        Delta_q = 2*Tq/(2^Nb - 1);
        xq = Delta_q/2 + Delta_q*floor(x/Delta_q);
    end
end

