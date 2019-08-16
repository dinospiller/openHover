

    function [result, hex, dec] = crc16(data)
    
    gx = zeros(1, 16);
    gx( [13 6 1] ) = 1;
     
    data_len = length(data);
     
    result = dec2bin( 0 , 16 ) - '0';
    for k=1:data_len
    temp = dec2bin( data(k),8 ) - '0';
    for m = 1:8
    if result(16) ~= temp(m)
    result(1:16) = [ 0 result(1:15) ];
    result = xor(result,gx);
    else
    result(1:16) = [0 result(1:15)];
    end
    end
    end
     
    str = num2str(fliplr(result));
    hex = dec2hex( bin2dec(str), 4 );
    dec = bin2dec(str);