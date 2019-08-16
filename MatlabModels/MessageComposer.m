%     Copyright 2017-2020 Dino Spiller (dinospiller@gmail.com)
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

classdef MessageComposer
    properties
        message;
        total_packet;
    end
    methods
        function obj = MessageComposer()
            obj.message="";
            obj.total_packet = "";
        end
        function obj =setCommand(obj,command,canForward)
            COMM_FORWARD_CAN=33;
            CAN_ADDR=0;
            if(canForward==true)
                obj.message = string([char(uint8(COMM_FORWARD_CAN)),char(uint8(CAN_ADDR)),char(uint8(command))]);
            else
                obj.message = string(char(uint8(command)));
            end            
        end
        function obj = addFloat(obj,floatVal)
            obj.message = obj.message + string((char(flip(typecast((single(floatVal)),'uint8')))));
        end
        function obj = addInt8(obj,intVal)
            obj.message = obj.message + string((char(typecast(int8(intVal),'uint8'))));
        end
        function obj = addInt32(obj,intVal)
            obj.message = obj.message + string((char(flip(typecast(int32(intVal),'uint8')))));
        end
        function obj = addCrcHeadersAndSend(obj,serialPortFile)
            obj.total_packet = string(char(uint8(2)));
            obj.total_packet = obj.total_packet+string(char(length(char(obj.message))));
            obj.total_packet = obj.total_packet+obj.message;
            [crcbit crchex crcdec]=crc16(char(obj.message));
            obj.total_packet = obj.total_packet+string(char(bitshift(uint16(crcdec),-8)));
            obj.total_packet = obj.total_packet+string(char(uint16(bitand((crcdec),255))));
            obj.total_packet = obj.total_packet+string(char(uint8(3)));
            uint8(char(obj.total_packet))
            fprintf(serialPortFile,"%s",obj.total_packet);
        end
        function r = resetMessage(obj)
            obj.message = "";
        end
    end
end
