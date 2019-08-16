if(timevector(1)~=0)
    timevector=timevector-timevector(1);
    timevector=timevector*0.01;
end


plot(timevector,-statevars(2,:)*180/pi,'r',timevector,statevars(4,:)*180/pi,'b');
title('Behavior of pitch angle and rate');
legend(["dot theta";"theta"]);
xlabel('time [s]')