%% Collection finished: here is data-processing

figure();
subplot(2,1,1);
plotSimAndExperimVar("Xb",timevector,var_names,statevars_sim.Data,statevars,"m");
subplot(2,1,2);
plotSimAndExperimVar("dotXb",timevector,var_names,statevars_sim.Data,statevars,"m/s");

figure();
subplot(2,1,1);
plotSimAndExperimVar("thetaP",timevector,var_names,statevars_sim.Data,statevars,"rad");
subplot(2,1,2);
plotSimAndExperimVar("dotThetaP",timevector,var_names,statevars_sim.Data,statevars,"rad/s");

figure();
subplot(2,1,1);
plotSimAndExperimVar("delta",timevector,var_names,statevars_sim.Data,statevars,"rad");
subplot(2,1,2);
plotSimAndExperimVar("dotDelta",timevector,var_names,statevars_sim.Data,statevars,"rad/s");

% plot(timevector,statevars);
% plot(timevector,statevars(1,:),'r',timevector,statevars(2,:),'g',timevector,statevars(3,:), '--r' ,timevector,statevars(4,:),'--g',timevector,statevars(5,:), 'b' ,timevector,statevars(6,:),'--b' ,timevector,statevars(7,:),'k',timevector,statevars(8,:), 'm' );
% plot(timevector,statevars(1,:),'r',timevector,statevars(2,:)*180/pi,'g',timevector,statevars(3,:), '--r' ,timevector,statevars(4,:)*180/pi,'--g',timevector,statevars(5,:), 'b' ,timevector,statevars(6,:),'--b' ,timevector,statevars(7,:),'k',timevector,statevars(8,:), 'm' );
% plot(timevector,statevars(1,:),'r',timevector,statevars(3,:), '--r' ,timevector,statevars(5,:),'--b');
% legend(var_names);
% 
% figure();
% 
% subplot(7,1,1);
% title('Horizontal position');
% ind1 =find(var_names=="Xb");
% ind2 =find(var_names=="ref Xb");
% plot(timevector,statevars(ind1,:),timevector,statevars(ind2,:));
% legendvars=[var_names(ind1);var_names(ind2)];
% legend(legendvars);
% 
% subplot(7,1,2);
% title('Horizontal velocity');
% ind1 =find(var_names=="dotXb");
% ind2 =find(var_names=="ref dotXb");
% plot(timevector,statevars(ind1,:),timevector,statevars(ind2,:));
% legendvars=[var_names(ind1);var_names(ind2)];
% legend(legendvars);
% 
% subplot(7,1,3);
% title('Pitch angle');
% ind1 =find(var_names=="thetaP");
% plot(timevector,statevars(ind1,:)*180/pi);
% legendvars=[var_names(ind1)];
% legend(legendvars);
% 
% subplot(7,1,4);
% title('Pitch angle speed');
% ind1 =find(var_names=="dotThetaP");
% plot(timevector,statevars(ind1,:)*180/pi);
% legendvars=[var_names(ind1)];
% legend(legendvars);
% 
% subplot(7,1,5);
% title('Yaw angle');
% ind1 =find(var_names=="delta");
% plot(timevector,statevars(ind1,:));
% legendvars=[var_names(ind1)];
% legend(legendvars);
% 
% subplot(7,1,6);
% title('Yaw angle speed');
% ind1 =find(var_names=="dotDelta");
% ind2 =find(var_names=="ref dotDelta");
% plot(timevector,statevars(ind1,:),timevector,statevars(ind2,:));
% legendvars=[var_names(ind1);var_names(ind2)];
% legend(legendvars);
% 
% subplot(7,1,7);
% title('Torques');
% ind1 =find(var_names=="torqueEq");
% ind2 =find(var_names=="torqueSteer");
% plot(timevector,statevars(ind1,:),timevector,statevars(ind2,:));
% legendvars=[var_names(ind1);var_names(ind2)];
% legend(legendvars);
