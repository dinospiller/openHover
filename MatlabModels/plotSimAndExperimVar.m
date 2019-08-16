function graph = plotSimAndExperimVar(varname,timevector,var_names,sim_vars,exper_vars,y_unit_str)
    temp_string = 'Plot of simulated and experimental value of: ' + varname;
    ind =find(var_names==varname);
    temp_string = "ref "+varname;
    ind_ref=find(var_names==temp_string);
    
    if(ind_ref)
        reference=sim_vars(1:length(timevector),ind_ref);
        %reference=exper_vars(ind_ref,:)
        graph=plot(timevector,reference,timevector,sim_vars(1:length(timevector),ind),timevector,exper_vars(ind,:));
        title(temp_string);
        legendvars=["reference";"simulated";"Experimental"]
        legend(legendvars);
    else
        graph=plot(timevector,sim_vars(1:length(timevector),ind),timevector,exper_vars(ind,:));
        title(temp_string);
        legendvars=["simulated";"Experimental"]
        legend(legendvars);
    end    
    xlabel('time [s]')
    temp_string = 'value of: ' + varname + ' [' +y_unit_str+']';
    ylabel(temp_string)
end

