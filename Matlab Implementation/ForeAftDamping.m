function [LocalVar] = ForeAftDamping(CntrPar, LocalVar, objInst)
    % Fore-aft damping controller, reducing the tower fore-aft vibrations using pitch

%     % Local variables
%     INTEGER(4) :: K    % Integer used to loop through turbine blades

%     TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
%     TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
%     TYPE(ObjectInstances), INTENT(INOUT)    :: objInst

    % Body
    LocalVar.FA_AccHPFI = PIController(LocalVar.FA_AccHPF, 0.0, CntrPar.FA_KI, -CntrPar.FA_IntSat, CntrPar.FA_IntSat, LocalVar.DT, 0.0, false, objInst.instPI);

    % Store the fore-aft pitch contribution to LocalVar data type
    for K = 1:LocalVar.NumBl
        LocalVar.FA_PitCom(K) = LocalVar.FA_AccHPFI;
    end

end