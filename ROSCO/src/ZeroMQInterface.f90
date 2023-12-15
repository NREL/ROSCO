module ZeroMQInterface
   USE, INTRINSIC :: ISO_C_BINDING, only: C_CHAR, C_DOUBLE, C_NULL_CHAR
   IMPLICIT NONE
   ! 

CONTAINS
    SUBROUTINE UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
        IMPLICIT NONE
        TYPE(LocalVariables),    INTENT(INOUT) :: LocalVar
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        TYPE(ErrorVariables),    INTENT(INOUT) :: ErrVar

        character(256) :: zmq_address
        real(C_DOUBLE) :: setpoints(5)
        real(C_DOUBLE) :: turbine_measurements(17)
        CHARACTER(*), PARAMETER                 :: RoutineName = 'UpdateZeroMQ'

        ! C interface with ZeroMQ client
#ifdef ZMQ_CLIENT
        interface
            subroutine zmq_client(zmq_address, measurements, setpoints) bind(C, name='zmq_client')
                import :: C_CHAR, C_DOUBLE
                implicit none
                character(C_CHAR), intent(out) :: zmq_address(*)
                real(C_DOUBLE) :: measurements(17)
                real(C_DOUBLE) :: setpoints(5)
            end subroutine zmq_client
        end interface
#endif

        ! Communicate if threshold has been reached
        IF ( MOD(LocalVar%n_DT, CntrPar%n_DT_ZMQ) == 0 .OR. LocalVar%iStatus == -1 ) THEN
            ! Collect measurements to be sent to ZeroMQ server
            turbine_measurements(1) = LocalVar%ZMQ_ID
            turbine_measurements(2) = LocalVar%iStatus
            turbine_measurements(3) = LocalVar%Time
            turbine_measurements(4) = LocalVar%VS_MechGenPwr
            turbine_measurements(5) = LocalVar%VS_GenPwr
            turbine_measurements(6) = LocalVar%GenSpeed
            turbine_measurements(7) = LocalVar%RotSpeed
            turbine_measurements(8) = LocalVar%GenTqMeas
            turbine_measurements(9) = LocalVar%NacHeading
            turbine_measurements(10) = LocalVar%NacVane
            turbine_measurements(11) = LocalVar%HorWindV
            turbine_measurements(12) = LocalVar%rootMOOP(1)
            turbine_measurements(13) = LocalVar%rootMOOP(2)
            turbine_measurements(14) = LocalVar%rootMOOP(3)
            turbine_measurements(15) = LocalVar%FA_Acc
            turbine_measurements(16) = LocalVar%NacIMU_FA_Acc
            turbine_measurements(17) = LocalVar%Azimuth

            write (zmq_address, '(A,A)') TRIM(CntrPar%ZMQ_CommAddress), C_NULL_CHAR
#ifdef ZMQ_CLIENT
            call zmq_client(zmq_address, turbine_measurements, setpoints)
#else
            ! Add RoutineName to error message
            ErrVar%aviFAIL = -1
            IF (CntrPar%ZMQ_Mode > 0) THEN
                ErrVar%ErrMsg = ' >> The ZeroMQ client has not been properly installed, ' &
                                //'please install it to use ZMQ_Mode > 0.'
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            ENDIF
#endif

            ! write (*,*) 'ZeroMQInterface: torque setpoint from ssc: ', setpoints(1)
            ! write (*,*) 'ZeroMQInterface: yaw setpoint from ssc: ', setpoints(2)
            ! write (*,*) 'ZeroMQInterface: pitch 1 setpoint from ssc: ', setpoints(3)
            ! write (*,*) 'ZeroMQInterface: pitch 2 setpoint from ssc: ', setpoints(4)
            ! write (*,*) 'ZeroMQInterface: pitch 3 setpoint from ssc: ', setpoints(5)
            LocalVar%ZMQ_TorqueOffset = setpoints(1)
            LocalVar%ZMQ_YawOffset = setpoints(2)
            LocalVar%ZMQ_PitOffset(1) = setpoints(3)
            LocalVar%ZMQ_PitOffset(2) = setpoints(4)
            LocalVar%ZMQ_PitOffset(3) = setpoints(5)
            
        ENDIF


    END SUBROUTINE UpdateZeroMQ
end module ZeroMQInterface
