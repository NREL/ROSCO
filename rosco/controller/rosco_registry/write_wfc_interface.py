import rosco.toolbox
import os
from rosco.toolbox.ofTools.util.FileTools import load_yaml
import re

this_dir = os.path.dirname(__file__)
src_dir = os.path.realpath(os.path.join(this_dir,'../src'))


def has_number_in_parentheses(input_string):
    pattern = r'\(\d+\)'
    match = re.search(pattern, input_string)
    if match:
        return match.group()
    else:
        return None


def main():

    # Read wfc_interface yaml and rosco_types for checking
    wfc_int = load_yaml(os.path.join(this_dir,'wfc_interface.yaml'))
    rosco_types = load_yaml(os.path.join(this_dir,'rosco_types.yaml'))

    local_vars = rosco_types['LocalVariables']

    # Check that measurements/setpoints are a LocalVar
    for ms in wfc_int['setpoints'] + wfc_int['measurements']:
        ind = has_number_in_parentheses(ms)
        if ind:
            ms = ms.strip(ind)
            ind_int = int(ind[1:-1])
        else:
            ind_int = 0

        if ms not in local_vars:
            raise Exception(f'WFC variable {ms} not in LocalVars')

        if ind_int:
            if local_vars[ms]['size'] < ind_int:
                raise Exception(f'Requested index ({ind_int}) of WFC variable {ms} is greater than LocalVars version.')


    write_zmq_f90(wfc_int)
    write_client_c(wfc_int)

def write_zmq_f90(wfc_int_yaml):


    n_measurements = len(wfc_int_yaml['measurements'])
    n_setpoints = len(wfc_int_yaml['setpoints'])


    with open(os.path.join(src_dir,'ZeroMQInterface.f90'),'w') as f:

        f.write("module ZeroMQInterface\n")
        f.write("   USE, INTRINSIC :: ISO_C_BINDING, only: C_CHAR, C_DOUBLE, C_NULL_CHAR\n")
        f.write("   IMPLICIT NONE\n")
        f.write("   ! \n")
        f.write("\n")
        f.write("CONTAINS\n")
        f.write("    SUBROUTINE UpdateZeroMQ(LocalVar, CntrPar, ErrVar)\n")
        f.write("        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables\n")
        f.write("        IMPLICIT NONE\n")
        f.write("        TYPE(LocalVariables),    INTENT(INOUT) :: LocalVar\n")
        f.write("        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar\n")
        f.write("        TYPE(ErrorVariables),    INTENT(INOUT) :: ErrVar\n")
        f.write("\n")
        f.write("        character(256) :: zmq_address\n")
        f.write(f"        real(C_DOUBLE) :: setpoints({n_setpoints})\n")
        f.write(f"        real(C_DOUBLE) :: turbine_measurements({n_measurements})\n")
        f.write("        CHARACTER(*), PARAMETER                 :: RoutineName = 'UpdateZeroMQ'\n")
        f.write("\n")
        f.write("        ! C interface with ZeroMQ client\n")
        f.write("#ifdef ZMQ_CLIENT\n")
        f.write("        interface\n")
        f.write("            subroutine zmq_client(zmq_address, measurements, setpoints) bind(C, name='zmq_client')\n")
        f.write("                import :: C_CHAR, C_DOUBLE\n")
        f.write("                implicit none\n")
        f.write("                character(C_CHAR), intent(out) :: zmq_address(*)\n")
        f.write(f"                real(C_DOUBLE) :: measurements({n_measurements})\n")
        f.write(f"                real(C_DOUBLE) :: setpoints({n_setpoints})\n")
        f.write("            end subroutine zmq_client\n")
        f.write("        end interface\n")
        f.write("#endif\n")
        f.write("\n")
        f.write("        ! Communicate if threshold has been reached\n")
        f.write("        IF ( MOD(LocalVar%n_DT, CntrPar%n_DT_ZMQ) == 0 .OR. LocalVar%iStatus == -1 ) THEN\n")
        f.write("            ! Collect measurements to be sent to ZeroMQ server\n")
        
        ## Write measurements
        for i_meas, meas in enumerate(wfc_int_yaml['measurements']):
            f.write(f"            turbine_measurements({i_meas+1}) = LocalVar%{meas}\n")
        f.write("\n")
        
        f.write("            write (zmq_address, '(A,A)') TRIM(CntrPar%ZMQ_CommAddress), C_NULL_CHAR\n")
        f.write("#ifdef ZMQ_CLIENT\n")
        f.write("            call zmq_client(zmq_address, turbine_measurements, setpoints)\n")
        f.write("#else\n")
        f.write("            ! Add RoutineName to error message\n")
        f.write("            ErrVar%aviFAIL = -1\n")
        f.write("            IF (CntrPar%ZMQ_Mode > 0) THEN\n")
        f.write("                ErrVar%ErrMsg = ' >> The ZeroMQ client has not been properly installed, ' &\n")
        f.write("                                //'please install it to use ZMQ_Mode > 0.'\n")
        f.write("                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)\n")
        f.write("            ENDIF\n")
        f.write("#endif\n")
        f.write("\n")
        f.write("            ! write (*,*) 'ZeroMQInterface: torque setpoint from ssc: ', setpoints(1)\n")
        f.write("            ! write (*,*) 'ZeroMQInterface: yaw setpoint from ssc: ', setpoints(2)\n")
        f.write("            ! write (*,*) 'ZeroMQInterface: pitch 1 setpoint from ssc: ', setpoints(3)\n")
        f.write("            ! write (*,*) 'ZeroMQInterface: pitch 2 setpoint from ssc: ', setpoints(4)\n")
        f.write("            ! write (*,*) 'ZeroMQInterface: pitch 3 setpoint from ssc: ', setpoints(5)\n")

        ## Write setpoints
        for i_set, set in enumerate(wfc_int_yaml['setpoints']):
            f.write(f"            LocalVar%{set} = setpoints({i_set+1})\n")

        f.write("            \n")
        f.write("        ENDIF\n")
        f.write("\n")
        f.write("\n")
        f.write("    END SUBROUTINE UpdateZeroMQ\n")
        f.write("end module ZeroMQInterface\n")

def write_client_c(wfc_int_yaml):
     
    n_measurements = len(wfc_int_yaml['measurements'])
    n_setpoints = len(wfc_int_yaml['setpoints'])

    with open(os.path.join(src_dir,'zmq_client.c'),'w') as f:
        f.write('#include <stdio.h>\n')
        f.write('#include <stdlib.h>\n')
        f.write('#include <string.h>\n')
        f.write('#include <math.h>\n')
        f.write('#include <zmq.h>\n')
        f.write('\n')
        f.write('\n')
        f.write('void delete_blank_spaces_in_string(char *s)\n')
        f.write('{\n')
        f.write('    int  i,k=0;\n')
        f.write('    for(i=0;s[i];i++)\n')
        f.write('    {\n')
        f.write('        s[i]=s[i+k];\n')
        f.write(r'        if(s[i]==" "|| s[i]=="\t")' + '\n')
        f.write('        {\n')
        f.write('        k++;\n')
        f.write('        i--;\n')
        f.write('        }\n')
        f.write('    }\n')
        f.write('}\n')
        f.write('\n')
        f.write('\n')
        f.write('int zmq_client (\n')
        f.write('    char *zmq_address,\n')
        f.write(f'    double measurements[{n_measurements}],\n')
        f.write(f'    double setpoints[{n_setpoints}]\n')
        f.write(')\n')
        f.write('{\n')
        f.write(f'    int num_measurements = {n_measurements};  // Number of setpoints and measurements, respectively, and float precision (character length)\n')
        f.write('    int char_buffer_size_single = 20; // Char buffer for a single measurement\n')
        f.write('    int char_buffer_size_array = (num_measurements * (char_buffer_size_single + 1));  // Char buffer for full messages to and from ROSCO\n')
        f.write('    char string_to_ssc[char_buffer_size_array];\n')
        f.write('    char string_from_ssc[char_buffer_size_array];\n')
        f.write('\n')
        f.write('    int verbose = 0; // Variable to define verbose\n')
        f.write('    \n')
        f.write('    if (verbose == 1) {\n')
        f.write(r'        printf ("Connecting to ZeroMQ server at %s...\n", zmq_address);' + '\n')
        f.write('    }\n')
        f.write('    // Open connection with ZeroMQ server\n')
        f.write('    void *context = zmq_ctx_new ();\n')
        f.write('    void *requester = zmq_socket (context, ZMQ_REQ);\n')
        f.write('    zmq_connect (requester, zmq_address);  // string_to_zmq is something like "tcp://localhost:5555"\n')
        f.write('\n')
        f.write('    // Create a string with measurements to be sent to ZeroMQ server (e.g., Python)\n')
        f.write('    char a[char_buffer_size_array], b[char_buffer_size_single];\n')
        f.write('    sprintf(b, "%.6e", measurements[0]);\n')
        f.write('    strncpy(a, b, char_buffer_size_single);\n')
        f.write(r'    //printf ("zmq_client.c: a[char_buffer_size_single]: measurements[0]: %s\n", a);'+ '\n')
        f.write('    int i = 1;\n')
        f.write('    while (i < num_measurements) {\n')
        f.write('        strcat(a, ",");  // Add a comma\n')
        f.write('        sprintf(b, "%.6e", measurements[i]);  // Add value\n')
        f.write('        strcat(a, b);  // Concatenate b to a\n')
        f.write(r'        //printf ("zmq_client.c: b[char_buffer_size_single]: measurements[i]: %s\n", b);'+ '\n')
        f.write(r'        //printf (" --> zmq_client.c: a[char_buffer_size_single]: measurements[i]: %s\n", a);'+ '\n')
        f.write('        i = i + 1;\n')
        f.write('    }\n')
        f.write('    strncpy(string_to_ssc, a, char_buffer_size_array);\n')
        f.write('\n')
        f.write('    // Print the string\n')
        f.write('    if (verbose == 1) {\n')
        f.write(r'        printf ("zmq_client.c: string_to_ssc: %s…\n", string_to_ssc);'+ '\n')
        f.write('    }\n')
        f.write('\n')
        f.write('    // Core ZeroMQ communication: receive data and send back signals\n')
        f.write('    zmq_send (requester, string_to_ssc, char_buffer_size_array, 0);\n')
        f.write('    zmq_recv (requester, string_from_ssc, char_buffer_size_array, 0);\n')
        f.write('\n')
        f.write('    if (verbose == 1) {\n')
        f.write(r'        printf ("zmq_client.c: Received a response: %s\n", string_from_ssc);'+ '\n')
        f.write('    }\n')
        f.write('\n')
        f.write('    // Convert string_from_ssc string to separate floats\n')
        f.write('    delete_blank_spaces_in_string(string_from_ssc);\n')
        f.write('    char *pt;\n')
        f.write('    pt = strtok (string_from_ssc,",");\n')
        f.write('    i = 0;\n')
        f.write('    while (pt != NULL) {\n')
        f.write('        double dtmp = atof(pt);\n')
        f.write('        if (verbose == 1) {\n')
        f.write(r'            printf("pt subloop: %s (var), %f (double) \n", pt, dtmp);'+ '\n')
        f.write(r'            printf("zmq_client.c: setpoint[%d]: %f \n", i, dtmp);'+ '\n')
        f.write('        }\n')
        f.write('        pt = strtok (NULL, ",");\n')
        f.write('        setpoints[i] = dtmp;  // Save values to setpoints\n')
        f.write('        i = i + 1;\n')
        f.write('    }\n')
        f.write('\n')
        f.write('    // Close connection\n')
        f.write('    zmq_close (requester);\n')
        f.write('    zmq_ctx_destroy (context);\n')
        f.write('    return 0;\n')
        f.write('}\n')


if __name__=="__main__":
    main()