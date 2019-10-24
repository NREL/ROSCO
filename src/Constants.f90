MODULE Constants
    REAL(4), PARAMETER          :: RPS2RPM = 9.5492966                  ! Factor to convert radians per second to revolutions per minute.
    REAL(4), PARAMETER          :: R2D = 57.295780                      ! Factor to convert radians to degrees.
    REAL(4), PARAMETER          :: D2R = 0.0175                         ! Factor to convert degrees to radians.
    REAL(4), PARAMETER          :: PI = 3.14159265359                   ! Mathematical constant pi
    INTEGER(4), PARAMETER       :: NP_1 = 1                             ! First rotational harmonic
    INTEGER(4), PARAMETER       :: NP_2 = 2                             ! Second rotational harmonic
END MODULE Constants