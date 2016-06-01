//---Impulse Disturbance Lead-Lag Compensator Biquad
    int         ImpulseController_ns = 1;     // number of sections
    static	struct	biquad ImpulseController[]={ 	 // define the array of floating point biquads
        {7.943164e+00, -7.941032e+00, 0.000000e+00, 1.000000e+00, -9.960670e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
    int         Butter_ns = 1;     // number of sections
    uint32_t    timeoutValue = 250;  // time interval - us; f_s = 4000 Hz
    static	struct	biquad Butter[]={ 	 // define the array of floating point biquads
        {5.371698e-04, 1.074340e-03, 5.371698e-04, 1.000000e+00, -1.933380e+00, 9.355289e-01, 0, 0, 0, 0, 0}
        };
