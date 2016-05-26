//---TF for input 1
    int         controller1_ns = 2;     // number of sections
    static	struct	biquad controller1[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 1.386615e-03, -9.986134e-01, 1.000000e+00, -1.935011e+00, 9.360623e-01, 0, 0, 0, 0, 0},
        {-7.038462e+01, 1.364996e+02, -6.617969e+01, 1.000000e+00, -1.932098e+00, 9.336101e-01, 0, 0, 0, 0, 0}
        };
    int         controller2_ns = 2;     // number of sections
    uint32_t    timeoutValue = 500;  // time interval - us; f_s = 2000 Hz
    static	struct	biquad controller2[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 4.278623e-03, -9.957214e-01, 1.000000e+00, -1.935011e+00, 9.360623e-01, 0, 0, 0, 0, 0},
        {8.751236e+00, -1.693140e+01, 8.189429e+00, 1.000000e+00, -1.932098e+00, 9.336101e-01, 0, 0, 0, 0, 0}
        };
