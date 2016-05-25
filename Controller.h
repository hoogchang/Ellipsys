//---TF for input 1
    int         controller1_ns = 2;     // number of sections
    static	struct	biquad controller1[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 2.048551e-03, -9.979514e-01, 1.000000e+00, -1.929678e+00, 9.308911e-01, 0, 0, 0, 0, 0},
        {-2.211234e+02, 4.288394e+02, -2.079192e+02, 1.000000e+00, -1.929787e+00, 9.316428e-01, 0, 0, 0, 0, 0}
        };
    int         controller2_ns = 2;     // number of sections
    uint32_t    timeoutValue = 500;  // time interval - us; f_s = 2000 Hz
    static	struct	biquad controller2[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 6.513306e-03, -9.934867e-01, 1.000000e+00, -1.929678e+00, 9.308911e-01, 0, 0, 0, 0, 0},
        {1.223484e+01, -2.359004e+01, 1.137062e+01, 1.000000e+00, -1.929787e+00, 9.316428e-01, 0, 0, 0, 0, 0}
        };