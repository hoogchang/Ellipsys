//---TF for input 1
    int         controller1_ns = 2;     // number of sections
    static	struct	biquad controller1[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 6.935480e-04, -9.993065e-01, 1.000000e+00, -1.967238e+00, 9.675054e-01, 0, 0, 0, 0, 0},
        {-3.582323e+01, 7.055159e+01, -3.473673e+01, 1.000000e+00, -1.965851e+00, 9.662351e-01, 0, 0, 0, 0, 0}
        };
    int         controller2_ns = 2;     // number of sections
    uint32_t    timeoutValue = 250;  // time interval - us; f_s = 4000 Hz
    static	struct	biquad controller2[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 2.141602e-03, -9.978584e-01, 1.000000e+00, -1.967238e+00, 9.675054e-01, 0, 0, 0, 0, 0},
        {4.445690e+00, -8.745125e+00, 4.300632e+00, 1.000000e+00, -1.965851e+00, 9.662351e-01, 0, 0, 0, 0, 0}
        };
    int         controller3_ns = 1;     // number of sections
    static	struct	biquad controller3[]={ 	 // define the array of floating point biquads
        {2.500000e-04, 2.500000e-04, 0.000000e+00, 1.000000e+00, -1.000000e+00, 0.000000e+00, 0, 0, 0, 0, 0}
        };
