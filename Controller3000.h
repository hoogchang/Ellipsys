//---TF for input 1
    int         controller1_ns = 2;     // number of sections
    static	struct	biquad controller1[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 9.246237e-04, -9.990754e-01, 1.000000e+00, -1.956437e+00, 9.569094e-01, 0, 0, 0, 0, 0},
        {-4.748161e+01, 9.303325e+01, -4.557124e+01, 1.000000e+00, -1.954556e+00, 9.552354e-01, 0, 0, 0, 0, 0}
        };
    int         controller2_ns = 2;     // number of sections
    uint32_t    timeoutValue = 3.333333e+02;  // time interval - us; f_s = 3000 Hz
    static	struct	biquad controller2[]={ 	 // define the array of floating point biquads
        {1.000000e+00, 2.854451e-03, -9.971455e-01, 1.000000e+00, -1.956437e+00, 9.569094e-01, 0, 0, 0, 0, 0},
        {5.896215e+00, -1.153451e+01, 5.641099e+00, 1.000000e+00, -1.954556e+00, 9.552354e-01, 0, 0, 0, 0, 0}
        };
