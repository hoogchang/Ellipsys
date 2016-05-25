//---Impulse Disturbance Lead-Lag Compensator Biquad
    int         ImpulseController_ns = 1;     // number of sections
    uint32_t    timeoutValue = 500;  // time interval - us; f_s = 2000 Hz
    static	struct	biquad ImpulseController[]={ 	 // define the array of floating point biquads
        {7.928638e+00, -7.924383e+00, 0.000000e+00, 1.000000e+00, -9.921494e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };
