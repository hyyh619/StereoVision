% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 380.515922733209610 ; 382.398202023175490 ];

%-- Principal point:
cc = [ 154.857714789310930 ; 129.978800851177710 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.055183428850803 ; -0.242058508127757 ; 0.006786858360876 ; -0.009649592118808 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.469507914676046 ; 2.499253119489089 ];

%-- Principal point uncertainty:
cc_error = [ 3.069888862376970 ; 2.417479423402131 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.022985610489104 ; 0.088367849420625 ; 0.002095276106123 ; 0.003004908207157 ; 0.000000000000000 ];

%-- Image size:
nx = 320;
ny = 240;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 30;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.155989e+00 ; -2.188905e+00 ; 3.464158e-02 ];
Tc_1  = [ -7.339875e+01 ; -1.133426e+02 ; 4.440601e+02 ];
omc_error_1 = [ 7.946394e-03 ; 8.759710e-03 ; 1.836038e-02 ];
Tc_error_1  = [ 3.627174e+00 ; 2.823987e+00 ; 3.496122e+00 ];

%-- Image #2:
omc_2 = [ -2.152493e+00 ; -2.144400e+00 ; -2.375944e-04 ];
Tc_2  = [ -4.691743e+01 ; -1.562749e+02 ; 5.036727e+02 ];
omc_error_2 = [ 1.030805e-02 ; 1.077984e-02 ; 2.182897e-02 ];
Tc_error_2  = [ 4.160050e+00 ; 3.214808e+00 ; 4.128712e+00 ];

%-- Image #3:
omc_3 = [ -2.211699e+00 ; -2.174418e+00 ; -7.090536e-02 ];
Tc_3  = [ -1.086590e+02 ; -1.558260e+02 ; 5.054758e+02 ];
omc_error_3 = [ 1.106853e-02 ; 1.013890e-02 ; 2.323061e-02 ];
Tc_error_3  = [ 4.171646e+00 ; 3.239048e+00 ; 4.309201e+00 ];

%-- Image #4:
omc_4 = [ -2.169334e+00 ; -2.180985e+00 ; 2.286520e-02 ];
Tc_4  = [ -8.239003e+01 ; -9.006322e+01 ; 4.627206e+02 ];
omc_error_4 = [ 8.193031e-03 ; 9.308376e-03 ; 1.951242e-02 ];
Tc_error_4  = [ 3.760330e+00 ; 2.947107e+00 ; 3.658582e+00 ];

%-- Image #5:
omc_5 = [ -2.207037e+00 ; -2.212994e+00 ; -6.825851e-02 ];
Tc_5  = [ -1.108864e+02 ; -9.080636e+01 ; 5.362025e+02 ];
omc_error_5 = [ 1.111815e-02 ; 1.160823e-02 ; 2.508333e-02 ];
Tc_error_5  = [ 4.371938e+00 ; 3.432607e+00 ; 4.537746e+00 ];

%-- Image #6:
omc_6 = [ -2.043611e+00 ; -1.926765e+00 ; -2.607190e-01 ];
Tc_6  = [ -7.322824e+01 ; -6.525101e+01 ; 4.412667e+02 ];
omc_error_6 = [ 5.720456e-03 ; 7.565449e-03 ; 1.429237e-02 ];
Tc_error_6  = [ 3.589511e+00 ; 2.806506e+00 ; 3.382384e+00 ];

%-- Image #7:
omc_7 = [ -1.948722e+00 ; -1.889636e+00 ; -3.559548e-01 ];
Tc_7  = [ -6.817162e+01 ; -1.464541e+02 ; 4.683241e+02 ];
omc_error_7 = [ 6.944386e-03 ; 8.129541e-03 ; 1.476168e-02 ];
Tc_error_7  = [ 3.897138e+00 ; 3.033944e+00 ; 3.813637e+00 ];

%-- Image #8:
omc_8 = [ 2.227224e+00 ; 2.077661e+00 ; -2.467936e-01 ];
Tc_8  = [ -7.415458e+01 ; -7.803209e+01 ; 5.606967e+02 ];
omc_error_8 = [ 1.092495e-02 ; 9.620602e-03 ; 2.192906e-02 ];
Tc_error_8  = [ 4.525707e+00 ; 3.518568e+00 ; 4.277986e+00 ];

%-- Image #9:
omc_9 = [ -2.597985e+00 ; -1.778036e+00 ; 4.765490e-02 ];
Tc_9  = [ -1.116030e+02 ; -6.138766e+01 ; 5.261473e+02 ];
omc_error_9 = [ 1.176475e-02 ; 8.740052e-03 ; 2.437997e-02 ];
Tc_error_9  = [ 4.273278e+00 ; 3.335587e+00 ; 4.310271e+00 ];

%-- Image #10:
omc_10 = [ -1.722212e+00 ; -2.561418e+00 ; 3.810474e-02 ];
Tc_10  = [ -3.410573e+01 ; -1.515414e+02 ; 5.191510e+02 ];
omc_error_10 = [ 8.753485e-03 ; 1.314838e-02 ; 2.243387e-02 ];
Tc_error_10  = [ 4.239110e+00 ; 3.305977e+00 ; 4.179049e+00 ];

%-- Image #11:
omc_11 = [ -2.353287e+00 ; -1.161804e+00 ; -1.057728e-01 ];
Tc_11  = [ -9.478548e+01 ; -1.463004e+00 ; 5.957249e+02 ];
omc_error_11 = [ 8.022497e-03 ; 6.327187e-03 ; 1.430601e-02 ];
Tc_error_11  = [ 4.815457e+00 ; 3.744997e+00 ; 4.132992e+00 ];

%-- Image #12:
omc_12 = [ -1.911362e+00 ; -1.862822e+00 ; -4.908652e-01 ];
Tc_12  = [ -8.013279e+01 ; -1.405493e+02 ; 4.586695e+02 ];
omc_error_12 = [ 6.334125e-03 ; 7.652487e-03 ; 1.370249e-02 ];
Tc_error_12  = [ 3.821515e+00 ; 2.964612e+00 ; 3.805961e+00 ];

%-- Image #13:
omc_13 = [ 1.876174e+00 ; 2.148134e+00 ; -9.042174e-01 ];
Tc_13  = [ -4.522962e+01 ; -1.178841e+02 ; 5.574131e+02 ];
omc_error_13 = [ 5.383992e-03 ; 7.472852e-03 ; 1.318330e-02 ];
Tc_error_13  = [ 4.544118e+00 ; 3.499256e+00 ; 3.346905e+00 ];

%-- Image #14:
omc_14 = [ 1.950445e+00 ; 1.334620e+00 ; -2.701668e-01 ];
Tc_14  = [ -1.009000e+02 ; -6.679328e+01 ; 4.733770e+02 ];
omc_error_14 = [ 6.331444e-03 ; 6.237682e-03 ; 1.022820e-02 ];
Tc_error_14  = [ 3.833144e+00 ; 2.972155e+00 ; 3.513932e+00 ];

%-- Image #15:
omc_15 = [ -1.869078e+00 ; -1.966704e+00 ; 1.131789e+00 ];
Tc_15  = [ -4.819338e+00 ; -9.807945e+01 ; 6.013530e+02 ];
omc_error_15 = [ 8.562330e-03 ; 5.649719e-03 ; 1.190125e-02 ];
Tc_error_15  = [ 4.897890e+00 ; 3.777906e+00 ; 3.340289e+00 ];

%-- Image #16:
omc_16 = [ -1.884567e+00 ; -1.984846e+00 ; 1.119032e+00 ];
Tc_16  = [ -7.889643e+01 ; -9.424267e+01 ; 5.919888e+02 ];
omc_error_16 = [ 8.620105e-03 ; 5.737498e-03 ; 1.152397e-02 ];
Tc_error_16  = [ 4.836476e+00 ; 3.727310e+00 ; 3.422866e+00 ];

%-- Image #17:
omc_17 = [ 1.836140e+00 ; 1.894085e+00 ; 6.218719e-01 ];
Tc_17  = [ -1.050342e+02 ; -1.270225e+02 ; 5.075454e+02 ];
omc_error_17 = [ 7.295720e-03 ; 7.325176e-03 ; 1.270245e-02 ];
Tc_error_17  = [ 4.241876e+00 ; 3.231140e+00 ; 4.450888e+00 ];

%-- Image #18:
omc_18 = [ -1.877321e+00 ; -1.948469e+00 ; 6.582391e-01 ];
Tc_18  = [ 2.369573e+01 ; -1.348126e+02 ; 6.895419e+02 ];
omc_error_18 = [ 8.790230e-03 ; 8.582501e-03 ; 1.659141e-02 ];
Tc_error_18  = [ 5.653908e+00 ; 4.340176e+00 ; 4.346155e+00 ];

%-- Image #19:
omc_19 = [ 1.885659e+00 ; 2.190158e+00 ; -1.039215e+00 ];
Tc_19  = [ -4.304247e+01 ; -1.185008e+02 ; 5.481337e+02 ];
omc_error_19 = [ 5.149607e-03 ; 7.447673e-03 ; 1.306931e-02 ];
Tc_error_19  = [ 4.475949e+00 ; 3.442223e+00 ; 3.192676e+00 ];

%-- Image #20:
omc_20 = [ -1.820508e+00 ; -1.802474e+00 ; -9.868907e-02 ];
Tc_20  = [ -8.898191e+01 ; -1.339529e+02 ; 4.623666e+02 ];
omc_error_20 = [ 5.946023e-03 ; 6.983586e-03 ; 1.238463e-02 ];
Tc_error_20  = [ 3.806413e+00 ; 2.946485e+00 ; 3.481322e+00 ];

%-- Image #21:
omc_21 = [ -1.830803e+00 ; -1.820149e+00 ; -1.959157e-01 ];
Tc_21  = [ -9.696069e+01 ; -1.025620e+02 ; 4.546045e+02 ];
omc_error_21 = [ 5.561507e-03 ; 7.057292e-03 ; 1.226296e-02 ];
Tc_error_21  = [ 3.708913e+00 ; 2.891178e+00 ; 3.453617e+00 ];

%-- Image #22:
omc_22 = [ 2.152727e+00 ; 1.802915e+00 ; 5.046454e-01 ];
Tc_22  = [ -8.970119e+01 ; -9.987723e+01 ; 4.557308e+02 ];
omc_error_22 = [ 7.773840e-03 ; 6.294037e-03 ; 1.354054e-02 ];
Tc_error_22  = [ 3.784371e+00 ; 2.887691e+00 ; 3.917267e+00 ];

%-- Image #23:
omc_23 = [ 1.956712e+00 ; 1.588492e+00 ; 6.899438e-01 ];
Tc_23  = [ -6.162978e+01 ; -1.025724e+02 ; 4.451270e+02 ];
omc_error_23 = [ 7.322661e-03 ; 6.079014e-03 ; 1.092375e-02 ];
Tc_error_23  = [ 3.683580e+00 ; 2.810518e+00 ; 3.873671e+00 ];

%-- Image #24:
omc_24 = [ -2.239947e+00 ; -2.153015e+00 ; -1.483520e-01 ];
Tc_24  = [ -7.693393e+01 ; -1.195313e+02 ; 4.269242e+02 ];
omc_error_24 = [ 7.812673e-03 ; 8.243688e-03 ; 1.756351e-02 ];
Tc_error_24  = [ 3.523473e+00 ; 2.729955e+00 ; 3.480187e+00 ];

%-- Image #25:
omc_25 = [ -2.144363e+00 ; -2.201914e+00 ; -5.835236e-02 ];
Tc_25  = [ -8.144602e+01 ; -1.265451e+02 ; 4.329494e+02 ];
omc_error_25 = [ 7.953429e-03 ; 8.596841e-03 ; 1.799421e-02 ];
Tc_error_25  = [ 3.561227e+00 ; 2.773502e+00 ; 3.500022e+00 ];

%-- Image #26:
omc_26 = [ -2.594797e+00 ; -1.680858e+00 ; -1.356196e-01 ];
Tc_26  = [ -8.317006e+01 ; -6.699059e+01 ; 5.126519e+02 ];
omc_error_26 = [ 1.063111e-02 ; 8.420105e-03 ; 2.172685e-02 ];
Tc_error_26  = [ 4.189297e+00 ; 3.253314e+00 ; 4.278570e+00 ];

%-- Image #27:
omc_27 = [ 1.725227e+00 ; 2.413891e+00 ; -2.902853e-01 ];
Tc_27  = [ -2.657052e+01 ; -1.532587e+02 ; 5.145040e+02 ];
omc_error_27 = [ 6.340013e-03 ; 9.167036e-03 ; 1.558824e-02 ];
Tc_error_27  = [ 4.197402e+00 ; 3.231954e+00 ; 3.686331e+00 ];

%-- Image #28:
omc_28 = [ 2.251750e+00 ; 1.556829e+00 ; -5.378998e-01 ];
Tc_28  = [ -1.125117e+02 ; -6.985013e+01 ; 5.385235e+02 ];
omc_error_28 = [ 6.791813e-03 ; 6.551004e-03 ; 1.332708e-02 ];
Tc_error_28  = [ 4.367532e+00 ; 3.378159e+00 ; 3.744114e+00 ];

%-- Image #29:
omc_29 = [ 2.260789e+00 ; 1.570009e+00 ; -4.935716e-01 ];
Tc_29  = [ -1.163393e+02 ; -7.497094e+01 ; 5.091831e+02 ];
omc_error_29 = [ 6.652444e-03 ; 6.442492e-03 ; 1.315089e-02 ];
Tc_error_29  = [ 4.132783e+00 ; 3.192830e+00 ; 3.592535e+00 ];

%-- Image #30:
omc_30 = [ 2.297778e+00 ; 1.816843e+00 ; -3.048859e-01 ];
Tc_30  = [ -9.223404e+01 ; -1.051554e+02 ; 4.823546e+02 ];
omc_error_30 = [ 7.393420e-03 ; 6.885447e-03 ; 1.492705e-02 ];
Tc_error_30  = [ 3.935941e+00 ; 3.018315e+00 ; 3.536379e+00 ];

