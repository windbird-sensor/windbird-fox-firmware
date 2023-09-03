/*
 * wb_compass_calibration.c
 *
 *  Created on: 29 Aug 2023
 *      Author: windbird-dev
 */

/*
 * Sample calculations of magnetic corrections using iterative techniques
 * No linear algebra libraries are required.
 * No eigenvalue-eigenvector routines are required.
 * Portions of original Python code Copyright 2020, Tom Judd
 * https://web.archive.org/web/20230901202740/http://juddzone.com/ALGORITHMS/least_squares_precision_3D_ellipsoid.html
 * Ported to C and memory-optimized for Windbird by N. Baldeck
*/

// #include <efm32.h>
#include <math.h>
#include <string.h>

#include "td_flash.h"
#include "wb_debug.h"

#include "wb_compass_calibration.h"

// TODO: Temperature compensation
// TODO: Improve calibration using accelerometer?

#define N_ACQ_SAMPLES 30
#define N_ACQ_REPEAT 3
#define SAMPLES_MIN_DISTANCE 70

#define N_ITERATIONS 7

static float calSamples[N_ACQ_SAMPLES * N_ACQ_REPEAT][3];
static int calSamplesCount = 0;
static float calMatrix[3][3];
static float calOffset[3];
static float calRadius;


void estimateCenter3D(float samples[][3], int nSamples, float center[3], float *radius) {
	int i;

	center[0] = 0.;
	center[1] = 0.;
	center[2] = 0.;

	for (i=0; i < nSamples; i++) {
		center[0] += samples[i][0];
		center[1] += samples[i][1];
		center[2] += samples[i][2];
	}
	center[0] /= nSamples;
	center[1] /= nSamples;
	center[2] /= nSamples;

	*radius = 0;
	for (i=0; i < nSamples; i++) {
		*radius += sqrt(
			pow(samples[i][0] - center[0], 2) +
			pow(samples[i][1] - center[1], 2) +
			pow(samples[i][2] - center[2], 2)
		);
	}
	*radius /= nSamples;
}

void param9toOfsMat(float params[9], float ofs[3], float mat[3][3]) {
	ofs[0] = params[0];
	ofs[1] = params[1];
	ofs[2] = params[2];
	mat[0][0] = params[3];
	mat[1][1] = params[4];
	mat[2][2] = params[5];
	mat[0][1] = params[6];
	mat[0][2] = params[7];
	mat[1][2] = params[8];
	mat[1][0] = params[6];
	mat[2][0] = params[7];
	mat[2][1] = params[8];
}

void ofsMatToParam9(float ofs[3], float mat[3][3], float params[9]) {
	params[0] = ofs[0];
	params[1] = ofs[1];
	params[2] = ofs[2];
	params[3] = mat[0][0];
	params[4] = mat[1][1];
	params[5] = mat[2][2];
	params[6] = mat[0][1];
	params[7] = mat[0][2];
	params[8] = mat[1][2];
}

void printNxN(char *description, float *array, int ni, int nj) {
	int i, j;
	WB_DEBUG("--- %s [%d][%d]: -----\n", description, ni, nj);
	for (i=0; i<ni; i++) {
		for (j=0; j<nj; j++) {
			WB_DEBUG("%d ", (int)(float)(*((array + i * nj) + j) * 1000.));
		}
		WB_DEBUG("\n");
	}
	WB_DEBUG("----------\n");
}


float radiusErr(float sample[3], float target, float params[9]) {
    // offset and transformation matrix from parameters
	float ofs[3];
	float mat[3][3];
	param9toOfsMat(params, ofs, mat);

	// subtract offset, then apply transformation matrix
	float mc[3];
	mc[0]= sample[0] - ofs[0];
	mc[1]= sample[1] - ofs[1];
	mc[2]= sample[2] - ofs[2];

	// mm = np.dot(mat,mc)
	float mm[3] = { 0., 0., 0. };
	int i;
	int j;
	for (i=0; i<3; i++) {
		for (j=0; j<3; j++) {
			mm[i] += mat[i][j] * mc[j];
		}
	}

	float radius = sqrt(mm[0]*mm[0] + mm[1]*mm[1] + mm[2]*mm[2]);
	float err = target - radius;
	return err;
}

float errorEstimateSymmetric(float samples[][3], int nSamples, float target, float params[9]) {
	float err;
	float err2sum = 0.;
	int i;
	for (i = 0; i < nSamples; i++) {
		err = radiusErr(samples[i], target, params);
		err2sum += err * err;
	}
	return sqrt(err2sum / nSamples);
}

void numericPartialRow(float sample[3], float target, float params[9], float step[9], float *err0, float pdiff[9]) {
	*err0 = radiusErr(sample, target, params);

	float errA;
	float errB;

	int i;
	for (i = 0; i < 9; i++) {
		params[i] = params[i] + step[i];
		errA = radiusErr(sample, target, params);
		params[i] = params[i] - 2.0 * step[i];
		errB = radiusErr(sample, target, params);
		params[i]= params[i] + step[i];
		pdiff[i] = (errB - errA) / (2.0 * step[i]);
	}
}

int inverseMatrix(float A[9][9], float inverse[9][9]) {

    // ChatGPT ....

	int i, j, k;

    float temp;

    // Initialize the inverse matrix as an identity matrix
    for (i = 0; i < 9; i++) {
        for (j = 0; j < 9; j++) {
            inverse[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Start Gauss-Jordan elimination
    for (i = 0; i < 9; i++) {
        // Make the diagonal contain all ones
        temp = A[i][i];
        for (j = 0; j < 9; j++) {
            A[i][j] /= temp;
            inverse[i][j] /= temp;
        }

        // Make the other rows contain zeros
        for (j = 0; j < 9; j++) {
            if (i != j) {
                temp = A[j][i];
                for (k = 0; k < 9; k++) {
                    A[j][k] -= A[i][k] * temp;
                    inverse[j][k] -= inverse[i][k] * temp;
                }
            }
        }
    }

    // Check if matrix is singular (i.e., not invertible)
    for (i = 0; i < 9; i++) {
        if (A[i][i] == 0) {
            return 0; // Matrix is singular, cannot find its inverse
        }
    }

    return 1; // Successful inversion
}

void ellipsoid_iterate_symmetric(float samples[][3], int nSamples, float params9[9], float *radius) {
	int i, j, k;

	float center[3];
	// estimate center
	estimateCenter3D(samples, nSamples, center, radius);

	// scale
	for (i=0; i<nSamples; i++) {
		samples[i][0] /= *radius;
		samples[i][1] /= *radius;
		samples[i][2] /= *radius;
	}
	center[0] /= *radius;
	center[1] /= *radius;
	center[2] /= *radius;

	memset(params9, 0, 9 * sizeof(float));
	float ofs[3] = {0., 0., 0.};
	float mat[3][3] = {
		{1., 0., 0.},
		{0., 1., 0.},
		{0., 0., 1.}
	};

	ofsMatToParam9(center, mat, params9);
	float sigma = errorEstimateSymmetric(samples, nSamples, 1., params9);

	WB_DEBUG("Initial sigma : %d e-3\n", (int)(float)(sigma * 1000.));

	float step[9];
	for (i=0; i<9; i++) step[i] = 1./5000.;

	int iteration;

	float DTD[9][9];
	float invDTD[9][9];
	float DTE[9];
	float f0;
	float pdiff[9];
	for (iteration = 0; iteration < N_ITERATIONS; iteration++) {
		memset(DTD, 0, 9 * 9 * sizeof(float));
		memset(DTE, 0, 9 * sizeof(float));

		for (i = 0; i < nSamples; i++) {
			numericPartialRow(samples[i], 1., params9, step, &f0, pdiff);
			for(j = 0; j < 9; j++) {
			    for(k = 0; k < 9; k++) {
			        DTD[j][k] += pdiff[j] * pdiff[k];
			    }
			    DTE[j] += f0 * pdiff[j];
			}

		}
		inverseMatrix(DTD, invDTD);
		for (i = 0; i < 9; i++) {
			for (j = 0; j < 9; j++) {
				params9[i] += invDTD[i][j] * DTE[j];
			}
		}

		param9toOfsMat(params9, ofs, mat);

		sigma = errorEstimateSymmetric(samples, nSamples, 1., params9);
		WB_DEBUG("iteration %d, sigma : %d e-3\n", iteration, (int)(float)(sigma * 1000.));
	}


}

/* test code
int main() {
	float samples[32][3] = {{ 321.13761,  592.68208, -110.92169},
		       { 369.5775 ,  820.62965,  -33.77537},
		       { 353.30059,  710.18674,   99.5011 },
		       { 309.06748,  296.01928,  231.44519},
		       { 239.27049, -177.83822,  273.04763},
		       { 192.47963, -434.44221,  198.11776},
		       { 195.28058, -304.18968,   53.81068},
		       { 259.38827,  110.18309,  -76.44162},
		       { 100.92073,  221.002  ,  -72.69141},
		       { -54.46045,  149.99361,   30.56772},
		       { -21.70442,   92.71619,  170.5606 },
		       { 187.70107,  115.00077,  253.07234},
		       { 454.83761,  180.06335,  238.11447},
		       { 619.01148,  266.21554,  129.1675 },
		       { 582.25116,  310.30544,  -10.64292},
		       { 370.97056,  295.59823,  -93.934  },
		       {  18.89031,  429.91097,   42.37785},
		       { -86.10727,  -54.91491,  100.34645},
		       {  28.97991, -389.40055,  145.12057},
		       { 292.69263, -372.17646,  151.2178 },
		       { 545.15406,  -42.78629,  117.71466},
		       { 653.36751,  455.18348,   62.37725},
		       { 534.53481,  800.83089,   14.2984 },
		       { 277.36174,  776.81465,    9.01628},
		       {  99.677  ,  606.21526,  -21.33528},
		       { -27.67416, -262.68674,   83.14308},
		       { 460.00349, -147.02142,   82.4087 },
		       { 585.50705,  715.83148,  -18.04456},
		       { 244.30505,  248.30528,  236.85131},
		       { 201.29068,    7.7006 ,  265.53074},
		       { 328.19573,   35.07879,  266.39908},
		       { 360.17339,  248.83504,  240.78515}};
	int nSamples = 32;
	float params9[9];
	float radius;
	ellipsoid_iterate_symmetric(samples, nSamples, params9, &radius);
	printf("radius %f\n", radius);
	float ofs[3];
	float mat[3][3];
	param9toOfsMat(params9, ofs, mat);
	print9(ofs, 3);
	print9x9(mat, 3);

	--- Expected output ----
	Sigma : 0.0098

	Radius : 397.900

	 Offsets  ( 3 )
	 0.708549 0.501864 0.201022

	 Transform Matrix norm1 ( 3 , 3 )
	 1.163677 -0.176644 -0.075369
	 -0.176644 0.694462 0.292957
	 -0.075369 0.292957 2.340037

} */


float distanceToSamples(float newSample[3]) {
	float minDistance = 99999999.;
	float distance;
	int i = floor((float)calSamplesCount / (float)N_ACQ_SAMPLES) * N_ACQ_SAMPLES;
	for (; i<calSamplesCount; i++) {
		distance = sqrt(
			pow(calSamples[i][0] - newSample[0], 2) +
			pow(calSamples[i][1] - newSample[1], 2) +
			pow(calSamples[i][2] - newSample[2], 2)
		);
		if (distance < minDistance) minDistance = distance;
	}
	return minDistance;
}

void WB_COMPASS_CALIBRATION_Init() {
	if (!TD_FLASH_DeclareVariable((uint8_t *) calMatrix, sizeof(float)*3*3, 0)) {
		WB_DEBUG("No calMatrix in Flash. Using default\n");
		calMatrix[0][0] = 0.925;
		calMatrix[0][1] = -0.010;
		calMatrix[0][2] = -0.020;
		calMatrix[1][0] = -0.010;
		calMatrix[1][1] = 0.951;
		calMatrix[1][2] = -0.002;
		calMatrix[2][0] = -0.020;
		calMatrix[2][1] = -0.002;
		calMatrix[2][2] = 0.909;
	} else {
		printNxN("Using calMatrix from Flash", &calMatrix[0][0], 3, 3);
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) calOffset, sizeof(float)*3, 0)) {
		WB_DEBUG("No calOffset in Flash. Using default\n");
		calOffset[0] = 179;
		calOffset[1] = -296;
		calOffset[2] = -4786;
	} else {
		printNxN("Using calOffset from Flash", calOffset, 3, 1);
	}

	if (!TD_FLASH_DeclareVariable((uint8_t *) &calRadius, sizeof(float), 0)) {
		WB_DEBUG("No calRadius in Flash. Using default\n");
		calRadius = 271;
	} else {
		WB_DEBUG("Using calRadius from Flash: %d\n", (int)calRadius);
	}
}

void WB_COMPASS_CALIBRATION_Transform(float *x, float *y, float *z) {

	*x /= calRadius;
	*y /= calRadius;
	*z /= calRadius;

	float xc = (*x) - calOffset[0];
	float yc = (*y) - calOffset[1];
	float zc = (*z) - calOffset[2];

	*x = calMatrix[0][0] * xc + calMatrix[0][1] * yc + calMatrix[0][2] * zc;
	*y = calMatrix[1][0] * xc + calMatrix[1][1] * yc + calMatrix[1][2] * zc;
	*z = calMatrix[2][0] * xc + calMatrix[2][1] * yc + calMatrix[2][2] * zc;
}

void WB_COMPASS_CALIBRATION_Begin() {
	calSamplesCount = 0;
}

WB_COMPASS_CALIBRATION_Result_t WB_COMPASS_CALIBRATION_AddSample(float newSample[3]) {
	float distance = distanceToSamples(newSample);
	if (distance < SAMPLES_MIN_DISTANCE) {
		 // discard this sample and continue acquisition
		return WB_COMPASS_CALIBRATION_SAMPLE_DISCARD;
	}
	calSamples[calSamplesCount][0] =  newSample[0];
	calSamples[calSamplesCount][1] =  newSample[1];
	calSamples[calSamplesCount][2] =  newSample[2];
	WB_DEBUG("cal: %d\t%d\t%d\t%d\t%d\n", (int)newSample[0], (int)newSample[1], (int)newSample[2], (int)distance, calSamplesCount);
	if (calSamplesCount < N_ACQ_SAMPLES * N_ACQ_REPEAT - 1) {
		calSamplesCount++;
		return WB_COMPASS_CALIBRATION_SAMPLE_OK;
	} else {
		// samples collection is now complete
		return WB_COMPASS_CALIBRATION_AQUISITION_COMPLETE;
	}
}

void WB_COMPASS_CALIBRATION_End() {
	float params[9];
	ellipsoid_iterate_symmetric(calSamples, calSamplesCount, params, &calRadius);
	param9toOfsMat(params, calOffset, calMatrix);
	WB_DEBUG("radius: %d\n", (int)calRadius);
	printNxN("Offsets", calOffset, 3, 1);
	printNxN("Transformation matrix", &calMatrix[0][0], 3, 3);
	TD_FLASH_WriteVariables();
}
