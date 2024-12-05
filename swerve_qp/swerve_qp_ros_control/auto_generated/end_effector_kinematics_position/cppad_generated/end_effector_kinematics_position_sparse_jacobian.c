#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void end_effector_kinematics_position_sparse_jacobian(double const *const * in,
                                                      double*const * out,
                                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[59];

   v[0] = cos(x[19]);
   v[1] = -0.999999999993254 * v[0];
   v[2] = -0.105929999359994 + 0.327454999998506 * v[1];
   v[0] = -3.67320510363811e-06 * v[0];
   v[3] = -0.000349286293238241 + 0.327454999998506 * v[0];
   v[4] = cos(x[18]);
   v[5] = 0.999999999993254 * v[4];
   v[6] = 0 - sin(x[19]);
   v[7] = 0.327454999998506 * v[6];
   v[8] = sin(x[18]);
   v[9] = 0.999999999993254 * v[8];
   v[10] = -0.006375 + v[2] * -3.67320510363811e-06 + v[3] * v[5] + v[7] * v[9];
   v[11] = -3.67320510363811e-06 * v[4];
   v[12] = -3.67320510363811e-06 * v[8];
   v[2] = 0.20843 + v[2] * -0.999999999993254 + v[3] * v[11] + v[7] * v[12];
   v[13] = cos(x[17]);
   v[14] = -7.34641020664359e-06 * v[13];
   v[8] = 0 - v[8];
   v[15] = v[3] * v[8] + v[7] * v[4];
   v[16] = sin(x[17]);
   v[17] = -7.34641020664359e-06 * v[16];
   v[18] = v[10] * -0.999999999973015 + v[2] * v[14] + v[15] * v[17];
   v[19] = -0.999999999973015 * v[13];
   v[20] = -0.999999999973015 * v[16];
   v[10] = -0.41 + v[10] * 7.34641020664359e-06 + v[2] * v[19] + v[15] * v[20];
   v[21] = cos(x[16]);
   v[22] = 0.999999999993254 * v[21];
   v[16] = 0 - v[16];
   v[23] = v[2] * v[16] + v[15] * v[13];
   v[24] = sin(x[16]);
   v[25] = 0.999999999993254 * v[24];
   v[26] = -0.12838 + v[18] * -3.67320510363811e-06 + v[10] * v[22] + v[23] * v[25];
   v[27] = -3.67320510363811e-06 * v[21];
   v[28] = -3.67320510363811e-06 * v[24];
   v[18] = 0.005375 + v[18] * -0.999999999993254 + v[10] * v[27] + v[23] * v[28];
   v[29] = cos(x[15]);
   v[30] = 7.34641020664359e-06 * v[29];
   v[24] = 0 - v[24];
   v[31] = v[10] * v[24] + v[23] * v[21];
   v[32] = sin(x[15]);
   v[33] = 7.34641020664359e-06 * v[32];
   v[34] = 0.43093 + v[26] * -0.999999999973015 + v[18] * v[30] + v[31] * v[33];
   v[35] = 2. * x[5];
   v[36] = -0.999999999973015 * v[29];
   v[37] = -0.999999999973015 * v[32];
   v[26] = v[26] * -7.34641020664359e-06 + v[18] * v[36] + v[31] * v[37];
   v[38] = 2. * x[4];
   jac[1] = v[34] * v[35] + v[26] * v[38];
   v[32] = 0 - v[32];
   v[39] = 0.2625 + v[18] * v[32] + v[31] * v[29];
   v[40] = 0 - v[39];
   jac[2] = v[40] * v[38] + (v[40] * x[4] + v[26] * x[3] + v[34] * x[6]) * 2.;
   v[39] = 0 - v[39];
   v[26] = 0 - v[26];
   jac[3] = v[39] * v[35] + (v[39] * x[5] + v[34] * x[3] + v[26] * x[6]) * 2.;
   jac[4] = v[26] * v[35] + v[34] * v[38];
   v[26] = v[35] * x[5];
   v[34] = v[38] * x[4];
   v[39] = 1 - v[26] - v[34];
   v[40] = v[35] * x[3];
   v[41] = v[38] * x[6];
   v[42] = v[40] + v[41];
   v[43] = v[38] * x[3];
   v[44] = v[35] * x[6];
   v[45] = v[43] - v[44];
   v[46] = sin(x[15]);
   v[47] = cos(x[15]);
   jac[5] = 0 - (v[31] * v[39] + v[18] * v[42] * 7.34641020664359e-06 + v[18] * v[45] * -0.999999999973015) * v[46] + (v[31] * v[42] * 7.34641020664359e-06 + v[31] * v[45] * -0.999999999973015 - v[18] * v[39]) * v[47];
   v[31] = v[42] * v[33] + v[45] * v[37] + v[39] * v[29];
   v[18] = -7.34641020664359e-06 * v[45] + -0.999999999973015 * v[42];
   v[45] = v[42] * v[30] + v[45] * v[36] + v[39] * v[32];
   v[42] = sin(x[16]);
   v[39] = cos(x[16]);
   jac[6] = 0 - (v[23] * v[31] + v[10] * v[18] * 0.999999999993254 + v[10] * v[45] * -3.67320510363811e-06) * v[42] + (v[23] * v[18] * 0.999999999993254 + v[23] * v[45] * -3.67320510363811e-06 - v[10] * v[31]) * v[39];
   v[23] = v[18] * v[25] + v[45] * v[28] + v[31] * v[21];
   v[10] = -0.999999999993254 * v[45] + -3.67320510363811e-06 * v[18];
   v[45] = v[18] * v[22] + v[45] * v[27] + v[31] * v[24];
   v[18] = sin(x[17]);
   v[31] = cos(x[17]);
   jac[7] = 0 - (v[15] * v[23] + v[2] * v[10] * -7.34641020664359e-06 + v[2] * v[45] * -0.999999999973015) * v[18] + (v[15] * v[10] * -7.34641020664359e-06 + v[15] * v[45] * -0.999999999973015 - v[2] * v[23]) * v[31];
   v[15] = v[10] * v[17] + v[45] * v[20] + v[23] * v[13];
   v[2] = 7.34641020664359e-06 * v[45] + -0.999999999973015 * v[10];
   v[45] = v[10] * v[14] + v[45] * v[19] + v[23] * v[16];
   v[10] = sin(x[18]);
   v[23] = cos(x[18]);
   jac[8] = 0 - (v[7] * v[15] + v[3] * v[2] * 0.999999999993254 + v[3] * v[45] * -3.67320510363811e-06) * v[10] + (v[7] * v[2] * 0.999999999993254 + v[7] * v[45] * -3.67320510363811e-06 - v[3] * v[15]) * v[23];
   v[7] = sin(x[19]);
   v[3] = cos(x[19]);
   jac[9] = 0 - (0.327454999998506 * (-0.999999999993254 * v[45] + -3.67320510363811e-06 * v[2]) * -0.999999999993254 + 0.327454999998506 * (v[2] * v[5] + v[45] * v[11] + v[15] * v[8]) * -3.67320510363811e-06) * v[7] + (0 - 0.327454999998506 * (v[2] * v[9] + v[45] * v[12] + v[15] * v[4])) * v[3];
   v[45] = -0.105929999359994 + 0.327454999998506 * v[1];
   v[2] = -0.000349286293238241 + 0.327454999998506 * v[0];
   v[15] = 0.327454999998506 * v[6];
   v[48] = -0.006375 + v[45] * -3.67320510363811e-06 + v[2] * v[5] + v[15] * v[9];
   v[45] = 0.20843 + v[45] * -0.999999999993254 + v[2] * v[11] + v[15] * v[12];
   v[49] = v[2] * v[8] + v[15] * v[4];
   v[50] = v[48] * -0.999999999973015 + v[45] * v[14] + v[49] * v[17];
   v[48] = -0.41 + v[48] * 7.34641020664359e-06 + v[45] * v[19] + v[49] * v[20];
   v[51] = v[45] * v[16] + v[49] * v[13];
   v[52] = 0.005375 + v[50] * -0.999999999993254 + v[48] * v[27] + v[51] * v[28];
   v[53] = v[48] * v[24] + v[51] * v[21];
   v[54] = 0.2625 + v[52] * v[32] + v[53] * v[29];
   v[50] = -0.12838 + v[50] * -3.67320510363811e-06 + v[48] * v[22] + v[51] * v[25];
   v[55] = v[50] * -7.34641020664359e-06 + v[52] * v[36] + v[53] * v[37];
   v[56] = 0 - v[55];
   v[57] = 2. * x[3];
   v[50] = 0.43093 + v[50] * -0.999999999973015 + v[52] * v[30] + v[53] * v[33];
   v[58] = 0 - v[50];
   jac[11] = v[54] * v[38] + v[56] * v[57] + (v[56] * x[3] + v[58] * x[6]) * 2.;
   jac[12] = v[50] * v[35] + v[54] * x[3] * 2.;
   v[55] = 0 - v[55];
   jac[13] = v[55] * v[35] + (v[55] * x[5] + v[50] * x[4] + v[54] * x[6]) * 2.;
   jac[14] = v[54] * v[35] + v[58] * v[57];
   v[44] = v[43] + v[44];
   v[43] = v[35] * x[4];
   v[58] = v[57] * x[6];
   v[54] = v[43] - v[58];
   v[55] = v[57] * x[3];
   v[26] = 1 - v[26] - v[55];
   jac[15] = 0 - (v[53] * v[44] + v[52] * v[54] * 7.34641020664359e-06 + v[52] * v[26] * -0.999999999973015) * v[46] + (v[53] * v[54] * 7.34641020664359e-06 + v[53] * v[26] * -0.999999999973015 - v[52] * v[44]) * v[47];
   v[53] = v[54] * v[33] + v[26] * v[37] + v[44] * v[29];
   v[52] = -7.34641020664359e-06 * v[26] + -0.999999999973015 * v[54];
   v[26] = v[54] * v[30] + v[26] * v[36] + v[44] * v[32];
   jac[16] = 0 - (v[51] * v[53] + v[48] * v[52] * 0.999999999993254 + v[48] * v[26] * -3.67320510363811e-06) * v[42] + (v[51] * v[52] * 0.999999999993254 + v[51] * v[26] * -3.67320510363811e-06 - v[48] * v[53]) * v[39];
   v[51] = v[52] * v[25] + v[26] * v[28] + v[53] * v[21];
   v[48] = -0.999999999993254 * v[26] + -3.67320510363811e-06 * v[52];
   v[26] = v[52] * v[22] + v[26] * v[27] + v[53] * v[24];
   jac[17] = 0 - (v[49] * v[51] + v[45] * v[48] * -7.34641020664359e-06 + v[45] * v[26] * -0.999999999973015) * v[18] + (v[49] * v[48] * -7.34641020664359e-06 + v[49] * v[26] * -0.999999999973015 - v[45] * v[51]) * v[31];
   v[49] = v[48] * v[17] + v[26] * v[20] + v[51] * v[13];
   v[45] = 7.34641020664359e-06 * v[26] + -0.999999999973015 * v[48];
   v[26] = v[48] * v[14] + v[26] * v[19] + v[51] * v[16];
   jac[18] = 0 - (v[15] * v[49] + v[2] * v[45] * 0.999999999993254 + v[2] * v[26] * -3.67320510363811e-06) * v[10] + (v[15] * v[45] * 0.999999999993254 + v[15] * v[26] * -3.67320510363811e-06 - v[2] * v[49]) * v[23];
   jac[19] = 0 - (0.327454999998506 * (-0.999999999993254 * v[26] + -3.67320510363811e-06 * v[45]) * -0.999999999993254 + 0.327454999998506 * (v[45] * v[5] + v[26] * v[11] + v[49] * v[8]) * -3.67320510363811e-06) * v[7] + (0 - 0.327454999998506 * (v[45] * v[9] + v[26] * v[12] + v[49] * v[4])) * v[3];
   v[1] = -0.105929999359994 + 0.327454999998506 * v[1];
   v[0] = -0.000349286293238241 + 0.327454999998506 * v[0];
   v[6] = 0.327454999998506 * v[6];
   v[26] = -0.006375 + v[1] * -3.67320510363811e-06 + v[0] * v[5] + v[6] * v[9];
   v[1] = 0.20843 + v[1] * -0.999999999993254 + v[0] * v[11] + v[6] * v[12];
   v[45] = v[0] * v[8] + v[6] * v[4];
   v[49] = v[26] * -0.999999999973015 + v[1] * v[14] + v[45] * v[17];
   v[26] = -0.41 + v[26] * 7.34641020664359e-06 + v[1] * v[19] + v[45] * v[20];
   v[15] = v[1] * v[16] + v[45] * v[13];
   v[2] = 0.005375 + v[49] * -0.999999999993254 + v[26] * v[27] + v[15] * v[28];
   v[48] = v[26] * v[24] + v[15] * v[21];
   v[51] = 0.2625 + v[2] * v[32] + v[48] * v[29];
   v[49] = -0.12838 + v[49] * -3.67320510363811e-06 + v[26] * v[22] + v[15] * v[25];
   v[52] = 0.43093 + v[49] * -0.999999999973015 + v[2] * v[30] + v[48] * v[33];
   v[53] = 0 - v[52];
   v[49] = v[49] * -7.34641020664359e-06 + v[2] * v[36] + v[48] * v[37];
   jac[21] = v[51] * v[35] + v[53] * v[57] + (v[53] * x[3] + v[49] * x[6]) * 2.;
   v[52] = 0 - v[52];
   v[53] = 0 - v[51];
   jac[22] = v[49] * v[35] + v[52] * v[38] + (v[52] * x[4] + v[53] * x[6]) * 2.;
   jac[23] = (v[49] * x[4] + v[51] * x[3]) * 2.;
   jac[24] = v[53] * v[38] + v[49] * v[57];
   v[41] = v[40] - v[41];
   v[55] = 1 - v[34] - v[55];
   v[58] = v[43] + v[58];
   jac[25] = 0 - (v[48] * v[41] + v[2] * v[55] * 7.34641020664359e-06 + v[2] * v[58] * -0.999999999973015) * v[46] + (v[48] * v[55] * 7.34641020664359e-06 + v[48] * v[58] * -0.999999999973015 - v[2] * v[41]) * v[47];
   v[37] = v[55] * v[33] + v[58] * v[37] + v[41] * v[29];
   v[33] = -7.34641020664359e-06 * v[58] + -0.999999999973015 * v[55];
   v[58] = v[55] * v[30] + v[58] * v[36] + v[41] * v[32];
   jac[26] = 0 - (v[15] * v[37] + v[26] * v[33] * 0.999999999993254 + v[26] * v[58] * -3.67320510363811e-06) * v[42] + (v[15] * v[33] * 0.999999999993254 + v[15] * v[58] * -3.67320510363811e-06 - v[26] * v[37]) * v[39];
   v[28] = v[33] * v[25] + v[58] * v[28] + v[37] * v[21];
   v[25] = -0.999999999993254 * v[58] + -3.67320510363811e-06 * v[33];
   v[58] = v[33] * v[22] + v[58] * v[27] + v[37] * v[24];
   jac[27] = 0 - (v[45] * v[28] + v[1] * v[25] * -7.34641020664359e-06 + v[1] * v[58] * -0.999999999973015) * v[18] + (v[45] * v[25] * -7.34641020664359e-06 + v[45] * v[58] * -0.999999999973015 - v[1] * v[28]) * v[31];
   v[20] = v[25] * v[17] + v[58] * v[20] + v[28] * v[13];
   v[17] = 7.34641020664359e-06 * v[58] + -0.999999999973015 * v[25];
   v[58] = v[25] * v[14] + v[58] * v[19] + v[28] * v[16];
   jac[28] = 0 - (v[6] * v[20] + v[0] * v[17] * 0.999999999993254 + v[0] * v[58] * -3.67320510363811e-06) * v[10] + (v[6] * v[17] * 0.999999999993254 + v[6] * v[58] * -3.67320510363811e-06 - v[0] * v[20]) * v[23];
   jac[29] = 0 - (0.327454999998506 * (-0.999999999993254 * v[58] + -3.67320510363811e-06 * v[17]) * -0.999999999993254 + 0.327454999998506 * (v[17] * v[5] + v[58] * v[11] + v[20] * v[8]) * -3.67320510363811e-06) * v[7] + (0 - 0.327454999998506 * (v[17] * v[9] + v[58] * v[12] + v[20] * v[4])) * v[3];
   // dependent variables without operations
   jac[0] = 1;
   jac[10] = 1;
   jac[20] = 1;
}

