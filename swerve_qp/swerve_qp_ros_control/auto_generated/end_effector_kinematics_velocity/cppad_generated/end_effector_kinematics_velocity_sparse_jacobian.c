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

void end_effector_kinematics_velocity_sparse_jacobian(double const *const * in,
                                                      double*const * out,
                                                      struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[171];

   v[0] = cos(x[19]);
   v[1] = -0.999999999993254 * v[0];
   v[2] = x[5] * x[21];
   v[2] = v[2] + v[2];
   v[3] = x[3] * x[22] - x[4] * x[21];
   v[3] = v[3] + v[3];
   v[4] = 0 - x[5] * x[22];
   v[4] = v[4] + v[4];
   v[5] = 0 - x[3] * x[23];
   v[5] = v[5] + v[5];
   v[6] = x[4] * x[23];
   v[6] = v[6] + v[6];
   v[7] = x[6] * v[5] + x[5] * v[6];
   v[8] = x[3] * v[2] + x[6] * v[3] - x[4] * v[4] - 0.2625 * v[7];
   v[9] = x[3] * v[5] + x[23] - x[4] * v[6];
   v[10] = x[6] * v[6] - x[5] * v[5];
   v[11] = x[6] * v[2] + x[22] + x[5] * v[4] + 0.2625 * v[9] - x[3] * v[3] - 0.43093 * v[10];
   v[12] = sin(x[15]);
   v[13] = 7.34641020664359e-06 * v[12];
   v[14] = -0.999999999973015 * v[12];
   v[15] = cos(x[15]);
   v[16] = v[13] * v[9] + v[14] * v[7] + v[15] * v[10];
   v[17] = -0.999999999973015 * v[8] + -7.34641020664359e-06 * v[11] + 0.005375 * v[16];
   v[18] = 7.34641020664359e-06 * v[15];
   v[19] = -0.999999999973015 * v[15];
   v[12] = 0 - v[12];
   v[20] = x[6] * v[4] + x[21] + x[4] * v[3] + 0.43093 * v[7] - x[5] * v[2];
   v[21] = v[18] * v[8] + v[19] * v[11] + v[12] * v[20] - -0.12838 * v[16];
   v[22] = sin(x[16]);
   v[23] = 0.999999999993254 * v[22];
   v[24] = -0.999999999973015 * v[9] + -7.34641020664359e-06 * v[7] + x[32];
   v[25] = -3.67320510363811e-06 * v[22];
   v[26] = v[18] * v[9] + v[19] * v[7] + v[12] * v[10];
   v[27] = cos(x[16]);
   v[28] = v[23] * v[24] + v[25] * v[26] + v[27] * v[16];
   v[29] = -3.67320510363811e-06 * v[17] + -0.999999999993254 * v[21] + -0.41 * v[28];
   v[30] = 0.999999999993254 * v[27];
   v[31] = -3.67320510363811e-06 * v[27];
   v[22] = 0 - v[22];
   v[32] = v[13] * v[8] + v[14] * v[11] + v[15] * v[20] + -0.12838 * v[26] - 0.005375 * v[24];
   v[33] = v[30] * v[17] + v[31] * v[21] + v[22] * v[32];
   v[34] = sin(x[17]);
   v[35] = -7.34641020664359e-06 * v[34];
   v[36] = -3.67320510363811e-06 * v[24] + -0.999999999993254 * v[26] + x[33];
   v[37] = -0.999999999973015 * v[34];
   v[38] = v[30] * v[24] + v[31] * v[26] + v[22] * v[16];
   v[39] = cos(x[17]);
   v[40] = v[35] * v[36] + v[37] * v[38] + v[39] * v[28];
   v[41] = -0.999999999973015 * v[29] + 7.34641020664359e-06 * v[33] + 0.20843 * v[40];
   v[42] = -7.34641020664359e-06 * v[39];
   v[43] = -0.999999999973015 * v[39];
   v[34] = 0 - v[34];
   v[44] = v[23] * v[17] + v[25] * v[21] + v[27] * v[32] - -0.41 * v[36];
   v[45] = v[42] * v[29] + v[43] * v[33] + v[34] * v[44] - -0.006375 * v[40];
   v[46] = sin(x[18]);
   v[47] = 0.999999999993254 * v[46];
   v[48] = -0.999999999973015 * v[36] + 7.34641020664359e-06 * v[38] + x[34];
   v[49] = -3.67320510363811e-06 * v[46];
   v[50] = v[42] * v[36] + v[43] * v[38] + v[34] * v[28];
   v[51] = cos(x[18]);
   v[52] = v[47] * v[48] + v[49] * v[50] + v[51] * v[40];
   v[53] = -3.67320510363811e-06 * v[41] + -0.999999999993254 * v[45] + -0.00017505 * v[52];
   v[54] = -3.67320510363811e-06 * v[0];
   v[55] = 0.999999999993254 * v[51];
   v[56] = -3.67320510363811e-06 * v[51];
   v[46] = 0 - v[46];
   v[57] = v[35] * v[29] + v[37] * v[33] + v[39] * v[44] + -0.006375 * v[50] - 0.20843 * v[48];
   v[58] = v[55] * v[41] + v[56] * v[45] + v[46] * v[57] - -0.10593 * v[52];
   v[59] = sin(x[19]);
   v[60] = 0 - v[59];
   v[61] = v[55] * v[48] + v[56] * v[50] + v[46] * v[40];
   v[62] = -3.67320510363811e-06 * v[48] + -0.999999999993254 * v[50] + x[35];
   v[63] = v[47] * v[41] + v[49] * v[45] + v[51] * v[57] + -0.10593 * v[61] - -0.00017505 * v[62];
   v[64] = -0.999999999993254 * v[59];
   v[59] = -3.67320510363811e-06 * v[59];
   v[65] = v[64] * v[62] + v[59] * v[61] + v[0] * v[52];
   v[66] = v[1] * v[53] + v[54] * v[58] + v[60] * v[63] - -0.00017505 * v[65];
   v[67] = -3.67320510363811e-06 * v[53] + 0.999999999993254 * v[58] + 0.10593 * v[65];
   v[68] = -0.999999999993254 * v[66] + -3.67320510363811e-06 * v[67];
   v[69] = cos(x[20]);
   v[70] = 0.999999999993254 * v[69];
   v[71] = -3.67320510363811e-06 * v[69];
   v[72] = sin(x[20]);
   v[73] = 0 - v[72];
   v[74] = v[1] * v[62] + v[54] * v[61] + v[60] * v[52];
   v[75] = -3.67320510363811e-06 * v[62] + 0.999999999993254 * v[61] + x[36];
   v[76] = v[64] * v[53] + v[59] * v[58] + v[0] * v[63] + -0.00017505 * v[74] - 0.10593 * v[75];
   v[77] = 0.999999999993254 * v[72];
   v[72] = -3.67320510363811e-06 * v[72];
   v[78] = v[70] * v[67] + v[71] * v[66] + v[73] * v[76] - -0.221525 * (v[77] * v[75] + v[72] * v[74] + v[69] * v[65]);
   v[79] = v[77] * v[67] + v[72] * v[66] + v[69] * v[76] + -0.221525 * (v[70] * v[75] + v[71] * v[74] + v[73] * v[65]);
   v[80] = v[68] * -3.67320510363811e-06 + v[78] * v[70] + v[79] * v[77];
   v[81] = v[68] * -0.999999999993254 + v[78] * v[71] + v[79] * v[72];
   v[82] = v[78] * v[73] + v[79] * v[69];
   v[83] = v[80] * -3.67320510363811e-06 + v[81] * v[1] + v[82] * v[64];
   v[80] = v[80] * 0.999999999993254 + v[81] * v[54] + v[82] * v[59];
   v[84] = v[81] * v[60] + v[82] * v[0];
   v[85] = v[83] * -3.67320510363811e-06 + v[80] * v[55] + v[84] * v[47];
   v[83] = v[83] * -0.999999999993254 + v[80] * v[56] + v[84] * v[49];
   v[86] = v[80] * v[46] + v[84] * v[51];
   v[87] = v[85] * -0.999999999973015 + v[83] * v[42] + v[86] * v[35];
   v[85] = v[85] * 7.34641020664359e-06 + v[83] * v[43] + v[86] * v[37];
   v[88] = v[83] * v[34] + v[86] * v[39];
   v[89] = v[87] * -3.67320510363811e-06 + v[85] * v[30] + v[88] * v[23];
   v[87] = v[87] * -0.999999999993254 + v[85] * v[31] + v[88] * v[25];
   v[90] = v[85] * v[22] + v[88] * v[27];
   v[91] = v[89] * -0.999999999973015 + v[87] * v[18] + v[90] * v[13];
   v[92] = 2. * x[5];
   v[89] = v[89] * -7.34641020664359e-06 + v[87] * v[19] + v[90] * v[14];
   v[93] = 2. * x[4];
   v[94] = v[93] * x[3];
   v[95] = v[92] * x[6];
   v[96] = v[94] - v[95];
   v[97] = v[92] * x[3];
   v[98] = v[93] * x[6];
   v[99] = v[97] + v[98];
   v[100] = -7.34641020664359e-06 * v[96] + -0.999999999973015 * v[99];
   v[101] = v[92] * x[5];
   v[102] = v[93] * x[4];
   v[103] = 1 - v[101] - v[102];
   v[104] = v[99] * v[18] + v[96] * v[19] + v[103] * v[12];
   v[105] = v[99] * v[13] + v[96] * v[14] + v[103] * v[15];
   v[106] = v[100] * v[30] + v[104] * v[31] + v[105] * v[22];
   v[107] = -0.999999999993254 * v[104] + -3.67320510363811e-06 * v[100];
   v[108] = 7.34641020664359e-06 * v[106] + -0.999999999973015 * v[107];
   v[109] = v[100] * v[23] + v[104] * v[25] + v[105] * v[27];
   v[110] = v[107] * v[42] + v[106] * v[43] + v[109] * v[34];
   v[111] = v[107] * v[35] + v[106] * v[37] + v[109] * v[39];
   v[112] = v[108] * v[55] + v[110] * v[56] + v[111] * v[46];
   v[113] = -0.999999999993254 * v[110] + -3.67320510363811e-06 * v[108];
   v[114] = 0.999999999993254 * v[112] + -3.67320510363811e-06 * v[113];
   v[115] = v[108] * v[47] + v[110] * v[49] + v[111] * v[51];
   v[116] = v[113] * v[1] + v[112] * v[54] + v[115] * v[60];
   v[117] = v[113] * v[64] + v[112] * v[59] + v[115] * v[0];
   v[118] = v[114] * v[77] + v[116] * v[72] + v[117] * v[69];
   v[119] = v[118] * -0.221525;
   v[120] = v[114] * v[70] + v[116] * v[71] + v[117] * v[73];
   v[121] = (0 - v[120]) * -0.221525;
   v[122] = v[120] * v[73] + v[118] * v[69];
   jac[17] = v[119] * v[70] + v[121] * v[77] + (0 - v[122]) * 0.10593;
   v[123] = v[119] * v[71] + v[121] * v[72] + v[122] * -0.00017505;
   v[124] = -0.999999999993254 * v[116] + -3.67320510363811e-06 * v[114];
   v[125] = v[124] * -0.999999999993254 + v[120] * v[71] + v[118] * v[72];
   v[124] = v[124] * -3.67320510363811e-06 + v[120] * v[70] + v[118] * v[77];
   v[126] = v[119] * v[73] + v[121] * v[69] + (0 - v[125]) * -0.00017505 + v[124] * 0.10593;
   v[127] = v[125] * v[60] + v[122] * v[0];
   jac[16] = jac[17] * -3.67320510363811e-06 + v[123] * v[1] + v[126] * v[64] + (0 - v[127]) * -0.00017505;
   v[128] = jac[17] * 0.999999999993254 + v[123] * v[54] + v[126] * v[59] + v[127] * -0.10593;
   v[129] = v[124] * 0.999999999993254 + v[125] * v[54] + v[122] * v[59];
   v[124] = v[124] * -3.67320510363811e-06 + v[125] * v[1] + v[122] * v[64];
   v[130] = v[123] * v[60] + v[126] * v[0] + (0 - v[129]) * -0.10593 + v[124] * -0.00017505;
   v[131] = v[129] * v[46] + v[127] * v[51];
   jac[15] = jac[16] * -3.67320510363811e-06 + v[128] * v[55] + v[130] * v[47] + (0 - v[131]) * 0.20843;
   v[132] = jac[16] * -0.999999999993254 + v[128] * v[56] + v[130] * v[49] + v[131] * -0.006375;
   v[133] = v[124] * -0.999999999993254 + v[129] * v[56] + v[127] * v[49];
   v[124] = v[124] * -3.67320510363811e-06 + v[129] * v[55] + v[127] * v[47];
   v[134] = v[128] * v[46] + v[130] * v[51] + (0 - v[133]) * -0.006375 + v[124] * 0.20843;
   v[135] = v[133] * v[34] + v[131] * v[39];
   jac[14] = jac[15] * -0.999999999973015 + v[132] * v[42] + v[134] * v[35] + (0 - v[135]) * -0.41;
   v[136] = jac[15] * 7.34641020664359e-06 + v[132] * v[43] + v[134] * v[37];
   v[137] = v[124] * -0.999999999973015 + v[133] * v[42] + v[131] * v[35];
   v[138] = v[132] * v[34] + v[134] * v[39] + v[137] * -0.41;
   v[124] = v[124] * 7.34641020664359e-06 + v[133] * v[43] + v[131] * v[37];
   v[139] = v[124] * v[22] + v[135] * v[27];
   jac[13] = jac[14] * -3.67320510363811e-06 + v[136] * v[30] + v[138] * v[23] + (0 - v[139]) * 0.005375;
   v[140] = jac[14] * -0.999999999993254 + v[136] * v[31] + v[138] * v[25] + v[139] * -0.12838;
   v[141] = v[137] * -0.999999999993254 + v[124] * v[31] + v[135] * v[25];
   v[137] = v[137] * -3.67320510363811e-06 + v[124] * v[30] + v[135] * v[23];
   v[142] = v[136] * v[22] + v[138] * v[27] + (0 - v[141]) * -0.12838 + v[137] * 0.005375;
   v[143] = v[137] * -7.34641020664359e-06 + v[141] * v[19] + v[139] * v[14];
   v[144] = jac[13] * -0.999999999973015 + v[140] * v[18] + v[142] * v[13] + v[143] * 0.2625;
   v[145] = v[141] * v[12] + v[139] * v[15];
   v[137] = v[137] * -0.999999999973015 + v[141] * v[18] + v[139] * v[13];
   v[146] = jac[13] * -7.34641020664359e-06 + v[140] * v[19] + v[142] * v[14] + v[145] * 0.43093 + (0 - v[137]) * 0.2625;
   v[147] = v[140] * v[12] + v[142] * v[15] + (0 - v[143]) * 0.43093;
   v[148] = 0 - v[147];
   v[149] = v[146] * x[6] + v[148] * x[5] + v[144] * x[3];
   v[149] = 0 - (v[149] + v[149]);
   v[150] = 0 - v[143];
   v[151] = v[137] * x[6] + v[145] * x[4] + v[150] * x[3];
   v[151] = v[151] + v[151];
   jac[0] = v[91] * v[92] + v[89] * v[93] + v[144] * v[5] + v[149] * x[23] + v[150] * v[3] + v[137] * v[2] + v[151] * x[22];
   v[150] = v[87] * v[12] + v[90] * v[15];
   v[152] = 0 - v[150];
   v[153] = 0 - v[144];
   v[154] = v[147] * x[6] + v[146] * x[5] + v[153] * x[4];
   v[154] = v[154] + v[154];
   v[155] = 0 - v[137];
   v[156] = 0 - v[151];
   jac[1] = v[152] * v[93] + (v[152] * x[4] + v[89] * x[3] + v[91] * x[6]) * 2. + v[153] * v[6] + v[154] * x[23] + v[145] * v[3] + v[155] * v[4] + v[156] * x[21];
   v[150] = 0 - v[150];
   v[89] = 0 - v[89];
   v[153] = 0 - v[145];
   v[155] = v[145] * x[6] + v[143] * x[5] + v[155] * x[4];
   v[155] = 0 - (v[155] + v[155]);
   v[152] = v[143] * x[6] + v[153] * x[5] + v[137] * x[3];
   v[152] = v[152] + v[152];
   jac[2] = v[150] * v[92] + (v[150] * x[5] + v[91] * x[3] + v[89] * x[6]) * 2. + v[148] * v[5] + v[146] * v[6] + v[153] * v[2] + v[143] * v[4] + v[155] * x[22] + v[152] * x[21];
   jac[3] = v[89] * v[92] + v[91] * v[93] + v[146] * v[5] + v[147] * v[6] + v[137] * v[3] + v[143] * v[2] + v[145] * v[4];
   v[89] = sin(x[15]);
   v[147] = cos(x[15]);
   jac[4] = 0 - (v[142] * v[10] + v[139] * v[20] + v[90] * v[103] + (v[140] * v[9] + v[141] * v[8] + v[87] * v[99]) * 7.34641020664359e-06 + (v[140] * v[7] + v[141] * v[11] + v[87] * v[96]) * -0.999999999973015) * v[89] + ((v[142] * v[9] + v[139] * v[8] + v[90] * v[99]) * 7.34641020664359e-06 + (v[142] * v[7] + v[139] * v[11] + v[90] * v[96]) * -0.999999999973015 - (v[140] * v[10] + v[141] * v[20] + v[87] * v[103])) * v[147];
   v[142] = sin(x[16]);
   v[141] = cos(x[16]);
   jac[5] = 0 - (v[138] * v[16] + v[135] * v[32] + v[88] * v[105] + (v[136] * v[24] + v[124] * v[17] + v[85] * v[100]) * 0.999999999993254 + (v[136] * v[26] + v[124] * v[21] + v[85] * v[104]) * -3.67320510363811e-06) * v[142] + ((v[138] * v[24] + v[135] * v[17] + v[88] * v[100]) * 0.999999999993254 + (v[138] * v[26] + v[135] * v[21] + v[88] * v[104]) * -3.67320510363811e-06 - (v[136] * v[16] + v[124] * v[32] + v[85] * v[105])) * v[141];
   v[124] = sin(x[17]);
   v[138] = cos(x[17]);
   jac[6] = 0 - (v[134] * v[28] + v[131] * v[44] + v[86] * v[109] + (v[132] * v[36] + v[133] * v[29] + v[83] * v[107]) * -7.34641020664359e-06 + (v[132] * v[38] + v[133] * v[33] + v[83] * v[106]) * -0.999999999973015) * v[124] + ((v[134] * v[36] + v[131] * v[29] + v[86] * v[107]) * -7.34641020664359e-06 + (v[134] * v[38] + v[131] * v[33] + v[86] * v[106]) * -0.999999999973015 - (v[132] * v[28] + v[133] * v[44] + v[83] * v[109])) * v[138];
   v[134] = sin(x[18]);
   v[133] = cos(x[18]);
   jac[7] = 0 - (v[130] * v[40] + v[127] * v[57] + v[84] * v[111] + (v[128] * v[48] + v[129] * v[41] + v[80] * v[108]) * 0.999999999993254 + (v[128] * v[50] + v[129] * v[45] + v[80] * v[110]) * -3.67320510363811e-06) * v[134] + ((v[130] * v[48] + v[127] * v[41] + v[84] * v[108]) * 0.999999999993254 + (v[130] * v[50] + v[127] * v[45] + v[84] * v[110]) * -3.67320510363811e-06 - (v[128] * v[40] + v[129] * v[57] + v[80] * v[111])) * v[133];
   v[130] = sin(x[19]);
   v[129] = cos(x[19]);
   jac[8] = 0 - (v[126] * v[52] + v[122] * v[63] + v[82] * v[115] + (v[123] * v[62] + v[125] * v[53] + v[81] * v[113]) * -0.999999999993254 + (v[123] * v[61] + v[125] * v[58] + v[81] * v[112]) * -3.67320510363811e-06) * v[130] + ((v[126] * v[62] + v[122] * v[53] + v[82] * v[113]) * -0.999999999993254 + (v[126] * v[61] + v[122] * v[58] + v[82] * v[112]) * -3.67320510363811e-06 - (v[123] * v[52] + v[125] * v[63] + v[81] * v[115])) * v[129];
   v[126] = sin(x[20]);
   v[125] = cos(x[20]);
   jac[9] = 0 - (v[121] * v[65] + v[118] * v[76] + v[79] * v[117] + (v[119] * v[75] + v[120] * v[67] + v[78] * v[114]) * 0.999999999993254 + (v[119] * v[74] + v[120] * v[66] + v[78] * v[116]) * -3.67320510363811e-06) * v[126] + ((v[121] * v[75] + v[118] * v[67] + v[79] * v[114]) * 0.999999999993254 + (v[121] * v[74] + v[118] * v[66] + v[79] * v[116]) * -3.67320510363811e-06 - (v[119] * v[65] + v[120] * v[76] + v[78] * v[117])) * v[125];
   jac[10] = v[145] + v[152] * x[5] + v[156] * x[4];
   jac[11] = v[143] + v[155] * x[5] + v[151] * x[3];
   jac[12] = v[144] + v[154] * x[4] + v[149] * x[3];
   v[154] = v[68] * -3.67320510363811e-06 + v[78] * v[70] + v[79] * v[77];
   v[149] = v[68] * -0.999999999993254 + v[78] * v[71] + v[79] * v[72];
   v[144] = v[78] * v[73] + v[79] * v[69];
   v[155] = v[154] * -3.67320510363811e-06 + v[149] * v[1] + v[144] * v[64];
   v[154] = v[154] * 0.999999999993254 + v[149] * v[54] + v[144] * v[59];
   v[151] = v[149] * v[60] + v[144] * v[0];
   v[143] = v[155] * -3.67320510363811e-06 + v[154] * v[55] + v[151] * v[47];
   v[155] = v[155] * -0.999999999993254 + v[154] * v[56] + v[151] * v[49];
   v[152] = v[154] * v[46] + v[151] * v[51];
   v[156] = v[143] * -0.999999999973015 + v[155] * v[42] + v[152] * v[35];
   v[143] = v[143] * 7.34641020664359e-06 + v[155] * v[43] + v[152] * v[37];
   v[145] = v[155] * v[34] + v[152] * v[39];
   v[121] = v[156] * -0.999999999993254 + v[143] * v[31] + v[145] * v[25];
   v[120] = v[143] * v[22] + v[145] * v[27];
   v[119] = v[121] * v[12] + v[120] * v[15];
   v[156] = v[156] * -3.67320510363811e-06 + v[143] * v[30] + v[145] * v[23];
   v[118] = v[156] * -7.34641020664359e-06 + v[121] * v[19] + v[120] * v[14];
   v[117] = 0 - v[118];
   v[116] = 2. * x[3];
   v[156] = v[156] * -0.999999999973015 + v[121] * v[18] + v[120] * v[13];
   v[114] = 0 - v[156];
   v[123] = v[116] * x[3];
   v[101] = 1 - v[101] - v[123];
   v[122] = v[92] * x[4];
   v[115] = v[116] * x[6];
   v[113] = v[122] - v[115];
   v[112] = -7.34641020664359e-06 * v[101] + -0.999999999973015 * v[113];
   v[95] = v[94] + v[95];
   v[94] = v[113] * v[18] + v[101] * v[19] + v[95] * v[12];
   v[82] = v[113] * v[13] + v[101] * v[14] + v[95] * v[15];
   v[81] = v[112] * v[30] + v[94] * v[31] + v[82] * v[22];
   v[128] = -0.999999999993254 * v[94] + -3.67320510363811e-06 * v[112];
   v[127] = 7.34641020664359e-06 * v[81] + -0.999999999973015 * v[128];
   v[111] = v[112] * v[23] + v[94] * v[25] + v[82] * v[27];
   v[110] = v[128] * v[42] + v[81] * v[43] + v[111] * v[34];
   v[108] = v[128] * v[35] + v[81] * v[37] + v[111] * v[39];
   v[84] = v[127] * v[55] + v[110] * v[56] + v[108] * v[46];
   v[80] = -0.999999999993254 * v[110] + -3.67320510363811e-06 * v[127];
   v[132] = 0.999999999993254 * v[84] + -3.67320510363811e-06 * v[80];
   v[131] = v[127] * v[47] + v[110] * v[49] + v[108] * v[51];
   v[109] = v[80] * v[1] + v[84] * v[54] + v[131] * v[60];
   v[107] = v[80] * v[64] + v[84] * v[59] + v[131] * v[0];
   v[106] = v[132] * v[77] + v[109] * v[72] + v[107] * v[69];
   v[86] = v[106] * -0.221525;
   v[83] = v[132] * v[70] + v[109] * v[71] + v[107] * v[73];
   v[136] = (0 - v[83]) * -0.221525;
   v[135] = v[83] * v[73] + v[106] * v[69];
   jac[35] = v[86] * v[70] + v[136] * v[77] + (0 - v[135]) * 0.10593;
   v[105] = v[86] * v[71] + v[136] * v[72] + v[135] * -0.00017505;
   v[104] = -0.999999999993254 * v[109] + -3.67320510363811e-06 * v[132];
   v[100] = v[104] * -0.999999999993254 + v[83] * v[71] + v[106] * v[72];
   v[104] = v[104] * -3.67320510363811e-06 + v[83] * v[70] + v[106] * v[77];
   v[88] = v[86] * v[73] + v[136] * v[69] + (0 - v[100]) * -0.00017505 + v[104] * 0.10593;
   v[85] = v[100] * v[60] + v[135] * v[0];
   jac[34] = jac[35] * -3.67320510363811e-06 + v[105] * v[1] + v[88] * v[64] + (0 - v[85]) * -0.00017505;
   v[140] = jac[35] * 0.999999999993254 + v[105] * v[54] + v[88] * v[59] + v[85] * -0.10593;
   v[139] = v[104] * 0.999999999993254 + v[100] * v[54] + v[135] * v[59];
   v[104] = v[104] * -3.67320510363811e-06 + v[100] * v[1] + v[135] * v[64];
   v[103] = v[105] * v[60] + v[88] * v[0] + (0 - v[139]) * -0.10593 + v[104] * -0.00017505;
   v[99] = v[139] * v[46] + v[85] * v[51];
   jac[33] = jac[34] * -3.67320510363811e-06 + v[140] * v[55] + v[103] * v[47] + (0 - v[99]) * 0.20843;
   v[96] = jac[34] * -0.999999999993254 + v[140] * v[56] + v[103] * v[49] + v[99] * -0.006375;
   v[90] = v[104] * -0.999999999993254 + v[139] * v[56] + v[85] * v[49];
   v[104] = v[104] * -3.67320510363811e-06 + v[139] * v[55] + v[85] * v[47];
   v[87] = v[140] * v[46] + v[103] * v[51] + (0 - v[90]) * -0.006375 + v[104] * 0.20843;
   v[146] = v[90] * v[34] + v[99] * v[39];
   jac[32] = jac[33] * -0.999999999973015 + v[96] * v[42] + v[87] * v[35] + (0 - v[146]) * -0.41;
   v[137] = jac[33] * 7.34641020664359e-06 + v[96] * v[43] + v[87] * v[37];
   v[91] = v[104] * -0.999999999973015 + v[90] * v[42] + v[99] * v[35];
   v[153] = v[96] * v[34] + v[87] * v[39] + v[91] * -0.41;
   v[104] = v[104] * 7.34641020664359e-06 + v[90] * v[43] + v[99] * v[37];
   v[150] = v[104] * v[22] + v[146] * v[27];
   jac[31] = jac[32] * -3.67320510363811e-06 + v[137] * v[30] + v[153] * v[23] + (0 - v[150]) * 0.005375;
   v[148] = jac[32] * -0.999999999993254 + v[137] * v[31] + v[153] * v[25] + v[150] * -0.12838;
   v[157] = v[91] * -0.999999999993254 + v[104] * v[31] + v[146] * v[25];
   v[91] = v[91] * -3.67320510363811e-06 + v[104] * v[30] + v[146] * v[23];
   v[158] = v[137] * v[22] + v[153] * v[27] + (0 - v[157]) * -0.12838 + v[91] * 0.005375;
   v[159] = v[91] * -7.34641020664359e-06 + v[157] * v[19] + v[150] * v[14];
   v[160] = jac[31] * -0.999999999973015 + v[148] * v[18] + v[158] * v[13] + v[159] * 0.2625;
   v[161] = v[157] * v[12] + v[150] * v[15];
   v[91] = v[91] * -0.999999999973015 + v[157] * v[18] + v[150] * v[13];
   v[162] = jac[31] * -7.34641020664359e-06 + v[148] * v[19] + v[158] * v[14] + v[161] * 0.43093 + (0 - v[91]) * 0.2625;
   v[163] = v[148] * v[12] + v[158] * v[15] + (0 - v[159]) * 0.43093;
   v[164] = 0 - v[163];
   v[165] = v[162] * x[6] + v[164] * x[5] + v[160] * x[3];
   v[165] = 0 - (v[165] + v[165]);
   v[166] = 0 - v[159];
   v[167] = v[91] * x[6] + v[161] * x[4] + v[166] * x[3];
   v[167] = v[167] + v[167];
   jac[18] = v[119] * v[93] + v[117] * v[116] + (v[117] * x[3] + v[114] * x[6]) * 2. + v[160] * v[5] + v[165] * x[23] + v[166] * v[3] + v[91] * v[2] + v[167] * x[22];
   v[166] = 0 - v[160];
   v[117] = v[163] * x[6] + v[162] * x[5] + v[166] * x[4];
   v[117] = v[117] + v[117];
   v[168] = 0 - v[91];
   v[169] = 0 - v[167];
   jac[19] = v[156] * v[92] + v[119] * x[3] * 2. + v[166] * v[6] + v[117] * x[23] + v[161] * v[3] + v[168] * v[4] + v[169] * x[21];
   v[118] = 0 - v[118];
   v[166] = 0 - v[161];
   v[168] = v[161] * x[6] + v[159] * x[5] + v[168] * x[4];
   v[168] = 0 - (v[168] + v[168]);
   v[170] = v[159] * x[6] + v[166] * x[5] + v[91] * x[3];
   v[170] = v[170] + v[170];
   jac[20] = v[118] * v[92] + (v[118] * x[5] + v[156] * x[4] + v[119] * x[6]) * 2. + v[164] * v[5] + v[162] * v[6] + v[166] * v[2] + v[159] * v[4] + v[168] * x[22] + v[170] * x[21];
   jac[21] = v[119] * v[92] + v[114] * v[116] + v[162] * v[5] + v[163] * v[6] + v[91] * v[3] + v[159] * v[2] + v[161] * v[4];
   jac[22] = 0 - (v[158] * v[10] + v[150] * v[20] + v[120] * v[95] + (v[148] * v[9] + v[157] * v[8] + v[121] * v[113]) * 7.34641020664359e-06 + (v[148] * v[7] + v[157] * v[11] + v[121] * v[101]) * -0.999999999973015) * v[89] + ((v[158] * v[9] + v[150] * v[8] + v[120] * v[113]) * 7.34641020664359e-06 + (v[158] * v[7] + v[150] * v[11] + v[120] * v[101]) * -0.999999999973015 - (v[148] * v[10] + v[157] * v[20] + v[121] * v[95])) * v[147];
   jac[23] = 0 - (v[153] * v[16] + v[146] * v[32] + v[145] * v[82] + (v[137] * v[24] + v[104] * v[17] + v[143] * v[112]) * 0.999999999993254 + (v[137] * v[26] + v[104] * v[21] + v[143] * v[94]) * -3.67320510363811e-06) * v[142] + ((v[153] * v[24] + v[146] * v[17] + v[145] * v[112]) * 0.999999999993254 + (v[153] * v[26] + v[146] * v[21] + v[145] * v[94]) * -3.67320510363811e-06 - (v[137] * v[16] + v[104] * v[32] + v[143] * v[82])) * v[141];
   jac[24] = 0 - (v[87] * v[28] + v[99] * v[44] + v[152] * v[111] + (v[96] * v[36] + v[90] * v[29] + v[155] * v[128]) * -7.34641020664359e-06 + (v[96] * v[38] + v[90] * v[33] + v[155] * v[81]) * -0.999999999973015) * v[124] + ((v[87] * v[36] + v[99] * v[29] + v[152] * v[128]) * -7.34641020664359e-06 + (v[87] * v[38] + v[99] * v[33] + v[152] * v[81]) * -0.999999999973015 - (v[96] * v[28] + v[90] * v[44] + v[155] * v[111])) * v[138];
   jac[25] = 0 - (v[103] * v[40] + v[85] * v[57] + v[151] * v[108] + (v[140] * v[48] + v[139] * v[41] + v[154] * v[127]) * 0.999999999993254 + (v[140] * v[50] + v[139] * v[45] + v[154] * v[110]) * -3.67320510363811e-06) * v[134] + ((v[103] * v[48] + v[85] * v[41] + v[151] * v[127]) * 0.999999999993254 + (v[103] * v[50] + v[85] * v[45] + v[151] * v[110]) * -3.67320510363811e-06 - (v[140] * v[40] + v[139] * v[57] + v[154] * v[108])) * v[133];
   jac[26] = 0 - (v[88] * v[52] + v[135] * v[63] + v[144] * v[131] + (v[105] * v[62] + v[100] * v[53] + v[149] * v[80]) * -0.999999999993254 + (v[105] * v[61] + v[100] * v[58] + v[149] * v[84]) * -3.67320510363811e-06) * v[130] + ((v[88] * v[62] + v[135] * v[53] + v[144] * v[80]) * -0.999999999993254 + (v[88] * v[61] + v[135] * v[58] + v[144] * v[84]) * -3.67320510363811e-06 - (v[105] * v[52] + v[100] * v[63] + v[149] * v[131])) * v[129];
   jac[27] = 0 - (v[136] * v[65] + v[106] * v[76] + v[79] * v[107] + (v[86] * v[75] + v[83] * v[67] + v[78] * v[132]) * 0.999999999993254 + (v[86] * v[74] + v[83] * v[66] + v[78] * v[109]) * -3.67320510363811e-06) * v[126] + ((v[136] * v[75] + v[106] * v[67] + v[79] * v[132]) * 0.999999999993254 + (v[136] * v[74] + v[106] * v[66] + v[79] * v[109]) * -3.67320510363811e-06 - (v[86] * v[65] + v[83] * v[76] + v[78] * v[107])) * v[125];
   jac[28] = v[161] + v[170] * x[5] + v[169] * x[4];
   jac[29] = v[159] + v[168] * x[5] + v[167] * x[3];
   jac[30] = v[160] + v[117] * x[4] + v[165] * x[3];
   v[117] = v[68] * -3.67320510363811e-06 + v[78] * v[70] + v[79] * v[77];
   v[68] = v[68] * -0.999999999993254 + v[78] * v[71] + v[79] * v[72];
   v[165] = v[78] * v[73] + v[79] * v[69];
   v[160] = v[117] * -3.67320510363811e-06 + v[68] * v[1] + v[165] * v[64];
   v[117] = v[117] * 0.999999999993254 + v[68] * v[54] + v[165] * v[59];
   v[168] = v[68] * v[60] + v[165] * v[0];
   v[167] = v[160] * -3.67320510363811e-06 + v[117] * v[55] + v[168] * v[47];
   v[160] = v[160] * -0.999999999993254 + v[117] * v[56] + v[168] * v[49];
   v[159] = v[117] * v[46] + v[168] * v[51];
   v[170] = v[167] * -0.999999999973015 + v[160] * v[42] + v[159] * v[35];
   v[167] = v[167] * 7.34641020664359e-06 + v[160] * v[43] + v[159] * v[37];
   v[169] = v[160] * v[34] + v[159] * v[39];
   v[161] = v[170] * -0.999999999993254 + v[167] * v[31] + v[169] * v[25];
   v[136] = v[167] * v[22] + v[169] * v[27];
   v[83] = v[161] * v[12] + v[136] * v[15];
   v[170] = v[170] * -3.67320510363811e-06 + v[167] * v[30] + v[169] * v[23];
   v[86] = v[170] * -0.999999999973015 + v[161] * v[18] + v[136] * v[13];
   v[106] = 0 - v[86];
   v[170] = v[170] * -7.34641020664359e-06 + v[161] * v[19] + v[136] * v[14];
   v[115] = v[122] + v[115];
   v[123] = 1 - v[102] - v[123];
   v[102] = -7.34641020664359e-06 * v[115] + -0.999999999973015 * v[123];
   v[98] = v[97] - v[98];
   v[97] = v[123] * v[18] + v[115] * v[19] + v[98] * v[12];
   v[122] = v[123] * v[13] + v[115] * v[14] + v[98] * v[15];
   v[107] = v[102] * v[30] + v[97] * v[31] + v[122] * v[22];
   v[109] = -0.999999999993254 * v[97] + -3.67320510363811e-06 * v[102];
   v[132] = 7.34641020664359e-06 * v[107] + -0.999999999973015 * v[109];
   v[88] = v[102] * v[23] + v[97] * v[25] + v[122] * v[27];
   v[100] = v[109] * v[42] + v[107] * v[43] + v[88] * v[34];
   v[105] = v[109] * v[35] + v[107] * v[37] + v[88] * v[39];
   v[135] = v[132] * v[55] + v[100] * v[56] + v[105] * v[46];
   v[131] = -0.999999999993254 * v[100] + -3.67320510363811e-06 * v[132];
   v[80] = 0.999999999993254 * v[135] + -3.67320510363811e-06 * v[131];
   v[84] = v[132] * v[47] + v[100] * v[49] + v[105] * v[51];
   v[144] = v[131] * v[1] + v[135] * v[54] + v[84] * v[60];
   v[149] = v[131] * v[64] + v[135] * v[59] + v[84] * v[0];
   v[103] = v[80] * v[77] + v[144] * v[72] + v[149] * v[69];
   v[139] = v[103] * -0.221525;
   v[140] = v[80] * v[70] + v[144] * v[71] + v[149] * v[73];
   v[85] = (0 - v[140]) * -0.221525;
   v[108] = v[140] * v[73] + v[103] * v[69];
   jac[53] = v[139] * v[70] + v[85] * v[77] + (0 - v[108]) * 0.10593;
   v[110] = v[139] * v[71] + v[85] * v[72] + v[108] * -0.00017505;
   v[127] = -0.999999999993254 * v[144] + -3.67320510363811e-06 * v[80];
   v[72] = v[127] * -0.999999999993254 + v[140] * v[71] + v[103] * v[72];
   v[127] = v[127] * -3.67320510363811e-06 + v[140] * v[70] + v[103] * v[77];
   v[73] = v[139] * v[73] + v[85] * v[69] + (0 - v[72]) * -0.00017505 + v[127] * 0.10593;
   v[69] = v[72] * v[60] + v[108] * v[0];
   jac[52] = jac[53] * -3.67320510363811e-06 + v[110] * v[1] + v[73] * v[64] + (0 - v[69]) * -0.00017505;
   v[77] = jac[53] * 0.999999999993254 + v[110] * v[54] + v[73] * v[59] + v[69] * -0.10593;
   v[59] = v[127] * 0.999999999993254 + v[72] * v[54] + v[108] * v[59];
   v[127] = v[127] * -3.67320510363811e-06 + v[72] * v[1] + v[108] * v[64];
   v[60] = v[110] * v[60] + v[73] * v[0] + (0 - v[59]) * -0.10593 + v[127] * -0.00017505;
   v[0] = v[59] * v[46] + v[69] * v[51];
   jac[51] = jac[52] * -3.67320510363811e-06 + v[77] * v[55] + v[60] * v[47] + (0 - v[0]) * 0.20843;
   v[64] = jac[52] * -0.999999999993254 + v[77] * v[56] + v[60] * v[49] + v[0] * -0.006375;
   v[56] = v[127] * -0.999999999993254 + v[59] * v[56] + v[69] * v[49];
   v[127] = v[127] * -3.67320510363811e-06 + v[59] * v[55] + v[69] * v[47];
   v[46] = v[77] * v[46] + v[60] * v[51] + (0 - v[56]) * -0.006375 + v[127] * 0.20843;
   v[51] = v[56] * v[34] + v[0] * v[39];
   jac[50] = jac[51] * -0.999999999973015 + v[64] * v[42] + v[46] * v[35] + (0 - v[51]) * -0.41;
   v[55] = jac[51] * 7.34641020664359e-06 + v[64] * v[43] + v[46] * v[37];
   v[42] = v[127] * -0.999999999973015 + v[56] * v[42] + v[0] * v[35];
   v[34] = v[64] * v[34] + v[46] * v[39] + v[42] * -0.41;
   v[127] = v[127] * 7.34641020664359e-06 + v[56] * v[43] + v[0] * v[37];
   v[43] = v[127] * v[22] + v[51] * v[27];
   jac[49] = jac[50] * -3.67320510363811e-06 + v[55] * v[30] + v[34] * v[23] + (0 - v[43]) * 0.005375;
   v[37] = jac[50] * -0.999999999993254 + v[55] * v[31] + v[34] * v[25] + v[43] * -0.12838;
   v[31] = v[42] * -0.999999999993254 + v[127] * v[31] + v[51] * v[25];
   v[42] = v[42] * -3.67320510363811e-06 + v[127] * v[30] + v[51] * v[23];
   v[22] = v[55] * v[22] + v[34] * v[27] + (0 - v[31]) * -0.12838 + v[42] * 0.005375;
   v[27] = v[42] * -7.34641020664359e-06 + v[31] * v[19] + v[43] * v[14];
   v[30] = jac[49] * -0.999999999973015 + v[37] * v[18] + v[22] * v[13] + v[27] * 0.2625;
   v[23] = v[31] * v[12] + v[43] * v[15];
   v[42] = v[42] * -0.999999999973015 + v[31] * v[18] + v[43] * v[13];
   v[19] = jac[49] * -7.34641020664359e-06 + v[37] * v[19] + v[22] * v[14] + v[23] * 0.43093 + (0 - v[42]) * 0.2625;
   v[12] = v[37] * v[12] + v[22] * v[15] + (0 - v[27]) * 0.43093;
   v[15] = 0 - v[12];
   v[14] = v[19] * x[6] + v[15] * x[5] + v[30] * x[3];
   v[14] = 0 - (v[14] + v[14]);
   v[18] = 0 - v[27];
   v[13] = v[42] * x[6] + v[23] * x[4] + v[18] * x[3];
   v[13] = v[13] + v[13];
   jac[36] = v[83] * v[92] + v[106] * v[116] + (v[106] * x[3] + v[170] * x[6]) * 2. + v[30] * v[5] + v[14] * x[23] + v[18] * v[3] + v[42] * v[2] + v[13] * x[22];
   v[86] = 0 - v[86];
   v[18] = 0 - v[83];
   v[106] = 0 - v[30];
   v[25] = v[12] * x[6] + v[19] * x[5] + v[106] * x[4];
   v[25] = v[25] + v[25];
   v[39] = 0 - v[42];
   v[35] = 0 - v[13];
   jac[37] = v[170] * v[92] + v[86] * v[93] + (v[86] * x[4] + v[18] * x[6]) * 2. + v[106] * v[6] + v[25] * x[23] + v[23] * v[3] + v[39] * v[4] + v[35] * x[21];
   v[106] = 0 - v[23];
   v[39] = v[23] * x[6] + v[27] * x[5] + v[39] * x[4];
   v[39] = 0 - (v[39] + v[39]);
   v[86] = v[27] * x[6] + v[106] * x[5] + v[42] * x[3];
   v[86] = v[86] + v[86];
   jac[38] = (v[170] * x[4] + v[83] * x[3]) * 2. + v[15] * v[5] + v[19] * v[6] + v[106] * v[2] + v[27] * v[4] + v[39] * x[22] + v[86] * x[21];
   jac[39] = v[18] * v[93] + v[170] * v[116] + v[19] * v[5] + v[12] * v[6] + v[42] * v[3] + v[27] * v[2] + v[23] * v[4];
   jac[40] = 0 - (v[22] * v[10] + v[43] * v[20] + v[136] * v[98] + (v[37] * v[9] + v[31] * v[8] + v[161] * v[123]) * 7.34641020664359e-06 + (v[37] * v[7] + v[31] * v[11] + v[161] * v[115]) * -0.999999999973015) * v[89] + ((v[22] * v[9] + v[43] * v[8] + v[136] * v[123]) * 7.34641020664359e-06 + (v[22] * v[7] + v[43] * v[11] + v[136] * v[115]) * -0.999999999973015 - (v[37] * v[10] + v[31] * v[20] + v[161] * v[98])) * v[147];
   jac[41] = 0 - (v[34] * v[16] + v[51] * v[32] + v[169] * v[122] + (v[55] * v[24] + v[127] * v[17] + v[167] * v[102]) * 0.999999999993254 + (v[55] * v[26] + v[127] * v[21] + v[167] * v[97]) * -3.67320510363811e-06) * v[142] + ((v[34] * v[24] + v[51] * v[17] + v[169] * v[102]) * 0.999999999993254 + (v[34] * v[26] + v[51] * v[21] + v[169] * v[97]) * -3.67320510363811e-06 - (v[55] * v[16] + v[127] * v[32] + v[167] * v[122])) * v[141];
   jac[42] = 0 - (v[46] * v[28] + v[0] * v[44] + v[159] * v[88] + (v[64] * v[36] + v[56] * v[29] + v[160] * v[109]) * -7.34641020664359e-06 + (v[64] * v[38] + v[56] * v[33] + v[160] * v[107]) * -0.999999999973015) * v[124] + ((v[46] * v[36] + v[0] * v[29] + v[159] * v[109]) * -7.34641020664359e-06 + (v[46] * v[38] + v[0] * v[33] + v[159] * v[107]) * -0.999999999973015 - (v[64] * v[28] + v[56] * v[44] + v[160] * v[88])) * v[138];
   jac[43] = 0 - (v[60] * v[40] + v[69] * v[57] + v[168] * v[105] + (v[77] * v[48] + v[59] * v[41] + v[117] * v[132]) * 0.999999999993254 + (v[77] * v[50] + v[59] * v[45] + v[117] * v[100]) * -3.67320510363811e-06) * v[134] + ((v[60] * v[48] + v[69] * v[41] + v[168] * v[132]) * 0.999999999993254 + (v[60] * v[50] + v[69] * v[45] + v[168] * v[100]) * -3.67320510363811e-06 - (v[77] * v[40] + v[59] * v[57] + v[117] * v[105])) * v[133];
   jac[44] = 0 - (v[73] * v[52] + v[108] * v[63] + v[165] * v[84] + (v[110] * v[62] + v[72] * v[53] + v[68] * v[131]) * -0.999999999993254 + (v[110] * v[61] + v[72] * v[58] + v[68] * v[135]) * -3.67320510363811e-06) * v[130] + ((v[73] * v[62] + v[108] * v[53] + v[165] * v[131]) * -0.999999999993254 + (v[73] * v[61] + v[108] * v[58] + v[165] * v[135]) * -3.67320510363811e-06 - (v[110] * v[52] + v[72] * v[63] + v[68] * v[84])) * v[129];
   jac[45] = 0 - (v[85] * v[65] + v[103] * v[76] + v[79] * v[149] + (v[139] * v[75] + v[140] * v[67] + v[78] * v[80]) * 0.999999999993254 + (v[139] * v[74] + v[140] * v[66] + v[78] * v[144]) * -3.67320510363811e-06) * v[126] + ((v[85] * v[75] + v[103] * v[67] + v[79] * v[80]) * 0.999999999993254 + (v[85] * v[74] + v[103] * v[66] + v[79] * v[144]) * -3.67320510363811e-06 - (v[139] * v[65] + v[140] * v[76] + v[78] * v[149])) * v[125];
   jac[46] = v[23] + v[86] * x[5] + v[35] * x[4];
   jac[47] = v[27] + v[39] * x[5] + v[13] * x[3];
   jac[48] = v[30] + v[25] * x[4] + v[14] * x[3];
}

