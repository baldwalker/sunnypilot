#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3545767590363888806) {
   out_3545767590363888806[0] = delta_x[0] + nom_x[0];
   out_3545767590363888806[1] = delta_x[1] + nom_x[1];
   out_3545767590363888806[2] = delta_x[2] + nom_x[2];
   out_3545767590363888806[3] = delta_x[3] + nom_x[3];
   out_3545767590363888806[4] = delta_x[4] + nom_x[4];
   out_3545767590363888806[5] = delta_x[5] + nom_x[5];
   out_3545767590363888806[6] = delta_x[6] + nom_x[6];
   out_3545767590363888806[7] = delta_x[7] + nom_x[7];
   out_3545767590363888806[8] = delta_x[8] + nom_x[8];
   out_3545767590363888806[9] = delta_x[9] + nom_x[9];
   out_3545767590363888806[10] = delta_x[10] + nom_x[10];
   out_3545767590363888806[11] = delta_x[11] + nom_x[11];
   out_3545767590363888806[12] = delta_x[12] + nom_x[12];
   out_3545767590363888806[13] = delta_x[13] + nom_x[13];
   out_3545767590363888806[14] = delta_x[14] + nom_x[14];
   out_3545767590363888806[15] = delta_x[15] + nom_x[15];
   out_3545767590363888806[16] = delta_x[16] + nom_x[16];
   out_3545767590363888806[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6258502841145403309) {
   out_6258502841145403309[0] = -nom_x[0] + true_x[0];
   out_6258502841145403309[1] = -nom_x[1] + true_x[1];
   out_6258502841145403309[2] = -nom_x[2] + true_x[2];
   out_6258502841145403309[3] = -nom_x[3] + true_x[3];
   out_6258502841145403309[4] = -nom_x[4] + true_x[4];
   out_6258502841145403309[5] = -nom_x[5] + true_x[5];
   out_6258502841145403309[6] = -nom_x[6] + true_x[6];
   out_6258502841145403309[7] = -nom_x[7] + true_x[7];
   out_6258502841145403309[8] = -nom_x[8] + true_x[8];
   out_6258502841145403309[9] = -nom_x[9] + true_x[9];
   out_6258502841145403309[10] = -nom_x[10] + true_x[10];
   out_6258502841145403309[11] = -nom_x[11] + true_x[11];
   out_6258502841145403309[12] = -nom_x[12] + true_x[12];
   out_6258502841145403309[13] = -nom_x[13] + true_x[13];
   out_6258502841145403309[14] = -nom_x[14] + true_x[14];
   out_6258502841145403309[15] = -nom_x[15] + true_x[15];
   out_6258502841145403309[16] = -nom_x[16] + true_x[16];
   out_6258502841145403309[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8724642416341409662) {
   out_8724642416341409662[0] = 1.0;
   out_8724642416341409662[1] = 0.0;
   out_8724642416341409662[2] = 0.0;
   out_8724642416341409662[3] = 0.0;
   out_8724642416341409662[4] = 0.0;
   out_8724642416341409662[5] = 0.0;
   out_8724642416341409662[6] = 0.0;
   out_8724642416341409662[7] = 0.0;
   out_8724642416341409662[8] = 0.0;
   out_8724642416341409662[9] = 0.0;
   out_8724642416341409662[10] = 0.0;
   out_8724642416341409662[11] = 0.0;
   out_8724642416341409662[12] = 0.0;
   out_8724642416341409662[13] = 0.0;
   out_8724642416341409662[14] = 0.0;
   out_8724642416341409662[15] = 0.0;
   out_8724642416341409662[16] = 0.0;
   out_8724642416341409662[17] = 0.0;
   out_8724642416341409662[18] = 0.0;
   out_8724642416341409662[19] = 1.0;
   out_8724642416341409662[20] = 0.0;
   out_8724642416341409662[21] = 0.0;
   out_8724642416341409662[22] = 0.0;
   out_8724642416341409662[23] = 0.0;
   out_8724642416341409662[24] = 0.0;
   out_8724642416341409662[25] = 0.0;
   out_8724642416341409662[26] = 0.0;
   out_8724642416341409662[27] = 0.0;
   out_8724642416341409662[28] = 0.0;
   out_8724642416341409662[29] = 0.0;
   out_8724642416341409662[30] = 0.0;
   out_8724642416341409662[31] = 0.0;
   out_8724642416341409662[32] = 0.0;
   out_8724642416341409662[33] = 0.0;
   out_8724642416341409662[34] = 0.0;
   out_8724642416341409662[35] = 0.0;
   out_8724642416341409662[36] = 0.0;
   out_8724642416341409662[37] = 0.0;
   out_8724642416341409662[38] = 1.0;
   out_8724642416341409662[39] = 0.0;
   out_8724642416341409662[40] = 0.0;
   out_8724642416341409662[41] = 0.0;
   out_8724642416341409662[42] = 0.0;
   out_8724642416341409662[43] = 0.0;
   out_8724642416341409662[44] = 0.0;
   out_8724642416341409662[45] = 0.0;
   out_8724642416341409662[46] = 0.0;
   out_8724642416341409662[47] = 0.0;
   out_8724642416341409662[48] = 0.0;
   out_8724642416341409662[49] = 0.0;
   out_8724642416341409662[50] = 0.0;
   out_8724642416341409662[51] = 0.0;
   out_8724642416341409662[52] = 0.0;
   out_8724642416341409662[53] = 0.0;
   out_8724642416341409662[54] = 0.0;
   out_8724642416341409662[55] = 0.0;
   out_8724642416341409662[56] = 0.0;
   out_8724642416341409662[57] = 1.0;
   out_8724642416341409662[58] = 0.0;
   out_8724642416341409662[59] = 0.0;
   out_8724642416341409662[60] = 0.0;
   out_8724642416341409662[61] = 0.0;
   out_8724642416341409662[62] = 0.0;
   out_8724642416341409662[63] = 0.0;
   out_8724642416341409662[64] = 0.0;
   out_8724642416341409662[65] = 0.0;
   out_8724642416341409662[66] = 0.0;
   out_8724642416341409662[67] = 0.0;
   out_8724642416341409662[68] = 0.0;
   out_8724642416341409662[69] = 0.0;
   out_8724642416341409662[70] = 0.0;
   out_8724642416341409662[71] = 0.0;
   out_8724642416341409662[72] = 0.0;
   out_8724642416341409662[73] = 0.0;
   out_8724642416341409662[74] = 0.0;
   out_8724642416341409662[75] = 0.0;
   out_8724642416341409662[76] = 1.0;
   out_8724642416341409662[77] = 0.0;
   out_8724642416341409662[78] = 0.0;
   out_8724642416341409662[79] = 0.0;
   out_8724642416341409662[80] = 0.0;
   out_8724642416341409662[81] = 0.0;
   out_8724642416341409662[82] = 0.0;
   out_8724642416341409662[83] = 0.0;
   out_8724642416341409662[84] = 0.0;
   out_8724642416341409662[85] = 0.0;
   out_8724642416341409662[86] = 0.0;
   out_8724642416341409662[87] = 0.0;
   out_8724642416341409662[88] = 0.0;
   out_8724642416341409662[89] = 0.0;
   out_8724642416341409662[90] = 0.0;
   out_8724642416341409662[91] = 0.0;
   out_8724642416341409662[92] = 0.0;
   out_8724642416341409662[93] = 0.0;
   out_8724642416341409662[94] = 0.0;
   out_8724642416341409662[95] = 1.0;
   out_8724642416341409662[96] = 0.0;
   out_8724642416341409662[97] = 0.0;
   out_8724642416341409662[98] = 0.0;
   out_8724642416341409662[99] = 0.0;
   out_8724642416341409662[100] = 0.0;
   out_8724642416341409662[101] = 0.0;
   out_8724642416341409662[102] = 0.0;
   out_8724642416341409662[103] = 0.0;
   out_8724642416341409662[104] = 0.0;
   out_8724642416341409662[105] = 0.0;
   out_8724642416341409662[106] = 0.0;
   out_8724642416341409662[107] = 0.0;
   out_8724642416341409662[108] = 0.0;
   out_8724642416341409662[109] = 0.0;
   out_8724642416341409662[110] = 0.0;
   out_8724642416341409662[111] = 0.0;
   out_8724642416341409662[112] = 0.0;
   out_8724642416341409662[113] = 0.0;
   out_8724642416341409662[114] = 1.0;
   out_8724642416341409662[115] = 0.0;
   out_8724642416341409662[116] = 0.0;
   out_8724642416341409662[117] = 0.0;
   out_8724642416341409662[118] = 0.0;
   out_8724642416341409662[119] = 0.0;
   out_8724642416341409662[120] = 0.0;
   out_8724642416341409662[121] = 0.0;
   out_8724642416341409662[122] = 0.0;
   out_8724642416341409662[123] = 0.0;
   out_8724642416341409662[124] = 0.0;
   out_8724642416341409662[125] = 0.0;
   out_8724642416341409662[126] = 0.0;
   out_8724642416341409662[127] = 0.0;
   out_8724642416341409662[128] = 0.0;
   out_8724642416341409662[129] = 0.0;
   out_8724642416341409662[130] = 0.0;
   out_8724642416341409662[131] = 0.0;
   out_8724642416341409662[132] = 0.0;
   out_8724642416341409662[133] = 1.0;
   out_8724642416341409662[134] = 0.0;
   out_8724642416341409662[135] = 0.0;
   out_8724642416341409662[136] = 0.0;
   out_8724642416341409662[137] = 0.0;
   out_8724642416341409662[138] = 0.0;
   out_8724642416341409662[139] = 0.0;
   out_8724642416341409662[140] = 0.0;
   out_8724642416341409662[141] = 0.0;
   out_8724642416341409662[142] = 0.0;
   out_8724642416341409662[143] = 0.0;
   out_8724642416341409662[144] = 0.0;
   out_8724642416341409662[145] = 0.0;
   out_8724642416341409662[146] = 0.0;
   out_8724642416341409662[147] = 0.0;
   out_8724642416341409662[148] = 0.0;
   out_8724642416341409662[149] = 0.0;
   out_8724642416341409662[150] = 0.0;
   out_8724642416341409662[151] = 0.0;
   out_8724642416341409662[152] = 1.0;
   out_8724642416341409662[153] = 0.0;
   out_8724642416341409662[154] = 0.0;
   out_8724642416341409662[155] = 0.0;
   out_8724642416341409662[156] = 0.0;
   out_8724642416341409662[157] = 0.0;
   out_8724642416341409662[158] = 0.0;
   out_8724642416341409662[159] = 0.0;
   out_8724642416341409662[160] = 0.0;
   out_8724642416341409662[161] = 0.0;
   out_8724642416341409662[162] = 0.0;
   out_8724642416341409662[163] = 0.0;
   out_8724642416341409662[164] = 0.0;
   out_8724642416341409662[165] = 0.0;
   out_8724642416341409662[166] = 0.0;
   out_8724642416341409662[167] = 0.0;
   out_8724642416341409662[168] = 0.0;
   out_8724642416341409662[169] = 0.0;
   out_8724642416341409662[170] = 0.0;
   out_8724642416341409662[171] = 1.0;
   out_8724642416341409662[172] = 0.0;
   out_8724642416341409662[173] = 0.0;
   out_8724642416341409662[174] = 0.0;
   out_8724642416341409662[175] = 0.0;
   out_8724642416341409662[176] = 0.0;
   out_8724642416341409662[177] = 0.0;
   out_8724642416341409662[178] = 0.0;
   out_8724642416341409662[179] = 0.0;
   out_8724642416341409662[180] = 0.0;
   out_8724642416341409662[181] = 0.0;
   out_8724642416341409662[182] = 0.0;
   out_8724642416341409662[183] = 0.0;
   out_8724642416341409662[184] = 0.0;
   out_8724642416341409662[185] = 0.0;
   out_8724642416341409662[186] = 0.0;
   out_8724642416341409662[187] = 0.0;
   out_8724642416341409662[188] = 0.0;
   out_8724642416341409662[189] = 0.0;
   out_8724642416341409662[190] = 1.0;
   out_8724642416341409662[191] = 0.0;
   out_8724642416341409662[192] = 0.0;
   out_8724642416341409662[193] = 0.0;
   out_8724642416341409662[194] = 0.0;
   out_8724642416341409662[195] = 0.0;
   out_8724642416341409662[196] = 0.0;
   out_8724642416341409662[197] = 0.0;
   out_8724642416341409662[198] = 0.0;
   out_8724642416341409662[199] = 0.0;
   out_8724642416341409662[200] = 0.0;
   out_8724642416341409662[201] = 0.0;
   out_8724642416341409662[202] = 0.0;
   out_8724642416341409662[203] = 0.0;
   out_8724642416341409662[204] = 0.0;
   out_8724642416341409662[205] = 0.0;
   out_8724642416341409662[206] = 0.0;
   out_8724642416341409662[207] = 0.0;
   out_8724642416341409662[208] = 0.0;
   out_8724642416341409662[209] = 1.0;
   out_8724642416341409662[210] = 0.0;
   out_8724642416341409662[211] = 0.0;
   out_8724642416341409662[212] = 0.0;
   out_8724642416341409662[213] = 0.0;
   out_8724642416341409662[214] = 0.0;
   out_8724642416341409662[215] = 0.0;
   out_8724642416341409662[216] = 0.0;
   out_8724642416341409662[217] = 0.0;
   out_8724642416341409662[218] = 0.0;
   out_8724642416341409662[219] = 0.0;
   out_8724642416341409662[220] = 0.0;
   out_8724642416341409662[221] = 0.0;
   out_8724642416341409662[222] = 0.0;
   out_8724642416341409662[223] = 0.0;
   out_8724642416341409662[224] = 0.0;
   out_8724642416341409662[225] = 0.0;
   out_8724642416341409662[226] = 0.0;
   out_8724642416341409662[227] = 0.0;
   out_8724642416341409662[228] = 1.0;
   out_8724642416341409662[229] = 0.0;
   out_8724642416341409662[230] = 0.0;
   out_8724642416341409662[231] = 0.0;
   out_8724642416341409662[232] = 0.0;
   out_8724642416341409662[233] = 0.0;
   out_8724642416341409662[234] = 0.0;
   out_8724642416341409662[235] = 0.0;
   out_8724642416341409662[236] = 0.0;
   out_8724642416341409662[237] = 0.0;
   out_8724642416341409662[238] = 0.0;
   out_8724642416341409662[239] = 0.0;
   out_8724642416341409662[240] = 0.0;
   out_8724642416341409662[241] = 0.0;
   out_8724642416341409662[242] = 0.0;
   out_8724642416341409662[243] = 0.0;
   out_8724642416341409662[244] = 0.0;
   out_8724642416341409662[245] = 0.0;
   out_8724642416341409662[246] = 0.0;
   out_8724642416341409662[247] = 1.0;
   out_8724642416341409662[248] = 0.0;
   out_8724642416341409662[249] = 0.0;
   out_8724642416341409662[250] = 0.0;
   out_8724642416341409662[251] = 0.0;
   out_8724642416341409662[252] = 0.0;
   out_8724642416341409662[253] = 0.0;
   out_8724642416341409662[254] = 0.0;
   out_8724642416341409662[255] = 0.0;
   out_8724642416341409662[256] = 0.0;
   out_8724642416341409662[257] = 0.0;
   out_8724642416341409662[258] = 0.0;
   out_8724642416341409662[259] = 0.0;
   out_8724642416341409662[260] = 0.0;
   out_8724642416341409662[261] = 0.0;
   out_8724642416341409662[262] = 0.0;
   out_8724642416341409662[263] = 0.0;
   out_8724642416341409662[264] = 0.0;
   out_8724642416341409662[265] = 0.0;
   out_8724642416341409662[266] = 1.0;
   out_8724642416341409662[267] = 0.0;
   out_8724642416341409662[268] = 0.0;
   out_8724642416341409662[269] = 0.0;
   out_8724642416341409662[270] = 0.0;
   out_8724642416341409662[271] = 0.0;
   out_8724642416341409662[272] = 0.0;
   out_8724642416341409662[273] = 0.0;
   out_8724642416341409662[274] = 0.0;
   out_8724642416341409662[275] = 0.0;
   out_8724642416341409662[276] = 0.0;
   out_8724642416341409662[277] = 0.0;
   out_8724642416341409662[278] = 0.0;
   out_8724642416341409662[279] = 0.0;
   out_8724642416341409662[280] = 0.0;
   out_8724642416341409662[281] = 0.0;
   out_8724642416341409662[282] = 0.0;
   out_8724642416341409662[283] = 0.0;
   out_8724642416341409662[284] = 0.0;
   out_8724642416341409662[285] = 1.0;
   out_8724642416341409662[286] = 0.0;
   out_8724642416341409662[287] = 0.0;
   out_8724642416341409662[288] = 0.0;
   out_8724642416341409662[289] = 0.0;
   out_8724642416341409662[290] = 0.0;
   out_8724642416341409662[291] = 0.0;
   out_8724642416341409662[292] = 0.0;
   out_8724642416341409662[293] = 0.0;
   out_8724642416341409662[294] = 0.0;
   out_8724642416341409662[295] = 0.0;
   out_8724642416341409662[296] = 0.0;
   out_8724642416341409662[297] = 0.0;
   out_8724642416341409662[298] = 0.0;
   out_8724642416341409662[299] = 0.0;
   out_8724642416341409662[300] = 0.0;
   out_8724642416341409662[301] = 0.0;
   out_8724642416341409662[302] = 0.0;
   out_8724642416341409662[303] = 0.0;
   out_8724642416341409662[304] = 1.0;
   out_8724642416341409662[305] = 0.0;
   out_8724642416341409662[306] = 0.0;
   out_8724642416341409662[307] = 0.0;
   out_8724642416341409662[308] = 0.0;
   out_8724642416341409662[309] = 0.0;
   out_8724642416341409662[310] = 0.0;
   out_8724642416341409662[311] = 0.0;
   out_8724642416341409662[312] = 0.0;
   out_8724642416341409662[313] = 0.0;
   out_8724642416341409662[314] = 0.0;
   out_8724642416341409662[315] = 0.0;
   out_8724642416341409662[316] = 0.0;
   out_8724642416341409662[317] = 0.0;
   out_8724642416341409662[318] = 0.0;
   out_8724642416341409662[319] = 0.0;
   out_8724642416341409662[320] = 0.0;
   out_8724642416341409662[321] = 0.0;
   out_8724642416341409662[322] = 0.0;
   out_8724642416341409662[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3607734036645673030) {
   out_3607734036645673030[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3607734036645673030[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3607734036645673030[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3607734036645673030[3] = dt*state[12] + state[3];
   out_3607734036645673030[4] = dt*state[13] + state[4];
   out_3607734036645673030[5] = dt*state[14] + state[5];
   out_3607734036645673030[6] = state[6];
   out_3607734036645673030[7] = state[7];
   out_3607734036645673030[8] = state[8];
   out_3607734036645673030[9] = state[9];
   out_3607734036645673030[10] = state[10];
   out_3607734036645673030[11] = state[11];
   out_3607734036645673030[12] = state[12];
   out_3607734036645673030[13] = state[13];
   out_3607734036645673030[14] = state[14];
   out_3607734036645673030[15] = state[15];
   out_3607734036645673030[16] = state[16];
   out_3607734036645673030[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2129463630195495818) {
   out_2129463630195495818[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2129463630195495818[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2129463630195495818[2] = 0;
   out_2129463630195495818[3] = 0;
   out_2129463630195495818[4] = 0;
   out_2129463630195495818[5] = 0;
   out_2129463630195495818[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2129463630195495818[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2129463630195495818[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2129463630195495818[9] = 0;
   out_2129463630195495818[10] = 0;
   out_2129463630195495818[11] = 0;
   out_2129463630195495818[12] = 0;
   out_2129463630195495818[13] = 0;
   out_2129463630195495818[14] = 0;
   out_2129463630195495818[15] = 0;
   out_2129463630195495818[16] = 0;
   out_2129463630195495818[17] = 0;
   out_2129463630195495818[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2129463630195495818[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2129463630195495818[20] = 0;
   out_2129463630195495818[21] = 0;
   out_2129463630195495818[22] = 0;
   out_2129463630195495818[23] = 0;
   out_2129463630195495818[24] = 0;
   out_2129463630195495818[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2129463630195495818[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2129463630195495818[27] = 0;
   out_2129463630195495818[28] = 0;
   out_2129463630195495818[29] = 0;
   out_2129463630195495818[30] = 0;
   out_2129463630195495818[31] = 0;
   out_2129463630195495818[32] = 0;
   out_2129463630195495818[33] = 0;
   out_2129463630195495818[34] = 0;
   out_2129463630195495818[35] = 0;
   out_2129463630195495818[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2129463630195495818[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2129463630195495818[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2129463630195495818[39] = 0;
   out_2129463630195495818[40] = 0;
   out_2129463630195495818[41] = 0;
   out_2129463630195495818[42] = 0;
   out_2129463630195495818[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2129463630195495818[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2129463630195495818[45] = 0;
   out_2129463630195495818[46] = 0;
   out_2129463630195495818[47] = 0;
   out_2129463630195495818[48] = 0;
   out_2129463630195495818[49] = 0;
   out_2129463630195495818[50] = 0;
   out_2129463630195495818[51] = 0;
   out_2129463630195495818[52] = 0;
   out_2129463630195495818[53] = 0;
   out_2129463630195495818[54] = 0;
   out_2129463630195495818[55] = 0;
   out_2129463630195495818[56] = 0;
   out_2129463630195495818[57] = 1;
   out_2129463630195495818[58] = 0;
   out_2129463630195495818[59] = 0;
   out_2129463630195495818[60] = 0;
   out_2129463630195495818[61] = 0;
   out_2129463630195495818[62] = 0;
   out_2129463630195495818[63] = 0;
   out_2129463630195495818[64] = 0;
   out_2129463630195495818[65] = 0;
   out_2129463630195495818[66] = dt;
   out_2129463630195495818[67] = 0;
   out_2129463630195495818[68] = 0;
   out_2129463630195495818[69] = 0;
   out_2129463630195495818[70] = 0;
   out_2129463630195495818[71] = 0;
   out_2129463630195495818[72] = 0;
   out_2129463630195495818[73] = 0;
   out_2129463630195495818[74] = 0;
   out_2129463630195495818[75] = 0;
   out_2129463630195495818[76] = 1;
   out_2129463630195495818[77] = 0;
   out_2129463630195495818[78] = 0;
   out_2129463630195495818[79] = 0;
   out_2129463630195495818[80] = 0;
   out_2129463630195495818[81] = 0;
   out_2129463630195495818[82] = 0;
   out_2129463630195495818[83] = 0;
   out_2129463630195495818[84] = 0;
   out_2129463630195495818[85] = dt;
   out_2129463630195495818[86] = 0;
   out_2129463630195495818[87] = 0;
   out_2129463630195495818[88] = 0;
   out_2129463630195495818[89] = 0;
   out_2129463630195495818[90] = 0;
   out_2129463630195495818[91] = 0;
   out_2129463630195495818[92] = 0;
   out_2129463630195495818[93] = 0;
   out_2129463630195495818[94] = 0;
   out_2129463630195495818[95] = 1;
   out_2129463630195495818[96] = 0;
   out_2129463630195495818[97] = 0;
   out_2129463630195495818[98] = 0;
   out_2129463630195495818[99] = 0;
   out_2129463630195495818[100] = 0;
   out_2129463630195495818[101] = 0;
   out_2129463630195495818[102] = 0;
   out_2129463630195495818[103] = 0;
   out_2129463630195495818[104] = dt;
   out_2129463630195495818[105] = 0;
   out_2129463630195495818[106] = 0;
   out_2129463630195495818[107] = 0;
   out_2129463630195495818[108] = 0;
   out_2129463630195495818[109] = 0;
   out_2129463630195495818[110] = 0;
   out_2129463630195495818[111] = 0;
   out_2129463630195495818[112] = 0;
   out_2129463630195495818[113] = 0;
   out_2129463630195495818[114] = 1;
   out_2129463630195495818[115] = 0;
   out_2129463630195495818[116] = 0;
   out_2129463630195495818[117] = 0;
   out_2129463630195495818[118] = 0;
   out_2129463630195495818[119] = 0;
   out_2129463630195495818[120] = 0;
   out_2129463630195495818[121] = 0;
   out_2129463630195495818[122] = 0;
   out_2129463630195495818[123] = 0;
   out_2129463630195495818[124] = 0;
   out_2129463630195495818[125] = 0;
   out_2129463630195495818[126] = 0;
   out_2129463630195495818[127] = 0;
   out_2129463630195495818[128] = 0;
   out_2129463630195495818[129] = 0;
   out_2129463630195495818[130] = 0;
   out_2129463630195495818[131] = 0;
   out_2129463630195495818[132] = 0;
   out_2129463630195495818[133] = 1;
   out_2129463630195495818[134] = 0;
   out_2129463630195495818[135] = 0;
   out_2129463630195495818[136] = 0;
   out_2129463630195495818[137] = 0;
   out_2129463630195495818[138] = 0;
   out_2129463630195495818[139] = 0;
   out_2129463630195495818[140] = 0;
   out_2129463630195495818[141] = 0;
   out_2129463630195495818[142] = 0;
   out_2129463630195495818[143] = 0;
   out_2129463630195495818[144] = 0;
   out_2129463630195495818[145] = 0;
   out_2129463630195495818[146] = 0;
   out_2129463630195495818[147] = 0;
   out_2129463630195495818[148] = 0;
   out_2129463630195495818[149] = 0;
   out_2129463630195495818[150] = 0;
   out_2129463630195495818[151] = 0;
   out_2129463630195495818[152] = 1;
   out_2129463630195495818[153] = 0;
   out_2129463630195495818[154] = 0;
   out_2129463630195495818[155] = 0;
   out_2129463630195495818[156] = 0;
   out_2129463630195495818[157] = 0;
   out_2129463630195495818[158] = 0;
   out_2129463630195495818[159] = 0;
   out_2129463630195495818[160] = 0;
   out_2129463630195495818[161] = 0;
   out_2129463630195495818[162] = 0;
   out_2129463630195495818[163] = 0;
   out_2129463630195495818[164] = 0;
   out_2129463630195495818[165] = 0;
   out_2129463630195495818[166] = 0;
   out_2129463630195495818[167] = 0;
   out_2129463630195495818[168] = 0;
   out_2129463630195495818[169] = 0;
   out_2129463630195495818[170] = 0;
   out_2129463630195495818[171] = 1;
   out_2129463630195495818[172] = 0;
   out_2129463630195495818[173] = 0;
   out_2129463630195495818[174] = 0;
   out_2129463630195495818[175] = 0;
   out_2129463630195495818[176] = 0;
   out_2129463630195495818[177] = 0;
   out_2129463630195495818[178] = 0;
   out_2129463630195495818[179] = 0;
   out_2129463630195495818[180] = 0;
   out_2129463630195495818[181] = 0;
   out_2129463630195495818[182] = 0;
   out_2129463630195495818[183] = 0;
   out_2129463630195495818[184] = 0;
   out_2129463630195495818[185] = 0;
   out_2129463630195495818[186] = 0;
   out_2129463630195495818[187] = 0;
   out_2129463630195495818[188] = 0;
   out_2129463630195495818[189] = 0;
   out_2129463630195495818[190] = 1;
   out_2129463630195495818[191] = 0;
   out_2129463630195495818[192] = 0;
   out_2129463630195495818[193] = 0;
   out_2129463630195495818[194] = 0;
   out_2129463630195495818[195] = 0;
   out_2129463630195495818[196] = 0;
   out_2129463630195495818[197] = 0;
   out_2129463630195495818[198] = 0;
   out_2129463630195495818[199] = 0;
   out_2129463630195495818[200] = 0;
   out_2129463630195495818[201] = 0;
   out_2129463630195495818[202] = 0;
   out_2129463630195495818[203] = 0;
   out_2129463630195495818[204] = 0;
   out_2129463630195495818[205] = 0;
   out_2129463630195495818[206] = 0;
   out_2129463630195495818[207] = 0;
   out_2129463630195495818[208] = 0;
   out_2129463630195495818[209] = 1;
   out_2129463630195495818[210] = 0;
   out_2129463630195495818[211] = 0;
   out_2129463630195495818[212] = 0;
   out_2129463630195495818[213] = 0;
   out_2129463630195495818[214] = 0;
   out_2129463630195495818[215] = 0;
   out_2129463630195495818[216] = 0;
   out_2129463630195495818[217] = 0;
   out_2129463630195495818[218] = 0;
   out_2129463630195495818[219] = 0;
   out_2129463630195495818[220] = 0;
   out_2129463630195495818[221] = 0;
   out_2129463630195495818[222] = 0;
   out_2129463630195495818[223] = 0;
   out_2129463630195495818[224] = 0;
   out_2129463630195495818[225] = 0;
   out_2129463630195495818[226] = 0;
   out_2129463630195495818[227] = 0;
   out_2129463630195495818[228] = 1;
   out_2129463630195495818[229] = 0;
   out_2129463630195495818[230] = 0;
   out_2129463630195495818[231] = 0;
   out_2129463630195495818[232] = 0;
   out_2129463630195495818[233] = 0;
   out_2129463630195495818[234] = 0;
   out_2129463630195495818[235] = 0;
   out_2129463630195495818[236] = 0;
   out_2129463630195495818[237] = 0;
   out_2129463630195495818[238] = 0;
   out_2129463630195495818[239] = 0;
   out_2129463630195495818[240] = 0;
   out_2129463630195495818[241] = 0;
   out_2129463630195495818[242] = 0;
   out_2129463630195495818[243] = 0;
   out_2129463630195495818[244] = 0;
   out_2129463630195495818[245] = 0;
   out_2129463630195495818[246] = 0;
   out_2129463630195495818[247] = 1;
   out_2129463630195495818[248] = 0;
   out_2129463630195495818[249] = 0;
   out_2129463630195495818[250] = 0;
   out_2129463630195495818[251] = 0;
   out_2129463630195495818[252] = 0;
   out_2129463630195495818[253] = 0;
   out_2129463630195495818[254] = 0;
   out_2129463630195495818[255] = 0;
   out_2129463630195495818[256] = 0;
   out_2129463630195495818[257] = 0;
   out_2129463630195495818[258] = 0;
   out_2129463630195495818[259] = 0;
   out_2129463630195495818[260] = 0;
   out_2129463630195495818[261] = 0;
   out_2129463630195495818[262] = 0;
   out_2129463630195495818[263] = 0;
   out_2129463630195495818[264] = 0;
   out_2129463630195495818[265] = 0;
   out_2129463630195495818[266] = 1;
   out_2129463630195495818[267] = 0;
   out_2129463630195495818[268] = 0;
   out_2129463630195495818[269] = 0;
   out_2129463630195495818[270] = 0;
   out_2129463630195495818[271] = 0;
   out_2129463630195495818[272] = 0;
   out_2129463630195495818[273] = 0;
   out_2129463630195495818[274] = 0;
   out_2129463630195495818[275] = 0;
   out_2129463630195495818[276] = 0;
   out_2129463630195495818[277] = 0;
   out_2129463630195495818[278] = 0;
   out_2129463630195495818[279] = 0;
   out_2129463630195495818[280] = 0;
   out_2129463630195495818[281] = 0;
   out_2129463630195495818[282] = 0;
   out_2129463630195495818[283] = 0;
   out_2129463630195495818[284] = 0;
   out_2129463630195495818[285] = 1;
   out_2129463630195495818[286] = 0;
   out_2129463630195495818[287] = 0;
   out_2129463630195495818[288] = 0;
   out_2129463630195495818[289] = 0;
   out_2129463630195495818[290] = 0;
   out_2129463630195495818[291] = 0;
   out_2129463630195495818[292] = 0;
   out_2129463630195495818[293] = 0;
   out_2129463630195495818[294] = 0;
   out_2129463630195495818[295] = 0;
   out_2129463630195495818[296] = 0;
   out_2129463630195495818[297] = 0;
   out_2129463630195495818[298] = 0;
   out_2129463630195495818[299] = 0;
   out_2129463630195495818[300] = 0;
   out_2129463630195495818[301] = 0;
   out_2129463630195495818[302] = 0;
   out_2129463630195495818[303] = 0;
   out_2129463630195495818[304] = 1;
   out_2129463630195495818[305] = 0;
   out_2129463630195495818[306] = 0;
   out_2129463630195495818[307] = 0;
   out_2129463630195495818[308] = 0;
   out_2129463630195495818[309] = 0;
   out_2129463630195495818[310] = 0;
   out_2129463630195495818[311] = 0;
   out_2129463630195495818[312] = 0;
   out_2129463630195495818[313] = 0;
   out_2129463630195495818[314] = 0;
   out_2129463630195495818[315] = 0;
   out_2129463630195495818[316] = 0;
   out_2129463630195495818[317] = 0;
   out_2129463630195495818[318] = 0;
   out_2129463630195495818[319] = 0;
   out_2129463630195495818[320] = 0;
   out_2129463630195495818[321] = 0;
   out_2129463630195495818[322] = 0;
   out_2129463630195495818[323] = 1;
}
void h_4(double *state, double *unused, double *out_6529939596965481659) {
   out_6529939596965481659[0] = state[6] + state[9];
   out_6529939596965481659[1] = state[7] + state[10];
   out_6529939596965481659[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8967940655313429347) {
   out_8967940655313429347[0] = 0;
   out_8967940655313429347[1] = 0;
   out_8967940655313429347[2] = 0;
   out_8967940655313429347[3] = 0;
   out_8967940655313429347[4] = 0;
   out_8967940655313429347[5] = 0;
   out_8967940655313429347[6] = 1;
   out_8967940655313429347[7] = 0;
   out_8967940655313429347[8] = 0;
   out_8967940655313429347[9] = 1;
   out_8967940655313429347[10] = 0;
   out_8967940655313429347[11] = 0;
   out_8967940655313429347[12] = 0;
   out_8967940655313429347[13] = 0;
   out_8967940655313429347[14] = 0;
   out_8967940655313429347[15] = 0;
   out_8967940655313429347[16] = 0;
   out_8967940655313429347[17] = 0;
   out_8967940655313429347[18] = 0;
   out_8967940655313429347[19] = 0;
   out_8967940655313429347[20] = 0;
   out_8967940655313429347[21] = 0;
   out_8967940655313429347[22] = 0;
   out_8967940655313429347[23] = 0;
   out_8967940655313429347[24] = 0;
   out_8967940655313429347[25] = 1;
   out_8967940655313429347[26] = 0;
   out_8967940655313429347[27] = 0;
   out_8967940655313429347[28] = 1;
   out_8967940655313429347[29] = 0;
   out_8967940655313429347[30] = 0;
   out_8967940655313429347[31] = 0;
   out_8967940655313429347[32] = 0;
   out_8967940655313429347[33] = 0;
   out_8967940655313429347[34] = 0;
   out_8967940655313429347[35] = 0;
   out_8967940655313429347[36] = 0;
   out_8967940655313429347[37] = 0;
   out_8967940655313429347[38] = 0;
   out_8967940655313429347[39] = 0;
   out_8967940655313429347[40] = 0;
   out_8967940655313429347[41] = 0;
   out_8967940655313429347[42] = 0;
   out_8967940655313429347[43] = 0;
   out_8967940655313429347[44] = 1;
   out_8967940655313429347[45] = 0;
   out_8967940655313429347[46] = 0;
   out_8967940655313429347[47] = 1;
   out_8967940655313429347[48] = 0;
   out_8967940655313429347[49] = 0;
   out_8967940655313429347[50] = 0;
   out_8967940655313429347[51] = 0;
   out_8967940655313429347[52] = 0;
   out_8967940655313429347[53] = 0;
}
void h_10(double *state, double *unused, double *out_4206098625503837771) {
   out_4206098625503837771[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4206098625503837771[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4206098625503837771[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5859467736028250385) {
   out_5859467736028250385[0] = 0;
   out_5859467736028250385[1] = 9.8100000000000005*cos(state[1]);
   out_5859467736028250385[2] = 0;
   out_5859467736028250385[3] = 0;
   out_5859467736028250385[4] = -state[8];
   out_5859467736028250385[5] = state[7];
   out_5859467736028250385[6] = 0;
   out_5859467736028250385[7] = state[5];
   out_5859467736028250385[8] = -state[4];
   out_5859467736028250385[9] = 0;
   out_5859467736028250385[10] = 0;
   out_5859467736028250385[11] = 0;
   out_5859467736028250385[12] = 1;
   out_5859467736028250385[13] = 0;
   out_5859467736028250385[14] = 0;
   out_5859467736028250385[15] = 1;
   out_5859467736028250385[16] = 0;
   out_5859467736028250385[17] = 0;
   out_5859467736028250385[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5859467736028250385[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5859467736028250385[20] = 0;
   out_5859467736028250385[21] = state[8];
   out_5859467736028250385[22] = 0;
   out_5859467736028250385[23] = -state[6];
   out_5859467736028250385[24] = -state[5];
   out_5859467736028250385[25] = 0;
   out_5859467736028250385[26] = state[3];
   out_5859467736028250385[27] = 0;
   out_5859467736028250385[28] = 0;
   out_5859467736028250385[29] = 0;
   out_5859467736028250385[30] = 0;
   out_5859467736028250385[31] = 1;
   out_5859467736028250385[32] = 0;
   out_5859467736028250385[33] = 0;
   out_5859467736028250385[34] = 1;
   out_5859467736028250385[35] = 0;
   out_5859467736028250385[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5859467736028250385[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5859467736028250385[38] = 0;
   out_5859467736028250385[39] = -state[7];
   out_5859467736028250385[40] = state[6];
   out_5859467736028250385[41] = 0;
   out_5859467736028250385[42] = state[4];
   out_5859467736028250385[43] = -state[3];
   out_5859467736028250385[44] = 0;
   out_5859467736028250385[45] = 0;
   out_5859467736028250385[46] = 0;
   out_5859467736028250385[47] = 0;
   out_5859467736028250385[48] = 0;
   out_5859467736028250385[49] = 0;
   out_5859467736028250385[50] = 1;
   out_5859467736028250385[51] = 0;
   out_5859467736028250385[52] = 0;
   out_5859467736028250385[53] = 1;
}
void h_13(double *state, double *unused, double *out_948693449030853671) {
   out_948693449030853671[0] = state[3];
   out_948693449030853671[1] = state[4];
   out_948693449030853671[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1357309446996728418) {
   out_1357309446996728418[0] = 0;
   out_1357309446996728418[1] = 0;
   out_1357309446996728418[2] = 0;
   out_1357309446996728418[3] = 1;
   out_1357309446996728418[4] = 0;
   out_1357309446996728418[5] = 0;
   out_1357309446996728418[6] = 0;
   out_1357309446996728418[7] = 0;
   out_1357309446996728418[8] = 0;
   out_1357309446996728418[9] = 0;
   out_1357309446996728418[10] = 0;
   out_1357309446996728418[11] = 0;
   out_1357309446996728418[12] = 0;
   out_1357309446996728418[13] = 0;
   out_1357309446996728418[14] = 0;
   out_1357309446996728418[15] = 0;
   out_1357309446996728418[16] = 0;
   out_1357309446996728418[17] = 0;
   out_1357309446996728418[18] = 0;
   out_1357309446996728418[19] = 0;
   out_1357309446996728418[20] = 0;
   out_1357309446996728418[21] = 0;
   out_1357309446996728418[22] = 1;
   out_1357309446996728418[23] = 0;
   out_1357309446996728418[24] = 0;
   out_1357309446996728418[25] = 0;
   out_1357309446996728418[26] = 0;
   out_1357309446996728418[27] = 0;
   out_1357309446996728418[28] = 0;
   out_1357309446996728418[29] = 0;
   out_1357309446996728418[30] = 0;
   out_1357309446996728418[31] = 0;
   out_1357309446996728418[32] = 0;
   out_1357309446996728418[33] = 0;
   out_1357309446996728418[34] = 0;
   out_1357309446996728418[35] = 0;
   out_1357309446996728418[36] = 0;
   out_1357309446996728418[37] = 0;
   out_1357309446996728418[38] = 0;
   out_1357309446996728418[39] = 0;
   out_1357309446996728418[40] = 0;
   out_1357309446996728418[41] = 1;
   out_1357309446996728418[42] = 0;
   out_1357309446996728418[43] = 0;
   out_1357309446996728418[44] = 0;
   out_1357309446996728418[45] = 0;
   out_1357309446996728418[46] = 0;
   out_1357309446996728418[47] = 0;
   out_1357309446996728418[48] = 0;
   out_1357309446996728418[49] = 0;
   out_1357309446996728418[50] = 0;
   out_1357309446996728418[51] = 0;
   out_1357309446996728418[52] = 0;
   out_1357309446996728418[53] = 0;
}
void h_14(double *state, double *unused, double *out_3564627351806134308) {
   out_3564627351806134308[0] = state[6];
   out_3564627351806134308[1] = state[7];
   out_3564627351806134308[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5004699798973944818) {
   out_5004699798973944818[0] = 0;
   out_5004699798973944818[1] = 0;
   out_5004699798973944818[2] = 0;
   out_5004699798973944818[3] = 0;
   out_5004699798973944818[4] = 0;
   out_5004699798973944818[5] = 0;
   out_5004699798973944818[6] = 1;
   out_5004699798973944818[7] = 0;
   out_5004699798973944818[8] = 0;
   out_5004699798973944818[9] = 0;
   out_5004699798973944818[10] = 0;
   out_5004699798973944818[11] = 0;
   out_5004699798973944818[12] = 0;
   out_5004699798973944818[13] = 0;
   out_5004699798973944818[14] = 0;
   out_5004699798973944818[15] = 0;
   out_5004699798973944818[16] = 0;
   out_5004699798973944818[17] = 0;
   out_5004699798973944818[18] = 0;
   out_5004699798973944818[19] = 0;
   out_5004699798973944818[20] = 0;
   out_5004699798973944818[21] = 0;
   out_5004699798973944818[22] = 0;
   out_5004699798973944818[23] = 0;
   out_5004699798973944818[24] = 0;
   out_5004699798973944818[25] = 1;
   out_5004699798973944818[26] = 0;
   out_5004699798973944818[27] = 0;
   out_5004699798973944818[28] = 0;
   out_5004699798973944818[29] = 0;
   out_5004699798973944818[30] = 0;
   out_5004699798973944818[31] = 0;
   out_5004699798973944818[32] = 0;
   out_5004699798973944818[33] = 0;
   out_5004699798973944818[34] = 0;
   out_5004699798973944818[35] = 0;
   out_5004699798973944818[36] = 0;
   out_5004699798973944818[37] = 0;
   out_5004699798973944818[38] = 0;
   out_5004699798973944818[39] = 0;
   out_5004699798973944818[40] = 0;
   out_5004699798973944818[41] = 0;
   out_5004699798973944818[42] = 0;
   out_5004699798973944818[43] = 0;
   out_5004699798973944818[44] = 1;
   out_5004699798973944818[45] = 0;
   out_5004699798973944818[46] = 0;
   out_5004699798973944818[47] = 0;
   out_5004699798973944818[48] = 0;
   out_5004699798973944818[49] = 0;
   out_5004699798973944818[50] = 0;
   out_5004699798973944818[51] = 0;
   out_5004699798973944818[52] = 0;
   out_5004699798973944818[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_3545767590363888806) {
  err_fun(nom_x, delta_x, out_3545767590363888806);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6258502841145403309) {
  inv_err_fun(nom_x, true_x, out_6258502841145403309);
}
void pose_H_mod_fun(double *state, double *out_8724642416341409662) {
  H_mod_fun(state, out_8724642416341409662);
}
void pose_f_fun(double *state, double dt, double *out_3607734036645673030) {
  f_fun(state,  dt, out_3607734036645673030);
}
void pose_F_fun(double *state, double dt, double *out_2129463630195495818) {
  F_fun(state,  dt, out_2129463630195495818);
}
void pose_h_4(double *state, double *unused, double *out_6529939596965481659) {
  h_4(state, unused, out_6529939596965481659);
}
void pose_H_4(double *state, double *unused, double *out_8967940655313429347) {
  H_4(state, unused, out_8967940655313429347);
}
void pose_h_10(double *state, double *unused, double *out_4206098625503837771) {
  h_10(state, unused, out_4206098625503837771);
}
void pose_H_10(double *state, double *unused, double *out_5859467736028250385) {
  H_10(state, unused, out_5859467736028250385);
}
void pose_h_13(double *state, double *unused, double *out_948693449030853671) {
  h_13(state, unused, out_948693449030853671);
}
void pose_H_13(double *state, double *unused, double *out_1357309446996728418) {
  H_13(state, unused, out_1357309446996728418);
}
void pose_h_14(double *state, double *unused, double *out_3564627351806134308) {
  h_14(state, unused, out_3564627351806134308);
}
void pose_H_14(double *state, double *unused, double *out_5004699798973944818) {
  H_14(state, unused, out_5004699798973944818);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
