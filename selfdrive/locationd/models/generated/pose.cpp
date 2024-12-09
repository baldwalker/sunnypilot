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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7863920857934757474) {
   out_7863920857934757474[0] = delta_x[0] + nom_x[0];
   out_7863920857934757474[1] = delta_x[1] + nom_x[1];
   out_7863920857934757474[2] = delta_x[2] + nom_x[2];
   out_7863920857934757474[3] = delta_x[3] + nom_x[3];
   out_7863920857934757474[4] = delta_x[4] + nom_x[4];
   out_7863920857934757474[5] = delta_x[5] + nom_x[5];
   out_7863920857934757474[6] = delta_x[6] + nom_x[6];
   out_7863920857934757474[7] = delta_x[7] + nom_x[7];
   out_7863920857934757474[8] = delta_x[8] + nom_x[8];
   out_7863920857934757474[9] = delta_x[9] + nom_x[9];
   out_7863920857934757474[10] = delta_x[10] + nom_x[10];
   out_7863920857934757474[11] = delta_x[11] + nom_x[11];
   out_7863920857934757474[12] = delta_x[12] + nom_x[12];
   out_7863920857934757474[13] = delta_x[13] + nom_x[13];
   out_7863920857934757474[14] = delta_x[14] + nom_x[14];
   out_7863920857934757474[15] = delta_x[15] + nom_x[15];
   out_7863920857934757474[16] = delta_x[16] + nom_x[16];
   out_7863920857934757474[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1293085161672310543) {
   out_1293085161672310543[0] = -nom_x[0] + true_x[0];
   out_1293085161672310543[1] = -nom_x[1] + true_x[1];
   out_1293085161672310543[2] = -nom_x[2] + true_x[2];
   out_1293085161672310543[3] = -nom_x[3] + true_x[3];
   out_1293085161672310543[4] = -nom_x[4] + true_x[4];
   out_1293085161672310543[5] = -nom_x[5] + true_x[5];
   out_1293085161672310543[6] = -nom_x[6] + true_x[6];
   out_1293085161672310543[7] = -nom_x[7] + true_x[7];
   out_1293085161672310543[8] = -nom_x[8] + true_x[8];
   out_1293085161672310543[9] = -nom_x[9] + true_x[9];
   out_1293085161672310543[10] = -nom_x[10] + true_x[10];
   out_1293085161672310543[11] = -nom_x[11] + true_x[11];
   out_1293085161672310543[12] = -nom_x[12] + true_x[12];
   out_1293085161672310543[13] = -nom_x[13] + true_x[13];
   out_1293085161672310543[14] = -nom_x[14] + true_x[14];
   out_1293085161672310543[15] = -nom_x[15] + true_x[15];
   out_1293085161672310543[16] = -nom_x[16] + true_x[16];
   out_1293085161672310543[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_271066960830845225) {
   out_271066960830845225[0] = 1.0;
   out_271066960830845225[1] = 0;
   out_271066960830845225[2] = 0;
   out_271066960830845225[3] = 0;
   out_271066960830845225[4] = 0;
   out_271066960830845225[5] = 0;
   out_271066960830845225[6] = 0;
   out_271066960830845225[7] = 0;
   out_271066960830845225[8] = 0;
   out_271066960830845225[9] = 0;
   out_271066960830845225[10] = 0;
   out_271066960830845225[11] = 0;
   out_271066960830845225[12] = 0;
   out_271066960830845225[13] = 0;
   out_271066960830845225[14] = 0;
   out_271066960830845225[15] = 0;
   out_271066960830845225[16] = 0;
   out_271066960830845225[17] = 0;
   out_271066960830845225[18] = 0;
   out_271066960830845225[19] = 1.0;
   out_271066960830845225[20] = 0;
   out_271066960830845225[21] = 0;
   out_271066960830845225[22] = 0;
   out_271066960830845225[23] = 0;
   out_271066960830845225[24] = 0;
   out_271066960830845225[25] = 0;
   out_271066960830845225[26] = 0;
   out_271066960830845225[27] = 0;
   out_271066960830845225[28] = 0;
   out_271066960830845225[29] = 0;
   out_271066960830845225[30] = 0;
   out_271066960830845225[31] = 0;
   out_271066960830845225[32] = 0;
   out_271066960830845225[33] = 0;
   out_271066960830845225[34] = 0;
   out_271066960830845225[35] = 0;
   out_271066960830845225[36] = 0;
   out_271066960830845225[37] = 0;
   out_271066960830845225[38] = 1.0;
   out_271066960830845225[39] = 0;
   out_271066960830845225[40] = 0;
   out_271066960830845225[41] = 0;
   out_271066960830845225[42] = 0;
   out_271066960830845225[43] = 0;
   out_271066960830845225[44] = 0;
   out_271066960830845225[45] = 0;
   out_271066960830845225[46] = 0;
   out_271066960830845225[47] = 0;
   out_271066960830845225[48] = 0;
   out_271066960830845225[49] = 0;
   out_271066960830845225[50] = 0;
   out_271066960830845225[51] = 0;
   out_271066960830845225[52] = 0;
   out_271066960830845225[53] = 0;
   out_271066960830845225[54] = 0;
   out_271066960830845225[55] = 0;
   out_271066960830845225[56] = 0;
   out_271066960830845225[57] = 1.0;
   out_271066960830845225[58] = 0;
   out_271066960830845225[59] = 0;
   out_271066960830845225[60] = 0;
   out_271066960830845225[61] = 0;
   out_271066960830845225[62] = 0;
   out_271066960830845225[63] = 0;
   out_271066960830845225[64] = 0;
   out_271066960830845225[65] = 0;
   out_271066960830845225[66] = 0;
   out_271066960830845225[67] = 0;
   out_271066960830845225[68] = 0;
   out_271066960830845225[69] = 0;
   out_271066960830845225[70] = 0;
   out_271066960830845225[71] = 0;
   out_271066960830845225[72] = 0;
   out_271066960830845225[73] = 0;
   out_271066960830845225[74] = 0;
   out_271066960830845225[75] = 0;
   out_271066960830845225[76] = 1.0;
   out_271066960830845225[77] = 0;
   out_271066960830845225[78] = 0;
   out_271066960830845225[79] = 0;
   out_271066960830845225[80] = 0;
   out_271066960830845225[81] = 0;
   out_271066960830845225[82] = 0;
   out_271066960830845225[83] = 0;
   out_271066960830845225[84] = 0;
   out_271066960830845225[85] = 0;
   out_271066960830845225[86] = 0;
   out_271066960830845225[87] = 0;
   out_271066960830845225[88] = 0;
   out_271066960830845225[89] = 0;
   out_271066960830845225[90] = 0;
   out_271066960830845225[91] = 0;
   out_271066960830845225[92] = 0;
   out_271066960830845225[93] = 0;
   out_271066960830845225[94] = 0;
   out_271066960830845225[95] = 1.0;
   out_271066960830845225[96] = 0;
   out_271066960830845225[97] = 0;
   out_271066960830845225[98] = 0;
   out_271066960830845225[99] = 0;
   out_271066960830845225[100] = 0;
   out_271066960830845225[101] = 0;
   out_271066960830845225[102] = 0;
   out_271066960830845225[103] = 0;
   out_271066960830845225[104] = 0;
   out_271066960830845225[105] = 0;
   out_271066960830845225[106] = 0;
   out_271066960830845225[107] = 0;
   out_271066960830845225[108] = 0;
   out_271066960830845225[109] = 0;
   out_271066960830845225[110] = 0;
   out_271066960830845225[111] = 0;
   out_271066960830845225[112] = 0;
   out_271066960830845225[113] = 0;
   out_271066960830845225[114] = 1.0;
   out_271066960830845225[115] = 0;
   out_271066960830845225[116] = 0;
   out_271066960830845225[117] = 0;
   out_271066960830845225[118] = 0;
   out_271066960830845225[119] = 0;
   out_271066960830845225[120] = 0;
   out_271066960830845225[121] = 0;
   out_271066960830845225[122] = 0;
   out_271066960830845225[123] = 0;
   out_271066960830845225[124] = 0;
   out_271066960830845225[125] = 0;
   out_271066960830845225[126] = 0;
   out_271066960830845225[127] = 0;
   out_271066960830845225[128] = 0;
   out_271066960830845225[129] = 0;
   out_271066960830845225[130] = 0;
   out_271066960830845225[131] = 0;
   out_271066960830845225[132] = 0;
   out_271066960830845225[133] = 1.0;
   out_271066960830845225[134] = 0;
   out_271066960830845225[135] = 0;
   out_271066960830845225[136] = 0;
   out_271066960830845225[137] = 0;
   out_271066960830845225[138] = 0;
   out_271066960830845225[139] = 0;
   out_271066960830845225[140] = 0;
   out_271066960830845225[141] = 0;
   out_271066960830845225[142] = 0;
   out_271066960830845225[143] = 0;
   out_271066960830845225[144] = 0;
   out_271066960830845225[145] = 0;
   out_271066960830845225[146] = 0;
   out_271066960830845225[147] = 0;
   out_271066960830845225[148] = 0;
   out_271066960830845225[149] = 0;
   out_271066960830845225[150] = 0;
   out_271066960830845225[151] = 0;
   out_271066960830845225[152] = 1.0;
   out_271066960830845225[153] = 0;
   out_271066960830845225[154] = 0;
   out_271066960830845225[155] = 0;
   out_271066960830845225[156] = 0;
   out_271066960830845225[157] = 0;
   out_271066960830845225[158] = 0;
   out_271066960830845225[159] = 0;
   out_271066960830845225[160] = 0;
   out_271066960830845225[161] = 0;
   out_271066960830845225[162] = 0;
   out_271066960830845225[163] = 0;
   out_271066960830845225[164] = 0;
   out_271066960830845225[165] = 0;
   out_271066960830845225[166] = 0;
   out_271066960830845225[167] = 0;
   out_271066960830845225[168] = 0;
   out_271066960830845225[169] = 0;
   out_271066960830845225[170] = 0;
   out_271066960830845225[171] = 1.0;
   out_271066960830845225[172] = 0;
   out_271066960830845225[173] = 0;
   out_271066960830845225[174] = 0;
   out_271066960830845225[175] = 0;
   out_271066960830845225[176] = 0;
   out_271066960830845225[177] = 0;
   out_271066960830845225[178] = 0;
   out_271066960830845225[179] = 0;
   out_271066960830845225[180] = 0;
   out_271066960830845225[181] = 0;
   out_271066960830845225[182] = 0;
   out_271066960830845225[183] = 0;
   out_271066960830845225[184] = 0;
   out_271066960830845225[185] = 0;
   out_271066960830845225[186] = 0;
   out_271066960830845225[187] = 0;
   out_271066960830845225[188] = 0;
   out_271066960830845225[189] = 0;
   out_271066960830845225[190] = 1.0;
   out_271066960830845225[191] = 0;
   out_271066960830845225[192] = 0;
   out_271066960830845225[193] = 0;
   out_271066960830845225[194] = 0;
   out_271066960830845225[195] = 0;
   out_271066960830845225[196] = 0;
   out_271066960830845225[197] = 0;
   out_271066960830845225[198] = 0;
   out_271066960830845225[199] = 0;
   out_271066960830845225[200] = 0;
   out_271066960830845225[201] = 0;
   out_271066960830845225[202] = 0;
   out_271066960830845225[203] = 0;
   out_271066960830845225[204] = 0;
   out_271066960830845225[205] = 0;
   out_271066960830845225[206] = 0;
   out_271066960830845225[207] = 0;
   out_271066960830845225[208] = 0;
   out_271066960830845225[209] = 1.0;
   out_271066960830845225[210] = 0;
   out_271066960830845225[211] = 0;
   out_271066960830845225[212] = 0;
   out_271066960830845225[213] = 0;
   out_271066960830845225[214] = 0;
   out_271066960830845225[215] = 0;
   out_271066960830845225[216] = 0;
   out_271066960830845225[217] = 0;
   out_271066960830845225[218] = 0;
   out_271066960830845225[219] = 0;
   out_271066960830845225[220] = 0;
   out_271066960830845225[221] = 0;
   out_271066960830845225[222] = 0;
   out_271066960830845225[223] = 0;
   out_271066960830845225[224] = 0;
   out_271066960830845225[225] = 0;
   out_271066960830845225[226] = 0;
   out_271066960830845225[227] = 0;
   out_271066960830845225[228] = 1.0;
   out_271066960830845225[229] = 0;
   out_271066960830845225[230] = 0;
   out_271066960830845225[231] = 0;
   out_271066960830845225[232] = 0;
   out_271066960830845225[233] = 0;
   out_271066960830845225[234] = 0;
   out_271066960830845225[235] = 0;
   out_271066960830845225[236] = 0;
   out_271066960830845225[237] = 0;
   out_271066960830845225[238] = 0;
   out_271066960830845225[239] = 0;
   out_271066960830845225[240] = 0;
   out_271066960830845225[241] = 0;
   out_271066960830845225[242] = 0;
   out_271066960830845225[243] = 0;
   out_271066960830845225[244] = 0;
   out_271066960830845225[245] = 0;
   out_271066960830845225[246] = 0;
   out_271066960830845225[247] = 1.0;
   out_271066960830845225[248] = 0;
   out_271066960830845225[249] = 0;
   out_271066960830845225[250] = 0;
   out_271066960830845225[251] = 0;
   out_271066960830845225[252] = 0;
   out_271066960830845225[253] = 0;
   out_271066960830845225[254] = 0;
   out_271066960830845225[255] = 0;
   out_271066960830845225[256] = 0;
   out_271066960830845225[257] = 0;
   out_271066960830845225[258] = 0;
   out_271066960830845225[259] = 0;
   out_271066960830845225[260] = 0;
   out_271066960830845225[261] = 0;
   out_271066960830845225[262] = 0;
   out_271066960830845225[263] = 0;
   out_271066960830845225[264] = 0;
   out_271066960830845225[265] = 0;
   out_271066960830845225[266] = 1.0;
   out_271066960830845225[267] = 0;
   out_271066960830845225[268] = 0;
   out_271066960830845225[269] = 0;
   out_271066960830845225[270] = 0;
   out_271066960830845225[271] = 0;
   out_271066960830845225[272] = 0;
   out_271066960830845225[273] = 0;
   out_271066960830845225[274] = 0;
   out_271066960830845225[275] = 0;
   out_271066960830845225[276] = 0;
   out_271066960830845225[277] = 0;
   out_271066960830845225[278] = 0;
   out_271066960830845225[279] = 0;
   out_271066960830845225[280] = 0;
   out_271066960830845225[281] = 0;
   out_271066960830845225[282] = 0;
   out_271066960830845225[283] = 0;
   out_271066960830845225[284] = 0;
   out_271066960830845225[285] = 1.0;
   out_271066960830845225[286] = 0;
   out_271066960830845225[287] = 0;
   out_271066960830845225[288] = 0;
   out_271066960830845225[289] = 0;
   out_271066960830845225[290] = 0;
   out_271066960830845225[291] = 0;
   out_271066960830845225[292] = 0;
   out_271066960830845225[293] = 0;
   out_271066960830845225[294] = 0;
   out_271066960830845225[295] = 0;
   out_271066960830845225[296] = 0;
   out_271066960830845225[297] = 0;
   out_271066960830845225[298] = 0;
   out_271066960830845225[299] = 0;
   out_271066960830845225[300] = 0;
   out_271066960830845225[301] = 0;
   out_271066960830845225[302] = 0;
   out_271066960830845225[303] = 0;
   out_271066960830845225[304] = 1.0;
   out_271066960830845225[305] = 0;
   out_271066960830845225[306] = 0;
   out_271066960830845225[307] = 0;
   out_271066960830845225[308] = 0;
   out_271066960830845225[309] = 0;
   out_271066960830845225[310] = 0;
   out_271066960830845225[311] = 0;
   out_271066960830845225[312] = 0;
   out_271066960830845225[313] = 0;
   out_271066960830845225[314] = 0;
   out_271066960830845225[315] = 0;
   out_271066960830845225[316] = 0;
   out_271066960830845225[317] = 0;
   out_271066960830845225[318] = 0;
   out_271066960830845225[319] = 0;
   out_271066960830845225[320] = 0;
   out_271066960830845225[321] = 0;
   out_271066960830845225[322] = 0;
   out_271066960830845225[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3845547857628505078) {
   out_3845547857628505078[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3845547857628505078[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3845547857628505078[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3845547857628505078[3] = dt*state[12] + state[3];
   out_3845547857628505078[4] = dt*state[13] + state[4];
   out_3845547857628505078[5] = dt*state[14] + state[5];
   out_3845547857628505078[6] = state[6];
   out_3845547857628505078[7] = state[7];
   out_3845547857628505078[8] = state[8];
   out_3845547857628505078[9] = state[9];
   out_3845547857628505078[10] = state[10];
   out_3845547857628505078[11] = state[11];
   out_3845547857628505078[12] = state[12];
   out_3845547857628505078[13] = state[13];
   out_3845547857628505078[14] = state[14];
   out_3845547857628505078[15] = state[15];
   out_3845547857628505078[16] = state[16];
   out_3845547857628505078[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1631823746137781300) {
   out_1631823746137781300[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1631823746137781300[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1631823746137781300[2] = 0;
   out_1631823746137781300[3] = 0;
   out_1631823746137781300[4] = 0;
   out_1631823746137781300[5] = 0;
   out_1631823746137781300[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1631823746137781300[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1631823746137781300[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1631823746137781300[9] = 0;
   out_1631823746137781300[10] = 0;
   out_1631823746137781300[11] = 0;
   out_1631823746137781300[12] = 0;
   out_1631823746137781300[13] = 0;
   out_1631823746137781300[14] = 0;
   out_1631823746137781300[15] = 0;
   out_1631823746137781300[16] = 0;
   out_1631823746137781300[17] = 0;
   out_1631823746137781300[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1631823746137781300[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1631823746137781300[20] = 0;
   out_1631823746137781300[21] = 0;
   out_1631823746137781300[22] = 0;
   out_1631823746137781300[23] = 0;
   out_1631823746137781300[24] = 0;
   out_1631823746137781300[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1631823746137781300[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1631823746137781300[27] = 0;
   out_1631823746137781300[28] = 0;
   out_1631823746137781300[29] = 0;
   out_1631823746137781300[30] = 0;
   out_1631823746137781300[31] = 0;
   out_1631823746137781300[32] = 0;
   out_1631823746137781300[33] = 0;
   out_1631823746137781300[34] = 0;
   out_1631823746137781300[35] = 0;
   out_1631823746137781300[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1631823746137781300[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1631823746137781300[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1631823746137781300[39] = 0;
   out_1631823746137781300[40] = 0;
   out_1631823746137781300[41] = 0;
   out_1631823746137781300[42] = 0;
   out_1631823746137781300[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1631823746137781300[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1631823746137781300[45] = 0;
   out_1631823746137781300[46] = 0;
   out_1631823746137781300[47] = 0;
   out_1631823746137781300[48] = 0;
   out_1631823746137781300[49] = 0;
   out_1631823746137781300[50] = 0;
   out_1631823746137781300[51] = 0;
   out_1631823746137781300[52] = 0;
   out_1631823746137781300[53] = 0;
   out_1631823746137781300[54] = 0;
   out_1631823746137781300[55] = 0;
   out_1631823746137781300[56] = 0;
   out_1631823746137781300[57] = 1;
   out_1631823746137781300[58] = 0;
   out_1631823746137781300[59] = 0;
   out_1631823746137781300[60] = 0;
   out_1631823746137781300[61] = 0;
   out_1631823746137781300[62] = 0;
   out_1631823746137781300[63] = 0;
   out_1631823746137781300[64] = 0;
   out_1631823746137781300[65] = 0;
   out_1631823746137781300[66] = dt;
   out_1631823746137781300[67] = 0;
   out_1631823746137781300[68] = 0;
   out_1631823746137781300[69] = 0;
   out_1631823746137781300[70] = 0;
   out_1631823746137781300[71] = 0;
   out_1631823746137781300[72] = 0;
   out_1631823746137781300[73] = 0;
   out_1631823746137781300[74] = 0;
   out_1631823746137781300[75] = 0;
   out_1631823746137781300[76] = 1;
   out_1631823746137781300[77] = 0;
   out_1631823746137781300[78] = 0;
   out_1631823746137781300[79] = 0;
   out_1631823746137781300[80] = 0;
   out_1631823746137781300[81] = 0;
   out_1631823746137781300[82] = 0;
   out_1631823746137781300[83] = 0;
   out_1631823746137781300[84] = 0;
   out_1631823746137781300[85] = dt;
   out_1631823746137781300[86] = 0;
   out_1631823746137781300[87] = 0;
   out_1631823746137781300[88] = 0;
   out_1631823746137781300[89] = 0;
   out_1631823746137781300[90] = 0;
   out_1631823746137781300[91] = 0;
   out_1631823746137781300[92] = 0;
   out_1631823746137781300[93] = 0;
   out_1631823746137781300[94] = 0;
   out_1631823746137781300[95] = 1;
   out_1631823746137781300[96] = 0;
   out_1631823746137781300[97] = 0;
   out_1631823746137781300[98] = 0;
   out_1631823746137781300[99] = 0;
   out_1631823746137781300[100] = 0;
   out_1631823746137781300[101] = 0;
   out_1631823746137781300[102] = 0;
   out_1631823746137781300[103] = 0;
   out_1631823746137781300[104] = dt;
   out_1631823746137781300[105] = 0;
   out_1631823746137781300[106] = 0;
   out_1631823746137781300[107] = 0;
   out_1631823746137781300[108] = 0;
   out_1631823746137781300[109] = 0;
   out_1631823746137781300[110] = 0;
   out_1631823746137781300[111] = 0;
   out_1631823746137781300[112] = 0;
   out_1631823746137781300[113] = 0;
   out_1631823746137781300[114] = 1;
   out_1631823746137781300[115] = 0;
   out_1631823746137781300[116] = 0;
   out_1631823746137781300[117] = 0;
   out_1631823746137781300[118] = 0;
   out_1631823746137781300[119] = 0;
   out_1631823746137781300[120] = 0;
   out_1631823746137781300[121] = 0;
   out_1631823746137781300[122] = 0;
   out_1631823746137781300[123] = 0;
   out_1631823746137781300[124] = 0;
   out_1631823746137781300[125] = 0;
   out_1631823746137781300[126] = 0;
   out_1631823746137781300[127] = 0;
   out_1631823746137781300[128] = 0;
   out_1631823746137781300[129] = 0;
   out_1631823746137781300[130] = 0;
   out_1631823746137781300[131] = 0;
   out_1631823746137781300[132] = 0;
   out_1631823746137781300[133] = 1;
   out_1631823746137781300[134] = 0;
   out_1631823746137781300[135] = 0;
   out_1631823746137781300[136] = 0;
   out_1631823746137781300[137] = 0;
   out_1631823746137781300[138] = 0;
   out_1631823746137781300[139] = 0;
   out_1631823746137781300[140] = 0;
   out_1631823746137781300[141] = 0;
   out_1631823746137781300[142] = 0;
   out_1631823746137781300[143] = 0;
   out_1631823746137781300[144] = 0;
   out_1631823746137781300[145] = 0;
   out_1631823746137781300[146] = 0;
   out_1631823746137781300[147] = 0;
   out_1631823746137781300[148] = 0;
   out_1631823746137781300[149] = 0;
   out_1631823746137781300[150] = 0;
   out_1631823746137781300[151] = 0;
   out_1631823746137781300[152] = 1;
   out_1631823746137781300[153] = 0;
   out_1631823746137781300[154] = 0;
   out_1631823746137781300[155] = 0;
   out_1631823746137781300[156] = 0;
   out_1631823746137781300[157] = 0;
   out_1631823746137781300[158] = 0;
   out_1631823746137781300[159] = 0;
   out_1631823746137781300[160] = 0;
   out_1631823746137781300[161] = 0;
   out_1631823746137781300[162] = 0;
   out_1631823746137781300[163] = 0;
   out_1631823746137781300[164] = 0;
   out_1631823746137781300[165] = 0;
   out_1631823746137781300[166] = 0;
   out_1631823746137781300[167] = 0;
   out_1631823746137781300[168] = 0;
   out_1631823746137781300[169] = 0;
   out_1631823746137781300[170] = 0;
   out_1631823746137781300[171] = 1;
   out_1631823746137781300[172] = 0;
   out_1631823746137781300[173] = 0;
   out_1631823746137781300[174] = 0;
   out_1631823746137781300[175] = 0;
   out_1631823746137781300[176] = 0;
   out_1631823746137781300[177] = 0;
   out_1631823746137781300[178] = 0;
   out_1631823746137781300[179] = 0;
   out_1631823746137781300[180] = 0;
   out_1631823746137781300[181] = 0;
   out_1631823746137781300[182] = 0;
   out_1631823746137781300[183] = 0;
   out_1631823746137781300[184] = 0;
   out_1631823746137781300[185] = 0;
   out_1631823746137781300[186] = 0;
   out_1631823746137781300[187] = 0;
   out_1631823746137781300[188] = 0;
   out_1631823746137781300[189] = 0;
   out_1631823746137781300[190] = 1;
   out_1631823746137781300[191] = 0;
   out_1631823746137781300[192] = 0;
   out_1631823746137781300[193] = 0;
   out_1631823746137781300[194] = 0;
   out_1631823746137781300[195] = 0;
   out_1631823746137781300[196] = 0;
   out_1631823746137781300[197] = 0;
   out_1631823746137781300[198] = 0;
   out_1631823746137781300[199] = 0;
   out_1631823746137781300[200] = 0;
   out_1631823746137781300[201] = 0;
   out_1631823746137781300[202] = 0;
   out_1631823746137781300[203] = 0;
   out_1631823746137781300[204] = 0;
   out_1631823746137781300[205] = 0;
   out_1631823746137781300[206] = 0;
   out_1631823746137781300[207] = 0;
   out_1631823746137781300[208] = 0;
   out_1631823746137781300[209] = 1;
   out_1631823746137781300[210] = 0;
   out_1631823746137781300[211] = 0;
   out_1631823746137781300[212] = 0;
   out_1631823746137781300[213] = 0;
   out_1631823746137781300[214] = 0;
   out_1631823746137781300[215] = 0;
   out_1631823746137781300[216] = 0;
   out_1631823746137781300[217] = 0;
   out_1631823746137781300[218] = 0;
   out_1631823746137781300[219] = 0;
   out_1631823746137781300[220] = 0;
   out_1631823746137781300[221] = 0;
   out_1631823746137781300[222] = 0;
   out_1631823746137781300[223] = 0;
   out_1631823746137781300[224] = 0;
   out_1631823746137781300[225] = 0;
   out_1631823746137781300[226] = 0;
   out_1631823746137781300[227] = 0;
   out_1631823746137781300[228] = 1;
   out_1631823746137781300[229] = 0;
   out_1631823746137781300[230] = 0;
   out_1631823746137781300[231] = 0;
   out_1631823746137781300[232] = 0;
   out_1631823746137781300[233] = 0;
   out_1631823746137781300[234] = 0;
   out_1631823746137781300[235] = 0;
   out_1631823746137781300[236] = 0;
   out_1631823746137781300[237] = 0;
   out_1631823746137781300[238] = 0;
   out_1631823746137781300[239] = 0;
   out_1631823746137781300[240] = 0;
   out_1631823746137781300[241] = 0;
   out_1631823746137781300[242] = 0;
   out_1631823746137781300[243] = 0;
   out_1631823746137781300[244] = 0;
   out_1631823746137781300[245] = 0;
   out_1631823746137781300[246] = 0;
   out_1631823746137781300[247] = 1;
   out_1631823746137781300[248] = 0;
   out_1631823746137781300[249] = 0;
   out_1631823746137781300[250] = 0;
   out_1631823746137781300[251] = 0;
   out_1631823746137781300[252] = 0;
   out_1631823746137781300[253] = 0;
   out_1631823746137781300[254] = 0;
   out_1631823746137781300[255] = 0;
   out_1631823746137781300[256] = 0;
   out_1631823746137781300[257] = 0;
   out_1631823746137781300[258] = 0;
   out_1631823746137781300[259] = 0;
   out_1631823746137781300[260] = 0;
   out_1631823746137781300[261] = 0;
   out_1631823746137781300[262] = 0;
   out_1631823746137781300[263] = 0;
   out_1631823746137781300[264] = 0;
   out_1631823746137781300[265] = 0;
   out_1631823746137781300[266] = 1;
   out_1631823746137781300[267] = 0;
   out_1631823746137781300[268] = 0;
   out_1631823746137781300[269] = 0;
   out_1631823746137781300[270] = 0;
   out_1631823746137781300[271] = 0;
   out_1631823746137781300[272] = 0;
   out_1631823746137781300[273] = 0;
   out_1631823746137781300[274] = 0;
   out_1631823746137781300[275] = 0;
   out_1631823746137781300[276] = 0;
   out_1631823746137781300[277] = 0;
   out_1631823746137781300[278] = 0;
   out_1631823746137781300[279] = 0;
   out_1631823746137781300[280] = 0;
   out_1631823746137781300[281] = 0;
   out_1631823746137781300[282] = 0;
   out_1631823746137781300[283] = 0;
   out_1631823746137781300[284] = 0;
   out_1631823746137781300[285] = 1;
   out_1631823746137781300[286] = 0;
   out_1631823746137781300[287] = 0;
   out_1631823746137781300[288] = 0;
   out_1631823746137781300[289] = 0;
   out_1631823746137781300[290] = 0;
   out_1631823746137781300[291] = 0;
   out_1631823746137781300[292] = 0;
   out_1631823746137781300[293] = 0;
   out_1631823746137781300[294] = 0;
   out_1631823746137781300[295] = 0;
   out_1631823746137781300[296] = 0;
   out_1631823746137781300[297] = 0;
   out_1631823746137781300[298] = 0;
   out_1631823746137781300[299] = 0;
   out_1631823746137781300[300] = 0;
   out_1631823746137781300[301] = 0;
   out_1631823746137781300[302] = 0;
   out_1631823746137781300[303] = 0;
   out_1631823746137781300[304] = 1;
   out_1631823746137781300[305] = 0;
   out_1631823746137781300[306] = 0;
   out_1631823746137781300[307] = 0;
   out_1631823746137781300[308] = 0;
   out_1631823746137781300[309] = 0;
   out_1631823746137781300[310] = 0;
   out_1631823746137781300[311] = 0;
   out_1631823746137781300[312] = 0;
   out_1631823746137781300[313] = 0;
   out_1631823746137781300[314] = 0;
   out_1631823746137781300[315] = 0;
   out_1631823746137781300[316] = 0;
   out_1631823746137781300[317] = 0;
   out_1631823746137781300[318] = 0;
   out_1631823746137781300[319] = 0;
   out_1631823746137781300[320] = 0;
   out_1631823746137781300[321] = 0;
   out_1631823746137781300[322] = 0;
   out_1631823746137781300[323] = 1;
}
void h_4(double *state, double *unused, double *out_4955052215047598515) {
   out_4955052215047598515[0] = state[6] + state[9];
   out_4955052215047598515[1] = state[7] + state[10];
   out_4955052215047598515[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4190822113574178604) {
   out_4190822113574178604[0] = 0;
   out_4190822113574178604[1] = 0;
   out_4190822113574178604[2] = 0;
   out_4190822113574178604[3] = 0;
   out_4190822113574178604[4] = 0;
   out_4190822113574178604[5] = 0;
   out_4190822113574178604[6] = 1;
   out_4190822113574178604[7] = 0;
   out_4190822113574178604[8] = 0;
   out_4190822113574178604[9] = 1;
   out_4190822113574178604[10] = 0;
   out_4190822113574178604[11] = 0;
   out_4190822113574178604[12] = 0;
   out_4190822113574178604[13] = 0;
   out_4190822113574178604[14] = 0;
   out_4190822113574178604[15] = 0;
   out_4190822113574178604[16] = 0;
   out_4190822113574178604[17] = 0;
   out_4190822113574178604[18] = 0;
   out_4190822113574178604[19] = 0;
   out_4190822113574178604[20] = 0;
   out_4190822113574178604[21] = 0;
   out_4190822113574178604[22] = 0;
   out_4190822113574178604[23] = 0;
   out_4190822113574178604[24] = 0;
   out_4190822113574178604[25] = 1;
   out_4190822113574178604[26] = 0;
   out_4190822113574178604[27] = 0;
   out_4190822113574178604[28] = 1;
   out_4190822113574178604[29] = 0;
   out_4190822113574178604[30] = 0;
   out_4190822113574178604[31] = 0;
   out_4190822113574178604[32] = 0;
   out_4190822113574178604[33] = 0;
   out_4190822113574178604[34] = 0;
   out_4190822113574178604[35] = 0;
   out_4190822113574178604[36] = 0;
   out_4190822113574178604[37] = 0;
   out_4190822113574178604[38] = 0;
   out_4190822113574178604[39] = 0;
   out_4190822113574178604[40] = 0;
   out_4190822113574178604[41] = 0;
   out_4190822113574178604[42] = 0;
   out_4190822113574178604[43] = 0;
   out_4190822113574178604[44] = 1;
   out_4190822113574178604[45] = 0;
   out_4190822113574178604[46] = 0;
   out_4190822113574178604[47] = 1;
   out_4190822113574178604[48] = 0;
   out_4190822113574178604[49] = 0;
   out_4190822113574178604[50] = 0;
   out_4190822113574178604[51] = 0;
   out_4190822113574178604[52] = 0;
   out_4190822113574178604[53] = 0;
}
void h_10(double *state, double *unused, double *out_4337291095685463138) {
   out_4337291095685463138[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4337291095685463138[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4337291095685463138[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1798816701295712907) {
   out_1798816701295712907[0] = 0;
   out_1798816701295712907[1] = 9.8100000000000005*cos(state[1]);
   out_1798816701295712907[2] = 0;
   out_1798816701295712907[3] = 0;
   out_1798816701295712907[4] = -state[8];
   out_1798816701295712907[5] = state[7];
   out_1798816701295712907[6] = 0;
   out_1798816701295712907[7] = state[5];
   out_1798816701295712907[8] = -state[4];
   out_1798816701295712907[9] = 0;
   out_1798816701295712907[10] = 0;
   out_1798816701295712907[11] = 0;
   out_1798816701295712907[12] = 1;
   out_1798816701295712907[13] = 0;
   out_1798816701295712907[14] = 0;
   out_1798816701295712907[15] = 1;
   out_1798816701295712907[16] = 0;
   out_1798816701295712907[17] = 0;
   out_1798816701295712907[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1798816701295712907[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1798816701295712907[20] = 0;
   out_1798816701295712907[21] = state[8];
   out_1798816701295712907[22] = 0;
   out_1798816701295712907[23] = -state[6];
   out_1798816701295712907[24] = -state[5];
   out_1798816701295712907[25] = 0;
   out_1798816701295712907[26] = state[3];
   out_1798816701295712907[27] = 0;
   out_1798816701295712907[28] = 0;
   out_1798816701295712907[29] = 0;
   out_1798816701295712907[30] = 0;
   out_1798816701295712907[31] = 1;
   out_1798816701295712907[32] = 0;
   out_1798816701295712907[33] = 0;
   out_1798816701295712907[34] = 1;
   out_1798816701295712907[35] = 0;
   out_1798816701295712907[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1798816701295712907[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1798816701295712907[38] = 0;
   out_1798816701295712907[39] = -state[7];
   out_1798816701295712907[40] = state[6];
   out_1798816701295712907[41] = 0;
   out_1798816701295712907[42] = state[4];
   out_1798816701295712907[43] = -state[3];
   out_1798816701295712907[44] = 0;
   out_1798816701295712907[45] = 0;
   out_1798816701295712907[46] = 0;
   out_1798816701295712907[47] = 0;
   out_1798816701295712907[48] = 0;
   out_1798816701295712907[49] = 0;
   out_1798816701295712907[50] = 1;
   out_1798816701295712907[51] = 0;
   out_1798816701295712907[52] = 0;
   out_1798816701295712907[53] = 1;
}
void h_13(double *state, double *unused, double *out_2119852544367272368) {
   out_2119852544367272368[0] = state[3];
   out_2119852544367272368[1] = state[4];
   out_2119852544367272368[2] = state[5];
}
void H_13(double *state, double *unused, double *out_978548288241845803) {
   out_978548288241845803[0] = 0;
   out_978548288241845803[1] = 0;
   out_978548288241845803[2] = 0;
   out_978548288241845803[3] = 1;
   out_978548288241845803[4] = 0;
   out_978548288241845803[5] = 0;
   out_978548288241845803[6] = 0;
   out_978548288241845803[7] = 0;
   out_978548288241845803[8] = 0;
   out_978548288241845803[9] = 0;
   out_978548288241845803[10] = 0;
   out_978548288241845803[11] = 0;
   out_978548288241845803[12] = 0;
   out_978548288241845803[13] = 0;
   out_978548288241845803[14] = 0;
   out_978548288241845803[15] = 0;
   out_978548288241845803[16] = 0;
   out_978548288241845803[17] = 0;
   out_978548288241845803[18] = 0;
   out_978548288241845803[19] = 0;
   out_978548288241845803[20] = 0;
   out_978548288241845803[21] = 0;
   out_978548288241845803[22] = 1;
   out_978548288241845803[23] = 0;
   out_978548288241845803[24] = 0;
   out_978548288241845803[25] = 0;
   out_978548288241845803[26] = 0;
   out_978548288241845803[27] = 0;
   out_978548288241845803[28] = 0;
   out_978548288241845803[29] = 0;
   out_978548288241845803[30] = 0;
   out_978548288241845803[31] = 0;
   out_978548288241845803[32] = 0;
   out_978548288241845803[33] = 0;
   out_978548288241845803[34] = 0;
   out_978548288241845803[35] = 0;
   out_978548288241845803[36] = 0;
   out_978548288241845803[37] = 0;
   out_978548288241845803[38] = 0;
   out_978548288241845803[39] = 0;
   out_978548288241845803[40] = 0;
   out_978548288241845803[41] = 1;
   out_978548288241845803[42] = 0;
   out_978548288241845803[43] = 0;
   out_978548288241845803[44] = 0;
   out_978548288241845803[45] = 0;
   out_978548288241845803[46] = 0;
   out_978548288241845803[47] = 0;
   out_978548288241845803[48] = 0;
   out_978548288241845803[49] = 0;
   out_978548288241845803[50] = 0;
   out_978548288241845803[51] = 0;
   out_978548288241845803[52] = 0;
   out_978548288241845803[53] = 0;
}
void h_14(double *state, double *unused, double *out_178331072993193741) {
   out_178331072993193741[0] = state[6];
   out_178331072993193741[1] = state[7];
   out_178331072993193741[2] = state[8];
}
void H_14(double *state, double *unused, double *out_227581257234694075) {
   out_227581257234694075[0] = 0;
   out_227581257234694075[1] = 0;
   out_227581257234694075[2] = 0;
   out_227581257234694075[3] = 0;
   out_227581257234694075[4] = 0;
   out_227581257234694075[5] = 0;
   out_227581257234694075[6] = 1;
   out_227581257234694075[7] = 0;
   out_227581257234694075[8] = 0;
   out_227581257234694075[9] = 0;
   out_227581257234694075[10] = 0;
   out_227581257234694075[11] = 0;
   out_227581257234694075[12] = 0;
   out_227581257234694075[13] = 0;
   out_227581257234694075[14] = 0;
   out_227581257234694075[15] = 0;
   out_227581257234694075[16] = 0;
   out_227581257234694075[17] = 0;
   out_227581257234694075[18] = 0;
   out_227581257234694075[19] = 0;
   out_227581257234694075[20] = 0;
   out_227581257234694075[21] = 0;
   out_227581257234694075[22] = 0;
   out_227581257234694075[23] = 0;
   out_227581257234694075[24] = 0;
   out_227581257234694075[25] = 1;
   out_227581257234694075[26] = 0;
   out_227581257234694075[27] = 0;
   out_227581257234694075[28] = 0;
   out_227581257234694075[29] = 0;
   out_227581257234694075[30] = 0;
   out_227581257234694075[31] = 0;
   out_227581257234694075[32] = 0;
   out_227581257234694075[33] = 0;
   out_227581257234694075[34] = 0;
   out_227581257234694075[35] = 0;
   out_227581257234694075[36] = 0;
   out_227581257234694075[37] = 0;
   out_227581257234694075[38] = 0;
   out_227581257234694075[39] = 0;
   out_227581257234694075[40] = 0;
   out_227581257234694075[41] = 0;
   out_227581257234694075[42] = 0;
   out_227581257234694075[43] = 0;
   out_227581257234694075[44] = 1;
   out_227581257234694075[45] = 0;
   out_227581257234694075[46] = 0;
   out_227581257234694075[47] = 0;
   out_227581257234694075[48] = 0;
   out_227581257234694075[49] = 0;
   out_227581257234694075[50] = 0;
   out_227581257234694075[51] = 0;
   out_227581257234694075[52] = 0;
   out_227581257234694075[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7863920857934757474) {
  err_fun(nom_x, delta_x, out_7863920857934757474);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1293085161672310543) {
  inv_err_fun(nom_x, true_x, out_1293085161672310543);
}
void pose_H_mod_fun(double *state, double *out_271066960830845225) {
  H_mod_fun(state, out_271066960830845225);
}
void pose_f_fun(double *state, double dt, double *out_3845547857628505078) {
  f_fun(state,  dt, out_3845547857628505078);
}
void pose_F_fun(double *state, double dt, double *out_1631823746137781300) {
  F_fun(state,  dt, out_1631823746137781300);
}
void pose_h_4(double *state, double *unused, double *out_4955052215047598515) {
  h_4(state, unused, out_4955052215047598515);
}
void pose_H_4(double *state, double *unused, double *out_4190822113574178604) {
  H_4(state, unused, out_4190822113574178604);
}
void pose_h_10(double *state, double *unused, double *out_4337291095685463138) {
  h_10(state, unused, out_4337291095685463138);
}
void pose_H_10(double *state, double *unused, double *out_1798816701295712907) {
  H_10(state, unused, out_1798816701295712907);
}
void pose_h_13(double *state, double *unused, double *out_2119852544367272368) {
  h_13(state, unused, out_2119852544367272368);
}
void pose_H_13(double *state, double *unused, double *out_978548288241845803) {
  H_13(state, unused, out_978548288241845803);
}
void pose_h_14(double *state, double *unused, double *out_178331072993193741) {
  h_14(state, unused, out_178331072993193741);
}
void pose_H_14(double *state, double *unused, double *out_227581257234694075) {
  H_14(state, unused, out_227581257234694075);
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
