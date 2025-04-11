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
void err_fun(double *nom_x, double *delta_x, double *out_2258393448586014737) {
   out_2258393448586014737[0] = delta_x[0] + nom_x[0];
   out_2258393448586014737[1] = delta_x[1] + nom_x[1];
   out_2258393448586014737[2] = delta_x[2] + nom_x[2];
   out_2258393448586014737[3] = delta_x[3] + nom_x[3];
   out_2258393448586014737[4] = delta_x[4] + nom_x[4];
   out_2258393448586014737[5] = delta_x[5] + nom_x[5];
   out_2258393448586014737[6] = delta_x[6] + nom_x[6];
   out_2258393448586014737[7] = delta_x[7] + nom_x[7];
   out_2258393448586014737[8] = delta_x[8] + nom_x[8];
   out_2258393448586014737[9] = delta_x[9] + nom_x[9];
   out_2258393448586014737[10] = delta_x[10] + nom_x[10];
   out_2258393448586014737[11] = delta_x[11] + nom_x[11];
   out_2258393448586014737[12] = delta_x[12] + nom_x[12];
   out_2258393448586014737[13] = delta_x[13] + nom_x[13];
   out_2258393448586014737[14] = delta_x[14] + nom_x[14];
   out_2258393448586014737[15] = delta_x[15] + nom_x[15];
   out_2258393448586014737[16] = delta_x[16] + nom_x[16];
   out_2258393448586014737[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_150977374342042324) {
   out_150977374342042324[0] = -nom_x[0] + true_x[0];
   out_150977374342042324[1] = -nom_x[1] + true_x[1];
   out_150977374342042324[2] = -nom_x[2] + true_x[2];
   out_150977374342042324[3] = -nom_x[3] + true_x[3];
   out_150977374342042324[4] = -nom_x[4] + true_x[4];
   out_150977374342042324[5] = -nom_x[5] + true_x[5];
   out_150977374342042324[6] = -nom_x[6] + true_x[6];
   out_150977374342042324[7] = -nom_x[7] + true_x[7];
   out_150977374342042324[8] = -nom_x[8] + true_x[8];
   out_150977374342042324[9] = -nom_x[9] + true_x[9];
   out_150977374342042324[10] = -nom_x[10] + true_x[10];
   out_150977374342042324[11] = -nom_x[11] + true_x[11];
   out_150977374342042324[12] = -nom_x[12] + true_x[12];
   out_150977374342042324[13] = -nom_x[13] + true_x[13];
   out_150977374342042324[14] = -nom_x[14] + true_x[14];
   out_150977374342042324[15] = -nom_x[15] + true_x[15];
   out_150977374342042324[16] = -nom_x[16] + true_x[16];
   out_150977374342042324[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3607427627586762493) {
   out_3607427627586762493[0] = 1.0;
   out_3607427627586762493[1] = 0.0;
   out_3607427627586762493[2] = 0.0;
   out_3607427627586762493[3] = 0.0;
   out_3607427627586762493[4] = 0.0;
   out_3607427627586762493[5] = 0.0;
   out_3607427627586762493[6] = 0.0;
   out_3607427627586762493[7] = 0.0;
   out_3607427627586762493[8] = 0.0;
   out_3607427627586762493[9] = 0.0;
   out_3607427627586762493[10] = 0.0;
   out_3607427627586762493[11] = 0.0;
   out_3607427627586762493[12] = 0.0;
   out_3607427627586762493[13] = 0.0;
   out_3607427627586762493[14] = 0.0;
   out_3607427627586762493[15] = 0.0;
   out_3607427627586762493[16] = 0.0;
   out_3607427627586762493[17] = 0.0;
   out_3607427627586762493[18] = 0.0;
   out_3607427627586762493[19] = 1.0;
   out_3607427627586762493[20] = 0.0;
   out_3607427627586762493[21] = 0.0;
   out_3607427627586762493[22] = 0.0;
   out_3607427627586762493[23] = 0.0;
   out_3607427627586762493[24] = 0.0;
   out_3607427627586762493[25] = 0.0;
   out_3607427627586762493[26] = 0.0;
   out_3607427627586762493[27] = 0.0;
   out_3607427627586762493[28] = 0.0;
   out_3607427627586762493[29] = 0.0;
   out_3607427627586762493[30] = 0.0;
   out_3607427627586762493[31] = 0.0;
   out_3607427627586762493[32] = 0.0;
   out_3607427627586762493[33] = 0.0;
   out_3607427627586762493[34] = 0.0;
   out_3607427627586762493[35] = 0.0;
   out_3607427627586762493[36] = 0.0;
   out_3607427627586762493[37] = 0.0;
   out_3607427627586762493[38] = 1.0;
   out_3607427627586762493[39] = 0.0;
   out_3607427627586762493[40] = 0.0;
   out_3607427627586762493[41] = 0.0;
   out_3607427627586762493[42] = 0.0;
   out_3607427627586762493[43] = 0.0;
   out_3607427627586762493[44] = 0.0;
   out_3607427627586762493[45] = 0.0;
   out_3607427627586762493[46] = 0.0;
   out_3607427627586762493[47] = 0.0;
   out_3607427627586762493[48] = 0.0;
   out_3607427627586762493[49] = 0.0;
   out_3607427627586762493[50] = 0.0;
   out_3607427627586762493[51] = 0.0;
   out_3607427627586762493[52] = 0.0;
   out_3607427627586762493[53] = 0.0;
   out_3607427627586762493[54] = 0.0;
   out_3607427627586762493[55] = 0.0;
   out_3607427627586762493[56] = 0.0;
   out_3607427627586762493[57] = 1.0;
   out_3607427627586762493[58] = 0.0;
   out_3607427627586762493[59] = 0.0;
   out_3607427627586762493[60] = 0.0;
   out_3607427627586762493[61] = 0.0;
   out_3607427627586762493[62] = 0.0;
   out_3607427627586762493[63] = 0.0;
   out_3607427627586762493[64] = 0.0;
   out_3607427627586762493[65] = 0.0;
   out_3607427627586762493[66] = 0.0;
   out_3607427627586762493[67] = 0.0;
   out_3607427627586762493[68] = 0.0;
   out_3607427627586762493[69] = 0.0;
   out_3607427627586762493[70] = 0.0;
   out_3607427627586762493[71] = 0.0;
   out_3607427627586762493[72] = 0.0;
   out_3607427627586762493[73] = 0.0;
   out_3607427627586762493[74] = 0.0;
   out_3607427627586762493[75] = 0.0;
   out_3607427627586762493[76] = 1.0;
   out_3607427627586762493[77] = 0.0;
   out_3607427627586762493[78] = 0.0;
   out_3607427627586762493[79] = 0.0;
   out_3607427627586762493[80] = 0.0;
   out_3607427627586762493[81] = 0.0;
   out_3607427627586762493[82] = 0.0;
   out_3607427627586762493[83] = 0.0;
   out_3607427627586762493[84] = 0.0;
   out_3607427627586762493[85] = 0.0;
   out_3607427627586762493[86] = 0.0;
   out_3607427627586762493[87] = 0.0;
   out_3607427627586762493[88] = 0.0;
   out_3607427627586762493[89] = 0.0;
   out_3607427627586762493[90] = 0.0;
   out_3607427627586762493[91] = 0.0;
   out_3607427627586762493[92] = 0.0;
   out_3607427627586762493[93] = 0.0;
   out_3607427627586762493[94] = 0.0;
   out_3607427627586762493[95] = 1.0;
   out_3607427627586762493[96] = 0.0;
   out_3607427627586762493[97] = 0.0;
   out_3607427627586762493[98] = 0.0;
   out_3607427627586762493[99] = 0.0;
   out_3607427627586762493[100] = 0.0;
   out_3607427627586762493[101] = 0.0;
   out_3607427627586762493[102] = 0.0;
   out_3607427627586762493[103] = 0.0;
   out_3607427627586762493[104] = 0.0;
   out_3607427627586762493[105] = 0.0;
   out_3607427627586762493[106] = 0.0;
   out_3607427627586762493[107] = 0.0;
   out_3607427627586762493[108] = 0.0;
   out_3607427627586762493[109] = 0.0;
   out_3607427627586762493[110] = 0.0;
   out_3607427627586762493[111] = 0.0;
   out_3607427627586762493[112] = 0.0;
   out_3607427627586762493[113] = 0.0;
   out_3607427627586762493[114] = 1.0;
   out_3607427627586762493[115] = 0.0;
   out_3607427627586762493[116] = 0.0;
   out_3607427627586762493[117] = 0.0;
   out_3607427627586762493[118] = 0.0;
   out_3607427627586762493[119] = 0.0;
   out_3607427627586762493[120] = 0.0;
   out_3607427627586762493[121] = 0.0;
   out_3607427627586762493[122] = 0.0;
   out_3607427627586762493[123] = 0.0;
   out_3607427627586762493[124] = 0.0;
   out_3607427627586762493[125] = 0.0;
   out_3607427627586762493[126] = 0.0;
   out_3607427627586762493[127] = 0.0;
   out_3607427627586762493[128] = 0.0;
   out_3607427627586762493[129] = 0.0;
   out_3607427627586762493[130] = 0.0;
   out_3607427627586762493[131] = 0.0;
   out_3607427627586762493[132] = 0.0;
   out_3607427627586762493[133] = 1.0;
   out_3607427627586762493[134] = 0.0;
   out_3607427627586762493[135] = 0.0;
   out_3607427627586762493[136] = 0.0;
   out_3607427627586762493[137] = 0.0;
   out_3607427627586762493[138] = 0.0;
   out_3607427627586762493[139] = 0.0;
   out_3607427627586762493[140] = 0.0;
   out_3607427627586762493[141] = 0.0;
   out_3607427627586762493[142] = 0.0;
   out_3607427627586762493[143] = 0.0;
   out_3607427627586762493[144] = 0.0;
   out_3607427627586762493[145] = 0.0;
   out_3607427627586762493[146] = 0.0;
   out_3607427627586762493[147] = 0.0;
   out_3607427627586762493[148] = 0.0;
   out_3607427627586762493[149] = 0.0;
   out_3607427627586762493[150] = 0.0;
   out_3607427627586762493[151] = 0.0;
   out_3607427627586762493[152] = 1.0;
   out_3607427627586762493[153] = 0.0;
   out_3607427627586762493[154] = 0.0;
   out_3607427627586762493[155] = 0.0;
   out_3607427627586762493[156] = 0.0;
   out_3607427627586762493[157] = 0.0;
   out_3607427627586762493[158] = 0.0;
   out_3607427627586762493[159] = 0.0;
   out_3607427627586762493[160] = 0.0;
   out_3607427627586762493[161] = 0.0;
   out_3607427627586762493[162] = 0.0;
   out_3607427627586762493[163] = 0.0;
   out_3607427627586762493[164] = 0.0;
   out_3607427627586762493[165] = 0.0;
   out_3607427627586762493[166] = 0.0;
   out_3607427627586762493[167] = 0.0;
   out_3607427627586762493[168] = 0.0;
   out_3607427627586762493[169] = 0.0;
   out_3607427627586762493[170] = 0.0;
   out_3607427627586762493[171] = 1.0;
   out_3607427627586762493[172] = 0.0;
   out_3607427627586762493[173] = 0.0;
   out_3607427627586762493[174] = 0.0;
   out_3607427627586762493[175] = 0.0;
   out_3607427627586762493[176] = 0.0;
   out_3607427627586762493[177] = 0.0;
   out_3607427627586762493[178] = 0.0;
   out_3607427627586762493[179] = 0.0;
   out_3607427627586762493[180] = 0.0;
   out_3607427627586762493[181] = 0.0;
   out_3607427627586762493[182] = 0.0;
   out_3607427627586762493[183] = 0.0;
   out_3607427627586762493[184] = 0.0;
   out_3607427627586762493[185] = 0.0;
   out_3607427627586762493[186] = 0.0;
   out_3607427627586762493[187] = 0.0;
   out_3607427627586762493[188] = 0.0;
   out_3607427627586762493[189] = 0.0;
   out_3607427627586762493[190] = 1.0;
   out_3607427627586762493[191] = 0.0;
   out_3607427627586762493[192] = 0.0;
   out_3607427627586762493[193] = 0.0;
   out_3607427627586762493[194] = 0.0;
   out_3607427627586762493[195] = 0.0;
   out_3607427627586762493[196] = 0.0;
   out_3607427627586762493[197] = 0.0;
   out_3607427627586762493[198] = 0.0;
   out_3607427627586762493[199] = 0.0;
   out_3607427627586762493[200] = 0.0;
   out_3607427627586762493[201] = 0.0;
   out_3607427627586762493[202] = 0.0;
   out_3607427627586762493[203] = 0.0;
   out_3607427627586762493[204] = 0.0;
   out_3607427627586762493[205] = 0.0;
   out_3607427627586762493[206] = 0.0;
   out_3607427627586762493[207] = 0.0;
   out_3607427627586762493[208] = 0.0;
   out_3607427627586762493[209] = 1.0;
   out_3607427627586762493[210] = 0.0;
   out_3607427627586762493[211] = 0.0;
   out_3607427627586762493[212] = 0.0;
   out_3607427627586762493[213] = 0.0;
   out_3607427627586762493[214] = 0.0;
   out_3607427627586762493[215] = 0.0;
   out_3607427627586762493[216] = 0.0;
   out_3607427627586762493[217] = 0.0;
   out_3607427627586762493[218] = 0.0;
   out_3607427627586762493[219] = 0.0;
   out_3607427627586762493[220] = 0.0;
   out_3607427627586762493[221] = 0.0;
   out_3607427627586762493[222] = 0.0;
   out_3607427627586762493[223] = 0.0;
   out_3607427627586762493[224] = 0.0;
   out_3607427627586762493[225] = 0.0;
   out_3607427627586762493[226] = 0.0;
   out_3607427627586762493[227] = 0.0;
   out_3607427627586762493[228] = 1.0;
   out_3607427627586762493[229] = 0.0;
   out_3607427627586762493[230] = 0.0;
   out_3607427627586762493[231] = 0.0;
   out_3607427627586762493[232] = 0.0;
   out_3607427627586762493[233] = 0.0;
   out_3607427627586762493[234] = 0.0;
   out_3607427627586762493[235] = 0.0;
   out_3607427627586762493[236] = 0.0;
   out_3607427627586762493[237] = 0.0;
   out_3607427627586762493[238] = 0.0;
   out_3607427627586762493[239] = 0.0;
   out_3607427627586762493[240] = 0.0;
   out_3607427627586762493[241] = 0.0;
   out_3607427627586762493[242] = 0.0;
   out_3607427627586762493[243] = 0.0;
   out_3607427627586762493[244] = 0.0;
   out_3607427627586762493[245] = 0.0;
   out_3607427627586762493[246] = 0.0;
   out_3607427627586762493[247] = 1.0;
   out_3607427627586762493[248] = 0.0;
   out_3607427627586762493[249] = 0.0;
   out_3607427627586762493[250] = 0.0;
   out_3607427627586762493[251] = 0.0;
   out_3607427627586762493[252] = 0.0;
   out_3607427627586762493[253] = 0.0;
   out_3607427627586762493[254] = 0.0;
   out_3607427627586762493[255] = 0.0;
   out_3607427627586762493[256] = 0.0;
   out_3607427627586762493[257] = 0.0;
   out_3607427627586762493[258] = 0.0;
   out_3607427627586762493[259] = 0.0;
   out_3607427627586762493[260] = 0.0;
   out_3607427627586762493[261] = 0.0;
   out_3607427627586762493[262] = 0.0;
   out_3607427627586762493[263] = 0.0;
   out_3607427627586762493[264] = 0.0;
   out_3607427627586762493[265] = 0.0;
   out_3607427627586762493[266] = 1.0;
   out_3607427627586762493[267] = 0.0;
   out_3607427627586762493[268] = 0.0;
   out_3607427627586762493[269] = 0.0;
   out_3607427627586762493[270] = 0.0;
   out_3607427627586762493[271] = 0.0;
   out_3607427627586762493[272] = 0.0;
   out_3607427627586762493[273] = 0.0;
   out_3607427627586762493[274] = 0.0;
   out_3607427627586762493[275] = 0.0;
   out_3607427627586762493[276] = 0.0;
   out_3607427627586762493[277] = 0.0;
   out_3607427627586762493[278] = 0.0;
   out_3607427627586762493[279] = 0.0;
   out_3607427627586762493[280] = 0.0;
   out_3607427627586762493[281] = 0.0;
   out_3607427627586762493[282] = 0.0;
   out_3607427627586762493[283] = 0.0;
   out_3607427627586762493[284] = 0.0;
   out_3607427627586762493[285] = 1.0;
   out_3607427627586762493[286] = 0.0;
   out_3607427627586762493[287] = 0.0;
   out_3607427627586762493[288] = 0.0;
   out_3607427627586762493[289] = 0.0;
   out_3607427627586762493[290] = 0.0;
   out_3607427627586762493[291] = 0.0;
   out_3607427627586762493[292] = 0.0;
   out_3607427627586762493[293] = 0.0;
   out_3607427627586762493[294] = 0.0;
   out_3607427627586762493[295] = 0.0;
   out_3607427627586762493[296] = 0.0;
   out_3607427627586762493[297] = 0.0;
   out_3607427627586762493[298] = 0.0;
   out_3607427627586762493[299] = 0.0;
   out_3607427627586762493[300] = 0.0;
   out_3607427627586762493[301] = 0.0;
   out_3607427627586762493[302] = 0.0;
   out_3607427627586762493[303] = 0.0;
   out_3607427627586762493[304] = 1.0;
   out_3607427627586762493[305] = 0.0;
   out_3607427627586762493[306] = 0.0;
   out_3607427627586762493[307] = 0.0;
   out_3607427627586762493[308] = 0.0;
   out_3607427627586762493[309] = 0.0;
   out_3607427627586762493[310] = 0.0;
   out_3607427627586762493[311] = 0.0;
   out_3607427627586762493[312] = 0.0;
   out_3607427627586762493[313] = 0.0;
   out_3607427627586762493[314] = 0.0;
   out_3607427627586762493[315] = 0.0;
   out_3607427627586762493[316] = 0.0;
   out_3607427627586762493[317] = 0.0;
   out_3607427627586762493[318] = 0.0;
   out_3607427627586762493[319] = 0.0;
   out_3607427627586762493[320] = 0.0;
   out_3607427627586762493[321] = 0.0;
   out_3607427627586762493[322] = 0.0;
   out_3607427627586762493[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_961891625828540708) {
   out_961891625828540708[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_961891625828540708[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_961891625828540708[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_961891625828540708[3] = dt*state[12] + state[3];
   out_961891625828540708[4] = dt*state[13] + state[4];
   out_961891625828540708[5] = dt*state[14] + state[5];
   out_961891625828540708[6] = state[6];
   out_961891625828540708[7] = state[7];
   out_961891625828540708[8] = state[8];
   out_961891625828540708[9] = state[9];
   out_961891625828540708[10] = state[10];
   out_961891625828540708[11] = state[11];
   out_961891625828540708[12] = state[12];
   out_961891625828540708[13] = state[13];
   out_961891625828540708[14] = state[14];
   out_961891625828540708[15] = state[15];
   out_961891625828540708[16] = state[16];
   out_961891625828540708[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3085087175776285280) {
   out_3085087175776285280[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3085087175776285280[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3085087175776285280[2] = 0;
   out_3085087175776285280[3] = 0;
   out_3085087175776285280[4] = 0;
   out_3085087175776285280[5] = 0;
   out_3085087175776285280[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3085087175776285280[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3085087175776285280[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3085087175776285280[9] = 0;
   out_3085087175776285280[10] = 0;
   out_3085087175776285280[11] = 0;
   out_3085087175776285280[12] = 0;
   out_3085087175776285280[13] = 0;
   out_3085087175776285280[14] = 0;
   out_3085087175776285280[15] = 0;
   out_3085087175776285280[16] = 0;
   out_3085087175776285280[17] = 0;
   out_3085087175776285280[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3085087175776285280[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3085087175776285280[20] = 0;
   out_3085087175776285280[21] = 0;
   out_3085087175776285280[22] = 0;
   out_3085087175776285280[23] = 0;
   out_3085087175776285280[24] = 0;
   out_3085087175776285280[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3085087175776285280[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3085087175776285280[27] = 0;
   out_3085087175776285280[28] = 0;
   out_3085087175776285280[29] = 0;
   out_3085087175776285280[30] = 0;
   out_3085087175776285280[31] = 0;
   out_3085087175776285280[32] = 0;
   out_3085087175776285280[33] = 0;
   out_3085087175776285280[34] = 0;
   out_3085087175776285280[35] = 0;
   out_3085087175776285280[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3085087175776285280[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3085087175776285280[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3085087175776285280[39] = 0;
   out_3085087175776285280[40] = 0;
   out_3085087175776285280[41] = 0;
   out_3085087175776285280[42] = 0;
   out_3085087175776285280[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3085087175776285280[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3085087175776285280[45] = 0;
   out_3085087175776285280[46] = 0;
   out_3085087175776285280[47] = 0;
   out_3085087175776285280[48] = 0;
   out_3085087175776285280[49] = 0;
   out_3085087175776285280[50] = 0;
   out_3085087175776285280[51] = 0;
   out_3085087175776285280[52] = 0;
   out_3085087175776285280[53] = 0;
   out_3085087175776285280[54] = 0;
   out_3085087175776285280[55] = 0;
   out_3085087175776285280[56] = 0;
   out_3085087175776285280[57] = 1;
   out_3085087175776285280[58] = 0;
   out_3085087175776285280[59] = 0;
   out_3085087175776285280[60] = 0;
   out_3085087175776285280[61] = 0;
   out_3085087175776285280[62] = 0;
   out_3085087175776285280[63] = 0;
   out_3085087175776285280[64] = 0;
   out_3085087175776285280[65] = 0;
   out_3085087175776285280[66] = dt;
   out_3085087175776285280[67] = 0;
   out_3085087175776285280[68] = 0;
   out_3085087175776285280[69] = 0;
   out_3085087175776285280[70] = 0;
   out_3085087175776285280[71] = 0;
   out_3085087175776285280[72] = 0;
   out_3085087175776285280[73] = 0;
   out_3085087175776285280[74] = 0;
   out_3085087175776285280[75] = 0;
   out_3085087175776285280[76] = 1;
   out_3085087175776285280[77] = 0;
   out_3085087175776285280[78] = 0;
   out_3085087175776285280[79] = 0;
   out_3085087175776285280[80] = 0;
   out_3085087175776285280[81] = 0;
   out_3085087175776285280[82] = 0;
   out_3085087175776285280[83] = 0;
   out_3085087175776285280[84] = 0;
   out_3085087175776285280[85] = dt;
   out_3085087175776285280[86] = 0;
   out_3085087175776285280[87] = 0;
   out_3085087175776285280[88] = 0;
   out_3085087175776285280[89] = 0;
   out_3085087175776285280[90] = 0;
   out_3085087175776285280[91] = 0;
   out_3085087175776285280[92] = 0;
   out_3085087175776285280[93] = 0;
   out_3085087175776285280[94] = 0;
   out_3085087175776285280[95] = 1;
   out_3085087175776285280[96] = 0;
   out_3085087175776285280[97] = 0;
   out_3085087175776285280[98] = 0;
   out_3085087175776285280[99] = 0;
   out_3085087175776285280[100] = 0;
   out_3085087175776285280[101] = 0;
   out_3085087175776285280[102] = 0;
   out_3085087175776285280[103] = 0;
   out_3085087175776285280[104] = dt;
   out_3085087175776285280[105] = 0;
   out_3085087175776285280[106] = 0;
   out_3085087175776285280[107] = 0;
   out_3085087175776285280[108] = 0;
   out_3085087175776285280[109] = 0;
   out_3085087175776285280[110] = 0;
   out_3085087175776285280[111] = 0;
   out_3085087175776285280[112] = 0;
   out_3085087175776285280[113] = 0;
   out_3085087175776285280[114] = 1;
   out_3085087175776285280[115] = 0;
   out_3085087175776285280[116] = 0;
   out_3085087175776285280[117] = 0;
   out_3085087175776285280[118] = 0;
   out_3085087175776285280[119] = 0;
   out_3085087175776285280[120] = 0;
   out_3085087175776285280[121] = 0;
   out_3085087175776285280[122] = 0;
   out_3085087175776285280[123] = 0;
   out_3085087175776285280[124] = 0;
   out_3085087175776285280[125] = 0;
   out_3085087175776285280[126] = 0;
   out_3085087175776285280[127] = 0;
   out_3085087175776285280[128] = 0;
   out_3085087175776285280[129] = 0;
   out_3085087175776285280[130] = 0;
   out_3085087175776285280[131] = 0;
   out_3085087175776285280[132] = 0;
   out_3085087175776285280[133] = 1;
   out_3085087175776285280[134] = 0;
   out_3085087175776285280[135] = 0;
   out_3085087175776285280[136] = 0;
   out_3085087175776285280[137] = 0;
   out_3085087175776285280[138] = 0;
   out_3085087175776285280[139] = 0;
   out_3085087175776285280[140] = 0;
   out_3085087175776285280[141] = 0;
   out_3085087175776285280[142] = 0;
   out_3085087175776285280[143] = 0;
   out_3085087175776285280[144] = 0;
   out_3085087175776285280[145] = 0;
   out_3085087175776285280[146] = 0;
   out_3085087175776285280[147] = 0;
   out_3085087175776285280[148] = 0;
   out_3085087175776285280[149] = 0;
   out_3085087175776285280[150] = 0;
   out_3085087175776285280[151] = 0;
   out_3085087175776285280[152] = 1;
   out_3085087175776285280[153] = 0;
   out_3085087175776285280[154] = 0;
   out_3085087175776285280[155] = 0;
   out_3085087175776285280[156] = 0;
   out_3085087175776285280[157] = 0;
   out_3085087175776285280[158] = 0;
   out_3085087175776285280[159] = 0;
   out_3085087175776285280[160] = 0;
   out_3085087175776285280[161] = 0;
   out_3085087175776285280[162] = 0;
   out_3085087175776285280[163] = 0;
   out_3085087175776285280[164] = 0;
   out_3085087175776285280[165] = 0;
   out_3085087175776285280[166] = 0;
   out_3085087175776285280[167] = 0;
   out_3085087175776285280[168] = 0;
   out_3085087175776285280[169] = 0;
   out_3085087175776285280[170] = 0;
   out_3085087175776285280[171] = 1;
   out_3085087175776285280[172] = 0;
   out_3085087175776285280[173] = 0;
   out_3085087175776285280[174] = 0;
   out_3085087175776285280[175] = 0;
   out_3085087175776285280[176] = 0;
   out_3085087175776285280[177] = 0;
   out_3085087175776285280[178] = 0;
   out_3085087175776285280[179] = 0;
   out_3085087175776285280[180] = 0;
   out_3085087175776285280[181] = 0;
   out_3085087175776285280[182] = 0;
   out_3085087175776285280[183] = 0;
   out_3085087175776285280[184] = 0;
   out_3085087175776285280[185] = 0;
   out_3085087175776285280[186] = 0;
   out_3085087175776285280[187] = 0;
   out_3085087175776285280[188] = 0;
   out_3085087175776285280[189] = 0;
   out_3085087175776285280[190] = 1;
   out_3085087175776285280[191] = 0;
   out_3085087175776285280[192] = 0;
   out_3085087175776285280[193] = 0;
   out_3085087175776285280[194] = 0;
   out_3085087175776285280[195] = 0;
   out_3085087175776285280[196] = 0;
   out_3085087175776285280[197] = 0;
   out_3085087175776285280[198] = 0;
   out_3085087175776285280[199] = 0;
   out_3085087175776285280[200] = 0;
   out_3085087175776285280[201] = 0;
   out_3085087175776285280[202] = 0;
   out_3085087175776285280[203] = 0;
   out_3085087175776285280[204] = 0;
   out_3085087175776285280[205] = 0;
   out_3085087175776285280[206] = 0;
   out_3085087175776285280[207] = 0;
   out_3085087175776285280[208] = 0;
   out_3085087175776285280[209] = 1;
   out_3085087175776285280[210] = 0;
   out_3085087175776285280[211] = 0;
   out_3085087175776285280[212] = 0;
   out_3085087175776285280[213] = 0;
   out_3085087175776285280[214] = 0;
   out_3085087175776285280[215] = 0;
   out_3085087175776285280[216] = 0;
   out_3085087175776285280[217] = 0;
   out_3085087175776285280[218] = 0;
   out_3085087175776285280[219] = 0;
   out_3085087175776285280[220] = 0;
   out_3085087175776285280[221] = 0;
   out_3085087175776285280[222] = 0;
   out_3085087175776285280[223] = 0;
   out_3085087175776285280[224] = 0;
   out_3085087175776285280[225] = 0;
   out_3085087175776285280[226] = 0;
   out_3085087175776285280[227] = 0;
   out_3085087175776285280[228] = 1;
   out_3085087175776285280[229] = 0;
   out_3085087175776285280[230] = 0;
   out_3085087175776285280[231] = 0;
   out_3085087175776285280[232] = 0;
   out_3085087175776285280[233] = 0;
   out_3085087175776285280[234] = 0;
   out_3085087175776285280[235] = 0;
   out_3085087175776285280[236] = 0;
   out_3085087175776285280[237] = 0;
   out_3085087175776285280[238] = 0;
   out_3085087175776285280[239] = 0;
   out_3085087175776285280[240] = 0;
   out_3085087175776285280[241] = 0;
   out_3085087175776285280[242] = 0;
   out_3085087175776285280[243] = 0;
   out_3085087175776285280[244] = 0;
   out_3085087175776285280[245] = 0;
   out_3085087175776285280[246] = 0;
   out_3085087175776285280[247] = 1;
   out_3085087175776285280[248] = 0;
   out_3085087175776285280[249] = 0;
   out_3085087175776285280[250] = 0;
   out_3085087175776285280[251] = 0;
   out_3085087175776285280[252] = 0;
   out_3085087175776285280[253] = 0;
   out_3085087175776285280[254] = 0;
   out_3085087175776285280[255] = 0;
   out_3085087175776285280[256] = 0;
   out_3085087175776285280[257] = 0;
   out_3085087175776285280[258] = 0;
   out_3085087175776285280[259] = 0;
   out_3085087175776285280[260] = 0;
   out_3085087175776285280[261] = 0;
   out_3085087175776285280[262] = 0;
   out_3085087175776285280[263] = 0;
   out_3085087175776285280[264] = 0;
   out_3085087175776285280[265] = 0;
   out_3085087175776285280[266] = 1;
   out_3085087175776285280[267] = 0;
   out_3085087175776285280[268] = 0;
   out_3085087175776285280[269] = 0;
   out_3085087175776285280[270] = 0;
   out_3085087175776285280[271] = 0;
   out_3085087175776285280[272] = 0;
   out_3085087175776285280[273] = 0;
   out_3085087175776285280[274] = 0;
   out_3085087175776285280[275] = 0;
   out_3085087175776285280[276] = 0;
   out_3085087175776285280[277] = 0;
   out_3085087175776285280[278] = 0;
   out_3085087175776285280[279] = 0;
   out_3085087175776285280[280] = 0;
   out_3085087175776285280[281] = 0;
   out_3085087175776285280[282] = 0;
   out_3085087175776285280[283] = 0;
   out_3085087175776285280[284] = 0;
   out_3085087175776285280[285] = 1;
   out_3085087175776285280[286] = 0;
   out_3085087175776285280[287] = 0;
   out_3085087175776285280[288] = 0;
   out_3085087175776285280[289] = 0;
   out_3085087175776285280[290] = 0;
   out_3085087175776285280[291] = 0;
   out_3085087175776285280[292] = 0;
   out_3085087175776285280[293] = 0;
   out_3085087175776285280[294] = 0;
   out_3085087175776285280[295] = 0;
   out_3085087175776285280[296] = 0;
   out_3085087175776285280[297] = 0;
   out_3085087175776285280[298] = 0;
   out_3085087175776285280[299] = 0;
   out_3085087175776285280[300] = 0;
   out_3085087175776285280[301] = 0;
   out_3085087175776285280[302] = 0;
   out_3085087175776285280[303] = 0;
   out_3085087175776285280[304] = 1;
   out_3085087175776285280[305] = 0;
   out_3085087175776285280[306] = 0;
   out_3085087175776285280[307] = 0;
   out_3085087175776285280[308] = 0;
   out_3085087175776285280[309] = 0;
   out_3085087175776285280[310] = 0;
   out_3085087175776285280[311] = 0;
   out_3085087175776285280[312] = 0;
   out_3085087175776285280[313] = 0;
   out_3085087175776285280[314] = 0;
   out_3085087175776285280[315] = 0;
   out_3085087175776285280[316] = 0;
   out_3085087175776285280[317] = 0;
   out_3085087175776285280[318] = 0;
   out_3085087175776285280[319] = 0;
   out_3085087175776285280[320] = 0;
   out_3085087175776285280[321] = 0;
   out_3085087175776285280[322] = 0;
   out_3085087175776285280[323] = 1;
}
void h_4(double *state, double *unused, double *out_2716213592127562262) {
   out_2716213592127562262[0] = state[6] + state[9];
   out_2716213592127562262[1] = state[7] + state[10];
   out_2716213592127562262[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_1545090757452318242) {
   out_1545090757452318242[0] = 0;
   out_1545090757452318242[1] = 0;
   out_1545090757452318242[2] = 0;
   out_1545090757452318242[3] = 0;
   out_1545090757452318242[4] = 0;
   out_1545090757452318242[5] = 0;
   out_1545090757452318242[6] = 1;
   out_1545090757452318242[7] = 0;
   out_1545090757452318242[8] = 0;
   out_1545090757452318242[9] = 1;
   out_1545090757452318242[10] = 0;
   out_1545090757452318242[11] = 0;
   out_1545090757452318242[12] = 0;
   out_1545090757452318242[13] = 0;
   out_1545090757452318242[14] = 0;
   out_1545090757452318242[15] = 0;
   out_1545090757452318242[16] = 0;
   out_1545090757452318242[17] = 0;
   out_1545090757452318242[18] = 0;
   out_1545090757452318242[19] = 0;
   out_1545090757452318242[20] = 0;
   out_1545090757452318242[21] = 0;
   out_1545090757452318242[22] = 0;
   out_1545090757452318242[23] = 0;
   out_1545090757452318242[24] = 0;
   out_1545090757452318242[25] = 1;
   out_1545090757452318242[26] = 0;
   out_1545090757452318242[27] = 0;
   out_1545090757452318242[28] = 1;
   out_1545090757452318242[29] = 0;
   out_1545090757452318242[30] = 0;
   out_1545090757452318242[31] = 0;
   out_1545090757452318242[32] = 0;
   out_1545090757452318242[33] = 0;
   out_1545090757452318242[34] = 0;
   out_1545090757452318242[35] = 0;
   out_1545090757452318242[36] = 0;
   out_1545090757452318242[37] = 0;
   out_1545090757452318242[38] = 0;
   out_1545090757452318242[39] = 0;
   out_1545090757452318242[40] = 0;
   out_1545090757452318242[41] = 0;
   out_1545090757452318242[42] = 0;
   out_1545090757452318242[43] = 0;
   out_1545090757452318242[44] = 1;
   out_1545090757452318242[45] = 0;
   out_1545090757452318242[46] = 0;
   out_1545090757452318242[47] = 1;
   out_1545090757452318242[48] = 0;
   out_1545090757452318242[49] = 0;
   out_1545090757452318242[50] = 0;
   out_1545090757452318242[51] = 0;
   out_1545090757452318242[52] = 0;
   out_1545090757452318242[53] = 0;
}
void h_10(double *state, double *unused, double *out_4417817159878647707) {
   out_4417817159878647707[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4417817159878647707[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4417817159878647707[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6890704536492367764) {
   out_6890704536492367764[0] = 0;
   out_6890704536492367764[1] = 9.8100000000000005*cos(state[1]);
   out_6890704536492367764[2] = 0;
   out_6890704536492367764[3] = 0;
   out_6890704536492367764[4] = -state[8];
   out_6890704536492367764[5] = state[7];
   out_6890704536492367764[6] = 0;
   out_6890704536492367764[7] = state[5];
   out_6890704536492367764[8] = -state[4];
   out_6890704536492367764[9] = 0;
   out_6890704536492367764[10] = 0;
   out_6890704536492367764[11] = 0;
   out_6890704536492367764[12] = 1;
   out_6890704536492367764[13] = 0;
   out_6890704536492367764[14] = 0;
   out_6890704536492367764[15] = 1;
   out_6890704536492367764[16] = 0;
   out_6890704536492367764[17] = 0;
   out_6890704536492367764[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6890704536492367764[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6890704536492367764[20] = 0;
   out_6890704536492367764[21] = state[8];
   out_6890704536492367764[22] = 0;
   out_6890704536492367764[23] = -state[6];
   out_6890704536492367764[24] = -state[5];
   out_6890704536492367764[25] = 0;
   out_6890704536492367764[26] = state[3];
   out_6890704536492367764[27] = 0;
   out_6890704536492367764[28] = 0;
   out_6890704536492367764[29] = 0;
   out_6890704536492367764[30] = 0;
   out_6890704536492367764[31] = 1;
   out_6890704536492367764[32] = 0;
   out_6890704536492367764[33] = 0;
   out_6890704536492367764[34] = 1;
   out_6890704536492367764[35] = 0;
   out_6890704536492367764[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6890704536492367764[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6890704536492367764[38] = 0;
   out_6890704536492367764[39] = -state[7];
   out_6890704536492367764[40] = state[6];
   out_6890704536492367764[41] = 0;
   out_6890704536492367764[42] = state[4];
   out_6890704536492367764[43] = -state[3];
   out_6890704536492367764[44] = 0;
   out_6890704536492367764[45] = 0;
   out_6890704536492367764[46] = 0;
   out_6890704536492367764[47] = 0;
   out_6890704536492367764[48] = 0;
   out_6890704536492367764[49] = 0;
   out_6890704536492367764[50] = 1;
   out_6890704536492367764[51] = 0;
   out_6890704536492367764[52] = 0;
   out_6890704536492367764[53] = 1;
}
void h_13(double *state, double *unused, double *out_8627506231444867801) {
   out_8627506231444867801[0] = state[3];
   out_8627506231444867801[1] = state[4];
   out_8627506231444867801[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4757364582784651043) {
   out_4757364582784651043[0] = 0;
   out_4757364582784651043[1] = 0;
   out_4757364582784651043[2] = 0;
   out_4757364582784651043[3] = 1;
   out_4757364582784651043[4] = 0;
   out_4757364582784651043[5] = 0;
   out_4757364582784651043[6] = 0;
   out_4757364582784651043[7] = 0;
   out_4757364582784651043[8] = 0;
   out_4757364582784651043[9] = 0;
   out_4757364582784651043[10] = 0;
   out_4757364582784651043[11] = 0;
   out_4757364582784651043[12] = 0;
   out_4757364582784651043[13] = 0;
   out_4757364582784651043[14] = 0;
   out_4757364582784651043[15] = 0;
   out_4757364582784651043[16] = 0;
   out_4757364582784651043[17] = 0;
   out_4757364582784651043[18] = 0;
   out_4757364582784651043[19] = 0;
   out_4757364582784651043[20] = 0;
   out_4757364582784651043[21] = 0;
   out_4757364582784651043[22] = 1;
   out_4757364582784651043[23] = 0;
   out_4757364582784651043[24] = 0;
   out_4757364582784651043[25] = 0;
   out_4757364582784651043[26] = 0;
   out_4757364582784651043[27] = 0;
   out_4757364582784651043[28] = 0;
   out_4757364582784651043[29] = 0;
   out_4757364582784651043[30] = 0;
   out_4757364582784651043[31] = 0;
   out_4757364582784651043[32] = 0;
   out_4757364582784651043[33] = 0;
   out_4757364582784651043[34] = 0;
   out_4757364582784651043[35] = 0;
   out_4757364582784651043[36] = 0;
   out_4757364582784651043[37] = 0;
   out_4757364582784651043[38] = 0;
   out_4757364582784651043[39] = 0;
   out_4757364582784651043[40] = 0;
   out_4757364582784651043[41] = 1;
   out_4757364582784651043[42] = 0;
   out_4757364582784651043[43] = 0;
   out_4757364582784651043[44] = 0;
   out_4757364582784651043[45] = 0;
   out_4757364582784651043[46] = 0;
   out_4757364582784651043[47] = 0;
   out_4757364582784651043[48] = 0;
   out_4757364582784651043[49] = 0;
   out_4757364582784651043[50] = 0;
   out_4757364582784651043[51] = 0;
   out_4757364582784651043[52] = 0;
   out_4757364582784651043[53] = 0;
}
void h_14(double *state, double *unused, double *out_44121701262927063) {
   out_44121701262927063[0] = state[6];
   out_44121701262927063[1] = state[7];
   out_44121701262927063[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5508331613791802771) {
   out_5508331613791802771[0] = 0;
   out_5508331613791802771[1] = 0;
   out_5508331613791802771[2] = 0;
   out_5508331613791802771[3] = 0;
   out_5508331613791802771[4] = 0;
   out_5508331613791802771[5] = 0;
   out_5508331613791802771[6] = 1;
   out_5508331613791802771[7] = 0;
   out_5508331613791802771[8] = 0;
   out_5508331613791802771[9] = 0;
   out_5508331613791802771[10] = 0;
   out_5508331613791802771[11] = 0;
   out_5508331613791802771[12] = 0;
   out_5508331613791802771[13] = 0;
   out_5508331613791802771[14] = 0;
   out_5508331613791802771[15] = 0;
   out_5508331613791802771[16] = 0;
   out_5508331613791802771[17] = 0;
   out_5508331613791802771[18] = 0;
   out_5508331613791802771[19] = 0;
   out_5508331613791802771[20] = 0;
   out_5508331613791802771[21] = 0;
   out_5508331613791802771[22] = 0;
   out_5508331613791802771[23] = 0;
   out_5508331613791802771[24] = 0;
   out_5508331613791802771[25] = 1;
   out_5508331613791802771[26] = 0;
   out_5508331613791802771[27] = 0;
   out_5508331613791802771[28] = 0;
   out_5508331613791802771[29] = 0;
   out_5508331613791802771[30] = 0;
   out_5508331613791802771[31] = 0;
   out_5508331613791802771[32] = 0;
   out_5508331613791802771[33] = 0;
   out_5508331613791802771[34] = 0;
   out_5508331613791802771[35] = 0;
   out_5508331613791802771[36] = 0;
   out_5508331613791802771[37] = 0;
   out_5508331613791802771[38] = 0;
   out_5508331613791802771[39] = 0;
   out_5508331613791802771[40] = 0;
   out_5508331613791802771[41] = 0;
   out_5508331613791802771[42] = 0;
   out_5508331613791802771[43] = 0;
   out_5508331613791802771[44] = 1;
   out_5508331613791802771[45] = 0;
   out_5508331613791802771[46] = 0;
   out_5508331613791802771[47] = 0;
   out_5508331613791802771[48] = 0;
   out_5508331613791802771[49] = 0;
   out_5508331613791802771[50] = 0;
   out_5508331613791802771[51] = 0;
   out_5508331613791802771[52] = 0;
   out_5508331613791802771[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_2258393448586014737) {
  err_fun(nom_x, delta_x, out_2258393448586014737);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_150977374342042324) {
  inv_err_fun(nom_x, true_x, out_150977374342042324);
}
void pose_H_mod_fun(double *state, double *out_3607427627586762493) {
  H_mod_fun(state, out_3607427627586762493);
}
void pose_f_fun(double *state, double dt, double *out_961891625828540708) {
  f_fun(state,  dt, out_961891625828540708);
}
void pose_F_fun(double *state, double dt, double *out_3085087175776285280) {
  F_fun(state,  dt, out_3085087175776285280);
}
void pose_h_4(double *state, double *unused, double *out_2716213592127562262) {
  h_4(state, unused, out_2716213592127562262);
}
void pose_H_4(double *state, double *unused, double *out_1545090757452318242) {
  H_4(state, unused, out_1545090757452318242);
}
void pose_h_10(double *state, double *unused, double *out_4417817159878647707) {
  h_10(state, unused, out_4417817159878647707);
}
void pose_H_10(double *state, double *unused, double *out_6890704536492367764) {
  H_10(state, unused, out_6890704536492367764);
}
void pose_h_13(double *state, double *unused, double *out_8627506231444867801) {
  h_13(state, unused, out_8627506231444867801);
}
void pose_H_13(double *state, double *unused, double *out_4757364582784651043) {
  H_13(state, unused, out_4757364582784651043);
}
void pose_h_14(double *state, double *unused, double *out_44121701262927063) {
  h_14(state, unused, out_44121701262927063);
}
void pose_H_14(double *state, double *unused, double *out_5508331613791802771) {
  H_14(state, unused, out_5508331613791802771);
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
