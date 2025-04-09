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
void err_fun(double *nom_x, double *delta_x, double *out_3068681635687019270) {
   out_3068681635687019270[0] = delta_x[0] + nom_x[0];
   out_3068681635687019270[1] = delta_x[1] + nom_x[1];
   out_3068681635687019270[2] = delta_x[2] + nom_x[2];
   out_3068681635687019270[3] = delta_x[3] + nom_x[3];
   out_3068681635687019270[4] = delta_x[4] + nom_x[4];
   out_3068681635687019270[5] = delta_x[5] + nom_x[5];
   out_3068681635687019270[6] = delta_x[6] + nom_x[6];
   out_3068681635687019270[7] = delta_x[7] + nom_x[7];
   out_3068681635687019270[8] = delta_x[8] + nom_x[8];
   out_3068681635687019270[9] = delta_x[9] + nom_x[9];
   out_3068681635687019270[10] = delta_x[10] + nom_x[10];
   out_3068681635687019270[11] = delta_x[11] + nom_x[11];
   out_3068681635687019270[12] = delta_x[12] + nom_x[12];
   out_3068681635687019270[13] = delta_x[13] + nom_x[13];
   out_3068681635687019270[14] = delta_x[14] + nom_x[14];
   out_3068681635687019270[15] = delta_x[15] + nom_x[15];
   out_3068681635687019270[16] = delta_x[16] + nom_x[16];
   out_3068681635687019270[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6108564159138725733) {
   out_6108564159138725733[0] = -nom_x[0] + true_x[0];
   out_6108564159138725733[1] = -nom_x[1] + true_x[1];
   out_6108564159138725733[2] = -nom_x[2] + true_x[2];
   out_6108564159138725733[3] = -nom_x[3] + true_x[3];
   out_6108564159138725733[4] = -nom_x[4] + true_x[4];
   out_6108564159138725733[5] = -nom_x[5] + true_x[5];
   out_6108564159138725733[6] = -nom_x[6] + true_x[6];
   out_6108564159138725733[7] = -nom_x[7] + true_x[7];
   out_6108564159138725733[8] = -nom_x[8] + true_x[8];
   out_6108564159138725733[9] = -nom_x[9] + true_x[9];
   out_6108564159138725733[10] = -nom_x[10] + true_x[10];
   out_6108564159138725733[11] = -nom_x[11] + true_x[11];
   out_6108564159138725733[12] = -nom_x[12] + true_x[12];
   out_6108564159138725733[13] = -nom_x[13] + true_x[13];
   out_6108564159138725733[14] = -nom_x[14] + true_x[14];
   out_6108564159138725733[15] = -nom_x[15] + true_x[15];
   out_6108564159138725733[16] = -nom_x[16] + true_x[16];
   out_6108564159138725733[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8310243639589261475) {
   out_8310243639589261475[0] = 1.0;
   out_8310243639589261475[1] = 0.0;
   out_8310243639589261475[2] = 0.0;
   out_8310243639589261475[3] = 0.0;
   out_8310243639589261475[4] = 0.0;
   out_8310243639589261475[5] = 0.0;
   out_8310243639589261475[6] = 0.0;
   out_8310243639589261475[7] = 0.0;
   out_8310243639589261475[8] = 0.0;
   out_8310243639589261475[9] = 0.0;
   out_8310243639589261475[10] = 0.0;
   out_8310243639589261475[11] = 0.0;
   out_8310243639589261475[12] = 0.0;
   out_8310243639589261475[13] = 0.0;
   out_8310243639589261475[14] = 0.0;
   out_8310243639589261475[15] = 0.0;
   out_8310243639589261475[16] = 0.0;
   out_8310243639589261475[17] = 0.0;
   out_8310243639589261475[18] = 0.0;
   out_8310243639589261475[19] = 1.0;
   out_8310243639589261475[20] = 0.0;
   out_8310243639589261475[21] = 0.0;
   out_8310243639589261475[22] = 0.0;
   out_8310243639589261475[23] = 0.0;
   out_8310243639589261475[24] = 0.0;
   out_8310243639589261475[25] = 0.0;
   out_8310243639589261475[26] = 0.0;
   out_8310243639589261475[27] = 0.0;
   out_8310243639589261475[28] = 0.0;
   out_8310243639589261475[29] = 0.0;
   out_8310243639589261475[30] = 0.0;
   out_8310243639589261475[31] = 0.0;
   out_8310243639589261475[32] = 0.0;
   out_8310243639589261475[33] = 0.0;
   out_8310243639589261475[34] = 0.0;
   out_8310243639589261475[35] = 0.0;
   out_8310243639589261475[36] = 0.0;
   out_8310243639589261475[37] = 0.0;
   out_8310243639589261475[38] = 1.0;
   out_8310243639589261475[39] = 0.0;
   out_8310243639589261475[40] = 0.0;
   out_8310243639589261475[41] = 0.0;
   out_8310243639589261475[42] = 0.0;
   out_8310243639589261475[43] = 0.0;
   out_8310243639589261475[44] = 0.0;
   out_8310243639589261475[45] = 0.0;
   out_8310243639589261475[46] = 0.0;
   out_8310243639589261475[47] = 0.0;
   out_8310243639589261475[48] = 0.0;
   out_8310243639589261475[49] = 0.0;
   out_8310243639589261475[50] = 0.0;
   out_8310243639589261475[51] = 0.0;
   out_8310243639589261475[52] = 0.0;
   out_8310243639589261475[53] = 0.0;
   out_8310243639589261475[54] = 0.0;
   out_8310243639589261475[55] = 0.0;
   out_8310243639589261475[56] = 0.0;
   out_8310243639589261475[57] = 1.0;
   out_8310243639589261475[58] = 0.0;
   out_8310243639589261475[59] = 0.0;
   out_8310243639589261475[60] = 0.0;
   out_8310243639589261475[61] = 0.0;
   out_8310243639589261475[62] = 0.0;
   out_8310243639589261475[63] = 0.0;
   out_8310243639589261475[64] = 0.0;
   out_8310243639589261475[65] = 0.0;
   out_8310243639589261475[66] = 0.0;
   out_8310243639589261475[67] = 0.0;
   out_8310243639589261475[68] = 0.0;
   out_8310243639589261475[69] = 0.0;
   out_8310243639589261475[70] = 0.0;
   out_8310243639589261475[71] = 0.0;
   out_8310243639589261475[72] = 0.0;
   out_8310243639589261475[73] = 0.0;
   out_8310243639589261475[74] = 0.0;
   out_8310243639589261475[75] = 0.0;
   out_8310243639589261475[76] = 1.0;
   out_8310243639589261475[77] = 0.0;
   out_8310243639589261475[78] = 0.0;
   out_8310243639589261475[79] = 0.0;
   out_8310243639589261475[80] = 0.0;
   out_8310243639589261475[81] = 0.0;
   out_8310243639589261475[82] = 0.0;
   out_8310243639589261475[83] = 0.0;
   out_8310243639589261475[84] = 0.0;
   out_8310243639589261475[85] = 0.0;
   out_8310243639589261475[86] = 0.0;
   out_8310243639589261475[87] = 0.0;
   out_8310243639589261475[88] = 0.0;
   out_8310243639589261475[89] = 0.0;
   out_8310243639589261475[90] = 0.0;
   out_8310243639589261475[91] = 0.0;
   out_8310243639589261475[92] = 0.0;
   out_8310243639589261475[93] = 0.0;
   out_8310243639589261475[94] = 0.0;
   out_8310243639589261475[95] = 1.0;
   out_8310243639589261475[96] = 0.0;
   out_8310243639589261475[97] = 0.0;
   out_8310243639589261475[98] = 0.0;
   out_8310243639589261475[99] = 0.0;
   out_8310243639589261475[100] = 0.0;
   out_8310243639589261475[101] = 0.0;
   out_8310243639589261475[102] = 0.0;
   out_8310243639589261475[103] = 0.0;
   out_8310243639589261475[104] = 0.0;
   out_8310243639589261475[105] = 0.0;
   out_8310243639589261475[106] = 0.0;
   out_8310243639589261475[107] = 0.0;
   out_8310243639589261475[108] = 0.0;
   out_8310243639589261475[109] = 0.0;
   out_8310243639589261475[110] = 0.0;
   out_8310243639589261475[111] = 0.0;
   out_8310243639589261475[112] = 0.0;
   out_8310243639589261475[113] = 0.0;
   out_8310243639589261475[114] = 1.0;
   out_8310243639589261475[115] = 0.0;
   out_8310243639589261475[116] = 0.0;
   out_8310243639589261475[117] = 0.0;
   out_8310243639589261475[118] = 0.0;
   out_8310243639589261475[119] = 0.0;
   out_8310243639589261475[120] = 0.0;
   out_8310243639589261475[121] = 0.0;
   out_8310243639589261475[122] = 0.0;
   out_8310243639589261475[123] = 0.0;
   out_8310243639589261475[124] = 0.0;
   out_8310243639589261475[125] = 0.0;
   out_8310243639589261475[126] = 0.0;
   out_8310243639589261475[127] = 0.0;
   out_8310243639589261475[128] = 0.0;
   out_8310243639589261475[129] = 0.0;
   out_8310243639589261475[130] = 0.0;
   out_8310243639589261475[131] = 0.0;
   out_8310243639589261475[132] = 0.0;
   out_8310243639589261475[133] = 1.0;
   out_8310243639589261475[134] = 0.0;
   out_8310243639589261475[135] = 0.0;
   out_8310243639589261475[136] = 0.0;
   out_8310243639589261475[137] = 0.0;
   out_8310243639589261475[138] = 0.0;
   out_8310243639589261475[139] = 0.0;
   out_8310243639589261475[140] = 0.0;
   out_8310243639589261475[141] = 0.0;
   out_8310243639589261475[142] = 0.0;
   out_8310243639589261475[143] = 0.0;
   out_8310243639589261475[144] = 0.0;
   out_8310243639589261475[145] = 0.0;
   out_8310243639589261475[146] = 0.0;
   out_8310243639589261475[147] = 0.0;
   out_8310243639589261475[148] = 0.0;
   out_8310243639589261475[149] = 0.0;
   out_8310243639589261475[150] = 0.0;
   out_8310243639589261475[151] = 0.0;
   out_8310243639589261475[152] = 1.0;
   out_8310243639589261475[153] = 0.0;
   out_8310243639589261475[154] = 0.0;
   out_8310243639589261475[155] = 0.0;
   out_8310243639589261475[156] = 0.0;
   out_8310243639589261475[157] = 0.0;
   out_8310243639589261475[158] = 0.0;
   out_8310243639589261475[159] = 0.0;
   out_8310243639589261475[160] = 0.0;
   out_8310243639589261475[161] = 0.0;
   out_8310243639589261475[162] = 0.0;
   out_8310243639589261475[163] = 0.0;
   out_8310243639589261475[164] = 0.0;
   out_8310243639589261475[165] = 0.0;
   out_8310243639589261475[166] = 0.0;
   out_8310243639589261475[167] = 0.0;
   out_8310243639589261475[168] = 0.0;
   out_8310243639589261475[169] = 0.0;
   out_8310243639589261475[170] = 0.0;
   out_8310243639589261475[171] = 1.0;
   out_8310243639589261475[172] = 0.0;
   out_8310243639589261475[173] = 0.0;
   out_8310243639589261475[174] = 0.0;
   out_8310243639589261475[175] = 0.0;
   out_8310243639589261475[176] = 0.0;
   out_8310243639589261475[177] = 0.0;
   out_8310243639589261475[178] = 0.0;
   out_8310243639589261475[179] = 0.0;
   out_8310243639589261475[180] = 0.0;
   out_8310243639589261475[181] = 0.0;
   out_8310243639589261475[182] = 0.0;
   out_8310243639589261475[183] = 0.0;
   out_8310243639589261475[184] = 0.0;
   out_8310243639589261475[185] = 0.0;
   out_8310243639589261475[186] = 0.0;
   out_8310243639589261475[187] = 0.0;
   out_8310243639589261475[188] = 0.0;
   out_8310243639589261475[189] = 0.0;
   out_8310243639589261475[190] = 1.0;
   out_8310243639589261475[191] = 0.0;
   out_8310243639589261475[192] = 0.0;
   out_8310243639589261475[193] = 0.0;
   out_8310243639589261475[194] = 0.0;
   out_8310243639589261475[195] = 0.0;
   out_8310243639589261475[196] = 0.0;
   out_8310243639589261475[197] = 0.0;
   out_8310243639589261475[198] = 0.0;
   out_8310243639589261475[199] = 0.0;
   out_8310243639589261475[200] = 0.0;
   out_8310243639589261475[201] = 0.0;
   out_8310243639589261475[202] = 0.0;
   out_8310243639589261475[203] = 0.0;
   out_8310243639589261475[204] = 0.0;
   out_8310243639589261475[205] = 0.0;
   out_8310243639589261475[206] = 0.0;
   out_8310243639589261475[207] = 0.0;
   out_8310243639589261475[208] = 0.0;
   out_8310243639589261475[209] = 1.0;
   out_8310243639589261475[210] = 0.0;
   out_8310243639589261475[211] = 0.0;
   out_8310243639589261475[212] = 0.0;
   out_8310243639589261475[213] = 0.0;
   out_8310243639589261475[214] = 0.0;
   out_8310243639589261475[215] = 0.0;
   out_8310243639589261475[216] = 0.0;
   out_8310243639589261475[217] = 0.0;
   out_8310243639589261475[218] = 0.0;
   out_8310243639589261475[219] = 0.0;
   out_8310243639589261475[220] = 0.0;
   out_8310243639589261475[221] = 0.0;
   out_8310243639589261475[222] = 0.0;
   out_8310243639589261475[223] = 0.0;
   out_8310243639589261475[224] = 0.0;
   out_8310243639589261475[225] = 0.0;
   out_8310243639589261475[226] = 0.0;
   out_8310243639589261475[227] = 0.0;
   out_8310243639589261475[228] = 1.0;
   out_8310243639589261475[229] = 0.0;
   out_8310243639589261475[230] = 0.0;
   out_8310243639589261475[231] = 0.0;
   out_8310243639589261475[232] = 0.0;
   out_8310243639589261475[233] = 0.0;
   out_8310243639589261475[234] = 0.0;
   out_8310243639589261475[235] = 0.0;
   out_8310243639589261475[236] = 0.0;
   out_8310243639589261475[237] = 0.0;
   out_8310243639589261475[238] = 0.0;
   out_8310243639589261475[239] = 0.0;
   out_8310243639589261475[240] = 0.0;
   out_8310243639589261475[241] = 0.0;
   out_8310243639589261475[242] = 0.0;
   out_8310243639589261475[243] = 0.0;
   out_8310243639589261475[244] = 0.0;
   out_8310243639589261475[245] = 0.0;
   out_8310243639589261475[246] = 0.0;
   out_8310243639589261475[247] = 1.0;
   out_8310243639589261475[248] = 0.0;
   out_8310243639589261475[249] = 0.0;
   out_8310243639589261475[250] = 0.0;
   out_8310243639589261475[251] = 0.0;
   out_8310243639589261475[252] = 0.0;
   out_8310243639589261475[253] = 0.0;
   out_8310243639589261475[254] = 0.0;
   out_8310243639589261475[255] = 0.0;
   out_8310243639589261475[256] = 0.0;
   out_8310243639589261475[257] = 0.0;
   out_8310243639589261475[258] = 0.0;
   out_8310243639589261475[259] = 0.0;
   out_8310243639589261475[260] = 0.0;
   out_8310243639589261475[261] = 0.0;
   out_8310243639589261475[262] = 0.0;
   out_8310243639589261475[263] = 0.0;
   out_8310243639589261475[264] = 0.0;
   out_8310243639589261475[265] = 0.0;
   out_8310243639589261475[266] = 1.0;
   out_8310243639589261475[267] = 0.0;
   out_8310243639589261475[268] = 0.0;
   out_8310243639589261475[269] = 0.0;
   out_8310243639589261475[270] = 0.0;
   out_8310243639589261475[271] = 0.0;
   out_8310243639589261475[272] = 0.0;
   out_8310243639589261475[273] = 0.0;
   out_8310243639589261475[274] = 0.0;
   out_8310243639589261475[275] = 0.0;
   out_8310243639589261475[276] = 0.0;
   out_8310243639589261475[277] = 0.0;
   out_8310243639589261475[278] = 0.0;
   out_8310243639589261475[279] = 0.0;
   out_8310243639589261475[280] = 0.0;
   out_8310243639589261475[281] = 0.0;
   out_8310243639589261475[282] = 0.0;
   out_8310243639589261475[283] = 0.0;
   out_8310243639589261475[284] = 0.0;
   out_8310243639589261475[285] = 1.0;
   out_8310243639589261475[286] = 0.0;
   out_8310243639589261475[287] = 0.0;
   out_8310243639589261475[288] = 0.0;
   out_8310243639589261475[289] = 0.0;
   out_8310243639589261475[290] = 0.0;
   out_8310243639589261475[291] = 0.0;
   out_8310243639589261475[292] = 0.0;
   out_8310243639589261475[293] = 0.0;
   out_8310243639589261475[294] = 0.0;
   out_8310243639589261475[295] = 0.0;
   out_8310243639589261475[296] = 0.0;
   out_8310243639589261475[297] = 0.0;
   out_8310243639589261475[298] = 0.0;
   out_8310243639589261475[299] = 0.0;
   out_8310243639589261475[300] = 0.0;
   out_8310243639589261475[301] = 0.0;
   out_8310243639589261475[302] = 0.0;
   out_8310243639589261475[303] = 0.0;
   out_8310243639589261475[304] = 1.0;
   out_8310243639589261475[305] = 0.0;
   out_8310243639589261475[306] = 0.0;
   out_8310243639589261475[307] = 0.0;
   out_8310243639589261475[308] = 0.0;
   out_8310243639589261475[309] = 0.0;
   out_8310243639589261475[310] = 0.0;
   out_8310243639589261475[311] = 0.0;
   out_8310243639589261475[312] = 0.0;
   out_8310243639589261475[313] = 0.0;
   out_8310243639589261475[314] = 0.0;
   out_8310243639589261475[315] = 0.0;
   out_8310243639589261475[316] = 0.0;
   out_8310243639589261475[317] = 0.0;
   out_8310243639589261475[318] = 0.0;
   out_8310243639589261475[319] = 0.0;
   out_8310243639589261475[320] = 0.0;
   out_8310243639589261475[321] = 0.0;
   out_8310243639589261475[322] = 0.0;
   out_8310243639589261475[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7527557940114523882) {
   out_7527557940114523882[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7527557940114523882[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7527557940114523882[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7527557940114523882[3] = dt*state[12] + state[3];
   out_7527557940114523882[4] = dt*state[13] + state[4];
   out_7527557940114523882[5] = dt*state[14] + state[5];
   out_7527557940114523882[6] = state[6];
   out_7527557940114523882[7] = state[7];
   out_7527557940114523882[8] = state[8];
   out_7527557940114523882[9] = state[9];
   out_7527557940114523882[10] = state[10];
   out_7527557940114523882[11] = state[11];
   out_7527557940114523882[12] = state[12];
   out_7527557940114523882[13] = state[13];
   out_7527557940114523882[14] = state[14];
   out_7527557940114523882[15] = state[15];
   out_7527557940114523882[16] = state[16];
   out_7527557940114523882[17] = state[17];
}
void F_fun(double *state, double dt, double *out_8348964164843467013) {
   out_8348964164843467013[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8348964164843467013[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8348964164843467013[2] = 0;
   out_8348964164843467013[3] = 0;
   out_8348964164843467013[4] = 0;
   out_8348964164843467013[5] = 0;
   out_8348964164843467013[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8348964164843467013[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8348964164843467013[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_8348964164843467013[9] = 0;
   out_8348964164843467013[10] = 0;
   out_8348964164843467013[11] = 0;
   out_8348964164843467013[12] = 0;
   out_8348964164843467013[13] = 0;
   out_8348964164843467013[14] = 0;
   out_8348964164843467013[15] = 0;
   out_8348964164843467013[16] = 0;
   out_8348964164843467013[17] = 0;
   out_8348964164843467013[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8348964164843467013[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8348964164843467013[20] = 0;
   out_8348964164843467013[21] = 0;
   out_8348964164843467013[22] = 0;
   out_8348964164843467013[23] = 0;
   out_8348964164843467013[24] = 0;
   out_8348964164843467013[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8348964164843467013[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_8348964164843467013[27] = 0;
   out_8348964164843467013[28] = 0;
   out_8348964164843467013[29] = 0;
   out_8348964164843467013[30] = 0;
   out_8348964164843467013[31] = 0;
   out_8348964164843467013[32] = 0;
   out_8348964164843467013[33] = 0;
   out_8348964164843467013[34] = 0;
   out_8348964164843467013[35] = 0;
   out_8348964164843467013[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8348964164843467013[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8348964164843467013[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8348964164843467013[39] = 0;
   out_8348964164843467013[40] = 0;
   out_8348964164843467013[41] = 0;
   out_8348964164843467013[42] = 0;
   out_8348964164843467013[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8348964164843467013[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_8348964164843467013[45] = 0;
   out_8348964164843467013[46] = 0;
   out_8348964164843467013[47] = 0;
   out_8348964164843467013[48] = 0;
   out_8348964164843467013[49] = 0;
   out_8348964164843467013[50] = 0;
   out_8348964164843467013[51] = 0;
   out_8348964164843467013[52] = 0;
   out_8348964164843467013[53] = 0;
   out_8348964164843467013[54] = 0;
   out_8348964164843467013[55] = 0;
   out_8348964164843467013[56] = 0;
   out_8348964164843467013[57] = 1;
   out_8348964164843467013[58] = 0;
   out_8348964164843467013[59] = 0;
   out_8348964164843467013[60] = 0;
   out_8348964164843467013[61] = 0;
   out_8348964164843467013[62] = 0;
   out_8348964164843467013[63] = 0;
   out_8348964164843467013[64] = 0;
   out_8348964164843467013[65] = 0;
   out_8348964164843467013[66] = dt;
   out_8348964164843467013[67] = 0;
   out_8348964164843467013[68] = 0;
   out_8348964164843467013[69] = 0;
   out_8348964164843467013[70] = 0;
   out_8348964164843467013[71] = 0;
   out_8348964164843467013[72] = 0;
   out_8348964164843467013[73] = 0;
   out_8348964164843467013[74] = 0;
   out_8348964164843467013[75] = 0;
   out_8348964164843467013[76] = 1;
   out_8348964164843467013[77] = 0;
   out_8348964164843467013[78] = 0;
   out_8348964164843467013[79] = 0;
   out_8348964164843467013[80] = 0;
   out_8348964164843467013[81] = 0;
   out_8348964164843467013[82] = 0;
   out_8348964164843467013[83] = 0;
   out_8348964164843467013[84] = 0;
   out_8348964164843467013[85] = dt;
   out_8348964164843467013[86] = 0;
   out_8348964164843467013[87] = 0;
   out_8348964164843467013[88] = 0;
   out_8348964164843467013[89] = 0;
   out_8348964164843467013[90] = 0;
   out_8348964164843467013[91] = 0;
   out_8348964164843467013[92] = 0;
   out_8348964164843467013[93] = 0;
   out_8348964164843467013[94] = 0;
   out_8348964164843467013[95] = 1;
   out_8348964164843467013[96] = 0;
   out_8348964164843467013[97] = 0;
   out_8348964164843467013[98] = 0;
   out_8348964164843467013[99] = 0;
   out_8348964164843467013[100] = 0;
   out_8348964164843467013[101] = 0;
   out_8348964164843467013[102] = 0;
   out_8348964164843467013[103] = 0;
   out_8348964164843467013[104] = dt;
   out_8348964164843467013[105] = 0;
   out_8348964164843467013[106] = 0;
   out_8348964164843467013[107] = 0;
   out_8348964164843467013[108] = 0;
   out_8348964164843467013[109] = 0;
   out_8348964164843467013[110] = 0;
   out_8348964164843467013[111] = 0;
   out_8348964164843467013[112] = 0;
   out_8348964164843467013[113] = 0;
   out_8348964164843467013[114] = 1;
   out_8348964164843467013[115] = 0;
   out_8348964164843467013[116] = 0;
   out_8348964164843467013[117] = 0;
   out_8348964164843467013[118] = 0;
   out_8348964164843467013[119] = 0;
   out_8348964164843467013[120] = 0;
   out_8348964164843467013[121] = 0;
   out_8348964164843467013[122] = 0;
   out_8348964164843467013[123] = 0;
   out_8348964164843467013[124] = 0;
   out_8348964164843467013[125] = 0;
   out_8348964164843467013[126] = 0;
   out_8348964164843467013[127] = 0;
   out_8348964164843467013[128] = 0;
   out_8348964164843467013[129] = 0;
   out_8348964164843467013[130] = 0;
   out_8348964164843467013[131] = 0;
   out_8348964164843467013[132] = 0;
   out_8348964164843467013[133] = 1;
   out_8348964164843467013[134] = 0;
   out_8348964164843467013[135] = 0;
   out_8348964164843467013[136] = 0;
   out_8348964164843467013[137] = 0;
   out_8348964164843467013[138] = 0;
   out_8348964164843467013[139] = 0;
   out_8348964164843467013[140] = 0;
   out_8348964164843467013[141] = 0;
   out_8348964164843467013[142] = 0;
   out_8348964164843467013[143] = 0;
   out_8348964164843467013[144] = 0;
   out_8348964164843467013[145] = 0;
   out_8348964164843467013[146] = 0;
   out_8348964164843467013[147] = 0;
   out_8348964164843467013[148] = 0;
   out_8348964164843467013[149] = 0;
   out_8348964164843467013[150] = 0;
   out_8348964164843467013[151] = 0;
   out_8348964164843467013[152] = 1;
   out_8348964164843467013[153] = 0;
   out_8348964164843467013[154] = 0;
   out_8348964164843467013[155] = 0;
   out_8348964164843467013[156] = 0;
   out_8348964164843467013[157] = 0;
   out_8348964164843467013[158] = 0;
   out_8348964164843467013[159] = 0;
   out_8348964164843467013[160] = 0;
   out_8348964164843467013[161] = 0;
   out_8348964164843467013[162] = 0;
   out_8348964164843467013[163] = 0;
   out_8348964164843467013[164] = 0;
   out_8348964164843467013[165] = 0;
   out_8348964164843467013[166] = 0;
   out_8348964164843467013[167] = 0;
   out_8348964164843467013[168] = 0;
   out_8348964164843467013[169] = 0;
   out_8348964164843467013[170] = 0;
   out_8348964164843467013[171] = 1;
   out_8348964164843467013[172] = 0;
   out_8348964164843467013[173] = 0;
   out_8348964164843467013[174] = 0;
   out_8348964164843467013[175] = 0;
   out_8348964164843467013[176] = 0;
   out_8348964164843467013[177] = 0;
   out_8348964164843467013[178] = 0;
   out_8348964164843467013[179] = 0;
   out_8348964164843467013[180] = 0;
   out_8348964164843467013[181] = 0;
   out_8348964164843467013[182] = 0;
   out_8348964164843467013[183] = 0;
   out_8348964164843467013[184] = 0;
   out_8348964164843467013[185] = 0;
   out_8348964164843467013[186] = 0;
   out_8348964164843467013[187] = 0;
   out_8348964164843467013[188] = 0;
   out_8348964164843467013[189] = 0;
   out_8348964164843467013[190] = 1;
   out_8348964164843467013[191] = 0;
   out_8348964164843467013[192] = 0;
   out_8348964164843467013[193] = 0;
   out_8348964164843467013[194] = 0;
   out_8348964164843467013[195] = 0;
   out_8348964164843467013[196] = 0;
   out_8348964164843467013[197] = 0;
   out_8348964164843467013[198] = 0;
   out_8348964164843467013[199] = 0;
   out_8348964164843467013[200] = 0;
   out_8348964164843467013[201] = 0;
   out_8348964164843467013[202] = 0;
   out_8348964164843467013[203] = 0;
   out_8348964164843467013[204] = 0;
   out_8348964164843467013[205] = 0;
   out_8348964164843467013[206] = 0;
   out_8348964164843467013[207] = 0;
   out_8348964164843467013[208] = 0;
   out_8348964164843467013[209] = 1;
   out_8348964164843467013[210] = 0;
   out_8348964164843467013[211] = 0;
   out_8348964164843467013[212] = 0;
   out_8348964164843467013[213] = 0;
   out_8348964164843467013[214] = 0;
   out_8348964164843467013[215] = 0;
   out_8348964164843467013[216] = 0;
   out_8348964164843467013[217] = 0;
   out_8348964164843467013[218] = 0;
   out_8348964164843467013[219] = 0;
   out_8348964164843467013[220] = 0;
   out_8348964164843467013[221] = 0;
   out_8348964164843467013[222] = 0;
   out_8348964164843467013[223] = 0;
   out_8348964164843467013[224] = 0;
   out_8348964164843467013[225] = 0;
   out_8348964164843467013[226] = 0;
   out_8348964164843467013[227] = 0;
   out_8348964164843467013[228] = 1;
   out_8348964164843467013[229] = 0;
   out_8348964164843467013[230] = 0;
   out_8348964164843467013[231] = 0;
   out_8348964164843467013[232] = 0;
   out_8348964164843467013[233] = 0;
   out_8348964164843467013[234] = 0;
   out_8348964164843467013[235] = 0;
   out_8348964164843467013[236] = 0;
   out_8348964164843467013[237] = 0;
   out_8348964164843467013[238] = 0;
   out_8348964164843467013[239] = 0;
   out_8348964164843467013[240] = 0;
   out_8348964164843467013[241] = 0;
   out_8348964164843467013[242] = 0;
   out_8348964164843467013[243] = 0;
   out_8348964164843467013[244] = 0;
   out_8348964164843467013[245] = 0;
   out_8348964164843467013[246] = 0;
   out_8348964164843467013[247] = 1;
   out_8348964164843467013[248] = 0;
   out_8348964164843467013[249] = 0;
   out_8348964164843467013[250] = 0;
   out_8348964164843467013[251] = 0;
   out_8348964164843467013[252] = 0;
   out_8348964164843467013[253] = 0;
   out_8348964164843467013[254] = 0;
   out_8348964164843467013[255] = 0;
   out_8348964164843467013[256] = 0;
   out_8348964164843467013[257] = 0;
   out_8348964164843467013[258] = 0;
   out_8348964164843467013[259] = 0;
   out_8348964164843467013[260] = 0;
   out_8348964164843467013[261] = 0;
   out_8348964164843467013[262] = 0;
   out_8348964164843467013[263] = 0;
   out_8348964164843467013[264] = 0;
   out_8348964164843467013[265] = 0;
   out_8348964164843467013[266] = 1;
   out_8348964164843467013[267] = 0;
   out_8348964164843467013[268] = 0;
   out_8348964164843467013[269] = 0;
   out_8348964164843467013[270] = 0;
   out_8348964164843467013[271] = 0;
   out_8348964164843467013[272] = 0;
   out_8348964164843467013[273] = 0;
   out_8348964164843467013[274] = 0;
   out_8348964164843467013[275] = 0;
   out_8348964164843467013[276] = 0;
   out_8348964164843467013[277] = 0;
   out_8348964164843467013[278] = 0;
   out_8348964164843467013[279] = 0;
   out_8348964164843467013[280] = 0;
   out_8348964164843467013[281] = 0;
   out_8348964164843467013[282] = 0;
   out_8348964164843467013[283] = 0;
   out_8348964164843467013[284] = 0;
   out_8348964164843467013[285] = 1;
   out_8348964164843467013[286] = 0;
   out_8348964164843467013[287] = 0;
   out_8348964164843467013[288] = 0;
   out_8348964164843467013[289] = 0;
   out_8348964164843467013[290] = 0;
   out_8348964164843467013[291] = 0;
   out_8348964164843467013[292] = 0;
   out_8348964164843467013[293] = 0;
   out_8348964164843467013[294] = 0;
   out_8348964164843467013[295] = 0;
   out_8348964164843467013[296] = 0;
   out_8348964164843467013[297] = 0;
   out_8348964164843467013[298] = 0;
   out_8348964164843467013[299] = 0;
   out_8348964164843467013[300] = 0;
   out_8348964164843467013[301] = 0;
   out_8348964164843467013[302] = 0;
   out_8348964164843467013[303] = 0;
   out_8348964164843467013[304] = 1;
   out_8348964164843467013[305] = 0;
   out_8348964164843467013[306] = 0;
   out_8348964164843467013[307] = 0;
   out_8348964164843467013[308] = 0;
   out_8348964164843467013[309] = 0;
   out_8348964164843467013[310] = 0;
   out_8348964164843467013[311] = 0;
   out_8348964164843467013[312] = 0;
   out_8348964164843467013[313] = 0;
   out_8348964164843467013[314] = 0;
   out_8348964164843467013[315] = 0;
   out_8348964164843467013[316] = 0;
   out_8348964164843467013[317] = 0;
   out_8348964164843467013[318] = 0;
   out_8348964164843467013[319] = 0;
   out_8348964164843467013[320] = 0;
   out_8348964164843467013[321] = 0;
   out_8348964164843467013[322] = 0;
   out_8348964164843467013[323] = 1;
}
void h_4(double *state, double *unused, double *out_5762446430048339267) {
   out_5762446430048339267[0] = state[6] + state[9];
   out_5762446430048339267[1] = state[7] + state[10];
   out_5762446430048339267[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5036021560798312222) {
   out_5036021560798312222[0] = 0;
   out_5036021560798312222[1] = 0;
   out_5036021560798312222[2] = 0;
   out_5036021560798312222[3] = 0;
   out_5036021560798312222[4] = 0;
   out_5036021560798312222[5] = 0;
   out_5036021560798312222[6] = 1;
   out_5036021560798312222[7] = 0;
   out_5036021560798312222[8] = 0;
   out_5036021560798312222[9] = 1;
   out_5036021560798312222[10] = 0;
   out_5036021560798312222[11] = 0;
   out_5036021560798312222[12] = 0;
   out_5036021560798312222[13] = 0;
   out_5036021560798312222[14] = 0;
   out_5036021560798312222[15] = 0;
   out_5036021560798312222[16] = 0;
   out_5036021560798312222[17] = 0;
   out_5036021560798312222[18] = 0;
   out_5036021560798312222[19] = 0;
   out_5036021560798312222[20] = 0;
   out_5036021560798312222[21] = 0;
   out_5036021560798312222[22] = 0;
   out_5036021560798312222[23] = 0;
   out_5036021560798312222[24] = 0;
   out_5036021560798312222[25] = 1;
   out_5036021560798312222[26] = 0;
   out_5036021560798312222[27] = 0;
   out_5036021560798312222[28] = 1;
   out_5036021560798312222[29] = 0;
   out_5036021560798312222[30] = 0;
   out_5036021560798312222[31] = 0;
   out_5036021560798312222[32] = 0;
   out_5036021560798312222[33] = 0;
   out_5036021560798312222[34] = 0;
   out_5036021560798312222[35] = 0;
   out_5036021560798312222[36] = 0;
   out_5036021560798312222[37] = 0;
   out_5036021560798312222[38] = 0;
   out_5036021560798312222[39] = 0;
   out_5036021560798312222[40] = 0;
   out_5036021560798312222[41] = 0;
   out_5036021560798312222[42] = 0;
   out_5036021560798312222[43] = 0;
   out_5036021560798312222[44] = 1;
   out_5036021560798312222[45] = 0;
   out_5036021560798312222[46] = 0;
   out_5036021560798312222[47] = 1;
   out_5036021560798312222[48] = 0;
   out_5036021560798312222[49] = 0;
   out_5036021560798312222[50] = 0;
   out_5036021560798312222[51] = 0;
   out_5036021560798312222[52] = 0;
   out_5036021560798312222[53] = 0;
}
void h_10(double *state, double *unused, double *out_3165249718128103071) {
   out_3165249718128103071[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3165249718128103071[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3165249718128103071[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_697369030236560030) {
   out_697369030236560030[0] = 0;
   out_697369030236560030[1] = 9.8100000000000005*cos(state[1]);
   out_697369030236560030[2] = 0;
   out_697369030236560030[3] = 0;
   out_697369030236560030[4] = -state[8];
   out_697369030236560030[5] = state[7];
   out_697369030236560030[6] = 0;
   out_697369030236560030[7] = state[5];
   out_697369030236560030[8] = -state[4];
   out_697369030236560030[9] = 0;
   out_697369030236560030[10] = 0;
   out_697369030236560030[11] = 0;
   out_697369030236560030[12] = 1;
   out_697369030236560030[13] = 0;
   out_697369030236560030[14] = 0;
   out_697369030236560030[15] = 1;
   out_697369030236560030[16] = 0;
   out_697369030236560030[17] = 0;
   out_697369030236560030[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_697369030236560030[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_697369030236560030[20] = 0;
   out_697369030236560030[21] = state[8];
   out_697369030236560030[22] = 0;
   out_697369030236560030[23] = -state[6];
   out_697369030236560030[24] = -state[5];
   out_697369030236560030[25] = 0;
   out_697369030236560030[26] = state[3];
   out_697369030236560030[27] = 0;
   out_697369030236560030[28] = 0;
   out_697369030236560030[29] = 0;
   out_697369030236560030[30] = 0;
   out_697369030236560030[31] = 1;
   out_697369030236560030[32] = 0;
   out_697369030236560030[33] = 0;
   out_697369030236560030[34] = 1;
   out_697369030236560030[35] = 0;
   out_697369030236560030[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_697369030236560030[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_697369030236560030[38] = 0;
   out_697369030236560030[39] = -state[7];
   out_697369030236560030[40] = state[6];
   out_697369030236560030[41] = 0;
   out_697369030236560030[42] = state[4];
   out_697369030236560030[43] = -state[3];
   out_697369030236560030[44] = 0;
   out_697369030236560030[45] = 0;
   out_697369030236560030[46] = 0;
   out_697369030236560030[47] = 0;
   out_697369030236560030[48] = 0;
   out_697369030236560030[49] = 0;
   out_697369030236560030[50] = 1;
   out_697369030236560030[51] = 0;
   out_697369030236560030[52] = 0;
   out_697369030236560030[53] = 1;
}
void h_13(double *state, double *unused, double *out_2411777720830163540) {
   out_2411777720830163540[0] = state[3];
   out_2411777720830163540[1] = state[4];
   out_2411777720830163540[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8248295386130645023) {
   out_8248295386130645023[0] = 0;
   out_8248295386130645023[1] = 0;
   out_8248295386130645023[2] = 0;
   out_8248295386130645023[3] = 1;
   out_8248295386130645023[4] = 0;
   out_8248295386130645023[5] = 0;
   out_8248295386130645023[6] = 0;
   out_8248295386130645023[7] = 0;
   out_8248295386130645023[8] = 0;
   out_8248295386130645023[9] = 0;
   out_8248295386130645023[10] = 0;
   out_8248295386130645023[11] = 0;
   out_8248295386130645023[12] = 0;
   out_8248295386130645023[13] = 0;
   out_8248295386130645023[14] = 0;
   out_8248295386130645023[15] = 0;
   out_8248295386130645023[16] = 0;
   out_8248295386130645023[17] = 0;
   out_8248295386130645023[18] = 0;
   out_8248295386130645023[19] = 0;
   out_8248295386130645023[20] = 0;
   out_8248295386130645023[21] = 0;
   out_8248295386130645023[22] = 1;
   out_8248295386130645023[23] = 0;
   out_8248295386130645023[24] = 0;
   out_8248295386130645023[25] = 0;
   out_8248295386130645023[26] = 0;
   out_8248295386130645023[27] = 0;
   out_8248295386130645023[28] = 0;
   out_8248295386130645023[29] = 0;
   out_8248295386130645023[30] = 0;
   out_8248295386130645023[31] = 0;
   out_8248295386130645023[32] = 0;
   out_8248295386130645023[33] = 0;
   out_8248295386130645023[34] = 0;
   out_8248295386130645023[35] = 0;
   out_8248295386130645023[36] = 0;
   out_8248295386130645023[37] = 0;
   out_8248295386130645023[38] = 0;
   out_8248295386130645023[39] = 0;
   out_8248295386130645023[40] = 0;
   out_8248295386130645023[41] = 1;
   out_8248295386130645023[42] = 0;
   out_8248295386130645023[43] = 0;
   out_8248295386130645023[44] = 0;
   out_8248295386130645023[45] = 0;
   out_8248295386130645023[46] = 0;
   out_8248295386130645023[47] = 0;
   out_8248295386130645023[48] = 0;
   out_8248295386130645023[49] = 0;
   out_8248295386130645023[50] = 0;
   out_8248295386130645023[51] = 0;
   out_8248295386130645023[52] = 0;
   out_8248295386130645023[53] = 0;
}
void h_14(double *state, double *unused, double *out_5831462969446676566) {
   out_5831462969446676566[0] = state[6];
   out_5831462969446676566[1] = state[7];
   out_5831462969446676566[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8999262417137796751) {
   out_8999262417137796751[0] = 0;
   out_8999262417137796751[1] = 0;
   out_8999262417137796751[2] = 0;
   out_8999262417137796751[3] = 0;
   out_8999262417137796751[4] = 0;
   out_8999262417137796751[5] = 0;
   out_8999262417137796751[6] = 1;
   out_8999262417137796751[7] = 0;
   out_8999262417137796751[8] = 0;
   out_8999262417137796751[9] = 0;
   out_8999262417137796751[10] = 0;
   out_8999262417137796751[11] = 0;
   out_8999262417137796751[12] = 0;
   out_8999262417137796751[13] = 0;
   out_8999262417137796751[14] = 0;
   out_8999262417137796751[15] = 0;
   out_8999262417137796751[16] = 0;
   out_8999262417137796751[17] = 0;
   out_8999262417137796751[18] = 0;
   out_8999262417137796751[19] = 0;
   out_8999262417137796751[20] = 0;
   out_8999262417137796751[21] = 0;
   out_8999262417137796751[22] = 0;
   out_8999262417137796751[23] = 0;
   out_8999262417137796751[24] = 0;
   out_8999262417137796751[25] = 1;
   out_8999262417137796751[26] = 0;
   out_8999262417137796751[27] = 0;
   out_8999262417137796751[28] = 0;
   out_8999262417137796751[29] = 0;
   out_8999262417137796751[30] = 0;
   out_8999262417137796751[31] = 0;
   out_8999262417137796751[32] = 0;
   out_8999262417137796751[33] = 0;
   out_8999262417137796751[34] = 0;
   out_8999262417137796751[35] = 0;
   out_8999262417137796751[36] = 0;
   out_8999262417137796751[37] = 0;
   out_8999262417137796751[38] = 0;
   out_8999262417137796751[39] = 0;
   out_8999262417137796751[40] = 0;
   out_8999262417137796751[41] = 0;
   out_8999262417137796751[42] = 0;
   out_8999262417137796751[43] = 0;
   out_8999262417137796751[44] = 1;
   out_8999262417137796751[45] = 0;
   out_8999262417137796751[46] = 0;
   out_8999262417137796751[47] = 0;
   out_8999262417137796751[48] = 0;
   out_8999262417137796751[49] = 0;
   out_8999262417137796751[50] = 0;
   out_8999262417137796751[51] = 0;
   out_8999262417137796751[52] = 0;
   out_8999262417137796751[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3068681635687019270) {
  err_fun(nom_x, delta_x, out_3068681635687019270);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6108564159138725733) {
  inv_err_fun(nom_x, true_x, out_6108564159138725733);
}
void pose_H_mod_fun(double *state, double *out_8310243639589261475) {
  H_mod_fun(state, out_8310243639589261475);
}
void pose_f_fun(double *state, double dt, double *out_7527557940114523882) {
  f_fun(state,  dt, out_7527557940114523882);
}
void pose_F_fun(double *state, double dt, double *out_8348964164843467013) {
  F_fun(state,  dt, out_8348964164843467013);
}
void pose_h_4(double *state, double *unused, double *out_5762446430048339267) {
  h_4(state, unused, out_5762446430048339267);
}
void pose_H_4(double *state, double *unused, double *out_5036021560798312222) {
  H_4(state, unused, out_5036021560798312222);
}
void pose_h_10(double *state, double *unused, double *out_3165249718128103071) {
  h_10(state, unused, out_3165249718128103071);
}
void pose_H_10(double *state, double *unused, double *out_697369030236560030) {
  H_10(state, unused, out_697369030236560030);
}
void pose_h_13(double *state, double *unused, double *out_2411777720830163540) {
  h_13(state, unused, out_2411777720830163540);
}
void pose_H_13(double *state, double *unused, double *out_8248295386130645023) {
  H_13(state, unused, out_8248295386130645023);
}
void pose_h_14(double *state, double *unused, double *out_5831462969446676566) {
  h_14(state, unused, out_5831462969446676566);
}
void pose_H_14(double *state, double *unused, double *out_8999262417137796751) {
  H_14(state, unused, out_8999262417137796751);
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
