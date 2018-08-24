
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>
#include "Interpolator.h"

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif


#define TIMESTEPS 84
#define MOTORS 8 

enum TBMode
{
	TB_SIT = 0,
	TB_STAND,
};

float pos[TIMESTEPS][MOTORS] = {{0.456580,0.215020,0.837780,1.625513,0.215119,0.456482,1.625524,0.837784},
{0.458896,0.215851,0.839891,1.628079,0.215932,0.458864,1.628090,0.839896},
{0.461286,0.216499,0.842176,1.630833,0.216598,0.461188,1.630845,0.842180},
{0.463604,0.217333,0.844284,1.633389,0.217413,0.463572,1.633401,0.844289},
{0.465998,0.217985,0.846568,1.636134,0.218084,0.465900,1.636146,0.846572},
{0.468319,0.218823,0.848676,1.638681,0.218903,0.468287,1.638692,0.848680},
{0.470714,0.219479,0.850959,1.641416,0.219578,0.470616,1.641428,0.850963},
{0.473039,0.220321,0.853065,1.643954,0.220401,0.473006,1.643965,0.853069},
{0.475434,0.220981,0.855346,1.646680,0.221080,0.475335,1.646691,0.855350},
{0.477762,0.221826,0.857451,1.649208,0.221906,0.477729,1.649219,0.857455},
{0.480158,0.222490,0.859731,1.651924,0.222589,0.480059,1.651936,0.859735},
{0.482490,0.223339,0.861834,1.654443,0.223419,0.482457,1.654454,0.861839},
{0.484886,0.224006,0.864113,1.657151,0.224106,0.484787,1.657162,0.864117},
{0.487221,0.224860,0.866215,1.659660,0.224940,0.487188,1.659671,0.866220},
{0.489618,0.225531,0.868492,1.662358,0.225630,0.489519,1.662370,0.868496},
{0.491957,0.226387,0.870592,1.664859,0.226467,0.491924,1.664870,0.870597},
{0.494354,0.227062,0.872869,1.667548,0.227161,0.494255,1.667559,0.872873},
{0.496696,0.227923,0.874968,1.670039,0.228003,0.496663,1.670051,0.874973},
{0.499093,0.228601,0.877242,1.672719,0.228700,0.498994,1.672731,0.877246},
{0.501439,0.229465,0.879340,1.675201,0.229546,0.501405,1.675213,0.879345},
{0.503836,0.230147,0.881613,1.677873,0.230247,0.503737,1.677884,0.881617},
{0.503736,0.230047,0.881513,1.677773,0.230147,0.503836,1.677784,0.881517},
{0.433257,0.208666,0.917353,1.732804,0.208696,0.433344,1.732815,0.917356},
{0.383392,0.216608,0.946268,1.760331,0.216568,0.383465,1.760342,0.946271},
{0.350902,0.249098,0.970632,1.768031,0.249059,0.350941,1.768042,0.970583},
{0.334258,0.303670,0.990909,1.759397,0.303631,0.334264,1.759407,0.990912},
{0.334625,0.381893,1.007433,1.736623,0.381854,0.334631,1.736633,1.007434},
{0.351061,0.481579,1.020068,1.701112,0.481540,0.351067,1.701122,1.020070},
{0.380882,0.599637,1.029728,1.655103,0.599599,0.380888,1.655007,1.029731},
{0.420566,0.729634,1.036853,1.600236,0.729596,0.420571,1.600244,1.036858},
{0.465293,0.865162,1.042909,1.538116,0.865125,0.465297,1.538124,1.042914},
{0.511940,1.000374,1.048554,1.470629,1.000337,0.511944,1.470637,1.048559},
{0.557279,1.131156,1.055333,1.398920,1.131119,0.557283,1.398928,1.055337},
{0.600386,1.255854,1.063832,1.325112,1.255818,0.600389,1.325120,1.063836},
{0.640101,1.373141,1.075086,1.250646,1.373105,0.640104,1.250653,1.075090},
{0.676737,1.483559,1.089677,1.177967,1.483523,0.676739,1.177974,1.089680},
{0.709999,1.587140,1.107351,1.108953,1.587104,0.710001,1.108985,1.107354},
{0.740571,1.684942,1.128191,1.045846,1.684906,0.740573,1.045853,1.128193},
{0.768396,1.777287,1.151433,0.990034,1.777252,0.768398,0.990041,1.151449},
{0.794181,1.865205,1.176224,0.942517,1.865170,0.794182,0.942523,1.176226},
{0.818101,1.948981,1.201952,0.904425,1.948946,0.818102,0.904432,1.201953},
{0.839927,2.029193,1.228159,0.876733,2.029158,0.839928,0.876739,1.228160},
{0.840027,2.029093,1.228259,0.876633,2.029058,0.840028,0.876639,1.228159},
{0.840784,2.031811,1.229372,0.875867,2.031776,0.840785,0.875869,1.229293},
{0.841390,2.034711,1.230358,0.875010,2.034676,0.841391,0.875016,1.230258},
{0.841986,2.037422,1.231345,0.874240,2.037388,0.841987,0.874243,1.231305},
{0.842753,2.040316,1.232460,0.873394,2.040281,0.842754,0.873400,1.232360},
{0.843348,2.043020,1.233450,0.872627,2.042985,0.843349,0.872630,1.233409},
{0.844114,2.045907,1.234566,0.871784,2.045872,0.844115,0.871791,1.234466},
{0.844709,2.048604,1.235558,0.871021,2.048570,0.844710,0.871024,1.235518},
{0.845474,2.051485,1.236676,0.870181,2.051450,0.845475,0.870188,1.236576},
{0.846069,2.054176,1.237669,0.869422,2.054142,0.846069,0.869424,1.237629},
{0.846834,2.057050,1.238789,0.868585,2.057016,0.846834,0.868592,1.238689},
{0.847427,2.059735,1.239784,0.867829,2.059701,0.847428,0.867832,1.239745},
{0.848192,2.062603,1.240905,0.866996,2.062568,0.848193,0.867002,1.240805},
{0.848785,2.065281,1.241903,0.866243,2.065247,0.848786,0.866246,1.241863},
{0.849549,2.068142,1.243025,0.865413,2.068107,0.849550,0.865419,1.242925},
{0.850142,2.070814,1.244025,0.864664,2.070780,0.850143,0.864667,1.243985},
{0.850905,2.073668,1.245148,0.863837,2.073634,0.850906,0.863844,1.245048},
{0.851498,2.076334,1.246150,0.863092,2.076300,0.851498,0.863095,1.246111},
{0.852260,2.079182,1.247274,0.862268,2.079147,0.852261,0.862274,1.247174},
{0.852852,2.081841,1.248277,0.861527,2.081807,0.852853,0.861529,1.248236},
{0.853614,2.084683,1.249404,0.860706,2.084648,0.853615,0.860712,1.249304},
{0.853514,2.084583,1.249304,0.860606,2.084548,0.853515,0.860612,1.249286},
{0.865081,2.146565,1.125639,0.814393,2.146531,0.865082,0.814400,1.125622},
{0.858847,2.170244,0.998816,0.761381,2.170211,0.858848,0.761388,0.998799},
{0.838996,2.164239,0.867548,0.700631,2.164206,0.838997,0.700639,0.867531},
{0.807450,2.131612,0.733574,0.632462,2.131580,0.807451,0.632470,0.733557},
{0.765781,2.073027,0.599712,0.559885,2.073047,0.765781,0.559893,0.599696},
{0.715068,1.989149,0.472533,0.489464,1.989117,0.715067,0.489473,0.472518},
{0.657371,1.880361,0.359828,0.431822,1.880330,0.657414,0.431832,0.359813},
{0.594249,1.748407,0.269255,0.397188,1.748376,0.594246,0.397198,0.269240},
{0.527285,1.593922,0.205683,0.394319,1.593892,0.527237,0.394331,0.205669},
{0.460485,1.422441,0.171886,0.428114,1.422412,0.460388,0.428128,0.171872},
{0.399695,1.243221,0.170355,0.502165,1.243192,0.399599,0.502179,0.170343},
{0.349491,1.063851,0.200114,0.611814,1.063823,0.349392,0.611827,0.200102},
{0.311725,0.888446,0.257601,0.749150,0.888481,0.311628,0.749163,0.257590},
{0.288297,0.721839,0.334882,0.899547,0.721936,0.288198,0.899560,0.334871},
{0.279791,0.569930,0.421574,1.048906,0.570027,0.279693,1.048918,0.421563},
{0.286686,0.437951,0.511224,1.189075,0.438049,0.286587,1.189088,0.511214},
{0.308171,0.331721,0.598926,1.316163,0.331820,0.308073,1.316175,0.598916},
{0.344118,0.255882,0.682841,1.430585,0.255982,0.344020,1.430597,0.682832},
{0.393365,0.215163,0.762928,1.533217,0.215262,0.393266,1.533229,0.762919},
{0.456680,0.215120,0.837693,1.625613,0.215219,0.456580,1.625624,0.837684}};

float vel[TIMESTEPS][MOTORS] = {{15.208288,5.196631,14.856636,17.943012,5.196727,15.208387,17.942915,14.856657},
{15.219876,5.214287,14.852277,17.908215,5.214312,15.219862,17.908171,14.852279},
{15.229264,5.229532,14.847982,17.875181,5.229534,15.229234,17.875157,14.847983},
{15.236333,5.241885,14.843530,17.843828,5.241900,15.236301,17.843811,14.843526},
{15.243243,5.254293,14.839175,17.812726,5.254300,15.243222,17.812711,14.839173},
{15.250185,5.266646,14.834690,17.781559,5.266665,15.250159,17.781543,14.834686},
{15.256966,5.279047,14.830306,17.750640,5.279065,15.256959,17.750623,14.830302},
{15.263789,5.291399,14.825791,17.719654,5.291420,15.263765,17.719638,14.825785},
{15.270449,5.303791,14.821378,17.688917,5.303810,15.270443,17.688900,14.821372},
{15.277156,5.316127,14.816833,17.658111,5.316147,15.277129,17.658095,14.816826},
{15.283696,5.328506,14.812393,17.627555,5.328523,15.283685,17.627540,14.812384},
{15.290279,5.340831,14.807821,17.596932,5.340852,15.290250,17.596917,14.807811},
{15.296697,5.353204,14.803355,17.566559,5.353218,15.296681,17.566545,14.803343},
{15.303156,5.365523,14.798756,17.536120,5.365543,15.303124,17.536106,14.798746},
{15.309452,5.377893,14.794264,17.505931,5.377902,15.309429,17.505919,14.794254},
{15.315786,5.390205,14.789640,17.475674,5.390222,15.315750,17.475665,14.789633},
{15.321962,5.402575,14.785123,17.445668,5.402573,15.321927,17.445664,14.785121},
{15.328168,5.414881,14.780476,17.415595,5.414893,15.328126,17.415597,14.780477},
{15.334214,5.427248,14.775944,17.385780,5.427242,15.334183,17.385775,14.775937},
{15.340021,5.439183,14.771269,17.356125,5.439205,15.339998,17.356093,14.771242},
{15.347205,5.453313,14.766847,17.325431,5.453413,15.347304,17.325359,14.766749},
{-19.673651,-9.046189,9.614278,17.211650,-9.046089,-19.673551,17.211623,9.614376},
{-14.545107,-1.519314,7.767744,9.739048,-1.519389,-14.545206,9.738982,7.767720},
{-9.772452,5.273335,6.352611,3.968632,5.273312,-9.772371,3.968550,6.352547},
{-6.014181,10.514746,5.396396,-0.179676,10.514845,-6.014279,-0.179764,5.396345},
{-1.989448,16.080142,4.466648,-3.856989,16.080241,-1.989460,-3.857063,4.466596},
{2.108394,21.709637,3.499284,-7.134030,21.709664,2.108296,-7.134095,3.499203},
{5.707638,26.587177,2.677460,-9.937867,26.587276,5.707737,-9.937953,2.677394},
{8.598003,30.369536,2.009803,-12.287837,30.369635,8.597904,-12.287931,2.009754},
{10.393881,32.474358,1.544800,-14.228296,32.474458,10.393782,-14.228381,1.544707},
{11.195537,33.037770,1.363396,-15.765808,33.037868,11.195438,-15.765896,1.363307},
{11.228281,32.417733,1.441862,-16.928201,32.417820,11.228193,-16.928297,1.441764},
{10.749230,31.038398,1.807177,-17.705665,31.038451,10.749133,-17.705762,1.807084},
{10.048712,29.369953,2.372152,-18.043249,29.370031,10.048621,-18.043331,2.372056},
{9.241858,27.605630,3.115350,-17.915821,27.605729,9.241759,-17.915805,3.115259},
{8.458544,25.941672,3.896688,-17.272711,25.941771,8.458444,-17.272656,3.896601},
{7.713256,24.399932,4.678399,-16.069114,24.400015,7.713158,-16.069212,4.678305},
{7.056583,23.042253,5.336806,-14.499892,23.042310,7.056496,-14.499793,5.336904},
{6.475851,21.844653,5.841245,-12.555375,21.844664,6.475752,-12.555475,5.841145},
{5.969950,20.797958,6.170448,-10.380347,20.797966,5.969881,-10.380247,6.170548},
{5.546991,19.902005,6.260377,-8.041681,19.902052,5.546991,-8.041781,6.260277},
{5.050813,18.972803,6.499525,-5.239572,18.972753,5.050738,-5.239472,6.499625},
{4.785155,18.708782,6.524463,-4.988881,18.708777,4.785076,-4.988783,6.524547},
{4.782772,18.685883,6.531585,-4.977229,18.685926,4.782778,-4.977229,6.531581},
{4.780763,18.663694,6.538027,-4.965855,18.663749,4.780786,-4.965880,6.538011},
{4.778924,18.641993,6.543955,-4.955024,18.642049,4.778953,-4.955050,6.543923},
{4.777209,18.620494,6.549730,-4.944062,18.620547,4.777240,-4.944088,6.549695},
{4.775402,18.598905,6.555572,-4.933205,18.598957,4.775429,-4.933226,6.555537},
{4.773716,18.577519,6.561262,-4.922215,18.577565,4.773739,-4.922229,6.561232},
{4.771929,18.556039,6.567021,-4.911319,18.556090,4.771953,-4.911336,6.566988},
{4.770268,18.534765,6.572628,-4.900294,18.534810,4.770290,-4.900307,6.572601},
{4.768508,18.513396,6.578306,-4.889365,18.513447,4.768534,-4.889384,6.578272},
{4.766874,18.492232,6.583832,-4.878308,18.492279,4.766900,-4.878324,6.583805},
{4.765143,18.470975,6.589432,-4.867351,18.471027,4.765170,-4.867371,6.589395},
{4.763534,18.449921,6.594878,-4.856264,18.449971,4.763564,-4.856283,6.594849},
{4.761829,18.428774,6.600398,-4.845279,18.428828,4.761858,-4.845301,6.600359},
{4.760245,18.407831,6.605764,-4.834164,18.407881,4.760275,-4.834184,6.605732},
{4.758564,18.386794,6.611203,-4.823150,18.386846,4.758591,-4.823171,6.611161},
{4.757006,18.365961,6.616488,-4.812005,18.366006,4.757029,-4.812025,6.616454},
{4.755349,18.345034,6.621847,-4.800963,18.345079,4.755368,-4.800984,6.621802},
{4.753814,18.324307,6.627049,-4.789790,18.324348,4.753830,-4.789808,6.627014},
{4.752206,18.303545,6.632320,-4.778728,18.303603,4.752236,-4.778747,6.632274},
{4.750497,18.282574,6.637467,-4.767488,18.282660,4.750552,-4.767504,6.637437},
{4.444696,17.389367,-25.378543,-9.012775,17.389466,4.444795,-9.012676,-25.378643},
{0.422581,8.634837,-26.025995,-10.291638,8.634935,0.422541,-10.291541,-26.025896},
{-2.872046,1.549230,-26.774947,-11.756252,1.549322,-2.872038,-11.756154,-26.774936},
{-5.397138,-4.035490,-27.675041,-13.484422,-4.035402,-5.397184,-13.484324,-27.674969},
{-7.650886,-9.434425,-27.983014,-14.757776,-9.434328,-7.650979,-14.757686,-27.982918},
{-9.672593,-14.829387,-27.391745,-15.173740,-14.829294,-9.672677,-15.173656,-27.391647},
{-11.327592,-20.020867,-25.287917,-13.778152,-20.020774,-11.327666,-13.778062,-25.287821},
{-12.633791,-25.075078,-21.340449,-9.923380,-25.074980,-12.633878,-9.923282,-21.340351},
{-13.537871,-29.734599,-16.170016,-4.130883,-29.734516,-13.537953,-4.130785,-16.169920},
{-14.095548,-34.145246,-10.224690,3.071006,-34.145158,-14.095598,3.071070,-10.224591},
{-13.552855,-36.882765,-3.709449,11.193847,-36.882669,-13.552946,11.193845,-3.709353},
{-11.621591,-37.407862,3.008677,19.335505,-37.407764,-11.621620,19.335455,3.008769},
{-9.235185,-37.027906,9.214824,26.075262,-37.027814,-9.235096,26.075217,9.214917},
{-6.406997,-35.722082,14.349770,30.454783,-35.721987,-6.406930,30.454756,14.349864},
{-3.357700,-33.288216,17.360078,31.588402,-33.288118,-3.357615,31.588371,17.360166},
{-0.167567,-29.687963,18.507781,30.290433,-29.687867,-0.167535,30.290398,18.507875},
{2.961213,-24.920178,18.507503,27.851708,-24.920125,2.961295,27.851662,18.507595},
{5.974606,-19.086310,17.856988,25.076050,-19.086211,5.974629,25.075998,17.857079},
{8.890534,-12.203544,17.016357,22.503369,-12.203447,8.890632,22.503316,17.016448},
{11.603815,-4.485356,16.148690,20.251577,-4.485455,11.603716,20.251516,16.148766},
{14.821626,4.789595,14.886701,18.165096,4.789694,14.821724,18.164999,14.886672}};

float u[TIMESTEPS][MOTORS] = {{0.039207,0.061161,0.003953,-0.078132,0.060999,0.039064,-0.077895,0.003909},
{0.039207,0.061161,0.003953,-0.078132,0.060999,0.039064,-0.077895,0.003909},
{0.026988,0.045214,-0.000039,-0.062277,0.045221,0.026996,-0.062279,-0.000028},
{0.026988,0.045214,-0.000039,-0.062277,0.045221,0.026996,-0.062279,-0.000028},
{0.026949,0.045667,-0.000340,-0.062311,0.045660,0.026944,-0.062302,-0.000330},
{0.026949,0.045667,-0.000340,-0.062311,0.045660,0.026944,-0.062302,-0.000330},
{0.026901,0.046109,-0.000639,-0.062337,0.046107,0.026899,-0.062332,-0.000634},
{0.026901,0.046109,-0.000639,-0.062337,0.046107,0.026899,-0.062332,-0.000634},
{0.026854,0.046550,-0.000941,-0.062365,0.046548,0.026852,-0.062359,-0.000939},
{0.026854,0.046550,-0.000941,-0.062365,0.046548,0.026852,-0.062359,-0.000939},
{0.026807,0.046987,-0.001244,-0.062394,0.046983,0.026807,-0.062386,-0.001245},
{0.026807,0.046987,-0.001244,-0.062394,0.046983,0.026807,-0.062386,-0.001245},
{0.026757,0.047424,-0.001548,-0.062423,0.047412,0.026761,-0.062411,-0.001552},
{0.026757,0.047424,-0.001548,-0.062423,0.047412,0.026761,-0.062411,-0.001552},
{0.026704,0.047855,-0.001853,-0.062451,0.047843,0.026708,-0.062435,-0.001860},
{0.026704,0.047855,-0.001853,-0.062451,0.047843,0.026708,-0.062435,-0.001860},
{0.026652,0.048278,-0.002163,-0.062478,0.048267,0.026656,-0.062459,-0.002171},
{0.026652,0.048278,-0.002163,-0.062478,0.048267,0.026656,-0.062459,-0.002171},
{0.026600,0.048699,-0.002469,-0.062512,0.048687,0.026604,-0.062478,-0.002486},
{0.026600,0.048699,-0.002469,-0.062512,0.048687,0.026604,-0.062478,-0.002486},
{0.038794,0.067164,-0.000007,-0.078504,0.067151,0.038790,-0.078449,-0.000014},
{0.720008,1.246191,0.062036,-1.420027,1.246233,0.719656,-1.420017,0.062089},
{0.720008,1.246191,0.062036,-1.420027,1.246233,0.719656,-1.420017,0.062089},
{0.672793,0.688173,0.023033,-0.889000,0.687832,0.673183,-0.889012,0.023028},
{0.672793,0.688173,0.023033,-0.889000,0.687832,0.673183,-0.889012,0.023028},
{0.808753,0.657883,0.032837,-0.677691,0.658156,0.808478,-0.677706,0.032841},
{0.808753,0.657883,0.032837,-0.677691,0.658156,0.808478,-0.677706,0.032841},
{0.768471,0.612266,0.083231,-0.466071,0.612906,0.767824,-0.466087,0.083232},
{0.768471,0.612266,0.083231,-0.466071,0.612906,0.767824,-0.466087,0.083232},
{0.733845,0.400226,0.116494,-0.258886,0.400231,0.733860,-0.258892,0.116495},
{0.733845,0.400226,0.116494,-0.258886,0.400231,0.733860,-0.258892,0.116495},
{0.654835,0.223808,0.101620,-0.069872,0.223814,0.654850,-0.069878,0.101630},
{0.654835,0.223808,0.101620,-0.069872,0.223814,0.654850,-0.069878,0.101630},
{0.566301,0.131259,0.040606,0.100100,0.131264,0.566326,0.100083,0.040590},
{0.566301,0.131259,0.040606,0.100100,0.131264,0.566326,0.100083,0.040590},
{0.494659,0.082267,-0.038315,0.264566,0.082266,0.494676,0.264637,-0.038243},
{0.494659,0.082267,-0.038315,0.264566,0.082266,0.494676,0.264637,-0.038243},
{0.517614,0.075204,-0.073363,0.300386,0.075187,0.517576,0.300268,-0.073416},
{0.517614,0.075204,-0.073363,0.300386,0.075187,0.517576,0.300268,-0.073416},
{0.514338,0.061061,-0.118291,0.364531,0.061144,0.514716,0.364492,-0.118234},
{0.514338,0.061061,-0.118291,0.364531,0.061144,0.514716,0.364492,-0.118234},
{-0.128372,-0.077703,-0.145700,0.559950,-0.078177,-0.130646,0.560515,-0.145008},
{-0.018186,-0.038603,0.004242,0.020523,-0.038418,-0.018120,0.020358,0.004064},
{-0.018186,-0.038603,0.004242,0.020523,-0.038418,-0.018120,0.020358,0.004064},
{-0.014615,-0.033432,0.001341,0.016587,-0.033425,-0.014630,0.016591,0.001326},
{-0.014615,-0.033432,0.001341,0.016587,-0.033425,-0.014630,0.016591,0.001326},
{-0.014551,-0.033329,0.001086,0.016905,-0.033317,-0.014557,0.016899,0.001066},
{-0.014551,-0.033329,0.001086,0.016905,-0.033317,-0.014557,0.016899,0.001066},
{-0.014483,-0.033222,0.000825,0.017213,-0.033216,-0.014488,0.017212,0.000813},
{-0.014483,-0.033222,0.000825,0.017213,-0.033216,-0.014488,0.017212,0.000813},
{-0.014417,-0.033117,0.000566,0.017522,-0.033114,-0.014419,0.017521,0.000557},
{-0.014417,-0.033117,0.000566,0.017522,-0.033114,-0.014419,0.017521,0.000557},
{-0.014350,-0.033012,0.000308,0.017829,-0.033012,-0.014351,0.017829,0.000302},
{-0.014350,-0.033012,0.000308,0.017829,-0.033012,-0.014351,0.017829,0.000302},
{-0.014285,-0.032907,0.000050,0.018135,-0.032909,-0.014284,0.018135,0.000047},
{-0.014285,-0.032907,0.000050,0.018135,-0.032909,-0.014284,0.018135,0.000047},
{-0.014219,-0.032802,-0.000207,0.018440,-0.032806,-0.014217,0.018440,-0.000210},
{-0.014219,-0.032802,-0.000207,0.018440,-0.032806,-0.014217,0.018440,-0.000210},
{-0.014154,-0.032697,-0.000466,0.018745,-0.032702,-0.014152,0.018744,-0.000468},
{-0.014154,-0.032697,-0.000466,0.018745,-0.032702,-0.014152,0.018744,-0.000468},
{-0.014090,-0.032591,-0.000726,0.019048,-0.032600,-0.014087,0.019048,-0.000727},
{-0.014090,-0.032591,-0.000726,0.019048,-0.032600,-0.014087,0.019048,-0.000727},
{-0.017212,-0.037102,0.001438,0.024521,-0.037113,-0.017212,0.024494,0.001431},
{-0.509879,-1.394311,-0.199851,0.703856,-1.394333,-0.509879,0.705141,-0.201098},
{-0.509879,-1.394311,-0.199851,0.703856,-1.394333,-0.509879,0.705141,-0.201098},
{-0.310862,-0.905084,-0.007363,0.626698,-0.905090,-0.310859,0.627072,-0.007733},
{-0.310862,-0.905084,-0.007363,0.626698,-0.905090,-0.310859,0.627072,-0.007733},
{-0.226209,-0.746817,0.198092,0.770727,-0.746819,-0.226212,0.770827,0.198057},
{-0.226209,-0.746817,0.198092,0.770727,-0.746819,-0.226212,0.770827,0.198057},
{-0.125439,-0.529108,0.415374,0.902617,-0.529112,-0.125454,0.903282,0.414847},
{-0.125439,-0.529108,0.415374,0.902617,-0.529112,-0.125454,0.903282,0.414847},
{-0.035860,-0.313469,0.531773,0.951710,-0.313485,-0.035874,0.952999,0.530645},
{-0.035860,-0.313469,0.531773,0.951710,-0.313485,-0.035874,0.952999,0.530645},
{0.128003,0.158432,0.778171,0.981293,0.158431,0.127984,0.982132,0.777497},
{0.128003,0.158432,0.778171,0.981293,0.158431,0.127984,0.982132,0.777497},
{0.161472,0.275712,0.842845,0.952044,0.275718,0.161460,0.953277,0.841940},
{0.161472,0.275712,0.842845,0.952044,0.275718,0.161460,0.953277,0.841940},
{0.188544,0.431184,0.811423,0.603679,0.431185,0.188535,0.603676,0.811563},
{0.188544,0.431184,0.811423,0.603679,0.431185,0.188535,0.603676,0.811563},
{0.198272,0.597502,0.680532,0.316573,0.597507,0.198243,0.316438,0.680402},
{0.198272,0.597502,0.680532,0.316573,0.597507,0.198243,0.316438,0.680402},
{0.213709,0.789726,0.534221,0.169805,0.789683,0.213613,0.169788,0.534262},
{0.213709,0.789726,0.534221,0.169805,0.789683,0.213613,0.169788,0.534262},
{0.411505,1.141278,-0.220036,-0.168463,1.141519,0.411791,-0.167959,-0.223169}};

	
float times[TIMESTEPS] = {0.000000,0.000151,0.000303,0.000454,0.000605,0.000757,0.000908,0.001059,0.001211,0.001362,0.001513,0.001665,0.001816,0.001967,0.002119,0.002270,0.002421,0.002573,0.002724,0.002875,0.003027,0.003027,0.007150,0.011274,0.015397,0.019521,0.023645,0.027768,0.031892,0.036015,0.040139,0.044262,0.048386,0.052510,0.056633,0.060757,0.064880,0.069004,0.073127,0.077251,0.081374,0.085498,0.085498,0.085651,0.085804,0.085957,0.086110,0.086263,0.086416,0.086569,0.086722,0.086875,0.087028,0.087181,0.087334,0.087487,0.087640,0.087793,0.087946,0.088099,0.088252,0.088405,0.088558,0.088558,0.093369,0.098180,0.102991,0.107802,0.112613,0.117424,0.122235,0.127046,0.131857,0.136669,0.141480,0.146291,0.151102,0.155913,0.160724,0.165535,0.170346,0.175157,0.179968,0.184779};

class TrajBound : public ReorientableBehavior
{
public:	

	Interpolator interp;

	TBMode mode = TB_SIT; //Current state within state-machine

	uint32_t tLast; //int used to store system time at various events

	float extDes;				 //The desired leg extension
	float angDes;				 // The desired leg angle
	float t;
	float test1;
	float test2;

	bool boolSlow = false;

	float posDes[MOTORS];
	float velDes[MOTORS];
	float uDes[MOTORS];
	float df[MOTORS];
	float posAct;
	float velAct;
	float kp = 0.8;
	float kd = 0.025; // 0.02
	float kpy = 0.1;//0.1;
	float kdy = 0;//0.05;
	float kiy = 0.02;
	float V = 16;
	float R = 0.23;
	float kt = 0.0954;
	float yawErrorInt = 0;

	float yawInit;
	float yawDes;
	float turnLeft;

	float finalTime = times[TIMESTEPS-1];


	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		if(sig > 1)
		{
			tLast = S->millis;
		}
			
	}

	void begin()
	{
		mode = TB_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		yawInit = S->imu.euler.z;
		yawErrorInt = 0;
	}

	void update()
	{
		if (isReorienting())
			return;

		for (int j = 0; j < MOTORS; j++)
	    {
	    	posDes[j] = 0;
	    	velDes[j] = 0;
	    	uDes[j] = 0;
	    	df[j] = 0;
	    }

		
		
		if (mode == TB_SIT)
		{
			C->mode = RobotCommand_Mode_LIMB;
			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
				limb[i].setGain(ANGLE, 0.8, .03);
				limb[i].setPosition(ANGLE, angDes);

				limb[i].setGain(EXTENSION, 120, 3);
				// Set the leg extension to 0.14 m
				limb[i].setPosition(EXTENSION, 0.12);
			}
		}
		else if (mode == TB_STAND)
		{
			C->mode = RobotCommand_Mode_JOINT;		



			t = 0.00001*(100*(S->millis - tLast) % (int)(100000*finalTime));

		    int index = 0;
		    for (int j = 0; j<TIMESTEPS; j++)
		    {
		      if (times[j] <= t && t<times[j + 1]) 
		        {
		          index = j;
		        }
		    }

   		    interp.getMultipleCubicSplineInterp(pos,vel,times,t,posDes);
   		    interp.getMultipleLinearInterp(vel,times,t,velDes);
   		    interp.getMultipleZOH(u,times,t,uDes);

			for (int i = 0; i<MOTORS; ++i)
			{
				joint[i].setGain(0.8);

				yawDes = yawInit + 0;
				yawErrorInt = yawErrorInt + 0.001*(yawDes - S->imu.euler.z);
				// turnLeft = kpy*(yawDes - S->imu.euler.z) + kdy*(-S->imu.angular_velocity.z) + 0.1;
				turnLeft = -map(C->behavior.twist.angular.z, -1.0, 1.0, -0.1, 0.1) - 0.03;

				if(i==0 || i==2 || i==4 || i==6)
				{
					posDes[i] = posDes[i] - turnLeft;
				} else if(i==1 || i==3 || i==5 || i==7)
				{
					posDes[i] = posDes[i] + turnLeft;
				}


				// if(i==1 || i==3 || i==4 || i==6)
				// {
				// 	posDes[i] = posDes[i] - S->imu.euler.y;
				// } else if(i==0 || i==2 || i==5 || i==7)
				// {
				// 	posDes[i] = posDes[i] + S->imu.euler.y;
				// }

				posAct = joint[i].getPosition();
				velAct = joint[i].getVelocity();

				df[i] = 1/(0.95*V)*(uDes[i]*R/kt + kt*joint[i].getVelocity());
				joint[i].setOpenLoop(df[i] + kp*(posDes[i] - posAct) + kd*(velDes[i] - velAct));
			}
		}
	}

	bool running()
	{
		return false;
	}
	void end()
	{
		mode = TB_SIT;
	}
};

TrajBound trajBound;

void debug()
{
	// printf("loop: ");
	// printf("%d, ", trajBound.mode);
	// printf("pos command: %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f. ", 
	// 	trajBound.uDes[0],trajBound.uDes[1],trajBound.uDes[2],trajBound.uDes[3],trajBound.uDes[4],trajBound.uDes[5],trajBound.uDes[6],trajBound.uDes[7]);
	printf("Time: %4.3fs. ", trajBound.t);
	printf("%4.3f, %4.3f, %4.3f ", trajBound.yawInit,trajBound.turnLeft, trajBound.kiy*trajBound.yawErrorInt);
	// printf("test1, test2: %4.3f, %4.3f.", trajBound.test1,trajBound.test2);
	printf("\n");
}

int main(int argc, char *argv[])
{
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
#else
#error "Define robot type in preprocessor"
#endif


	// Uncomment to clear Bound and Walk behaviors
	// behaviors.clear();
	
	behaviors.push_back(&trajBound);
	if (!trajBound.boolSlow)
	{
		for (int i = 0; i<TIMESTEPS; ++i)
			{
				times[i] = 1*times[i];
			}
		trajBound.boolSlow = true;
	}

	setDebugRate(100);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);


	// Run
	return begin();
}
