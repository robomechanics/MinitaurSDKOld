
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>
#include "Interpolator.h"

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
// const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
// const float motZeros[8] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
const float motZeros[8] = {0.93, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie
// const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 5.500, 1.078, 6.252}; //RML Odie
#endif

#define TIMESTEPS 42
#define MOTORS 8

enum TAMode
{
	TA_SIT = 0,
	TA_STAND,
	TA_GO
};

char myData[32];
float* myData_buf = (float*)myData;

// globals for power monitoring values
float voltage, current;
float oldvoltage, oldcurrent;
float power_int = 0;

#pragma pack(push, 1)
// We receive the serial protocol version number (in case we add more fields later)
// our behaviorCmd, and a checksum.
struct SerialCommandPacket
{
	float voltage, current;
};

const char ALIGNMENT_WORD[2] = {'G', 'R'};

#pragma pack(pop)

// Ground speed matching and impact minimization - energyOptimalBoundInstantaneousSwitchGSMClearance.mat
float times[TIMESTEPS] = {0.000000,0.012198,0.024397,0.036595,0.048793,0.060992,0.073190,0.085389,0.097587,0.109785,0.121984,0.134182,0.146380,0.158579,0.170777,0.182975,0.195174,0.207372,0.219571,0.231769,0.243967,0.243967,0.245602,0.247237,0.248871,0.250506,0.252141,0.253776,0.255410,0.257045,0.258680,0.260314,0.261949,0.263584,0.265219,0.266853,0.268488,0.270123,0.271757,0.273392,0.275027,0.276662};
float pos[TIMESTEPS][MOTORS] = {{1.570850,1.570884,1.570891,1.570892,1.570884,1.570851,1.570892,1.570891},{1.583152,1.604076,1.539260,1.560796,1.604076,1.583153,1.560794,1.539259},{1.608851,1.678343,1.460238,1.533047,1.678344,1.608852,1.533042,1.460237},{1.628402,1.750194,1.362864,1.493154,1.750195,1.628403,1.493149,1.362862},{1.634229,1.802455,1.260067,1.442998,1.802457,1.634230,1.442992,1.260064},{1.626997,1.835671,1.154050,1.383084,1.835673,1.626999,1.383077,1.154046},{1.606294,1.850472,1.039784,1.307824,1.850474,1.606296,1.307817,1.039780},{1.571437,1.849530,0.897078,1.197664,1.849534,1.571439,1.197655,0.897072},{1.521766,1.830548,0.723499,1.043712,1.830552,1.521768,1.043700,0.723490},{1.457159,1.786788,0.548123,0.865221,1.786792,1.457160,0.865207,0.548111},{1.374824,1.712069,0.397659,0.688405,1.712073,1.374826,0.688390,0.397642},{1.270980,1.603962,0.275886,0.523021,1.603967,1.270982,0.523010,0.275863},{1.134863,1.450307,0.207226,0.406590,1.450313,1.134865,0.406588,0.207197},{0.933429,1.204963,0.244037,0.404199,1.204969,0.933431,0.404206,0.244011},{0.648925,0.849501,0.334278,0.472133,0.849508,0.648928,0.472139,0.334263},{0.333709,0.479438,0.300337,0.433737,0.479444,0.333712,0.433757,0.300316},{0.207562,0.392499,0.213261,0.386792,0.392498,0.207562,0.386825,0.213229},{0.421607,0.757798,0.320162,0.627340,0.757794,0.421601,0.627357,0.320134},{0.778756,1.274664,0.596990,1.082941,1.274663,0.778751,1.082942,0.596981},{1.101719,1.689736,0.919428,1.534230,1.689738,1.101712,1.534234,0.919423},{1.400456,2.032737,1.235792,1.915351,2.032738,1.400455,1.915353,1.235791},{1.400534,2.032804,1.235886,1.915444,2.032805,1.400533,1.915446,1.235885},{1.440661,2.076987,1.277030,1.961106,2.076988,1.440661,1.961105,1.277030},{1.482419,2.122784,1.318529,2.005969,2.122784,1.482418,2.005970,1.318529},{1.524947,2.169367,1.360496,2.050063,2.169367,1.524947,2.050063,1.360496},{1.567771,2.216277,1.402974,2.093539,2.216277,1.567771,2.093539,1.402974},{1.610681,2.263296,1.446066,2.136435,2.263296,1.610681,2.136434,1.446066},{1.653493,2.310215,1.489820,2.178913,2.310215,1.653493,2.178913,1.489820},{1.696033,2.356825,1.534357,2.221025,2.356825,1.696033,2.221025,1.534357},{1.738127,2.402895,1.579751,2.262942,2.402895,1.738127,2.262942,1.579751},{1.779594,2.448190,1.626148,2.304733,2.448190,1.779594,2.304732,1.626148},{1.820217,2.492418,1.673657,2.346585,2.492418,1.820217,2.346584,1.673657},{1.859758,2.535272,1.722461,2.388594,2.535272,1.859758,2.388593,1.722461},{1.897879,2.576350,1.772717,2.430981,2.576350,1.897879,2.430981,1.772717},{1.934192,2.615218,1.824666,2.473886,2.615218,1.934192,2.473885,1.824666},{1.968144,2.651305,1.878543,2.517593,2.651305,1.968144,2.517593,1.878543},{1.999080,2.683968,1.934683,2.562319,2.683968,1.999080,2.562318,1.934683},{2.026212,2.712444,1.993461,2.608471,2.712444,2.026212,2.608471,1.993462},{2.048675,2.735891,2.055389,2.656418,2.735891,2.048675,2.656417,2.055389},{2.065587,2.753452,2.121124,2.706826,2.753451,2.065587,2.706827,2.121126},{2.075962,2.764089,2.191650,2.760446,2.764088,2.075961,2.760445,2.191652},{2.079403,2.767564,2.268038,2.818472,2.767561,2.079398,2.818478,2.268048}};
float vel[TIMESTEPS][MOTORS] = {{0.000006,0.000052,0.000094,0.000083,0.000052,0.000006,0.000083,0.000094},{1.788309,4.924792,-4.863698,-1.608745,4.924824,1.788321,-1.608774,-4.863738},{2.173429,6.706109,-7.738005,-2.862385,6.706156,2.173447,-2.862436,-7.738069},{1.028529,5.070382,-8.204030,-3.673417,5.070426,1.028546,-3.673472,-8.204096},{-0.065946,3.512914,-8.651511,-4.566989,3.512956,-0.065929,-4.567051,-8.651580},{-1.140082,1.940606,-8.867725,-5.386913,1.940646,-1.140067,-5.386982,-8.867796},{-2.262802,0.510913,-10.029550,-7.106524,0.510950,-2.262789,-7.106625,-10.029641},{-3.465163,-0.750247,-13.152588,-10.877031,-0.750212,-3.465155,-10.877215,-13.152750},{-4.681630,-2.431894,-15.123734,-14.318807,-2.431861,-4.681624,-14.319033,-15.123993},{-5.973624,-4.809615,-13.490614,-14.753064,-4.809586,-5.973621,-14.753214,-13.490950},{-7.573179,-7.489295,-11.068597,-14.073040,-7.489263,-7.573174,-14.072976,-11.069021},{-9.645995,-10.491437,-8.351777,-12.298262,-10.491397,-9.645984,-12.297714,-8.352323},{-12.852056,-14.941344,-2.381121,-6.065375,-14.941302,-12.852046,-6.064548,-2.381252},{-20.045584,-24.964205,6.806706,4.173417,-24.964173,-20.045579,4.173753,6.807393},{-26.460513,-32.978476,6.378093,5.467530,-32.978354,-26.460378,5.467574,6.378709},{-21.648456,-23.216335,-8.462845,-7.641186,-23.216717,-21.648484,-7.639513,-8.463994},{4.517934,13.431222,-2.311193,4.085860,13.430720,4.517420,4.085724,-2.311285},{26.995059,41.307485,17.776534,31.944973,41.307591,26.994820,31.943041,17.777902},{27.970314,38.276442,25.555847,39.318920,38.276611,27.970501,39.318794,25.556198},{25.228155,30.422932,26.746846,34.392166,30.422963,25.228309,34.392268,26.747103},{23.996124,26.452481,24.537598,27.808474,26.452429,23.996275,27.808497,24.537798},{23.876495,26.390822,25.071225,28.230867,26.390822,23.876495,28.230868,25.071224},{25.116232,27.605634,25.269094,27.701520,27.605634,25.116232,27.701520,25.269094},{25.872732,28.360985,25.515878,27.222898,28.360985,25.872732,27.222898,25.515878},{26.117665,28.626050,25.819850,26.795156,28.626050,26.117665,26.795156,25.819850},{26.234721,28.757706,26.163389,26.427350,28.757706,26.234721,26.427350,26.163389},{26.227009,28.761740,26.552244,26.121478,28.761741,26.227009,26.121478,26.552244},{26.114401,28.634709,26.993393,25.880484,28.634709,26.114401,25.880484,26.993393},{25.894645,28.379677,27.493310,25.707609,28.379677,25.894646,25.707609,27.493310},{25.566214,27.975157,28.061196,25.607386,27.975157,25.566214,25.607386,28.061196},{25.123401,27.421629,28.705604,25.585587,27.421629,25.123401,25.585587,28.705604},{24.531582,26.672419,29.439786,25.649835,26.672419,24.531582,25.649835,29.439786},{23.784891,25.724806,30.276567,25.810386,25.724806,23.784891,25.810386,30.276567},{22.792289,24.502224,31.235378,26.080589,24.502223,22.792289,26.080589,31.235377},{21.548948,22.998688,32.337210,26.478767,22.998688,21.548949,26.478767,32.337210},{19.903094,21.099390,33.613716,27.029526,21.099389,19.903094,27.029526,33.613716},{17.838908,18.790971,35.102470,27.766702,18.790972,17.838908,27.766703,35.102470},{15.253226,15.973865,36.861918,28.740689,15.973864,15.253225,28.740689,36.861918},{12.124634,12.635264,38.967979,30.019826,12.635266,12.124636,30.019826,38.967979},{8.469987,8.768333,41.544209,31.718560,8.768330,8.469984,31.718559,41.544209},{4.215947,4.285519,44.832557,34.027729,4.285523,4.215951,34.027730,44.832559},{0.000001,0.000000,48.746680,37.080712,-0.000001,-0.000002,37.080708,48.746683}};
float u[TIMESTEPS][MOTORS] = {{0.604008,2.938430,-0.084521,-0.067627,2.938404,0.603982,-0.067602,-0.084516},{0.604008,2.938430,-0.084521,-0.067627,2.938404,0.603982,-0.067602,-0.084516},{-0.077286,-0.049462,0.114000,0.865289,-0.049474,-0.077288,0.865350,0.113999},{-0.077286,-0.049462,0.114000,0.865289,-0.049474,-0.077288,0.865350,0.113999},{-0.078839,-0.057958,0.095042,0.908330,-0.057933,-0.078833,0.908353,0.095030},{-0.078839,-0.057958,0.095042,0.908330,-0.057933,-0.078833,0.908353,0.095030},{-0.081572,-0.065182,-0.073974,0.120629,-0.065225,-0.081584,0.120663,-0.073975},{-0.081572,-0.065182,-0.073974,0.120629,-0.065225,-0.081584,0.120663,-0.073975},{-0.084702,-0.082726,-0.122236,1.246927,-0.082721,-0.084701,1.247004,-0.122298},{-0.084702,-0.082726,-0.122236,1.246927,-0.082721,-0.084701,1.247004,-0.122298},{-0.098201,-0.111440,-0.284627,0.833814,-0.111621,-0.098250,0.833987,-0.284713},{-0.098201,-0.111440,-0.284627,0.833814,-0.111621,-0.098250,0.833987,-0.284713},{-0.153177,-0.223767,-0.179631,0.893484,-0.223809,-0.153187,0.893609,-0.179661},{-0.153177,-0.223767,-0.179631,0.893484,-0.223809,-0.153187,0.893609,-0.179661},{-0.110560,2.113759,-0.480674,0.188332,2.113699,-0.110580,0.188489,-0.480746},{-0.110560,2.113759,-0.480674,0.188332,2.113699,-0.110580,0.188489,-0.480746},{0.544606,2.998757,0.302950,2.780940,2.998795,0.544551,2.781284,0.302856},{0.544606,2.998757,0.302950,2.780940,2.998795,0.544551,2.781284,0.302856},{2.528399,2.998617,2.676587,2.999414,2.998699,2.528537,2.999431,2.676627},{2.528399,2.998617,2.676587,2.999414,2.998699,2.528537,2.999431,2.676627},{2.927861,2.994573,2.998208,2.829190,2.994785,2.928063,2.829305,2.998235},{0.611122,0.882574,2.992800,2.778608,0.882585,0.611113,2.778748,2.992847},{0.611122,0.882574,2.992800,2.778608,0.882585,0.611113,2.778748,2.992847},{0.048830,0.447616,2.993660,2.698379,0.447624,0.048827,2.698522,2.993717},{0.048830,0.447616,2.993660,2.698379,0.447624,0.048827,2.698522,2.993717},{-0.170058,0.296123,2.993365,2.621276,0.296131,-0.170059,2.621423,2.993433},{-0.170058,0.296123,2.993365,2.621276,0.296131,-0.170059,2.621423,2.993433},{-0.361175,0.128359,2.993132,2.545118,0.128365,-0.361174,2.545269,2.993212},{-0.361175,0.128359,2.993132,2.545118,0.128365,-0.361174,2.545269,2.993212},{-0.531544,-0.067413,2.992977,2.464717,-0.067408,-0.531541,2.464871,2.993065},{-0.531544,-0.067413,2.992977,2.464717,-0.067408,-0.531541,2.464871,2.993065},{-0.703159,-0.302890,2.992916,2.373775,-0.302888,-0.703154,2.373928,2.993010},{-0.703159,-0.302890,2.992916,2.373775,-0.302888,-0.703154,2.373928,2.993010},{-0.936345,-0.571448,2.992975,2.262794,-0.571450,-0.936337,2.262940,2.993066},{-0.936345,-0.571448,2.992975,2.262794,-0.571450,-0.936337,2.262940,2.993066},{-1.296369,-0.866545,2.993190,2.115999,-0.866552,-1.296356,2.116129,2.993268},{-1.296369,-0.866545,2.993190,2.115999,-0.866552,-1.296356,2.116129,2.993268},{-1.707186,-1.203427,2.993638,1.904265,-1.203443,-1.707167,1.904369,2.993691},{-1.707186,-1.203427,2.993638,1.904265,-1.203443,-1.707167,1.904369,2.993691},{-2.055898,-1.569667,2.994502,1.559784,-1.569695,-2.055873,1.559856,2.994521},{-2.055898,-1.569667,2.994502,1.559784,-1.569695,-2.055873,1.559856,2.994521},{-1.996268,-1.426926,1.387126,0.190282,-1.426998,-1.996298,0.190298,1.387088}};
float dfVec[TIMESTEPS][MOTORS] = {{0.117911,0.573625,-0.016499,-0.013201,0.573620,0.117906,-0.013196,-0.016498},{0.131725,0.611667,-0.054070,-0.025629,0.611662,0.131720,-0.025624,-0.054070},{0.001702,0.042147,-0.037519,0.146806,0.042145,0.001701,0.146818,-0.037520},{-0.007142,0.029511,-0.041119,0.140541,0.029510,-0.007143,0.140553,-0.041120},{-0.015900,0.015822,-0.048277,0.142041,0.015827,-0.015899,0.142045,-0.048280},{-0.024197,0.003676,-0.049947,0.135707,0.003682,-0.024196,0.135711,-0.049950},{-0.033404,-0.008778,-0.091916,-0.031347,-0.008786,-0.033406,-0.031341,-0.091917},{-0.042691,-0.018520,-0.116040,-0.060473,-0.018528,-0.042694,-0.060468,-0.116042},{-0.052699,-0.034935,-0.140688,0.132810,-0.034934,-0.052699,0.132823,-0.140703},{-0.062680,-0.053302,-0.128073,0.129456,-0.053301,-0.062679,0.129469,-0.128088},{-0.077671,-0.079607,-0.141065,0.054063,-0.079642,-0.077680,0.054097,-0.141085},{-0.093683,-0.102798,-0.120078,0.067772,-0.102833,-0.093692,0.067810,-0.120099},{-0.129181,-0.159100,-0.053460,0.127568,-0.159108,-0.129183,0.127599,-0.053467},{-0.184748,-0.236524,0.017513,0.206660,-0.236531,-0.184750,0.206687,0.017513},{-0.225982,0.157888,-0.044566,0.079000,0.157877,-0.225985,0.079031,-0.044575},{-0.188811,0.233298,-0.159208,-0.022261,0.233283,-0.188815,-0.022217,-0.159230},{0.141215,0.689154,0.041287,0.574442,0.689157,0.141200,0.574508,0.041268},{0.314844,0.904489,0.196459,0.789646,0.904497,0.314831,0.789698,0.196451},{0.709643,0.881048,0.719920,0.889256,0.881065,0.709671,0.889259,0.719931},{0.688461,0.820382,0.729120,0.851199,0.820398,0.688489,0.851203,0.729130},{0.756924,0.788922,0.774840,0.767112,0.788963,0.756965,0.767134,0.774847},{0.303739,0.376153,0.777906,0.760500,0.376155,0.303737,0.760527,0.777915},{0.313315,0.385537,0.779435,0.756411,0.385539,0.313313,0.756438,0.779444},{0.209391,0.306461,0.781509,0.737052,0.306463,0.209391,0.737080,0.781520},{0.211283,0.308509,0.783857,0.733748,0.308510,0.211283,0.733775,0.783868},{0.169457,0.279952,0.786453,0.715855,0.279954,0.169457,0.715884,0.786466},{0.169398,0.279983,0.789457,0.713492,0.279985,0.169398,0.713521,0.789470},{0.131219,0.246252,0.792819,0.696763,0.246253,0.131219,0.696793,0.792835},{0.129522,0.244282,0.796681,0.695428,0.244283,0.129522,0.695457,0.796696},{0.093726,0.202940,0.801037,0.678958,0.202941,0.093727,0.678988,0.801055},{0.090306,0.198664,0.806015,0.678790,0.198665,0.090306,0.678820,0.806032},{0.052232,0.146908,0.811675,0.661533,0.146908,0.052233,0.661563,0.811693},{0.046464,0.139588,0.818139,0.662773,0.139588,0.046465,0.662803,0.818157},{-0.006725,0.077717,0.825557,0.643195,0.077717,-0.006723,0.643224,0.825574},{-0.016329,0.066103,0.834068,0.646271,0.066103,-0.016328,0.646300,0.834086},{-0.099325,-0.006176,0.843970,0.621869,-0.006177,-0.099322,0.621894,0.843986},{-0.115270,-0.024008,0.855471,0.627563,-0.024009,-0.115268,0.627589,0.855486},{-0.215441,-0.111533,0.869149,0.593754,-0.111537,-0.215438,0.593774,0.869160},{-0.239609,-0.137323,0.885418,0.603635,-0.137326,-0.239605,0.603655,0.885428},{-0.335913,-0.238689,0.905487,0.549509,-0.238695,-0.335909,0.549523,0.905491},{-0.368775,-0.273318,0.930889,0.567347,-0.273323,-0.368770,0.567361,0.930892},{-0.389701,-0.278557,0.647341,0.323583,-0.278571,-0.389707,0.323586,0.647333}};


class TrajAccelerate : public ReorientableBehavior
{
public:
	Interpolator interp;

	TAMode mode = TA_SIT; //Current state within state-machine
	int logging = 0;

	uint32_t tLast; //int used to store system time at various events

	float extDes; //The desired leg extension
	float angDes; // The desired leg angle
	float t;
	uint32_t tOld;
	uint32_t tNew;
	uint32_t timeStep;
	uint32_t iter = 0;

	bool boolSlow = false;

	float posDes[MOTORS];
	float velDes[MOTORS];
	float uDes[MOTORS];
	float df[MOTORS];
	float dfDes[MOTORS];
	float posAct;
	float velAct;
	float kp = 0.8; // 0.80
	float kd = 0.020; // 0.02
	float kpy = 0.1;	//0.1;
	float kdy = 0;		//0.05;
	float kiy = 0.02;
	float V = 15;
	float R = 0.22;
	float kt = 0.0954;
	float yawErrorInt = 0;

	float yawInit;
	float yawDes;
	float turnLeft;

	float finalTime = times[TIMESTEPS - 1];

	// Parser state (for serial comms with raspi)
	// Goes from 0 to 1 to 2
	int numAlignmentSeen = 0;
	uint16_t rxPtr = 0;

	// Receive buffer and alignment
	const static uint16_t RX_BUF_SIZE = 100;
	SerialCommandPacket packet;

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		// tLast = S->millis;
		if (logging == 0) power_int = 0;
		// logging = 1;
		if (mode == TA_STAND)
		{
			mode = TA_GO;			// Start behavior in STAND mode
			tLast = S->millis;
		}	
	}

	void begin()
	{
		mode = TA_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition
		yawInit = S->imu.euler.z;
		yawErrorInt = 0;
		power_int = 0;
		// ioctl(LOGGER_FILENO, 0);//stop
	}

	void update()
	{
		tOld = tNew;
		tNew = clockTimeUS;
		timeStep = tNew - tOld;
		// if (tNew!=clockTimeUS)
		// {
		// 	tOld = tNew;
		// 	tNew = clockTimeUS;
		// 	timeStep = tNew - tOld;
		// }


		// Serial comms code
		oldvoltage = voltage;
		oldcurrent = current;

		// Character to store the latest received character
		uint8_t latestRX;

		// Loop through while there are new bytes available
		while (read(SERIAL_AUX_FILENO, &latestRX, 1) > 0)
		{
			if (numAlignmentSeen == 0 && latestRX == ALIGNMENT_WORD[0])
			{
				numAlignmentSeen = 1;
			}
			else if (numAlignmentSeen == 1 && latestRX == ALIGNMENT_WORD[1])
			{
				numAlignmentSeen = 2;
				rxPtr = 0;
			}
			else if (numAlignmentSeen == 2)
			{
				// Add the next byte to our memory space in serial_packet
				uint8_t *pSerialPacket = (uint8_t *)&packet;
				pSerialPacket[rxPtr++] = latestRX; // post-increment rxPtr

				// Check if we have read a whole packet
				if (rxPtr == sizeof(SerialCommandPacket))
				{
					// Copy voltage and current readings
					voltage = packet.voltage;
					current = packet.current;

					// Reset
					numAlignmentSeen = rxPtr = 0;
				}
			}
		}

		if (isReorienting())
			return;

		for (int j = 0; j < MOTORS; j++)
		{
			posDes[j] = 0;
			velDes[j] = 0;
			uDes[j] = 0;
			df[j] = 0;
		}

		if (mode == TA_SIT)
		{


			C->mode = RobotCommand_Mode_JOINT;
			for (int i = 0; i < P->joints_count; i++)
			{
				posDes[i] = 0.5;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				if(i==1 || i==3 || i==4 || i==6)
				{
					posDes[i] = posDes[i] - S->imu.euler.y;
				} else if(i==0 || i==2 || i==5 || i==7)
				{
					posDes[i] = posDes[i] + S->imu.euler.y;
				}

				joint[i].setGain(0.8, .003);
				joint[i].setPosition(posDes[i]);
			}
		}
		else if (mode == TA_STAND)
		{


			C->mode = RobotCommand_Mode_JOINT;
			for (int i = 0; i < P->joints_count; i++)
			{
				posDes[i] = 0.5;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
				// and aft-displacement of rear legs)
				// The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
				// and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body
				if(i==1 || i==3 || i==4 || i==6)
				{
					posDes[i] = pos[0][i] - S->imu.euler.y;
				} else if(i==0 || i==2 || i==5 || i==7)
				{
					posDes[i] = pos[0][i] + S->imu.euler.y;
				}

				joint[i].setGain(0.8, .003);
				joint[i].setPosition(posDes[i]);
			}
		}
		else if (mode == TA_GO)
		{
			if (voltage<10)
			{
				voltage = oldvoltage;
				current = oldcurrent;
			}
			power_int = power_int + 0.000001*timeStep*(voltage + oldvoltage)*0.5*(current + oldcurrent)*0.5;

			C->mode = RobotCommand_Mode_JOINT;

			// t = 0.00001 * (100 * (S->millis - tLast) % (int)(100000 * finalTime));
			t = 0.001*(S->millis - tLast);

			int index = 0;
			for (int j = 0; j < TIMESTEPS; j++)
			{
				if (times[j] <= t && t < times[j + 1])
				{
					index = j;
				}
			}

			interp.getMultipleCubicSplineInterp(pos, vel, times, t, posDes);
			interp.getMultipleLinearInterp(vel, times, t, velDes);
			interp.getMultipleZOH(u, times, t, uDes);
			// interp.getMultipleZOH(dfVec, times, t, dfDes);

			for (int i = 0; i < MOTORS; ++i)
			{

				// int ii;
				// if (i==0)
				// {
				// 	ii = 1;
				// } else if (i == 1) {
				// 	ii = 0;
				// } else {
				// 	ii = i;
				// }
				// joint[i].setGain(0.8);

				// yawDes = yawInit + 0;
				// yawErrorInt = yawErrorInt + 0.001 * (yawDes - S->imu.euler.z);
				// turnLeft = kpy*(yawDes - S->imu.euler.z) + kdy*(-S->imu.angular_velocity.z) + 0.1;
				turnLeft = -map(C->behavior.twist.angular.z, -1.0, 1.0, -0.1, 0.1) - 0.03;

				if (i == 0 || i == 2 || i == 4 || i == 6)
				{
					posDes[i] = posDes[i] - turnLeft;
				}
				else if (i == 1 || i == 3 || i == 5 || i == 7)
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

				df[i] = 1 / (0.95 * S->batt.voltage) * (uDes[i] * R / kt + kt * joint[i].getVelocity());
				joint[i].setOpenLoop(df[i] + kp * (posDes[i] - posAct) + kd * (velDes[i] - velAct));
				// joint[i].setOpenLoop(dfDes[i] + kp * (posDes[i] - posAct) + kd * (velDes[i] - velAct));

				if (t>=times[TIMESTEPS-1])
				{
					mode = TA_SIT;
					tLast = S->millis;
					t = 0;
				}
			}
		}
	}

	bool running()
	{
		return false;
	}
	void end()
	{
		mode = TA_SIT;
		logging = 0;
	}
};

TrajAccelerate trajAccelerate;

void debug()
{
	// printf("loop: ");
	printf("%d, ", trajAccelerate.mode);
	// printf("pos command: %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f, %4.3f. ",
	// 	trajAccelerate.uDes[0],trajAccelerate.uDes[1],trajAccelerate.uDes[2],trajAccelerate.uDes[3],trajAccelerate.uDes[4],trajAccelerate.uDes[5],trajAccelerate.uDes[6],trajAccelerate.uDes[7]);
	printf("Time: %4.3fs. ", trajAccelerate.t);
	// printf("%4.3f, %4.3f, %4.3f ", trajAccelerate.yawInit, trajAccelerate.turnLeft, trajAccelerate.kiy * trajAccelerate.yawErrorInt);
	// printf("test1, test2: %4.3f, %4.3f.", trajAccelerate.test1,trajAccelerate.test2);
	// printf("Voltage: %6.3f, Current: %6.3f", voltage,  current);

	// for (int i = 0; i < P->joints_count; i++)
	// {
	// 	printf("Motor %d, Pos: %4.3f, ", i, joint[i].getRawPosition());
	// }


	// printf("Power Int: %6.2f, Voltage: %5.2f, Current: %5.2f", power_int, voltage, current);
	printf("\n");

	// myData_buf[0] = trajAccelerate.logging;
	// myData_buf[1] = voltage;
	// myData_buf[2] = current;
	// myData_buf[3] = power_int;
	// write(LOGGER_FILENO, myData, 32);
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


	// Uncomment to clear Accelerate and Walk behaviors
	// behaviors.clear();
	
	behaviors.push_back(&trajAccelerate);

	setDebugRate(100);
	safetyShutoffEnable(false);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);


	// Run
	return begin();
}