
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Motor.h>
#include <ReorientableBehavior.h>


#if defined(ROBOT_MINITAUR)

const float motZeros[8] = { 5.5200, 5.34, 5.95, 3.81, 3.84, 4.07, 5.71, 5.38 }; // RML Ellie past

#endif

#define TIMESTEPS 42
#define MOTORS 8

enum TBMode
{
	TB_SIT = 0,
	TB_STAND,
};




#pragma pack(push, 1)
// We receive the serial protocol version number (in case we add more fields later)
// our behaviorCmd, and a checksum.

const char ALIGNMENT_WORD[2] = { 'G', 'R' };

#pragma pack(pop)

float times[TIMESTEPS] = { 0,0.02501377200,0.05002311600,0.07503688800,0.1000506600,0.1250600040,0.1500737760,0.1750831200,0.2000968920,0.2251106640,0.2501200080,0.2751337800,0.3001475520,0.3251568960,0.3501706680,0.3751844400,0.4001937840,0.4252075560,0.4502169000,0.4752306720,0.5002444440,0.5002444440,0.5252316480,0.5502188520,0.5752060560,0.6001932600,0.6251804640,0.6501720960,0.6751593000,0.7001465040,0.7251337080,0.7501209120,0.7751081160,0.8000953200,0.8250869520,0.8500741560,0.8750613600,0.9000485640,0.9250357680,0.9500229720,0.9750101760,0.9999973800 };
float pos[TIMESTEPS][MOTORS] = { {1.8530109,1.4029685,1.1965212,2.0924773,2.1024756,1.2020218,1.4094725,1.8485086},{1.8332518,1.4514959,1.1552540,2.1152489,2.1247492,1.1617538,1.4539962,1.8340013},{1.8029898,1.4900101,1.0859729,2.0329387,2.0409369,1.0899719,1.4950101,1.8109906},{1.7689748,1.5255315,0.99166918,1.8075180,1.8087606,0.98990756,1.5295273,1.7784727},{1.7379549,1.5630522,0.88984692,1.5220842,1.5155755,0.88285732,1.5650556,1.7444496},{1.7016815,1.5983096,0.78657562,1.2337532,1.2232487,0.78182340,1.6050707,1.7029204},{1.6659218,1.6310668,0.74712461,1.0297652,1.0097479,0.73710722,1.6380782,1.6679246},{1.6324124,1.6603342,0.81254965,0.93962538,0.91159844,0.79578948,1.6708273,1.6359191},{1.6006277,1.6922889,0.94086188,0.90650117,0.87387586,0.91957068,1.6999141,1.6056275},{1.5715954,1.7263330,1.1154867,0.90065479,0.87685704,1.0869035,1.7303689,1.5752859},{1.5368438,1.7615104,1.3261770,0.93309373,0.91144788,1.2951770,1.7628646,1.5431354},{1.5034908,1.7944533,1.5497816,0.98845935,0.97190648,1.5252845,1.7985650,1.5074348},{1.4688025,1.8269712,1.7591481,1.0422633,1.0366359,1.7465205,1.8332777,1.4724154},{1.4335166,1.8633828,1.9052354,1.0872686,1.0965104,1.9147187,1.8703828,1.4362884},{1.4009987,1.9022323,1.9638090,1.1244245,1.1450788,1.9840014,1.9068090,1.4003832},{1.3654596,1.9333811,1.9667783,1.1661780,1.1869653,1.9824064,1.9414876,1.3646189},{1.3325685,1.9630138,1.9397774,1.2133564,1.2291030,1.9487774,1.9723871,1.3351952},{1.2986717,1.9934900,1.9089696,1.2620051,1.2716262,1.9184293,1.9995707,1.3064293},{1.2642803,2.0283017,1.8830899,1.3078202,1.3164656,1.8930900,2.0320106,1.2759259},{1.2316208,2.0649614,1.8656712,1.3538473,1.3645688,1.8705068,2.0679615,1.2384564},{1.1967686,2.0984516,1.8513286,1.4009000,1.4152344,1.8536628,2.1037858,1.2021029},{1.1967686,2.0984516,1.8513286,1.4009000,1.4152344,1.8536628,2.1037858,1.2021029},{1.1547644,2.1068153,1.8297019,1.4442668,1.4605453,1.8365058,2.1187840,1.1627643},{1.0814589,2.0101209,1.7997463,1.4836943,1.4974741,1.8097463,2.0272903,1.0882386},{0.98681748,1.7994115,1.7659335,1.5199258,1.5312068,1.7776526,1.8025188,0.99023896},{0.88717401,1.5186553,1.7297246,1.5532753,1.5702932,1.7467070,1.5077085,0.88610297},{0.78544497,1.2151645,1.6922857,1.5907142,1.6091977,1.7065109,1.2044832,0.77412081},{0.76446319,1.0423715,1.6591504,1.6243112,1.6403879,1.6711503,1.0102934,0.74507821},{0.82325935,0.94945586,1.6278851,1.6567721,1.6711149,1.6352849,0.91277015,0.79305893},{0.95697075,0.90852362,1.5991498,1.6885698,1.6989437,1.6059628,0.87524325,0.90384018},{1.1367205,0.90756524,1.5664907,1.7186772,1.7315094,1.5745466,0.87311804,1.0722796},{1.3352841,0.94743156,1.5336738,1.7561579,1.7655474,1.5417578,0.90421051,1.2848630},{1.5496137,0.99907571,1.5009160,1.7914202,1.8010840,1.5069158,0.95806730,1.5168238},{1.7622654,1.0481594,1.4638804,1.8230996,1.8365116,1.4714885,1.0216111,1.7449132},{1.9178925,1.0929284,1.4293393,1.8555534,1.8715534,1.4396073,1.0891427,1.9094284},{1.9711206,1.1376026,1.3912151,1.8923910,1.9077849,1.4062151,1.1369967,1.9742410},{1.9666110,1.1807778,1.3582973,1.9300165,1.9412551,1.3699837,1.1782255,1.9762681},{1.9405620,1.2232562,1.3295619,1.9614379,1.9720414,1.3345537,1.2226611,1.9491570},{1.9121329,1.2647345,1.2976737,1.9892244,2.0012753,1.3007247,1.2699385,1.9226738},{1.8910849,1.3084687,1.2653543,2.0204649,2.0329189,1.2690811,1.3169225,1.8914466},{1.8726034,1.3560894,1.2319582,2.0560420,2.0661426,1.2363074,1.3626397,1.8654554},{1.8555138,1.4049633,1.1957266,2.0922735,2.1026716,1.2024257,1.4074678,1.8487084} };

float vel[TIMESTEPS][MOTORS] = { {-1.57985768799683,3.88005455554644,-3.29955833930205,1.82072499901254,1.78090693398821,-3.21966635020100,3.55993490306061,-1.15994500949317},{-1.99991939726425,3.48005510092573,-4.41988859710379,-2.38044347337340,-2.46041050301624,-4.47992484114744,3.41992290124430,-1.50002650774494},{-2.56989188758253,2.96005550713794,-6.54036905657776,-12.3035478237701,-12.6337031863429,-6.87067315038911,3.01984786393555,-2.22011759523337},{-2.59996373197932,2.92007538886978,-7.84071990421917,-20.4229294166430,-21.0028859301988,-8.28002190153489,2.80027738319515,-2.66017456303671},{-2.69048813352611,2.90977875108780,-8.19995139846946,-22.9399863854943,-23.4096532491099,-8.31952011945838,3.02033963657921,-3.02069547206935},{-2.87999252185730,2.71932680083344,-5.70625428451918,-19.6836598503780,-20.2237541539795,-5.82730991807867,2.91955423168761,-3.05958549243514},{-2.76948361233635,2.47983752151705,1.03848108942274,-11.7596760665609,-12.4602497773229,0.558385047424879,2.62904853827978,-2.67881353092838},{-2.61055708724742,2.44775235513118,7.74590971102240,-4.92828275631610,-5.43237010665230,7.29516569899404,2.47229300949585,-2.49073248455774},{-2.43134062307756,2.63849850394414,12.1108103967686,-1.55796534804906,-1.38889088778774,11.6381495761615,2.38035271129840,-2.42399267091745},{-2.55017700216836,2.76758049218684,15.4054825373133,1.06321085635689,1.50218630922552,15.0173099972421,2.51685640694594,-2.49852888012814},{-2.72292513725055,2.72355284704776,17.3637683826014,3.51055939817903,3.80022068197431,17.5271368540896,2.72658344594127,-2.71278982300902},{-2.72015352182790,2.61699035235470,17.3093086480520,4.36437855114375,5.00476377573123,18.0438000314387,2.81497328751538,-2.82724252863582},{-2.79767457908860,2.75590588958912,14.2115817015477,3.95054358468993,4.98185358944853,15.5701695991909,2.87138450151726,-2.84454091184563},{-2.71089869731426,3.00905285468422,8.18265299586694,3.28492931148072,4.33571151385291,9.49484634263886,2.93989282874741,-2.87995653849312},{-2.72078117606572,2.79839042268396,2.46036063653255,3.15463817292329,3.61620390559248,2.70601730918471,2.84262605415929,-2.86520161773282},{-2.73594311877732,2.43013649929364,-0.960819793792927,3.55563216013972,3.35941487531485,-1.40830891062444,2.62191183771919,-2.60631504842681},{-2.67028147546826,2.40324493180313,-2.31127944928501,3.83131270750907,3.38487110639010,-2.55790143101042,2.32225037720561,-2.32650840863252},{-2.73026574354145,2.61030920184981,-2.26645217383100,3.77680590709304,3.49288916748009,-2.22646665993378,2.38383790406020,-2.36967645118309},{-2.68079661410937,2.85753490446297,-1.73113566136104,3.67199036541425,3.71598602534076,-1.91601418832045,2.73436784705695,-2.71765957162684},{-2.69898118524467,2.80445108398686,-1.26975251873248,3.72114209724147,3.94857680800800,-1.57621969209602,2.86942728989453,-2.95129419105603},{-2.78664089526362,2.67774088610068,-1.14677626389175,3.76214351038301,4.05101637609873,-1.34677808688750,2.86436607801493,-2.90667876879984},{-3.36205683517052,0.669438645476290,-1.73102200630370,3.47112065839780,3.62672830461544,-1.37326289087806,1.20047044879451,-3.14869963041884},{-4.61475001364699,-3.53503737352928,-2.06434861619571,3.31346796544343,3.29127260497014,-1.75755958930019,-3.06138694029151,-4.55690440595115},{-6.72131703891320,-12.3024488854375,-2.55204223729874,3.02790980535477,2.82790743614212,-2.35533355392625,-12.6570864031046,-6.90454762365570},{-7.77537534811818,-19.6686912229155,-2.80230233042481,2.78466530308873,2.91425563260299,-2.52286330235268,-20.7939151575343,-8.08956576334031},{-8.05902533152568,-23.3818477649600,-2.94742060776388,2.83298603557245,3.12123357219158,-2.84712527259952,-23.9336742118085,-8.64915298246253},{-4.91051132123204,-19.0594194710737,-2.82416341188898,2.84263923233426,2.80497528994072,-3.02354780731587,-19.9050294008448,-5.64337912951794},{1.51321571394740,-10.6328462711697,-2.57711484116997,2.64343491313003,2.47773677642273,-2.85024645231834,-11.6734631434794,0.757845580877477},{7.70424574114015,-5.35665695129396,-2.40125305736488,2.57166027859700,2.34343146195950,-2.60883530626317,-5.40477237869431,6.35373089362060},{12.5448669647072,-1.67648289100293,-2.45703360808197,2.47747206930395,2.41701712604579,-2.43077616847408,-1.58689663717477,11.1745463798191},{15.1402834026568,1.55711459353355,-2.62038121592155,2.70490848035659,2.66551231582373,-2.56951518065006,1.15928376780371,15.2487177036695},{16.5241857392288,3.66229330820687,-2.62433123770071,2.91121007376416,2.78440917199059,-2.70661735502699,3.39971050782632,17.7908740809896},{17.0879983210606,4.03117691759350,-2.79316565390829,2.67903923944431,2.84002163667451,-2.81221140228414,4.69842844361458,18.4114317072050},{14.7373900424571,3.75569731155803,-2.86428039260457,2.56641431184992,2.81996963674784,-2.69348009625514,5.24523620358025,15.7108340818501},{8.35774566658575,3.57924302198634,-2.90784283171382,2.77282968334837,2.85213925350323,-2.61204162497902,4.61737844394776,9.17699643905272},{1.94973795387432,3.51577551453937,-2.84313523033630,2.98004930843803,2.78949577551774,-2.78637017571074,3.56513677960927,2.67495715006769},{-1.22296996494686,3.42789853558646,-2.46739090936304,2.76329036253916,2.57157623558042,-2.86792391817828,3.42833075681457,-1.00387382277745},{-2.18023993400783,3.35998777614335,-2.42618581894957,2.36952881963104,2.40203745885294,-2.77177870721350,3.67039865684853,-2.14486983017387},{-1.98009749310086,3.41024550005675,-2.56961923390869,2.36228911406014,2.43634701985864,-2.62024514627568,3.77238685848965,-2.30959814471439},{-1.58198972562116,3.65606732149783,-2.62996612185982,2.67407269736943,2.59602074725928,-2.57801152942122,3.70994689922089,-2.28990806654478},{-1.42357264142079,3.86176060354731,-2.78653425969548,2.87381493343553,2.79153682020605,-2.66758137485090,3.62366673758296,-1.71040345290333},{-1.36786812962346,3.91191427420209,-2.90001234231728,2.90000433822046,2.92381652625077,-2.71192407121660,3.58808452518337,-1.34044609392872} };


class TrajBound : public ReorientableBehavior
{
public:


	TBMode mode = TB_SIT; //Current state within state-machine
	int logging = 0;

	uint32_t tLast; //int used to store system time at various events

	float t;
	uint32_t tOld;
	uint32_t tNew;
	uint32_t timeStep;
	uint32_t iter = 0;

	bool boolSlow = false;

	float posDes;
	float velDes;


	float posAct;
	float velAct;

	//float kp = 0.05;//0.8
	//float kd = 0.0012; // 0.02
	//float kp = 0.4;//0.8
	//float kd = 0.01; // 0.02
	float kp = 0.3;//0.8
	float kd = 0.005; // 0.02


	float ss;
	float ee;
	float eedot;
	float uu;

	//float pq1 = 0.025;//best=0.02 best+afterfiltering=0.05
	float pq1 = 0.02;//best=0.02 best+afterfiltering=0.05


	float alpha1 = 0.0005;
	float eta = 0.0075;
	float alpha3 = 1.2; // It's a big number.
	float gamma = 50;
	float sigma1;
	float sigmapt1;
	float beta = 15;
	int ddd;
	float tfcycle = times[TIMESTEPS - 1];
	float deltat = 0.01;
	float sigmap1[(2 * 100 + 2)][MOTORS];



	int fff;
	int ii = 0;
	float xi1;
	int jabejayi = 5;

	float finalTime = times[TIMESTEPS - 1];


	/////////////////////////////////////////////////////////////////////////
	float R_hat0;
	float R_hat1;
	float one = 1;
	float numact = one + exp(-0.5);
	float R_hatprevious0[MOTORS] = { one,one,one,one,one,one,one,one };
	float R_hatprevious1[MOTORS] = { numact,numact,numact,numact,numact,numact,numact,numact };
	float Omega;
	float UU;
	float sgnss;
	int ii_ii2;
	int ff_tf_deltat;
	int i;
	int j;
	int index = 0;
	int tfc_deltat = tfcycle / deltat;
	int t_deltat;
	uint32_t smillis;
	int tf_t_deltat;
	int ff_tfc_deltat;
	int co2 = 2 * tfc_deltat + 1 - jabejayi;
	float etalgam = 1.3333 * eta * alpha1 / gamma;
	float deltat_d2 = deltat * 0.5;
	float exp_ang;
	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into FH_LEAP
	void signal(uint32_t sig)
	{
		// tLast = S->millis;
		if (logging == 0)
			logging = 1;
	}

	void begin()
	{
		mode = TB_STAND;			// Start behavior in STAND mode
		tLast = S->millis;		// Record the system time @ this transition


		// ioctl(LOGGER_FILENO, 0);//stop
	}

	void update()
	{
		tOld = tNew;
		tNew = clockTimeUS;
		timeStep = tNew - tOld;






		if (isReorienting())
			return;


		posDes = 0;
		velDes = 0;



		if (mode == TB_SIT)
		{


			C->mode = RobotCommand_Mode_JOINT;
			for (i = 0; i < P->joints_count; i++)
			{


				posDes = 0.5;
				// Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs

				if (i == 1 || i == 3 || i == 4 || i == 6)
				{
					posDes = posDes - S->imu.euler.y;
				}
				else if (i == 0 || i == 2 || i == 5 || i == 7)
				{
					posDes = posDes + S->imu.euler.y;
				}

				joint[i].setGain(0.4, .003);
				joint[i].setPosition(posDes);
			}
		}
		else if (mode == TB_STAND)
		{

			C->mode = RobotCommand_Mode_JOINT;
			smillis = S->millis;
			t = 0.00001 * (100 * (smillis - tLast) % (int)(100000 * finalTime));

			ii = 100 * (smillis - tLast) / (int)(100000 * finalTime) + 1;
			//ii = ii+1;

			for (j = 0; j < TIMESTEPS; j++)
			{
				if (times[j] <= t && t < times[j + 1])
				{
					index = j;
					break;
				}
			}


			fff = t / deltat + 1;
			ii_ii2 = abs(ii % 2);
			t_deltat = round(t / deltat) + 1;
			tf_t_deltat = tfc_deltat + t_deltat;
			ff_tfc_deltat = fff + tfc_deltat;

			for (i = 0; i < MOTORS; ++i)
			{
				posDes = pos[index][i];
				velDes = vel[index][i];
				posAct = joint[i].getPosition();
				velAct = joint[i].getVelocity();
				/////////////////////////////////////////////////////////////////

				ee = posDes - posAct;
				eedot = velDes - velAct;
				ss = +kp * ee + kd * eedot;
				if (ss >= 0)
				{
					sgnss = 1;
				}
				else
				{
					sgnss = -1;
				}

				/////////////////////////////////////////////////////////////////////////	


				xi1 = beta * cos(posAct);
				//xi1[i] = beta;

				if (ii == 1)
				{

					sigma1 = 0.0001;

				}
				else
				{
					if (ii_ii2 < 0.000001)
					{
						ddd = ff_tfc_deltat;

					}

					else
					{
						ddd = fff;


					}
					sigmapt1 = sigmap1[ddd][i];

					if (ddd > jabejayi&& ddd < co2)
					{
						sigmapt1 = 0.3333 * (sigmap1[ddd - jabejayi][i] + sigmap1[ddd][i] + sigmap1[ddd + jabejayi][i]);
					}
					else
					{
						sigmapt1 = sigmap1[ddd][i];
					}





					sigma1 = sigmapt1 - pq1 * xi1 * (4 * eta / 3 * pow(abs(ss), 0.33333) * sgnss + gamma * ss);
					//sigma1[i] =  - pq1 * xi1[i] * (4 * eta / 3 * pow(abs(ss[i]), 0.33333) * sgn(ss[i]) + gamma * ss[i]);
				}

				if (ii_ii2 < 0.000001)
				{

					sigmap1[t_deltat][i] = sigma1;
				}



				else
				{

					sigmap1[tf_t_deltat][i] = sigma1;
				}



				/////////////////////////////////////////////////////////////////



				//uu =  (-kd * sigma1 * xi1 +alpha3 * ss + 1.3333 * eta * alpha1 / gamma * sgnss);
				/////////////////////////////////////////////////////////////////////////


				UU = (-kd * sigma1 * xi1 + alpha3 * ss + etalgam * sgnss);

				R_hat0 = 2.001*(deltat_d2 * UU * ss) + R_hatprevious0[i];
				R_hat1 = 2.001*(deltat_d2 / (one + exp(-posAct)) * UU * ss )+ R_hatprevious1[i];
				R_hatprevious0[i] = R_hat0;
				R_hatprevious1[i] = R_hat1;
				Omega = R_hat0 * 0.5 + R_hat1 * 0.5 / (one + exp(-posAct));
				uu = Omega * UU;

				//joint[i].setOpenLoop(ss[i]);
				joint[i].setOpenLoop(uu);
				//joint[i].setOpenLoop(dfDes[i] + kp * (posDes[i] - posAct) + kd * (velDes[i] - velAct));
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
		logging = 0;
	}
};

TrajBound trajBound;
//////////////////////////////////////////
typedef struct OutStruct {
	uint16_t align = 0xbbaa;
	uint32_t millis;
	float jointPosition[8];
	float jointVelocity[8];
} __attribute__((packed)) OutStruct;

OutStruct outs;
//////////////////////////////////////
void debug()
{


}

int main(int argc, char* argv[])
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

	setDebugRate(10);
	safetyShutoffEnable(false);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);


	// Run
	return begin();
}