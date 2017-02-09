#ifdef _WIN32
#  include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vcclr.h>
#ifndef __APPLE__
#  include <GL/gl.h>
#  include <GL/glut.h>
#else
#  include <OpenGL/gl.h>
#  include <GLUT/glut.h>
#endif
#include <AR/ar.h>
#include <AR/gsub.h>
#include <AR/video.h>

#using <System.dll>
#include <Windows.h>
#include <string>
#include <cmath>

using namespace System;
using namespace System::IO::Ports;
using namespace System::Threading;
using namespace std;

#define             CPARA_NAME       "Data/camera_para.dat"
#define             VPARA_NAME       "Data/cameraSetting-%08x%08x.dat"
//#define             PATT_NAME        "Data/hiro.patt"

struct Mobile
{
	float x;
	float y;
	float theta;
};

ARHandle           *arHandle;
//ARPattHandle       *arPattHandle;
AR3DHandle         *ar3DHandle;
ARGViewportHandle  *vp;
gcroot<SerialPort^> NXT;
int                 xsize, ysize;
int                 flipMode = 0;
int                 patt_id;
double              patt_width = 60.0;
int                 Count = 0;
float               velLeftPrev;
float               velRightPrev;
char                fps[256];
char				xValue[8][256];
char				yValue[8][256];
char                velLeftVal[256];
char                velRightVal[256];
char				rVelVal[256];
char				deltaThetaVal[256];
char				WVal[256];
char                angle[256];
char				ur1[256];
char				ur2[256];
char				ur3[256];
char                ur4[256];
char                errValue[256];
int                 distF = 0;
int                 contF = 0;
ARParamLT          *gCparamLT = NULL;

//Settings for COM Port
Int32               baudRate = 9600;
Int32               readTimeout = 500; // time limit for attempting to write to port before exiting
Int32               writeTimeout = 500; // time limit for attempting to read from port before exiting
gcroot<String^>     portNum = "COM6"; // name for the bluetooth COM port

//PF Variables
const float         PI = 3.14159;
const float         unitDeg = 7.0;
const float         wheelR = 1.0625;
const float         baseR = 2.15625;
const float         T = 0.1;
const float         rho0[] = { 0, 450, 450, 450, 450 };
const float         nAttract = 1.05;
const float         nRepulse[] = { 0, 1500, 1500, 1500, 1500 };

static void   init(int argc, char *argv[]);
static void   keyFunc(unsigned char key, int x, int y);
static void   cleanup(void);
static void   mainLoop(void);
static void   draw(ARdouble trans[3][4]);
static void   diffSteer(Mobile &mobile, float xVel, float yVel, float xTarget, float yTarget);

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	init(argc, argv);

	argSetDispFunc(mainLoop, 1);
	argSetKeyFunc(keyFunc);
	Count = 0;
	fps[0] = '\0';
	arUtilTimerReset();

	//set pattern detection for 2-D barcode markers
	arSetPatternDetectionMode(arHandle, AR_MATRIX_CODE_DETECTION);
	arSetMatrixCodeType(arHandle, AR_MATRIX_CODE_3x3_HAMMING63);

	argMainLoop();
	return (0);
}
static void keyFunc(unsigned char key, int x, int y)
{
	int   value;

	switch (key) {
	case 0x1b:
		cleanup();
		exit(0);
		break;
	case '1':
	case '-':
		arGetLabelingThresh(arHandle, &value);
		value -= 5;
		if (value < 0) value = 0;
		arSetLabelingThresh(arHandle, value);
		ARLOG("thresh = %d\n", value);
		break;
	case '2':
	case '+':
		arGetLabelingThresh(arHandle, &value);
		value += 5;
		if (value > 255) value = 255;
		arSetLabelingThresh(arHandle, value);
		ARLOG("thresh = %d\n", value);
		break;
	case 'd':
	case 'D':
		arGetDebugMode(arHandle, &value);
		value = 1 - value;
		arSetDebugMode(arHandle, value);
		break;
	case 'h':
	case 'H':
		if (flipMode & AR_GL_FLIP_H) flipMode = flipMode & AR_GL_FLIP_V;
		else                         flipMode = flipMode | AR_GL_FLIP_H;
		argViewportSetFlipMode(vp, flipMode);
		break;
	case 'v':
	case 'V':
		if (flipMode & AR_GL_FLIP_V) flipMode = flipMode & AR_GL_FLIP_H;
		else                         flipMode = flipMode | AR_GL_FLIP_V;
		argViewportSetFlipMode(vp, flipMode);
		break;
	case ' ':
		distF = 1 - distF;
		if (distF) {
			argViewportSetDistortionMode(vp, AR_GL_DISTORTION_COMPENSATE_ENABLE);
		}
		else {
			argViewportSetDistortionMode(vp, AR_GL_DISTORTION_COMPENSATE_DISABLE);
		}
		break;
	case 'c':
		contF = 1 - contF;
		break;
	case '?':
	case '/':
		ARLOG("Keys:\n");
		ARLOG(" [esc]         Quit demo.\n");
		ARLOG(" - and +       Adjust threshhold.\n");
		ARLOG(" d             Activate / deactivate debug mode.\n");
		ARLOG(" h and v       Toggle horizontal / vertical flip mode.\n");
		ARLOG(" [space]       Toggle distortion compensation.\n");
		ARLOG(" ? or /        Show this help.\n");
		ARLOG("\nAdditionally, the ARVideo library supplied the following help text:\n");
		arVideoDispOption();
		break;
	default:
		break;
	}
}

static void mainLoop(void)
{
	static int      contF2 = 0;
	static ARdouble patt_trans[3][4];
	static ARUint8 *dataPtr = NULL;
	ARMarkerInfo   *markerInfo;
	int             markerNum;
	ARdouble        err;
	int             imageProcMode;
	double			xCoord[8];
	double			xCoordRel[8];
	double			yCoord[8];
	double			yCoordRel[8];
	int             debugMode;
	int             h, j, k;
	double			xReal = 73.0;
	double			yReal = 40.5;

	//PF Variables
	float rho[4];
	float ur_x[4];
	float ur_y[4];
	float xObstacle[5];
	float yObstacle[5];
	float xTarget;
	float yTarget;
	float xNext;
	float Angle;
	float A;
	float B;
	float yNext;
	float xVel;
	float yVel;
	Mobile mobile;
	mobile.x = 0.0;
	mobile.y = 0.0;
	mobile.theta = 90.0;

	xCoord[0] = 0.0;
	yCoord[0] = 0.0;

	// grab a video frame 
	if ((dataPtr = (ARUint8 *)arVideoGetImage()) == NULL) {
		arUtilSleep(2);
		return;
	}

	argDrawMode2D(vp);
	arGetDebugMode(arHandle, &debugMode);
	if (debugMode == 0) {
		argDrawImage(dataPtr);
	}
	else {
		arGetImageProcMode(arHandle, &imageProcMode);
		if (imageProcMode == AR_IMAGE_PROC_FRAME_IMAGE) {
			argDrawImage(arHandle->labelInfo.bwImage);
		}
		else {
			argDrawImageHalf(arHandle->labelInfo.bwImage);
		}
	}


	// detect the markers in the video frame
	if (arDetectMarker(arHandle, dataPtr) < 0) {
		cleanup();
		exit(0);
	}

	if (Count % 60 == 0) {
		//sprintf(fps, "%f[fps]", 60.0 / arUtilTimer());
		arUtilTimerReset();
	}
	Count++;
	//glColor3f(0.0f, 1.0f, 0.0f);
	//argDrawStringsByIdealPos(fps, 10, ysize - 30);

	markerNum = arGetMarkerNum(arHandle);
	if (markerNum == 0) {
		argSwapBuffers();
		return;
	}

	// check for object visibility 
	markerInfo = arGetMarker(arHandle);
	k = -1;
	glColor3f(0.0f, 1.0f, 0.0f);

	for (j = 0; j < markerNum; j++) {
		ARLOG("ID=%d, CF = %f\n", markerInfo[j].idMatrix, markerInfo[j].cfMatrix);

		for (h = 0; h < 8; h++) {

			if (markerInfo[j].idMatrix == h) {
				yCoord[h] = (yReal / ysize)*(ysize - markerInfo[j].pos[1]);
				xCoord[h] = (xReal / xsize)*(markerInfo[j].pos[0]);
				}

			if ((xCoord[0] != 0) && (yCoord[0] != 0) && (xCoord[h] < 1000) && (yCoord[h] < 1000) && (xCoord[h] > -1000) && (yCoord[h] > -1000)) {
				xCoordRel[h] = xCoord[h] - xCoord[0];
				yCoordRel[h] = yCoord[h] - yCoord[0];
				ARLOG("X(%d): %.1f; Y(%d): %.1f\n", h, xCoordRel[h], h, yCoordRel[h]);
				sprintf(xValue[h], "X(%d): %.1f", h, xCoordRel[h]);
				sprintf(yValue[h], "Y(%d): %.1f", h, yCoordRel[h]);
				argDrawStringsByIdealPos(xValue[h], 10, ysize - (25 * (h+1)));
				argDrawStringsByIdealPos(yValue[h], 130, ysize - (25 * (h+1)));
			}
		}

		if (patt_id == markerInfo[j].idMatrix) {
			if (k == -1) {
				if (markerInfo[j].cfMatrix >= 0.7) k = j;
			}
			else if (markerInfo[j].cfMatrix > markerInfo[k].cfMatrix) k = j;
		}

	}
	
	A = xCoordRel[7] - xCoordRel[6];
	B = yCoordRel[7] - yCoordRel[6];

	Angle = atan2(B, A)*(180 / PI);

	if (B < 0) {
		Angle += 360;
	}

	xTarget = xCoordRel[5];
	yTarget = yCoordRel[5];
	mobile.x = xCoordRel[7];
	mobile.y = yCoordRel[7];
	mobile.theta = Angle;
	for (h = 1; h < 5; h++) {
		xObstacle[h] = xCoordRel[h];
		yObstacle[h] = yCoordRel[h];
	}

	sprintf(angle, "Angle: %.2f  ", Angle);
	argDrawStringsByIdealPos(angle, 10, 150);


	for (j = 1; j < 5; j++) {
		rho[j] = sqrt(pow((yObstacle[j] - mobile.y), 2) + pow((xObstacle[j] - mobile.x), 2));

		// Calculate gradient (Eqns 11 and 13)
		if (rho[j] < rho0[j]) {
			ur_x[j] = nRepulse[j] * (xObstacle[j] - mobile.x)*((1 / rho[j]) - (1 / rho0[j])) / pow(rho[j], 3);
			ur_y[j] = nRepulse[j] * (yObstacle[j] - mobile.y)*((1 / rho[j]) - (1 / rho0[j])) / pow(rho[j], 3);
		}

		else {
			ur_x[j] = 0;
			ur_y[j] = 0;
		}

		}

	sprintf(ur1, "Ux(1): %.1f; Uy(1): %.1f", ur_x[1], ur_y[1]);
	//sprintf(ur2, "Ux(2): %.1f; Uy(2): %.1f", ur_x[2], ur_y[2]);
	argDrawStringsByIdealPos(ur1, 10, 25);
	//argDrawStringsByIdealPos(ur2, 10, 50);

	xNext = mobile.x - (T*nAttract*(mobile.x - xTarget)) - (T*ur_x[1]) - (T*ur_x[2]) - (T*ur_x[3]) - (T*ur_x[4]);
	yNext = mobile.y - (T*nAttract*(mobile.y - yTarget)) - (T*ur_y[1]) - (T*ur_y[2]) - (T*ur_y[3]) - (T*ur_y[4]);
	xVel = -nAttract*(mobile.x - xTarget) - ur_x[1] - ur_x[2] - ur_x[3] - ur_x[4];
	yVel = -nAttract*(mobile.y - yTarget) - ur_y[1] - ur_y[2] - ur_y[3] - ur_y[4];
	diffSteer(mobile, xVel, yVel, xTarget, yTarget);

	//Error calc
	if (k == -1) {
		contF2 = 0;
		argSwapBuffers();
		return;
	}

	if (contF && contF2) {
		err = arGetTransMatSquareCont(ar3DHandle, &(markerInfo[k]), patt_trans, patt_width, patt_trans);
	}
	else {
		err = arGetTransMatSquare(ar3DHandle, &(markerInfo[k]), patt_width, patt_trans);
	}

	//ARLOG("err = %f\n", err);

	contF2 = 1;
	//draw(patt_trans);

	argSwapBuffers();
}

static void   init(int argc, char *argv[])
{
	ARParam         cparam;
	ARGViewport     viewport;
	char            vconf[512];
	AR_PIXEL_FORMAT pixFormat;
	ARUint32        id0, id1;
	int             i;

	if (argc == 1) vconf[0] = '\0';
	else {
		strcpy(vconf, argv[1]);
		for (i = 2; i < argc; i++) { strcat(vconf, " "); strcat(vconf, argv[i]); }
	}

	// open the video path
	ARLOGi("Using video configuration '%s'.\n", vconf);
	if (arVideoOpen(vconf) < 0) exit(0);
	if (arVideoGetSize(&xsize, &ysize) < 0) exit(0);
	ARLOGi("Image size (x,y) = (%d,%d)\n", xsize, ysize);
	if ((pixFormat = arVideoGetPixelFormat()) < 0) exit(0);
	if (arVideoGetId(&id0, &id1) == 0) {
		ARLOGi("Camera ID = (%08x, %08x)\n", id1, id0);
		sprintf(vconf, VPARA_NAME, id1, id0);
		if (arVideoLoadParam(vconf) < 0) {
			ARLOGe("No camera setting data!!\n");
		}
	}

	// set the initial camera parameters
	if (arParamLoad(CPARA_NAME, 1, &cparam) < 0) {
		ARLOGe("Camera parameter load error !!\n");
		exit(0);
	}
	arParamChangeSize(&cparam, xsize, ysize, &cparam);
	ARLOG("*** Camera Parameter ***\n");
	arParamDisp(&cparam);
	if ((gCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
		ARLOGe("Error: arParamLTCreate.\n");
		exit(-1);
	}

	if ((arHandle = arCreateHandle(gCparamLT)) == NULL) {
		ARLOGe("Error: arCreateHandle.\n");
		exit(0);
	}
	if (arSetPixelFormat(arHandle, pixFormat) < 0) {
		ARLOGe("Error: arSetPixelFormat.\n");
		exit(0);
	}

	if ((ar3DHandle = ar3DCreateHandle(&cparam)) == NULL) {
		ARLOGe("Error: ar3DCreateHandle.\n");
		exit(0);
	}

	/*if ((arPattHandle = arPattCreateHandle()) == NULL) {
		ARLOGe("Error: arPattCreateHandle.\n");
		exit(0);
	}
	if ((patt_id = arPattLoad(arPattHandle, PATT_NAME)) < 0) {
		ARLOGe("pattern load error !!\n");
		exit(0);
	}
	arPattAttach(arHandle, arPattHandle);*/

	// open the graphics window
	int winSizeX, winSizeY;
	argCreateFullWindow();
	argGetScreenSize( &winSizeX, &winSizeY );
	viewport.sx = 0;
	viewport.sy = 0;
	viewport.xsize = winSizeX;
	viewport.ysize = winSizeY;
	viewport.sx = 0;
	viewport.sy = 0;
	viewport.xsize = xsize;
	viewport.ysize = ysize;
	if ((vp = argCreateViewport(&viewport)) == NULL) exit(0);
	argViewportSetCparam(vp, &cparam);
	argViewportSetPixFormat(vp, pixFormat);
	//argViewportSetDispMethod( vp, AR_GL_DISP_METHOD_GL_DRAW_PIXELS );
	argViewportSetDistortionMode(vp, AR_GL_DISTORTION_COMPENSATE_DISABLE);

	if (arVideoCapStart() != 0) {
		ARLOGe("video capture start error !!\n");
		exit(0);
	}

	//Bluetooth Setup

	//Creates Serial Port object named "NXT" for communication over Bluetooth
	NXT = gcnew SerialPort(portNum, baudRate, Parity::None, 8, StopBits::One); //creates Serial Port object named "NXT"
	NXT->ReadTimeout = readTimeout; // sets read timeout
	NXT->WriteTimeout = writeTimeout; //sets write timeout

    // Opens port for communication with NXT Brick over Bluetooth
	NXT->Open();
	Sleep(1000);
}

// cleanup function called when program exits
static void cleanup(void)
{
	arVideoCapStop();
	argCleanup();
	arPattDetach(arHandle);
	//arPattDeleteHandle(arPattHandle);
	ar3DDeleteHandle(&ar3DHandle);
	arDeleteHandle(arHandle);
	arParamLTFree(&gCparamLT);
	arVideoClose();

	Sleep(1000);
	NXT->Close();
}

static void draw(ARdouble trans[3][4])
{
	ARdouble  gl_para[16];
	GLfloat   mat_diffuse[] = { 0.0f, 0.0f, 1.0f, 0.0f };
	GLfloat   mat_flash[] = { 1.0f, 1.0f, 1.0f, 0.0f };
	GLfloat   mat_flash_shiny[] = { 50.0f };
	GLfloat   light_position[] = { 100.0f,-200.0f,200.0f,0.0f };
	GLfloat   light_ambi[] = { 0.1f, 0.1f, 0.1f, 0.0f };
	GLfloat   light_color[] = { 1.0f, 1.0f, 1.0f, 0.0f };

	argDrawMode3D(vp);
	glClearDepth(1.0);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	// load the camera transformation matrix
	argConvGlpara(trans, gl_para);
	glMatrixMode(GL_MODELVIEW);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(gl_para);
#else
	glLoadMatrixd(gl_para);
#endif

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambi);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_diffuse);

#if 1
	glTranslatef(0.0f, 0.0f, 30.0f);
	glutSolidCube(60.0);
#else
	glTranslatef(0.0f, 0.0f, 20.0f);
	glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
	glutSolidTeapot(40.0);
#endif
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	glDisable(GL_DEPTH_TEST);
}

static void diffSteer(Mobile &mobile, float xVel, float yVel, float xTarget, float yTarget)
{
	cli::array<Byte, 1>^ left = { 0x0C, 0x00, 0x00, 0x04, 0x01, 0x00, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00  };
	cli::array<Byte, 1>^ right = { 0x0C, 0x00, 0x00, 0x04, 0x00, 0x00, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00 };
	float rVel = sqrt(pow(xVel, 2) + pow(yVel, 2));
	sprintf(rVelVal, "rVel: %.2f", rVel);
	argDrawStringsByIdealPos(rVelVal, 10, 50);
	//float thetaTarget = ((PI / 2) - atan2((xNext - mobile.x), (yNext - mobile.y)))*(180 / PI);
	float thetaTarget = atan2(yVel, xVel)*(180 / PI);

	if (thetaTarget < 0)
		thetaTarget += 360;

	float deltaTheta = thetaTarget - mobile.theta;

	if (deltaTheta < -180)
		deltaTheta += 360;

	else if (deltaTheta > 180)
		deltaTheta -= 360;

	sprintf(deltaThetaVal, "dTheta: %.2f", deltaTheta);
	argDrawStringsByIdealPos(deltaThetaVal, 10, 75);
	float W = deltaTheta*(baseR / wheelR);
	sprintf(WVal, "W: %.2f", W);
	argDrawStringsByIdealPos(WVal, 10, 100);
	float rightVel = rVel + (W/2);
	float leftVel = rVel - (W/2);
	sprintf(velRightVal, "rightVel: %.2f", rightVel);
	sprintf(velLeftVal, "leftVel: %.2f", leftVel);
	argDrawStringsByIdealPos(velRightVal, 10, 125);
	argDrawStringsByIdealPos(velLeftVal, 200, 125);


	if (leftVel*velLeftPrev < 0)
	{
		left[5] = 0;
		NXT->Write(left, 0, 14);
	}
	if (rightVel*velRightPrev < 0)
	{
		right[5] = 0;
		NXT->Write(right, 0, 14);
	}

	left[5] = leftVel;
	right[5] = rightVel;

	if (leftVel < 20 && leftVel > 0)
		left[5] = 20;

	if (rightVel < 20 && rightVel > 0)
		right[5] = 20;

	if (leftVel > 100)
		left[5] = 100;

	else if (leftVel < -100)
		left[5] = -100;

	if (rightVel > 100)
		right[5] = 100;

	else if (rightVel < -100)
		right[5] = -100;

	if (abs(xTarget - mobile.x) < 4.0 && abs(yTarget - mobile.y) < 4.0) {
		left[5] = 20;
		right[5] = 20;
		NXT->Write(left, 0, 14);
		NXT->Write(right, 0, 14);
		Sleep(1800);
		left[5] = 0;
		right[5] = 0;
		NXT->Write(left, 0, 14);
		NXT->Write(right, 0, 14);
		exit(0);
	}

	NXT->Write(left, 0, 14);
	NXT->Write(right, 0, 14);

	velLeftPrev = leftVel;
	velRightPrev = rightVel;
}