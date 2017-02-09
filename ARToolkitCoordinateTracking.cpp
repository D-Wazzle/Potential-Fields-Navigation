// ARToolkitCoordinateTracking.cpp
// 
// Author: Dylan Wallace, Drones and Autonomous Systems Lab @ University of Nevada, Las Vegas
//
// This is an ARToolkit program for tracking the relative x & y coordinates of ARToolkit markers. 
// This is important for a future program that will use the markers as obstacles and targets for robotic navigation.

#ifdef _WIN32
#  include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef __APPLE__
#  include <GL/gl.h>
#  define GLUT_DISABLE_ATEXIT_HACK
#  include <GL/glut.h>
#else
#  include <OpenGL/gl.h>
#  include <GLUT/glut.h>
#endif
#include <AR/ar.h>
#include <AR/gsub.h>
#include <AR/video.h>

#define             CPARA_NAME       "Data/camera_para.dat"
#define             VPARA_NAME       "Data/cameraSetting-%08x%08x.dat"
//#define             PATT_NAME        "Data/hiro.patt"

ARHandle           *arHandle;
//ARPattHandle       *arPattHandle;
AR3DHandle         *ar3DHandle;
ARGViewportHandle  *vp;
int                 xsize, ysize;
int                 flipMode = 0;
int                 patt_id;
double              patt_width = 80.0;
int                 count = 0;
char                xValue[256];
char                yValue[256];
char                fps[256];
char                errValue[256];
int                 distF = 0;
int                 contF = 0;
ARParamLT          *gCparamLT = NULL;

static void   init(int argc, char *argv[]);
static void   keyFunc( unsigned char key, int x, int y );
static void   cleanup(void);
static void   mainLoop(void);
static void   draw( ARdouble trans[3][4] );

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
    init(argc, argv);

    argSetDispFunc( mainLoop, 1 );
	argSetKeyFunc( keyFunc );
	count = 0;
    fps[0] = '\0';

    if (flipMode & AR_GL_FLIP_V)
        flipMode = flipMode & AR_GL_FLIP_H;
    else
        flipMode = flipMode | AR_GL_FLIP_V;

    argViewportSetFlipMode(vp, flipMode);

    if (flipMode & AR_GL_FLIP_H)
        flipMode = flipMode & AR_GL_FLIP_V;
    else
        flipMode = flipMode | AR_GL_FLIP_H;

    argViewportSetFlipMode(vp, flipMode);

	arUtilTimerReset();

	arSetPatternDetectionMode(arHandle, AR_MATRIX_CODE_DETECTION);
	arSetMatrixCodeType(arHandle, AR_MATRIX_CODE_3x3_HAMMING63);

    argMainLoop();
	return (0);
}
static void keyFunc( unsigned char key, int x, int y )
{
    int   value;

    switch (key) {
		case 0x1b:
			cleanup();
			exit(0);
			break;
		case '1':
		case '-':
        	arGetLabelingThresh( arHandle, &value );
        	value -= 5;
        	if( value < 0 ) value = 0;
        	arSetLabelingThresh( arHandle, value );
        	ARLOG("thresh = %d\n", value);
        	break;
		case '2':
		case '+':
        	arGetLabelingThresh( arHandle, &value );
       		value += 5;
        	if( value > 255 ) value = 255;
        	arSetLabelingThresh( arHandle, value );
        	ARLOG("thresh = %d\n", value);
        	break;
		case 'd':
		case 'D':
        	arGetDebugMode( arHandle, &value );
       		value = 1 - value;
        	arSetDebugMode( arHandle, value );
            break;
        case 'h':
        case 'H':
            if( flipMode & AR_GL_FLIP_H ) flipMode = flipMode & AR_GL_FLIP_V;
            else                         flipMode = flipMode | AR_GL_FLIP_H;
            argViewportSetFlipMode( vp, flipMode );
            break;
        case 'v':
        case 'V':
            if( flipMode & AR_GL_FLIP_V ) flipMode = flipMode & AR_GL_FLIP_H;
            else                         flipMode = flipMode | AR_GL_FLIP_V;
            argViewportSetFlipMode( vp, flipMode );
        	break;
        case ' ':
            distF = 1 - distF;
            if( distF ) {
                argViewportSetDistortionMode( vp, AR_GL_DISTORTION_COMPENSATE_ENABLE );
            } else {
                argViewportSetDistortionMode( vp, AR_GL_DISTORTION_COMPENSATE_DISABLE );
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
    int             debugMode;
    int             j, k;
    double          xCoord;
    double          yCoord;
    double          xReal = 38.5;
    double          yReal = 26.5;

    /* grab a video frame */
    if( (dataPtr = (ARUint8 *)arVideoGetImage()) == NULL ) {
        arUtilSleep(2);
        return;
    }

    argDrawMode2D(vp);
    arGetDebugMode( arHandle, &debugMode );
    if( debugMode == 0 ) {
        argDrawImage( dataPtr );
    }
    else {
        arGetImageProcMode(arHandle, &imageProcMode);
        if( imageProcMode == AR_IMAGE_PROC_FRAME_IMAGE ) {
            argDrawImage( arHandle->labelInfo.bwImage );
        }
        else {
            argDrawImageHalf( arHandle->labelInfo.bwImage );
        }
    }

    /* detect the markers in the video frame */
    if( arDetectMarker(arHandle, dataPtr) < 0 ) {
        cleanup();
        exit(0);
    }

    if( count % 60 == 0 ) {
        sprintf(fps, "%f[fps]", 60.0/arUtilTimer());
        arUtilTimerReset();
    }
    count++;
    //glColor3f(0.0f, 1.0f, 0.0f);
    //argDrawStringsByIdealPos(fps, 10, ysize-30);

    markerNum = arGetMarkerNum( arHandle );
    if( markerNum == 0 ) {
        argSwapBuffers();
        return;
    }

    /* check for object visibility */
    markerInfo =  arGetMarker( arHandle );
    k = -1;
    for( j = 0; j < markerNum; j++ ) {
        ARLOG("ID=%d, CF = %f\n", markerInfo[j].id, markerInfo[j].cf);
	
	// match the barcode marker to the proper ID
        for (h = 0; h < 7; h++) {
            if (markerInfo[j].idMatrix == h) {
                yCoord[h] = (yReal / ysize)*(ysize - markerInfo[j].pos[1]);
                xCoord[h] = (xReal / xsize)*(markerInfo[j].pos[0]);
            }

	    // Create a coordinate system to keep track of the relative positions
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

        if( patt_id == markerInfo[j].id ) {
            if( k == -1 ) {
                if (markerInfo[j].cf >= 0.7) k = j;
            } else if( markerInfo[j].cf > markerInfo[k].cf ) k = j;
        }
    }
    if( k == -1 ) {
        contF2 = 0;
        argSwapBuffers();
        return;
    }

    if( contF && contF2 ) {
        err = arGetTransMatSquareCont(ar3DHandle, &(markerInfo[k]), patt_trans, patt_width, patt_trans);
    }
    else {
        err = arGetTransMatSquare(ar3DHandle, &(markerInfo[k]), patt_width, patt_trans);
    }
    //sprintf(errValue, "err = %f", err);
    glColor3f(0.0f, 1.0f, 0.0f);

    xCoord = markerInfo[k].pos[0];
    yCoord = markerInfo[k].pos[1];
    sprintf(yValue, "Y: %f", (yReal / ysize)*yCoord);
    sprintf(xValue, "X: %f", (xReal / xsize)*(xsize - xCoord));

    argDrawStringsByIdealPos(xValue, xsize - 10, 60);
    argDrawStringsByIdealPos(yValue, xsize - 10, 30);
    //argDrawStringsByIdealPos(fps, 10, ysize-30);
    //argDrawStringsByIdealPos(errValue, 10, ysize-60);
    //ARLOG("err = %f\n", err);

    contF2 = 1;
    draw(patt_trans);

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

    if( argc == 1 ) vconf[0] = '\0';
    else {
        strcpy( vconf, argv[1] );
        for( i = 2; i < argc; i++ ) {strcat(vconf, " "); strcat(vconf,argv[i]);}
    }

    /* open the video path */
	ARLOGi("Using video configuration '%s'.\n", vconf);
    if( arVideoOpen( vconf ) < 0 ) exit(0);
    if( arVideoGetSize(&xsize, &ysize) < 0 ) exit(0);
    ARLOGi("Image size (x,y) = (%d,%d)\n", xsize, ysize);
    if( (pixFormat=arVideoGetPixelFormat()) < 0 ) exit(0);
    if( arVideoGetId( &id0, &id1 ) == 0 ) {
        ARLOGi("Camera ID = (%08x, %08x)\n", id1, id0);
        sprintf(vconf, VPARA_NAME, id1, id0);
        if( arVideoLoadParam(vconf) < 0 ) {
            ARLOGe("No camera setting data!!\n");
        }
    }

    /* set the initial camera parameters */
    if( arParamLoad(CPARA_NAME, 1, &cparam) < 0 ) {
        ARLOGe("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize( &cparam, xsize, ysize, &cparam );
    ARLOG("*** Camera Parameter ***\n");
    arParamDisp( &cparam );
    if ((gCparamLT = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ARLOGe("Error: arParamLTCreate.\n");
        exit(-1);
    }

    if( (arHandle=arCreateHandle(gCparamLT)) == NULL ) {
        ARLOGe("Error: arCreateHandle.\n");
        exit(0);
    }
    if( arSetPixelFormat(arHandle, pixFormat) < 0 ) {
        ARLOGe("Error: arSetPixelFormat.\n");
        exit(0);
    }

    if( (ar3DHandle=ar3DCreateHandle(&cparam)) == NULL ) {
        ARLOGe("Error: ar3DCreateHandle.\n");
        exit(0);
    }

    if( (arPattHandle=arPattCreateHandle()) == NULL ) {
        ARLOGe("Error: arPattCreateHandle.\n");
        exit(0);
    }
    if( (patt_id=arPattLoad(arPattHandle, PATT_NAME)) < 0 ) {
        ARLOGe("pattern load error !!\n");
        exit(0);
    }
    arPattAttach( arHandle, arPattHandle );

    /* open the graphics window */
/*
    int winSizeX, winSizeY;
    argCreateFullWindow();
    argGetScreenSize( &winSizeX, &winSizeY );
    viewport.sx = 0;
    viewport.sy = 0;
    viewport.xsize = winSizeX;
    viewport.ysize = winSizeY;
*/
    viewport.sx = 0;
    viewport.sy = 0;
    viewport.xsize = xsize;
    viewport.ysize = ysize;
    if( (vp=argCreateViewport(&viewport)) == NULL ) exit(0);
    argViewportSetCparam( vp, &cparam );
    argViewportSetPixFormat( vp, pixFormat );
    //argViewportSetDispMethod( vp, AR_GL_DISP_METHOD_GL_DRAW_PIXELS );
    argViewportSetDistortionMode( vp, AR_GL_DISTORTION_COMPENSATE_DISABLE );

	if (arVideoCapStart() != 0) {
        ARLOGe("video capture start error !!\n");
        exit(0);
	}
}

/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    argCleanup();
	arPattDetach(arHandle);
	arPattDeleteHandle(arPattHandle);
	ar3DDeleteHandle(&ar3DHandle);
	arDeleteHandle(arHandle);
    arParamLTFree(&gCparamLT);
    arVideoClose();
}

static void draw( ARdouble trans[3][4] )
{
    ARdouble  gl_para[16];
    GLfloat   mat_diffuse[]     = {0.0f, 0.0f, 1.0f, 0.0f};
    GLfloat   mat_flash[]       = {1.0f, 1.0f, 1.0f, 0.0f};
    GLfloat   mat_flash_shiny[] = {50.0f};
    GLfloat   light_position[]  = {100.0f,-200.0f,200.0f,0.0f};
    GLfloat   light_ambi[]      = {0.1f, 0.1f, 0.1f, 0.0f};
    GLfloat   light_color[]     = {1.0f, 1.0f, 1.0f, 0.0f};

    argDrawMode3D(vp);
    glClearDepth( 1.0 );
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    /* load the camera transformation matrix */
    argConvGlpara(trans, gl_para);
    glMatrixMode(GL_MODELVIEW);
#ifdef ARDOUBLE_IS_FLOAT
    glLoadMatrixf( gl_para );
#else
    glLoadMatrixd( gl_para );
#endif

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambi);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_color);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_flash);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_diffuse);

#if 1
    glTranslatef( 0.0f, 0.0f, 0.0f );
    glutSolidCube(10.0);
#else
    glTranslatef( 0.0f, 0.0f, 20.0f );
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glutSolidTeapot(40.0);
#endif
    glDisable(GL_LIGHT0);
    glDisable( GL_LIGHTING );

    glDisable( GL_DEPTH_TEST );
}

