/*
 *  openal.cpp
 *  PIGE-OpenAL
 *
 *  Created by Chad Armstrong on Mon Jul 29 2002.
 *  Copyright (c) 2002 Edenwaith. All rights reserved.
 *
 *  Remember to add these frameworks: GLUT, OpenAL, OpenGL
 *  Otherwise, Undefined symbols: errors will result
 *
 *  Several prebind errors will occur, but I was still able to compile
 *  and run the program.
 *
 *  A VERY important step to get this to work is to copy the .wav files into
 *  the <app name>.app/Contents/MacOS folder.  You need to use Terminal to do
 *  this since the Finder windows interpret the <app name>.app as an executable
 *  file instead.
 */

#include <stdio.h>
#include <GL/glut.h>
#include <AL/alut.h>
#include <cmath>
#include "sonic_dog.h"
#include <vector>
#include <ctime>
#include <algorithm>
#include <string>
#include <sstream>

using std::vector;
using std::min;
using std::stringstream;
using std::string;

//  function prototypes ---------------------------------------------
void init();
void display();
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);
void specialKeys(int key, int x, int y);
void perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar );
float getDistance( ALfloat source[] );
float getAngle( ALfloat source[] );
void alertTime();
void displayDot ( void );
void checkObstacles();
void drawAngle();

// used to turn on and off the pink box and all related functions
#define BOX

ALfloat listenerPos[]={0.0,0.0,4.0};
ALfloat listenerVel[]={0.0,0.0,0.0};
ALfloat	listenerOri[]={0.0,0.0,1.0, 0.0,1.0,0.0};

ALfloat obs1_pos[]={ -2.0, 0.0, 1.0};
ALfloat obs1_vel[]={ 0.0, 0.0, 0.0};
float obs1_dist;

ALfloat obs2_pos[]={ 2.0, 0.0, 0.0};
ALfloat obs2_vel[]={ 0.0, 0.0, 0.0};
float obs2_dist;

// the pink box
ALfloat pink_box_pos[]={ 0.0, 0.0, -4.0};
ALfloat pink_box_vel[]={ 0.0, 0.0, 0.0};

int GLwin;

const GLdouble pi = 3.1415926535897932384626433832795;
SonicDog *sonny;
size_t pb;
time_t last_alert;
const time_t alert_inter = 0;
const float angle61 = 1.06465;
const float angle119 = 2.07694181;
const float WID = 0.5f;
const float ANGLE_80 = 1.3962634;
const float ARC_20 = 0.34906585;
bool arrived = false;
bool global_pause = false;
ALfloat *target = pink_box_pos;
ALfloat *targets[] = { pink_box_pos, obs2_pos, obs1_pos };
unsigned int cur_targ = 0;
bool draw_angle = false;

// ===================================================================
// void main(int argc, char** argv)
// ===================================================================
int main(int argc, char** argv) {
	//initialise glut
	glutInit(&argc, argv) ;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);   
	glutInitWindowSize(500,500);

	sonny = new SonicDog( 3 );

	GLwin = glutCreateWindow("Sonic Dog Demo") ;
	init();
	glutDisplayFunc(display) ;
	glutKeyboardFunc(keyboard) ;
	glutSpecialFunc(specialKeys);
	glutReshapeFunc(reshape) ;
	glutIdleFunc( checkObstacles );

	glutMainLoop();

	return 0;
}

// ===================================================================
// void init()
// ===================================================================
void init(void) {
#ifdef BOX
	float side = pink_box_pos[0] - listenerPos[0];
	float dist = listenerPos[2] - pink_box_pos[2];
	pb = sonny->addBeacon( side, dist );
#endif

	last_alert = 0;
	obs1_dist = getDistance( obs1_pos );
	obs2_dist = getDistance( obs2_pos );

	sonny->start();
}


// ===================================================================
// void display()
// ===================================================================
void display(void) {
	glLineWidth( 3.0 );
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPushMatrix() ;
	glRotatef(15.0,1.0,0.0,0.0);

	if ( draw_angle ) drawAngle();

	// obstacle
	glPushMatrix();
	glTranslatef(obs1_pos[0],obs1_pos[1],obs1_pos[2]);
	glColor3f(0.0,1.0,0.0);
	glutWireCube(WID);
	glPopMatrix();

	// obstacle
	glPushMatrix();
	glTranslatef(obs2_pos[0],obs2_pos[1],obs2_pos[2]);
	glColor3f(0.0,1.0,0.0);
	glutWireCube(WID);
	glPopMatrix();

#ifdef BOX
	// pink box
	glPushMatrix() ;
	glTranslatef(pink_box_pos[0],pink_box_pos[1],pink_box_pos[2]) ;
	glColor3f(1.0,0.2,0.6) ;
	glutWireCube(WID) ;
	glPopMatrix() ;
#endif

	//the listener
	glPushMatrix() ;
	glTranslatef(listenerPos[0],listenerPos[1],listenerPos[2]) ;
  glColor3f(1.0,1.0,1.0) ;
	glutWireCube(WID) ;
	glPopMatrix() ;

	glPopMatrix() ;
	glutSwapBuffers() ;
}

void drawAngle() {
  glColor3f(0.0,0.0,1.0) ;
	glBegin(GL_LINES);
	glVertex3f( listenerPos[0], listenerPos[1]-.25, listenerPos[2] );
	glVertex3f( target[0], target[1]-.25, target[2] );
	glEnd();

	float degrees = ( getAngle( target ) * 180 ) / pi;
	stringstream ss;
	ss << degrees;

	//glRotatef(10.0,0.0,0.0,0.0);
	glPushMatrix();
	glTranslatef( -3, 2, 0 );
	glScalef(0.01f,0.01f,0.01f);
	string text = ss.str();
	for ( size_t i = 0; i < text.length(); i++ ) {
		glutStrokeCharacter( GLUT_STROKE_ROMAN, text[i] );
	}
	glPopMatrix();
}

// ===================================================================
// void reshape(int w, int h)
// ===================================================================
void reshape(int w, int h) // the reshape function
{
  glViewport(0,0,(GLsizei)w,(GLsizei)h) ;
  glMatrixMode(GL_PROJECTION) ;
  glLoadIdentity() ;
  perspectiveGL(60.0,(GLfloat)w/(GLfloat)h,1.0,30.0) ;
  glMatrixMode(GL_MODELVIEW) ;
  glLoadIdentity() ;
  glTranslatef(0.0,0.0,-6.6) ;
}


// ===================================================================
// void keyboard(int key, int x, int y)
// ===================================================================
void keyboard(unsigned char key, int x, int y) {
	switch(key) {
		case '1': {
			sonny->pauseObject( pb );
			global_pause = true;
			break;
		}
		case '2': {
			sonny->unpauseObject( pb );
			global_pause = false;
			break;
		}
		case '3': {
			sonny->turnRegionsOn();
			break;
		}
		case '4': {
			sonny->turnRegularOn();
			break;
		}
		case '5': {
			sonny->turnCutoffOn();
			break;
		}
		case '6': {
			sonny->turnFrontOnlyOn();
			break;
		}
		case '7': {
			sonny->turnFrontOnlyOff();
			break;
		}
		case '8': {
			draw_angle = true;
			break;
		}
		case '9': {
			draw_angle = false;
			break;
		}
		case 27: {
			sonny->stop();
			delete sonny;

			glutDestroyWindow(GLwin) ;

			exit(0) ;
			break ;
		}
		case ']': {
			cur_targ++;
			target = targets[cur_targ % 3];
			break;
		}
		case '[': {
			cur_targ--;
			target = targets[cur_targ % 3];
			break;
		}
    default: break;
	}
	glutPostRedisplay() ;
}

// ===================================================================
// void specialKeys(int key, int x, int y)
// =================================================================== 
void specialKeys(int key, int x, int y) {
    switch(key) {
        case GLUT_KEY_RIGHT: {
					pink_box_pos[0] -= 0.1 ;
					obs1_pos[0] -= 0.1;
					obs2_pos[0] -= 0.1;
          break;
				}
        case GLUT_KEY_LEFT: {
					pink_box_pos[0] += 0.1;
					obs1_pos[0] += 0.1;
					obs2_pos[0] += 0.1;
          break;
				}
        case GLUT_KEY_UP: {
					pink_box_pos[2] += 0.1 ;
					obs1_pos[2] += 0.1;
					obs2_pos[2] += 0.1;
          break;
				}
        case GLUT_KEY_DOWN: {
					pink_box_pos[2] -= 0.1 ;
					obs1_pos[2] -= 0.1;
					obs2_pos[2] -= 0.1;
          break;
				}
    }
		float angle = getAngle( pink_box_pos );
		float len = getDistance( pink_box_pos );
		if ( angle > ANGLE_80 && angle < ( ANGLE_80 + ARC_20 ) && len < .7 && !arrived ) {
			sonny->playArrived();
			sonny->pauseObject( pb );
			arrived = true;
		} else if ( angle > ANGLE_80 && angle < ( ANGLE_80 + ARC_20 ) && len > .7 && !global_pause ) {
			sonny->unpauseObject( pb );
			arrived = false;
		}

#ifdef BOX
		// location information about the pink box
		float dist = listenerPos[2] - pink_box_pos[2];
		float side = pink_box_pos[0] - listenerPos[0];
		sonny->changeObjectLoc( pb, side, dist );					
#endif

		glutPostRedisplay() ;
}

void perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar ) {
	GLdouble fW, fH;
	fH = tan( fovY / 360 * pi ) * zNear;
	fW = fH * aspect;
	glFrustum( -fW, fW, -fH, fH, zNear, zFar );
}

float getDistance( ALfloat source[] ) {
	float x = listenerPos[0] - source[0];
	float z = source[2] - listenerPos[2];
	return sqrt( x*x + z*z );
}

float getAngle( ALfloat source[] ) {
	// printf( "list: %f\t src: %f\n", listenerPos[0], source[0] );
	float x = source[0] - listenerPos[0];
	float z = ( source[2] - listenerPos[2] )*-1.0f;
	float angle = atan2( z, x );
	// printf( "x: %f\t z: %f\t angle: %f\n", x, z, angle );
	return angle;
}

void alertTime() {
	float dist, side, angle;
	if ( time( NULL ) - last_alert > alert_inter ) {
		SonicDog::CoordinateVect obstacles;
		angle = getAngle( obs1_pos );
		if ( angle > angle61 && angle < angle119 && getDistance( obs1_pos ) < 2.5f ) {
			dist = listenerPos[2] - obs1_pos[2];
			side = obs1_pos[0] - listenerPos[0];
			obstacles.push_back( SonicDog::Coordinate( side, dist ) );
		}
		angle = getAngle( obs2_pos );
		if ( angle > angle61 && angle < angle119 && getDistance( obs2_pos ) < 2.5f ) {
			dist = listenerPos[2] - obs2_pos[2];
			side = obs2_pos[0] - listenerPos[0];
			obstacles.push_back( SonicDog::Coordinate( side, dist ) );
		}
		if ( obstacles.size() > 0 ) {
			sonny->alertObstacles( obstacles );
		}
		last_alert = time( NULL );
		obs1_dist = getDistance( obs1_pos );
		obs2_dist = getDistance( obs2_pos );
	}
}

void checkObstacles() {
	alertTime();
}
