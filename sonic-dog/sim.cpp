#include <ncurses.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <pthread.h>
#include <ctime>
#include <vector>
#include <algorithm>

#include "sonic_dog.h"

using std::cout;
using std::endl;
using std::find;

size_t beacon;
std::vector< size_t > objects;

bool init( SonicDog &sonny );
void userInteract( SonicDog &sonny );
void reset( SonicDog &sonny );

int main( int argc, char **argv ) {
	// initialize ncurses
	initscr();
	raw(); // disable line buffering to make characters available as soon as they're pressed
	keypad( stdscr, TRUE ); // enable capture of arrow keys and function keys
	noecho(); // don't echo characters pressed by user

	SonicDog sonny( 6 );

	init( sonny );
	userInteract( sonny );

	getch();
	endwin();

	return 0;
}

void changePosition( int ch, SonicDog &sonny ) {
	std::vector< size_t >::iterator itr;
	for ( itr = objects.begin(); itr != objects.end(); itr++ ) {
		ALfloat x, y, z;
		size_t id = *itr;
		sonny.getSoundPosition( id, &x, &y, &z );
		switch ( ch ) {
			case KEY_UP: { 
				sonny.changeObjectLoc( id, x, z-1.0f );
				break;
			}
			case KEY_DOWN: {
				sonny.changeObjectLoc( id, x, z+1.0f );
				break;
			}
			case KEY_LEFT: {
				sonny.changeObjectLoc( id, x+1.0f, z );
				break;
			}
			case KEY_RIGHT: { 
				sonny.changeObjectLoc( id, x-1.0f, z );
				break;
			}
			default:
				break;
		}
	}
}

void userInteract( SonicDog &sonny ) {
	printw( "Navigate toward the sound beacon using the arrow keys\n" );
	printw( "Press q to quit\n" );
	printw( "Press b to add a new beacon\n" );
	printw( "Press o to add an new obstacle\n" );
	printw( "Press r to remove object by id\n" );
	printw( "Press a to alert obstacles\n" );
	refresh();

	sonny.start();

	int ch;
	while( ( ch = getch() ) != 'q' ) {
		switch( ch ) {
			case KEY_UP:
			case KEY_DOWN:
			case KEY_LEFT:
			case KEY_RIGHT: {
				changePosition( ch, sonny );
				break;
			}
			case 'b': {
				reset( sonny );
				break;
			}
			case 'o': {
				srand( static_cast< unsigned int>( time( NULL ) ) );
				float x = static_cast< float >( ( rand() % 40 ) - 20 );
				float z = static_cast< float >( ( rand() % 40 ) - 20 );
				objects.push_back( sonny.addObstacle( x, z ) );
				break;
			}
			case 'a': {
				SonicDog::CoordinateVect obs;
				// obs.push_back( SonicDog::Coordinate( -5.0f, 0.0f ) );
				// obs.push_back( SonicDog::Coordinate( 5.0f, 0.0f ) );
				obs.push_back( SonicDog::Coordinate( 0.0f, -5.0f ) );
				obs.push_back( SonicDog::Coordinate( 0.0f, 5.0f ) );

				sonny.alertObstacles( obs, false );
				break;
			}
			case 'r': {
				printw( "Valid ids:" );
				for ( size_t i = 0; i < objects.size(); i++ ) {
					printw( " %zu", objects[i] );
				}
				printw( "\nOr type 'c' to cancel\n" );
				refresh();
				int input = ( getch() );
				if ( input == 'c' ) break;
				char buf[2];
				snprintf( buf, 2, "%c", input );	
				printw( "Deleting %s\n", buf );
				refresh();
				int id = atoi( buf );

				std::vector< size_t >::iterator itr = find( objects.begin(), objects.end(), static_cast< size_t>( id ) );
				if ( itr != objects.end() ) objects.erase( itr );

				sonny.removeObject( static_cast< size_t >( id ) );
				break;
			}
			default: {
				break;
			}
		}
	}

	printw( "\n\nBye Now!\n" );

	sonny.stop();
}

bool init( SonicDog &sonny ) {
	// srand( static_cast< unsigned int>( time( NULL ) ) );
	// float x = static_cast< float >( ( rand() % 40 ) - 20 );
	// float z = static_cast< float >( ( rand() % 40 ) - 20 );

	// beacon = sonny.addBeacon( x, z );
	// objects.push_back( beacon );

	return false;
}

void reset( SonicDog &sonny ) {
	srand( static_cast< unsigned int>( time( NULL ) ) );
	float x = static_cast< float >( ( rand() % 40 ) - 20 );
	float z = static_cast< float >( ( rand() % 40 ) - 20 );

	objects.push_back( sonny.addBeacon( x, z ) );
}

