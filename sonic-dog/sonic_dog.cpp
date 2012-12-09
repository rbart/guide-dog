#include <iostream>
#include <cmath>
#include <string>
#include <cstdio>

#include "sonic_dog.h"

SonicDog::SonicDog( size_t threads ) {
	num_threads_ = threads;

	// alutInit( NULL, NULL );
	// checkError( "clearing messages", AL );
	alutGetError();
	if ( alutInitWithoutContext( NULL, NULL ) == AL_FALSE ) {
		checkError( "alutInitWithoutContext", ALUT );
	}

	device_ = alcOpenDevice( NULL );
	if ( device_ ) {
		context_ = alcCreateContext( device_, NULL );
		if ( context_ ) {
			alcMakeContextCurrent( context_ );
		} else {
			checkError( "alcCreateContext", AL );
		}
	} else {
		checkError( "alcOpenDevice", AL );
	}

	// set the position of the listener at the origin
	alListener3f( AL_POSITION, 0.0f, 0.0f, 0.0f );
	alListener3f( AL_VELOCITY, 0.0f, 0.0f, 0.0f );
	// ALfloat ori[] = { 0.0f, 0.0f, 1.0f };
	// alListenerfv( AL_ORIENTATION, ori );

	// initialize the object id
	object_id_ = 1;

	// initialize the locks
	pthread_mutex_init( &play_lock_, NULL );
	pthread_mutex_init( &sources_lock_, NULL );
	pthread_mutex_init( &remove_lock_, NULL );
	pthread_mutex_init( &pause_lock_, NULL );
	pthread_mutex_init( &q_lock_, NULL );
	pthread_mutex_init( &turns_lock_, NULL );
	pthread_cond_init( &empty_q_lock_, NULL );
	pthread_cond_init( &pause_cond_lock_, NULL );

	// boolean to toggle regions on or off
	regions_ = false;
	cutoff_ = true;
}

SonicDog::~SonicDog() {
	pthread_mutex_lock( &q_lock_ );
	if ( !exit_ ) {
		pthread_mutex_unlock( &q_lock_ );
		stop();
	}
	pthread_mutex_unlock( &q_lock_ );

	// destroy the locks
	pthread_mutex_destroy( &play_lock_ );
	pthread_mutex_destroy( &sources_lock_ );
	pthread_mutex_destroy( &remove_lock_ );
	pthread_mutex_destroy( &pause_lock_ );
	pthread_mutex_destroy( &q_lock_ );
	pthread_mutex_destroy( &turns_lock_ );
	pthread_cond_destroy( &empty_q_lock_ );
	pthread_cond_destroy( &pause_cond_lock_ );

	alcMakeContextCurrent( NULL );
	alcDestroyContext( context_ );
	alcCloseDevice( device_ );

	// alutExit();
}

size_t SonicDog::addBeacon( float x, float z ) {
	checkError( "alutCreateBufferWavform", ALUT );
	start_d = z;
	return addObject( x, z, BEAC );
}

size_t SonicDog::addObstacle( float x, float z ) {
	checkError( "alutCreateBufferWavform", ALUT );
	return addObject( x, z, OBS );
}

size_t SonicDog::addObject( float x, float z, obj_t type ) {
	// instantiate the source and buffer for this sound source
	SoundSrc *src = new SoundSrc;
	src->id = object_id_;
	src->once = false;
	src->x = x;
	src->z = z;
	src->pause = 1.5;

	alGetError();
	alGenSources( 1, &(src->source) );
	checkError( "alGenSources", AL );

	switch ( type ) {
		case BEAC: {
			// src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_WHITENOISE, 500.0f, 0.0f, 0.5f );
			alutGetError();
			src->buffer = alutCreateBufferFromFile( "./beeps.wav" );
			checkError( "alutCreateBufferFromFile", ALUT );

			alSourcef( src->source, AL_GAIN, 5.0f );
			break;
		}
		case OBS: {
			alutGetError();
			src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_SQUARE, object_id_*100.0f + 100.0f, 0.0f, 0.7f );
			checkError( "alutCreateBufferWaveform", ALUT );

			alSourcef( src->source, AL_GAIN, 0.1f );
			break;
		}
		default: break;
	}

	alSourcef( src->source, AL_PITCH, 1 );
	alSource3f( src->source, AL_VELOCITY, 0.0f, 0.0f, 0.0f );
	alSourcei( src->source, AL_LOOPING, AL_FALSE );
	alSourcei( src->source, AL_BUFFER, src->buffer );

	if ( regions_ ) {
		// move the location of the source in the OpenAL world to one of our discrete zones
		ALfloat pos[] = { 0.0, 0.0, 0.0 };
		placeInRegion( x, z, pos );
		alSourcefv( src->source, AL_POSITION, pos );
	} else if ( cutoff_ ) {
		ALfloat pos[] = { 0.0, 0.0, 0.0 };
		placeInCutoff( x, z, pos );	
		alSourcefv( src->source, AL_POSITION, pos );
	} else {
		alSource3f( src->source, AL_POSITION, x, 0.0f, z );
	}

	// insert the src into our map
	pthread_mutex_lock( &sources_lock_ );
	sources_.insert( SoundMap::value_type( src->id, src ) );
	pthread_mutex_unlock( &sources_lock_ );

	// insert an entry for this source to our removed map
	pthread_mutex_lock( &remove_lock_ );
	removed_.insert( BoolMap::value_type( src->id, false ) );
	pthread_mutex_unlock( &remove_lock_ );

	// insert an entry for this source to our paused map
	pthread_mutex_lock( &pause_lock_ );
	paused_.insert( BoolMap::value_type( src->id, false ) );
	pthread_mutex_unlock( &pause_lock_ );

	// alSourcePlay( src->source );

	// then insert it into the play queue
	pthread_mutex_lock( &q_lock_ );
	play_q_.push( src );
	pthread_mutex_unlock( &q_lock_ );

	// alert the threads that a new object has been added
	pthread_cond_broadcast( &empty_q_lock_ );

	// increment the id counter
	object_id_++;

	return src->id;
}

void SonicDog::changeObjectLoc( size_t id, float x, float z ) {
	pthread_mutex_lock( &sources_lock_ );
	SoundMap::iterator beacon = sources_.find( id );
	pthread_mutex_unlock( &sources_lock_ );
	assert( beacon != sources_.end() );

	beacon->second->x = x;
	beacon->second->z = z;

	if ( regions_ ) {
		ALfloat pos[] = { 0.0, 0.0, 0.0 };
		placeInRegion( x, z, pos );
		alSourcefv( beacon->second->source, AL_POSITION, pos );
	} else if ( cutoff_ ) {
		ALfloat pos[] = { 0.0, 0.0, 0.0 };
		placeInCutoff( x, z, pos );
		alSourcefv( beacon->second->source, AL_POSITION, pos );
	} else {
		alSource3f( beacon->second->source, AL_POSITION, x, 0.0f, z );
	}
}

bool SonicDog::getSoundPosition( const size_t id, ALfloat *x, ALfloat *y, ALfloat *z ) {
	pthread_mutex_lock( &sources_lock_ );
	SoundMap::iterator object = sources_.find( id );

	if ( object != sources_.end() ) {
		alGetSource3f( object->second->source, AL_POSITION, x, y, z );
		pthread_mutex_unlock( &sources_lock_ );

		return true;
	} else {
		pthread_mutex_unlock( &sources_lock_ );
		return false;
	}
}

void SonicDog::start() {
	pthread_mutex_lock( &play_lock_ );
	playing_ = true;
	pthread_mutex_unlock( &play_lock_ );
	pthread_mutex_lock( &q_lock_ );
	exit_ = false;
	pthread_mutex_unlock( &q_lock_ );

	// create the thread pool
	// threads_ = new pthread_t[ NUM_THREADS ];

	data_ = new worker_dat;
	data_->sd = this;
	data_->func = &SonicDog::startPlaying;

	for ( size_t i = 0; i < num_threads_; i++ ) {
		pthread_t thread;
		pthread_create( &thread, NULL, &SonicDog::worker_fn, static_cast< void * >( data_ ) );
		threads_.push_back( thread );
	}
}

void SonicDog::startPlaying() {
	// find out if the client wants the sound to play
	bool can_play, once, removed;
	removed = false;
	pthread_mutex_lock( &play_lock_ );
	can_play = playing_;
	pthread_mutex_unlock( &play_lock_ );

	while ( can_play ) {
		pthread_mutex_lock( &q_lock_ );
		while ( play_q_.size() == 0 && !exit_ ) {
			pthread_cond_wait( &empty_q_lock_, &q_lock_ );
		}
		// check that we can keep playing after being woken up
		if ( exit_ ) {
			pthread_mutex_unlock( &q_lock_ );
			pthread_exit( 0 );
		}

		// there's stuff in the queue so pop off a source
		SoundSrc *src = play_q_.front();
		play_q_.pop();
		pthread_mutex_unlock( &q_lock_ );

		once = src->once;

		do {
			// play the sound at least once
			if ( once ) pthread_mutex_lock( &turns_lock_ );

			alSourcePlay( src->source );
			alutSleep( src->pause );
			if ( once ) pthread_mutex_unlock( &turns_lock_ );
			if ( !once ) alSourcef( src->source, AL_PITCH, calculatePitch( src->source ) );


			// check that we can keep playing
			pthread_mutex_lock( &play_lock_ );
			can_play = playing_;
			pthread_mutex_unlock( &play_lock_ );

			if ( !once ) {
				bool paused;

				// check if we're paused
				pthread_mutex_lock( &pause_lock_ );
				BoolMap::iterator p = paused_.find( src->id );
				assert( p != paused_.end() );
				paused = p->second;
				while ( paused ) {
					pthread_cond_wait( &pause_cond_lock_, &pause_lock_ );
					BoolMap::iterator p = paused_.find( src->id );
					assert( p != paused_.end() );
					paused = p->second;
				}
				pthread_mutex_unlock( &pause_lock_ );

				// check if we've been removed
				pthread_mutex_lock( &remove_lock_ );
				BoolMap::iterator cont = removed_.find( src->id );
				if ( cont != removed_.end() ) {
					removed = cont->second;
				}
				pthread_mutex_unlock( &remove_lock_ );
			}
		} while ( can_play && !once && !removed );

		// remove this source if wasn't meant to be played only once
		if ( !src->once ) {
			// remove this source from the source map if it's in there
			pthread_mutex_lock( &sources_lock_ );
			SoundMap::iterator itr = sources_.find( src->id );
			if ( itr != sources_.end() ) {
				sources_.erase( itr );
			}
			pthread_mutex_unlock( &sources_lock_ );
		}

		// we're done with the source so clean it up
		alDeleteSources( 1, &(src->source) );
		alDeleteBuffers( 1, &(src->buffer) );
		delete src;
	}

	// we've been stopped, so exit the thread
	pthread_exit( 0 );
}

void SonicDog::stop( void ) {
	pthread_mutex_lock( &play_lock_ );
	playing_ = false;
	pthread_mutex_unlock( &play_lock_ );
	pthread_mutex_lock( &q_lock_ );
	exit_ = true;
	pthread_mutex_unlock( &q_lock_ );
	pthread_cond_broadcast( &empty_q_lock_ );
	
	// indicate removal and then unpause anything in the paused map
	pthread_mutex_lock( &pause_lock_ );
	BoolMap::iterator p;
	for ( p = paused_.begin(); p != paused_.end(); p++ ) {
		removeObject( p->first );
		p->second = false;
	}
	pthread_mutex_unlock( &pause_lock_ );
	pthread_cond_broadcast( &pause_cond_lock_ );

	for ( size_t i = 0; i < threads_.size(); i++ ) {
		pthread_join( threads_[i], 0 );
	}
	threads_.clear();

	delete data_;
}

void SonicDog::checkError( const char *msg, al_t type ) {
	ALenum err;
	switch ( type ) {
		case ALUT: {
			err = alutGetError();
			if ( err == ALUT_ERROR_NO_ERROR ) {
				error_ = false;
			} else {
				displayAlutError( msg, err );
				error_ = true;
			}
		}
		case AL:
		default: {
			err = alGetError();
			if ( err == AL_NO_ERROR ) {
				error_ = false;
			} else {
				displayALError( msg, err );
				error_ = true;
			}
		}
	}
}

void SonicDog::displayALError( const char *msg, ALenum err ) {
	const char *errMsg = NULL;
	switch ( err ) {
		case AL_NO_ERROR: {
			errMsg = "None"; 
			break;
		}
		case AL_INVALID_NAME: {
			errMsg = "Invalid name.";
			break;
		}
		case AL_INVALID_ENUM: {
			errMsg = "Invalid enum.";
			break;
		}
		case AL_INVALID_VALUE: {
			errMsg = "Invalid value.";
			break;
		}
		case AL_INVALID_OPERATION: {
			errMsg = "Invalid operation.";
			break;
		}
		case AL_OUT_OF_MEMORY: {
			errMsg = "Out of memory.";
			break;
		}
		default: {
			errMsg = "Unknown error.";
			break;
		}
	}
	std::cerr << msg << " : " << errMsg << std::endl;
} 

void SonicDog::displayAlutError( const char *msg, ALenum err ) {
	std::cerr << msg << " : " << alutGetErrorString( err ) << std::endl;
}

float SonicDog::calculatePause( ALuint source ) { 
	ALfloat x, y, z;
	alGetSource3f( source, AL_POSITION, &x, &y, &z );
	float d = sqrt( (x*x) + (z*z) );

	return ( -1.5f*exp( 0.6f*d ) ) / ( -1.0f*exp( 0.6f*d ) - 1.5f );
}

float SonicDog::calculatePitch( ALuint source ) {
	ALfloat x, y, z;
	alGetSource3f( source, AL_POSITION, &x, &y, &z );
	float d = sqrt( (x*x) + (z*z) );

	float p = ( 1.0f - (d / start_d) ) / 2.0f;
	return p + 1.0f;
}

void SonicDog::alertObstacles( const CoordinateVect &obstacles, bool diff ) {
	int size = obstacles.size();
	// checkError( "alGenSources", AL );
	pthread_mutex_lock( &q_lock_ );
	if ( play_q_.size() > 0 ) {
		pthread_mutex_unlock( &q_lock_ );
		return;
	}

	for ( int i = 0; i < size; i++ ) {
		SoundSrc *src = new SoundSrc;
		src->once = true;
		src->x = obstacles[i].first;
		src->z = obstacles[i].second;
		src->pause = 1.5;

		alGetError();
		alGenSources( 1, &(src->source) );
		checkError( "alGenSources", AL );

		alutGetError();
		if ( diff ) {
			src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_IMPULSE, i*100.0f+200.0f, 0.0f, 0.5f );
		} else {
			// src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_IMPULSE, 400.0f, 0.0f, 0.5f );
			src->buffer = alutCreateBufferFromFile( "./ding.wav" );
		}

		if ( src->buffer == AL_NONE ) {
			checkError( "alutCreateBufferWaveform", ALUT );
		}

		if ( regions_ ) {
			// move the location of the source in the OpenAL world to one of our discrete zones
			ALfloat pos[] = { 0.0, 0.0, 0.0 };
			placeInRegion( src->x, src->z, pos );
			alSourcefv( src->source, AL_POSITION, pos );
		} else if ( cutoff_ ) {
			ALfloat pos[] = { 0.0, 0.0, 0.0 };
			placeInCutoff( src->x, src->z, pos );
			alSourcefv( src->source, AL_POSITION, pos );
		} else {
			alSource3f( src->source, AL_POSITION, src->x, 0.0f, src->z );
		}

		alSourcef( src->source, AL_PITCH, 1 );
		alSource3f( src->source, AL_VELOCITY, 0.0f, 0.0f, 0.0f );
		alSourcei( src->source, AL_LOOPING, AL_FALSE );
		alSourcei( src->source, AL_BUFFER, src->buffer );

		play_q_.push( src );
	}
	pthread_mutex_unlock( &q_lock_ );
	pthread_cond_broadcast( &empty_q_lock_ );
}

void *SonicDog::worker_fn( void *data ) {
	worker_dat *dat = static_cast< worker_dat * >( data );
	SonicDog *sd = dat->sd;
	void ((SonicDog::*f)()) = dat->func;
	(sd->*f)();
	return NULL;
}

void SonicDog::removeObject( size_t id ) {
	pthread_mutex_lock( &remove_lock_ );
	BoolMap::iterator src = removed_.find( id );
	pthread_mutex_unlock( &remove_lock_ );
	if ( src == removed_.end() ) return;

	src->second = true;
}

void SonicDog::pauseObject( size_t id ) {
	pthread_mutex_lock( &pause_lock_ );
	BoolMap::iterator p = paused_.find( id );
	if ( p != paused_.end() ) {
		p->second = true;
	}
	pthread_mutex_unlock( &pause_lock_ );
}

void SonicDog::unpauseObject( size_t id ) {
	pthread_mutex_lock( &pause_lock_ );
	BoolMap::iterator p = paused_.find( id );
	if ( p != paused_.end() ) {
		p->second = false;
	}
	pthread_mutex_unlock( &pause_lock_ );
	pthread_cond_broadcast( &pause_cond_lock_ );
}

float SonicDog::getAngle( float x, float z ) {
	return atan2( z, x );
}

void SonicDog::placeInRegion( float x, float z, ALfloat *pos ) {
	float angle = getAngle( x, z );
	// printf( "SD: x: %f\t z: %f\t angle: %f\n", x, z, angle );
	if ( angle < ANGLE_80 ) {
		// source is to the right of listener
		pos[0] = z;
	} else if ( angle > ( ANGLE_80 + ARC_20 ) ) {
		// source is to the left of listener
		pos[0] = -z;
	} else {
		// source is in front of the listener
		pos[2] = z;
	}
}

void SonicDog::placeInCutoff( float x, float z, ALfloat *pos ) {
	float angle = getAngle( x, z );
	float dist = sqrt( (x*x) + (z*z) );
//	printf( "SD: x: %f\t z: %f\t angle: %f\n", x, z, angle );
	if ( angle < ANGLE_70 ) {
		// source is considered to the right of the listener
		pos[0] = dist;
	} else if ( angle > ( ANGLE_70 + ARC_40 ) ) {
		// source is considered to the left of the listener
		pos[0] = -dist;
	} else if ( angle > ANGLE_80 && angle < ( ANGLE_80 + ARC_20 ) ) {
		// source is considered in front of the listener
		pos[2] = dist;
	} else if ( angle < ANGLE_80 || angle > ( ANGLE_80 + ARC_20 ) ) {
		// gradually move the source to one side
		float map_angle = 2*angle - 140;
		pos[0] = dist*cos( map_angle );
		pos[2] = dist*sin( map_angle );
	}
}

void SonicDog::turnRegionsOn( void ) {
	regions_ = true;
	cutoff_ = false;
	pthread_mutex_lock( &sources_lock_ );
	for ( SoundMap::iterator itr = sources_.begin(); itr != sources_.end(); itr++ ) {
		ALfloat pos[] = { 0.0, 0.0, 0.0 };
		placeInRegion( itr->second->x, itr->second->z, pos );
		alSourcefv( itr->second->source, AL_POSITION, pos );
	}
	pthread_mutex_unlock( &sources_lock_ );
}

void SonicDog::turnRegularOn( void ) {
	regions_ = false;
	cutoff_ = false;
	pthread_mutex_lock( &sources_lock_ );
	for ( SoundMap::iterator itr = sources_.begin(); itr != sources_.end(); itr++ ) {
		alSource3f( itr->second->source, AL_POSITION, itr->second->x, 0.0f, itr->second->z );
	}
	pthread_mutex_unlock( &sources_lock_ );
}

void SonicDog::turnCutoffOn( void ) {
	regions_ = false;
	cutoff_ = true;
	for ( SoundMap::iterator itr = sources_.begin(); itr != sources_.end(); itr++ ) {
		ALfloat pos[] = { 0.0, 0.0, 0.0 };
		placeInCutoff( itr->second->x, itr->second->z, pos );
		alSourcefv( itr->second->source, AL_POSITION, pos );
	}
	pthread_mutex_unlock( &sources_lock_ );
}

void SonicDog::playArrived( void ) {
	SoundSrc *src = new SoundSrc;
	src->once = true;
	src->x = 0.0;
	src->z = 0.0;
	src->pause = 2.5;

	alGetError();
	alGenSources( 1, &(src->source) );
	checkError( "alGenSources", AL );

	alutGetError();
	src->buffer = alutCreateBufferFromFile( "./radio_jingle.wav" );

	if ( src->buffer == AL_NONE ) {
		checkError( "alutCreateBufferWaveform", ALUT );
		return;
	}

	alSource3f( src->source, AL_POSITION, 0.0f, 0.0f, 0.0f );

	alSourcef( src->source, AL_PITCH, 1 );
	alSource3f( src->source, AL_VELOCITY, 0.0f, 0.0f, 0.0f );
	alSourcei( src->source, AL_LOOPING, AL_FALSE );
	alSourcei( src->source, AL_BUFFER, src->buffer );

	pthread_mutex_lock( &q_lock_ );
	play_q_.push( src );
	pthread_mutex_unlock( &q_lock_ );
	pthread_cond_broadcast( &empty_q_lock_ );
}
