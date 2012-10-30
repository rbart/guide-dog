#include <iostream>
#include <cmath>
#include <string>

#include "sonic_dog.h"

SonicDog::SonicDog( size_t threads ) {
	num_threads_ = threads;

	//alutInit( NULL, NULL );
	//checkError( "alutInit", ALUT );
	alutInitWithoutContext( NULL, NULL );

	device_ = alcOpenDevice( NULL );
	checkError( "alcOpenDevice", AL );
	if ( device_ ) {
		context_ = alcCreateContext( device_, NULL );
		checkError( "alcCreateContext", AL );
		alcMakeContextCurrent( context_ );
		checkError( "alcMakeContextCurrent", AL );
	}

	// set the position of the listener at the origin
	alListener3f( AL_POSITION, 0.0f, 0.0f, 1.0f );
	alListener3f( AL_VELOCITY, 0.0f, 0.0f, 0.0f );
	// ALfloat ori[] = { 0.0f, 0.0f, 1.0f };
	// alListenerfv( AL_ORIENTATION, ori );

	// initialize the object id
	object_id_ = 0;

	// initialize the locks
	pthread_mutex_init( &play_lock_, NULL );
	pthread_mutex_init( &sound_map_lock_, NULL );
	pthread_mutex_init( &q_lock_, NULL );
	pthread_cond_init( &empty_q_lock_, NULL );
	pthread_mutex_init( &turns_lock_, NULL );
}

SonicDog::~SonicDog() {
	pthread_mutex_lock( &q_lock_ );
	if ( !exit_ ) {
		pthread_mutex_unlock( &q_lock_ );
		stop();
	}
	pthread_mutex_unlock( &q_lock_ );

	alcMakeContextCurrent( NULL );
	alcDestroyContext( context_ );
	alcCloseDevice( device_ );

	// alutExit();
}

size_t SonicDog::addBeacon( float x, float z ) {
	checkError( "alutCreateBufferWavform", ALUT );
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
	alGenSources( 1, &(src->source) );
	checkError( "alGenSources", AL );
	switch ( type ) {
		case BEAC: {
			src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_WHITENOISE, 500.0f, 0.0f, 0.5f );
			alSourcef( src->source, AL_GAIN, 1.0f );
			break;
		}
		case OBS: {
			src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_SQUARE, object_id_*100.0f + 100.0f, 0.0f, 0.7f );
			alSourcef( src->source, AL_GAIN, 0.1f );
			break;
		}
		default: break;
	}

	alSourcef( src->source, AL_PITCH, 1 );
	alSource3f( src->source, AL_POSITION, x, z, 0.0f );
	alSource3f( src->source, AL_VELOCITY, 0.0f, 0.0f, 0.0f );
	alSourcei( src->source, AL_LOOPING, AL_FALSE );
	alSourcei( src->source, AL_BUFFER, src->buffer );

	// insert the src into our map
	pthread_mutex_lock( &sound_map_lock_ );
	sources_.insert( SoundMap::value_type( src->id, src ) );
	removed_.insert( BoolMap::value_type( src->id, false ) );
	pthread_mutex_unlock( &sound_map_lock_ );

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
	pthread_mutex_lock( &sound_map_lock_ );
	SoundMap::iterator beacon = sources_.find( id );
	pthread_mutex_unlock( &sound_map_lock_ );
	assert( beacon != sources_.end() );

	alSource3f( beacon->second->source, AL_POSITION, x, 0.0f, z );
}

bool SonicDog::getSoundPosition( const size_t id, ALfloat *x, ALfloat *y, ALfloat *z ) {
	SoundMap::iterator object = sources_.find( id );

	if ( object != sources_.end() ) {
		alGetSource3f( object->second->source, AL_POSITION, x, y, z );

		return true;
	} else {
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
		// then release the lock on the queue
		pthread_mutex_unlock( &q_lock_ );

		once = src->once;
		do {
			// play the sound at least once
			if ( once ) pthread_mutex_lock( &turns_lock_ );
			alSourcePlay( src->source );
			alutSleep( calculatePause( src->source ) );
			if ( once ) pthread_mutex_unlock( &turns_lock_ );

			// check that we can keep playing
			pthread_mutex_lock( &play_lock_ );
			can_play = playing_;
			pthread_mutex_unlock( &play_lock_ );

			if ( !once ) {
				pthread_mutex_lock( &sound_map_lock_ );
				BoolMap::iterator cont = removed_.find( src->id );
				if ( cont != removed_.end() ) {
					removed = cont->second;
				}
				pthread_mutex_unlock( &sound_map_lock_ );
			}
		} while ( can_play && !once && !removed );

		// remove this source from the source map if it's in there
		pthread_mutex_lock( &sound_map_lock_ );
		SoundMap::iterator itr = sources_.find( src->id );
		if ( itr != sources_.end() ) {
			sources_.erase( itr );
		}
		pthread_mutex_unlock( &sound_map_lock_ );

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

	return ( -1.0f*exp( 0.4f*d ) ) / ( -1.0f*exp( 0.4f*d ) - 1 );
}

void SonicDog::alertObstacles( const CoordinateVect &obstacles, bool diff ) {
	int size = obstacles.size();
	checkError( "alGenSources", AL );
	pthread_mutex_lock( &q_lock_ );
	for ( int i = 0; i < size; i++ ) {
		SoundSrc *src = new SoundSrc;
		src->once = true;
		alGenSources( 1, &(src->source) );
		if ( diff ) {
			src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_IMPULSE, i*100.0f+200.0f, 0.0f, 0.5f );
		} else {
			src->buffer = alutCreateBufferWaveform( ALUT_WAVEFORM_IMPULSE, 200.0f, 0.0f, 0.5f );
		}

		alSourcef( src->source, AL_PITCH, 1 );
		alSource3f( src->source, AL_POSITION, obstacles[i].first, 0.0f, obstacles[i].second );
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
	pthread_mutex_lock( &sound_map_lock_ );
	BoolMap::iterator src = removed_.find( id );
	pthread_mutex_unlock( &sound_map_lock_ );
	if ( src == removed_.end() ) return;

	src->second = true;
}
