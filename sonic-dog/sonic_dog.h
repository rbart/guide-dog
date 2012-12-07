#ifndef SONIC_DOG_h
#define SONIC_DOG_h

#include <string>
#include <AL/alut.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <pthread.h>
#include <ctime>
#include <vector>
#include <map>
#include <cassert>
#include <queue>

class SonicDog {

public:
	typedef std::string sdString;
	typedef std::pair< float, float > Coordinate;
	typedef std::vector< Coordinate > CoordinateVect;

	SonicDog( size_t threads );
	~SonicDog();
	size_t addBeacon( float x, float z );
	void changeObjectLoc( size_t id, float x, float z );
	bool getSoundPosition( const size_t id, ALfloat *x, ALfloat *y, ALfloat *z );
	void start( void );
	void stop( void );
	size_t addObstacle( float x, float z );
	void alertObstacles( const CoordinateVect &obstacles, bool diff = false );
	void startPlaying( void );
	void removeObject( size_t id );
	void pauseObject( size_t id );
	void unpauseObject( size_t id );
	void turnRegionsOn( void );
	void turnRegionsOff( void );

private:
	typedef std::vector< ALuint > AluVect;

	typedef enum {
		AL,
		ALUT
	} al_t;

	typedef enum {
		BEAC,
		OBS
	} obj_t;

	typedef struct {
		size_t id;
		ALuint source;
		ALuint buffer;
		bool once;
		float x;
		float z;
	} SoundSrc;

	typedef struct {
		SonicDog *sd;
		void (SonicDog::*func)();
	} worker_dat;

	typedef std::map< size_t, SoundSrc * > SoundMap;
	typedef std::queue< SoundSrc > SoundQueue;
	typedef std::queue< SoundSrc * > SoundQ;
	typedef std::vector< pthread_t > ThreadVect;
	typedef std::map< size_t, bool > BoolMap;

	// the number of radians to indicate a hard left/right
	static const float HARD_DIR = 1.3962634;
	// the 20 degree arc that's considered in the middle
	static const float STRAIGHT = 0.34906585;
	float start_d;

	size_t num_threads_;

	ALCcontext *context_;
	ALCdevice *device_;

	ThreadVect threads_;
	worker_dat *data_;

	bool error_;
	sdString error_str_;

	bool regions_;
	bool playing_;
	bool exit_;
	pthread_mutex_t play_lock_;
	pthread_mutex_t q_lock_;
	pthread_cond_t empty_q_lock_;
	pthread_cond_t pause_cond_lock_;
	pthread_mutex_t sources_lock_;
	pthread_mutex_t remove_lock_;
	pthread_mutex_t pause_lock_;

	/// @brief A lock that requires the threads playing a sound only once to play 
	//    them in succession instead of all at once.
	pthread_mutex_t turns_lock_;
	
	SoundMap sources_;
	size_t object_id_;
	SoundQ play_q_;
	BoolMap removed_;
	BoolMap paused_;

	static void *worker_fn( void *data );
	void (SonicDog::*func())() {
		return &SonicDog::startPlaying;
	}

	void checkError( const char *msg, al_t type );
	void displayALError( const char *msg, ALenum err );
	void displayAlutError( const char *msg, ALenum err );
	size_t addObject( float x, float z, obj_t type );
	float calculatePause( ALuint source );
	float calculatePitch( ALuint source );
	float getAngle( float x, float z );
	void placeInRegion( float x, float z, ALfloat *pos );

};

#endif /* SONIC_DOG_h */
