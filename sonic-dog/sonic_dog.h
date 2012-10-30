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

	size_t num_threads_;

	ALCcontext *context_;
	ALCdevice *device_;

	ThreadVect threads_;
	worker_dat *data_;

	bool error_;
	sdString error_str_;

	bool playing_;
	bool exit_;
	pthread_mutex_t play_lock_;
	pthread_mutex_t q_lock_;
	pthread_cond_t empty_q_lock_;
	pthread_mutex_t sound_map_lock_;

	/// @brief A lock that requires the threads playing a sound only once to play 
	//    them in succession instead of all at once.
	pthread_mutex_t turns_lock_;
	
	SoundMap sources_;
	size_t object_id_;
	SoundQ play_q_;
	BoolMap removed_;

	static void *worker_fn( void *data );
	void (SonicDog::*func())() {
		return &SonicDog::startPlaying;
	}

	void checkError( const char *msg, al_t type );
	void displayALError( const char *msg, ALenum err );
	void displayAlutError( const char *msg, ALenum err );
	size_t addObject( float x, float z, obj_t type );
	float calculatePause( ALuint source );

};

#endif /* SONIC_DOG_h */
