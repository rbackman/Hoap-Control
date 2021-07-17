
# ifndef CLIENT_H
# define CLIENT_H

# include <RakPeerInterface.h>
# include <MessageIdentifiers.h>
# include <GetTime.h>
# include <RakSleep.h>
# include "../../hoapnet/server/netapi_ids.h" // so that we use the same message ids
#include "../../hoapnet/server/robotapi.h"
class Client
 { private :
    bool _connected;
    char _ip[32];
    int _port;
    RakPeerInterface * _peer;

   public :
    Client();
   ~Client();
    
    void setPort ( int p ) { _port=p; }
    void setIp ( const char* ip );

    void start ();

    void connect ();

    void disconnect ();
	bool isConnected(){return _connected;}
    void update ( RakNetTime curTime );

    void handle ( Packet* p );

    void sendHello();
    void getPosture();
    void sendPosture(HoapPosture* posture);
	void sendPostureQueue(HoapPosture* posture);
};

# endif // CLIENT_H
