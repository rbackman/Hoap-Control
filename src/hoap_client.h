
# ifndef CLIENT_H
# define CLIENT_H

# include <RakPeerInterface.h>
# include <MessageIdentifiers.h>
# include <GetTime.h>
# include <RakSleep.h>
#include <gsim/gs_vars.h>

# include "../../hoapnet/server/netapi_ids.h" // so that we use the same message ids
#include "../../hoapnet/shared/hoap_control.h"

class HoapClient
 { private :
    bool _connected;
    char _ip[32];
    int _port;
    RakPeerInterface * _peer;

   public :
    HoapClient();
   ~HoapClient();
    
    void setPort ( int p ) { _port=p; }
    void setIp ( const char* ip );

    void start ();

    void connect ();

    void disconnect ();
	bool isConnected(){return _connected;}
    void update ( RakNetTime curTime );

    void handle ( Packet* p );
    void getData();
    void sendHello();

	void sendMessage(rt_msg* mesg);

};

# endif // CLIENT_H
