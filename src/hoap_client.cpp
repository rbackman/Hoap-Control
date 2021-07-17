
#include "hoap_client.h"
#include "app_main.h"
#include "app_main_win.h"

#include <RakPeerInterface.h>
#include <RakNetworkFactory.h>
#include <BitStream.h>
#include <Rand.h>
#include <RakNetStatistics.h>
#include <MessageIdentifiers.h>
#include <Kbhit.h>
#include <GetTime.h>
#include <RakAssert.h>
#include <RakSleep.h>
#include <BitStream.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#ifdef _WIN32
#include "WindowsIncludes.h" // Sleep
#pragma warning(disable : 4996)
#else
#include <unistd.h> // usleep
#include <cstdio>
#endif

#define SERVER_PORT 12345

#define PACKET_RELIABILITY RELIABLE_ORDERED
#define PACKET_PRIORITY HIGH_PRIORITY

HoapClient::HoapClient ()
 { 
   _peer = RakNetworkFactory::GetRakPeerInterface();
   _port = SERVER_PORT;
   strcpy ( _ip, "127.0.0.1" );
   _connected = false;
 }

HoapClient::~HoapClient ()
 {
   RakNetworkFactory::DestroyRakPeerInterface ( _peer );
 }

void HoapClient::setIp ( const char* ip )
 {
   strcpy ( _ip, ip );
 }

void HoapClient::start ()
 {
   SocketDescriptor socketDescriptor;
   socketDescriptor.port = 0;
   //Startup: The first parameter is the maximum mumber of connections. For a pure client, we use 1.
   //The second parameter (set to 30 in this example) is the thread sleep timer. 
   //A value of 0 is good for games that need fast responses, such as a shooter. 
   //Otherwise, a value of 30 will give good response times and will require very little CPU usage.
   _peer->Startup(1,30,&socketDescriptor,1);
   _connected = false;
   this->connect();
 }

void HoapClient::connect ()
 {
    bool b;
    b = _peer->Connect ( _ip, (unsigned short)_port, 0, 0, 0);
	if (b == false) {// if true does not mean that the server has accepted the connection
     App->mainwin->message("HoapClient connect call failed!");
	 
	}
	else {
	//	_connected = true;
		
	}
 }

void HoapClient::disconnect ()
 {
   _peer->CloseConnection( _peer->GetSystemAddressFromIndex(0) ,true ,0 );
   _connected = false;
 }

void HoapClient::update ( RakNetTime curTime )
 {
   while (true)
    { Packet *p = _peer->Receive();
      if ( !p ) break;
      switch (p->data[0])
       {
         case ID_CONNECTION_REQUEST_ACCEPTED:
              _connected = true;
			   App->mainwin->message("hoap connected");
              break;
              // print out errors
         case ID_CONNECTION_ATTEMPT_FAILED:
			  App->mainwin->message("hoap connection failed");
             // std::cout << "HoapClient Error: ID_CONNECTION_ATTEMPT_FAILED" << std::endl;
              _connected = false;
              this->connect();
              break;
         case ID_ALREADY_CONNECTED:
              App->mainwin->message("HoapClient Error: ID_ALREADY_CONNECTED");
              break;
         case ID_NO_FREE_INCOMING_CONNECTIONS:
              App->mainwin->message("HoapClient Error: ID_NO_FREE_INCOMING_CONNECTIONS");
              _connected = false;
              this->connect();
              break;
         case ID_DISCONNECTION_NOTIFICATION:
              _connected = false;
              break;
         case ID_CONNECTION_LOST:
               App->mainwin->message("HoapClient Error: ID_CONNECTION_LOST");
              _connected = false;
              this->connect();
              break;
         case ID_MODIFIED_PACKET:
               App->mainwin->message("HoapClient Error: ID_MODIFIED_PACKET");
              break;
         default:
              handle(p);
              break;
        }
       _peer->DeallocatePacket(p);
     }
 }

void HoapClient::handle ( Packet* p )
 {
	// printf("packed id %d\n",p->data[0]);
    switch (p->data[0])
     {

       case ID_HELLO_MSG:
            std::cout << "Hello from: " << p->systemAddress.ToString() << std::endl;
            break;


	   case ID_LOG:
		   unsigned char id;
		   RakNet::BitStream bs(p->data, p->length, true);

		    hoap_data_log log;
			bs.Read(id);
			bs.ReadAlignedBytes((unsigned char*) &log, sizeof(hoap_data_log));
			App->mainwin->showLog(log);
			//for (int i=0; i<8; i++) gsout << log.fsr[i]<<" ";
			//gsout<<gsnl;
			/*App->sk->update_global_matrices();
			GsVec dst = App->sk->joint("RightHandIKEnd")->gcenter()-App->sk->joint("Head")->gcenter();
			gsout<< dst <<gsnl;*/
		   break;
    }
 }

void HoapClient::sendHello()
 {
   RakNet::BitStream bs;
   bs.Write((unsigned char) ID_HELLO_MSG);
   _peer->Send(&bs, PACKET_PRIORITY, PACKET_RELIABILITY, 0, UNASSIGNED_SYSTEM_ADDRESS, true);
 }


void HoapClient::getData()
{
	RakNet::BitStream bs;
	bs.Write((unsigned char) ID_GET_DATA);
	_peer->Send(&bs, PACKET_PRIORITY, PACKET_RELIABILITY, 0, UNASSIGNED_SYSTEM_ADDRESS, true);
}


void HoapClient::sendMessage(rt_msg* mesg)
{
	RakNet::BitStream bs;
	bs.Write((unsigned char) ID_MESG);
	bs.WriteAlignedBytes((const unsigned char*) mesg, sizeof(rt_msg));
	_peer->Send(&bs, PACKET_PRIORITY, PACKET_RELIABILITY, 0, UNASSIGNED_SYSTEM_ADDRESS, true);
}



