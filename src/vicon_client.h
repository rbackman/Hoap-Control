# pragma once


#include "RakPeerInterface.h"
#include "RakNetworkFactory.h"
#include "BitStream.h"
#include "Rand.h"
#include "RakNetStatistics.h"
#include "MessageIdentifiers.h"
#include "Kbhit.h"
#include "GetTime.h"
#include "RakAssert.h"
#include "RakSleep.h"
#include "BitStream.h"

#include <iostream>
#include <string>

//# include "ctlapp_main.h"


//#define SERVER_IP "127.0.0.1"
#define SERVER_IP "169.236.143.161"
#define SERVER_PORT 12345
#define RANDOM_DATA_SIZE 200

#define PACKET_RELIABILITY RELIABLE_ORDERED
#define PACKET_PRIORITY HIGH_PRIORITY


using namespace RakNet;

static const int NUM_CLIENTS = 64;

enum MSGFromClient
{
    ID_VICON_HELLO_MSG = ID_USER_PACKET_ENUM,
    ID_VICON_STRING,
	ID_OBJ_NUM_POS_ROT
};


//struct CtlData
//{
//	float val[50];
//};

class ViconClient
{
public:

	ViconClient() { peer = RakNetworkFactory::GetRakPeerInterface();}
    ~ViconClient() { RakNetworkFactory::DestroyRakPeerInterface(peer); }

    RakPeerInterface * peer;

	int obj_num; // object number to send
	GsVec pos_data; // it's global position
	GsQuat ori_data;// it's global rotation


	bool isConnected;

    void Startup(void)
    {
        SocketDescriptor socketDescriptor;
        socketDescriptor.port = 0;
        peer->Startup(1,30,&socketDescriptor,1);
        isConnected = false;

        this->Connect();
    }
    void Connect(void)
    {
        bool b;
        b=peer->Connect(SERVER_IP, (unsigned short) SERVER_PORT, 0, 0, 0);
        if (b == false) // if true does not mean that the server has accepted the connection
            std::cout << "ViconClient connect call failed!" << std::endl;
		else isConnected = true;
    }
    void Disconnect(void)
    {
        peer->CloseConnection( peer->GetSystemAddressFromIndex(0) ,true ,0 );
        isConnected = false;
    }
    void Update(RakNetTime curTime)
    {
        Packet *p = peer->Receive();
        while (p)
        {
            switch (p->data[0])
            {
            case ID_CONNECTION_REQUEST_ACCEPTED:
                isConnected = true;
                break;
                // print out errors
            case ID_CONNECTION_ATTEMPT_FAILED:
                std::cout << "ViconClient Error: ID_CONNECTION_ATTEMPT_FAILED" << std::endl;
                isConnected = false;
                this->Connect();
                break;
            case ID_ALREADY_CONNECTED:
                std::cout << "ViconClient Error: ID_ALREADY_CONNECTED" << std::endl;
                break;
            case ID_NO_FREE_INCOMING_CONNECTIONS:
                std::cout << "ViconClient Error: ID_NO_FREE_INCOMING_CONNECTIONS" << std::endl;
                isConnected = false;
                this->Connect();
                break;
            case ID_DISCONNECTION_NOTIFICATION:
                isConnected = false;
                break;
            case ID_CONNECTION_LOST:
                std::cout << "ViconClient Error: ID_CONNECTION_LOST" << std::endl;
                isConnected = false;
                this->Connect();
                break;
            case ID_MODIFIED_PACKET:
                std::cout << "ViconClient Error: ID_MODIFIED_PACKET" << std::endl;
                break;
            default:
                handleUserMessages(p);
                break;
            }
            peer->DeallocatePacket(p);
            p = peer->Receive();
        }
    }

    void handleUserMessages(Packet* p)
    {
        switch (p->data[0])
        {
            case ID_VICON_HELLO_MSG:
                std::cout << "Hello from: " << p->systemAddress.ToString() << std::endl;
                break;
            case ID_VICON_STRING:
				{
					RakNet::BitStream bs(p->data, p->length, true);
					unsigned char id;
					unsigned int lenght;
					bs.Read(id);
					bs.ReadCompressed(lenght);
					char* cstr = new char[lenght];
					bs.ReadAlignedBytes((unsigned char*) &cstr, lenght);
	                
					std::string str(cstr);
					std::cout << "App: " << p->systemAddress.ToString() << " received string: " << str << std::endl;
					delete [] cstr;
				} break;

         

			case ID_OBJ_NUM_POS_ROT:
				{
					float number[8];
					//gsout<<"new pos data"<<gsnl;
					RakNet::BitStream bs(p->data, p->length, true);
					unsigned char id;
					bs.Read(id);
					bs.ReadAlignedBytes((unsigned char*) &number, sizeof(number) );
					obj_num = (int)number[0];
					pos_data.x = number[1];
					pos_data.y = number[2];
					pos_data.z = number[3];
					ori_data.w = number[4];
					ori_data.x = number[5];
					ori_data.y = number[6];
					ori_data.z = number[7];
				} break;

        }
    }

    void SendHello()
    {
        RakNet::BitStream bs;
        bs.Write((unsigned char) ID_VICON_HELLO_MSG);
        peer->Send(&bs, PACKET_PRIORITY, PACKET_RELIABILITY, 0, UNASSIGNED_SYSTEM_ADDRESS, true);
    }

    void SendString()
    {
        std::string str( "server text server text server text server text" );
        RakNet::BitStream bs;
        bs.Write((unsigned char) ID_VICON_STRING);
        bs.WriteCompressed((unsigned int) str.length() + 1);
        bs.WriteAlignedBytes((const unsigned char*) str.c_str(), str.length() + 1);
        peer->Send(&bs, PACKET_PRIORITY, PACKET_RELIABILITY, 0, UNASSIGNED_SYSTEM_ADDRESS, true);
    }

    
    void SendObjNumPosOri( float& number ) // position and orientation
    {
        RakNet::BitStream bs;
        bs.Write((unsigned char) ID_OBJ_NUM_POS_ROT);
        bs.WriteAlignedBytes((const unsigned char*) &number, sizeof(number)*8 );
        peer->Send(&bs, PACKET_PRIORITY, PACKET_RELIABILITY, 0, UNASSIGNED_SYSTEM_ADDRESS, true);
    }

};
