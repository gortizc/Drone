/*!
 * Simple chat program (server side).cpp - http://github.com/hassanyf
 * Version - 2.0.1
 *
 * Copyright (c) 2016 Hassan M. Yousuf
 */
#include <unistd.h>


#include "../cpp/driver/everloop_image.h"
#include "../cpp/driver/everloop.h"
#include "../cpp/driver/wishbone_bus.h"

#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>


namespace hal = matrix_hal;

const uint16_t kMCUBaseAddress = 0x3800;
const uint16_t kMemoryOffsetMCU = 0x90;
const uint16_t kMemoryOffsetSamCommunication=0xF0;


using namespace std;


void turnAllTheLeds(hal::EverloopImage* led_image,int color, int var)
{	
	for (hal::LedValue& led : led_image->leds) {
		switch (color)
		{
			case 1:
				led.red=var;
				led.green=0;
				led.blue=0;
				led.white=0;
			break;
			case 2:
				led.red=0;
				led.green=var;
				led.blue=0;
				led.white=0;
			break;
			case 3:
				led.red=0;
				led.green=0;
				led.blue=var;
				led.white=0;
			break;
			case 4:
				led.red=0;
				led.green=0;
				led.blue=0;
				led.white=var;
			break;
			case 5:
				led.red=0;
				led.green=0;
				led.blue=0;
				led.white=0;
			break;
			default:
				led.red=0;
				led.green=0;
				led.blue=0;
				led.white=0;
			break;

		}
	}
	
}

int main()
{
	hal::WishboneBus bus;
	bus.SpiInit();  	
	hal::Everloop everloop;
	hal::EverloopImage image1d;
	everloop.Setup(&bus);
	
	unsigned char wb_data_buffer[2];

	int client, server;
	int portNum = 1500;
	bool isExit = false;
	int bufsize = 1024;
	char buffer[bufsize];

	struct sockaddr_in server_addr;
	socklen_t size;


    client = socket(AF_INET, SOCK_STREAM, 0);

    if (client < 0) 
    {
        cout << "\nError establishing socket..." << endl;
        exit(1);
    }

    
    cout << "\n=> Socket server has been created..." << endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(portNum);


    if ((bind(client, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0) 
    {
        cout << "=> Error binding connection, the socket has already been established..." << endl;
        return -1;
    }


    size = sizeof(server_addr);
    cout << "=> Looking for clients..." << endl;

    listen(client, 1);

    bool manual=false;
    bool automatic=false;
    bool avail_auto=false;
    bool avail_manual=false;
    int  x;
    server = accept(client,(struct sockaddr *)&server_addr,&size);

    // first check if it is valid or not
    if (server < 0) 
        cout << "=> Error on accepting..." << endl;
        
    ///////////////COMUNICATION/////////////////////////////////
    cout << "Connection made" <<endl;
    while(!isExit && server>0)
    {		
    		for (hal::LedValue& led : image1d.leds) {
			led.red = 0;
			led.green = 0;
			led.blue = 0;
			led.white = 0;
		} 
		
		int i;
		for(i=0;i<bufsize;i++)
			buffer[i]=' ';
			
		recv(server, buffer, bufsize, 0);
		   
	   //MANUAL MODE---------------------------------------------------------------------------------------
	    if (buffer[0]=='m')  
		{ 	manual=true;
			automatic=false;
			avail_auto=false;
			if(manual==true&&buffer[0]=='m'&&buffer[1]=='a'){
			avail_manual=true;
			avail_auto=false;	
			turnAllTheLeds(&image1d,4,0);
			wb_data_buffer[0]=0x02;
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			}
			
		if(manual==true&&avail_manual==true){
			if(buffer[0]=='m'&&buffer[1]=='+'){
			turnAllTheLeds(&image1d,1,100);
			if(x>=20){
			x=20;
			turnAllTheLeds(&image1d,3,100);
			}else{
			x++ ;
			turnAllTheLeds(&image1d,4,x*5);
			wb_data_buffer[0]=0x02;
			wb_data_buffer[1]= x;
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			}
			}
			else if(buffer[0]=='m'&&buffer[1]=='-'){
			if(x==0){
			x=0;
			}else{
			x-- ;
			turnAllTheLeds(&image1d,4,x*5);
			wb_data_buffer[0]=0x02;
			wb_data_buffer[1]= x;
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			}
			}
			}
		}
			
	   //AUTOMATIC MODE--------------------------------------------------------------------------------
	   else if (buffer[0]=='a')
	   { 		automatic=true;
			manual=false;
			avail_manual=false;	
			if(automatic==true&&buffer[0]=='a'&&buffer[1]=='u'){
			avail_auto=true;
			avail_manual=false;	
			turnAllTheLeds(&image1d,3,0);
			wb_data_buffer[0]=0x01;
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			}
		if(automatic==true&&avail_auto==true){
			if(buffer[0]=='a'&&buffer[1]=='+'){
			turnAllTheLeds(&image1d,1,100);
			if(x>=20){
			x=20;
			turnAllTheLeds(&image1d,3,100);
			}else{
			x++ ;
			turnAllTheLeds(&image1d,2,x*5);
			wb_data_buffer[0]=0x01;
			wb_data_buffer[1]= x;
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			}
			}
			else if(buffer[0]=='a'&&buffer[1]=='-'){
			if(x==0){
			x=0;
			}else{
			x-- ;
			turnAllTheLeds(&image1d,2,x*5);
			wb_data_buffer[0]=0x01;
			wb_data_buffer[1]= x;
			bus.SpiWrite(kMCUBaseAddress+(kMemoryOffsetSamCommunication>>1), wb_data_buffer,0);
			}
			}
			}
		}	

		else{
			turnAllTheLeds(&image1d,5,0); 
		}
	
	        everloop.Write(&image1d);
	
	//PRINT DATA BUFFER
		for(i=0;i<5;i++)
			cout << buffer[i];
			cout <<endl;
        
        if(*buffer=='#'){
		isExit = true;
		}	
	if(*buffer==' '){
		isExit = true;
	    }
	
	}
		
	cout << "\n\n=> Connection terminated with IP " << inet_ntoa(server_addr.sin_addr);
	close(server);
	cout << "\nGoodbye..." << endl;
	isExit = false;
	exit(1);

	close(client);
	return 0;
}
