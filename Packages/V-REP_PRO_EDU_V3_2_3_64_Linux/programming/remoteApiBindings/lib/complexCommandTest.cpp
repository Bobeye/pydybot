// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.3 on November 24th 2015

// This example illustrates how to execute complex commands from
// a remote API client. You can also use a similar construct for
// commands that are not directly supported by the remote API.
//
// Load the demo scene 'remoteApiCommandServerExample.ttt' in V-REP, then 
// start the simulation and run this program.
//
// IMPORTANT: for each successful call to simxStart, there
// should be a corresponding call to simxFinish at the end!

#include <stdio.h>
#include <stdlib.h>
#include <string>

extern "C" {
    #include "extApi.h"
}

std::string getCmdString(int id,int cnt,std::string& data)
{
	int l=12+int(data.size());
	int header[3]={id,cnt,l};
	std::string retData((char*)header,sizeof(int)*3);
	return(retData+data);
}
	
std::string waitForCmdReply(int clId,int cnt)
{
	while (true)
	{
		simxUChar* sigVal;
		int sigLen;
		int result=simxReadStringStream(clId,"repliesToRemoteApiClient",&sigVal,&sigLen,simx_opmode_streaming);
		if (result==simx_return_ok)
		{
			std::string str((char*)sigVal,sigLen);
			while (str.size()>0)
			{
				std::string headerPacked=str.substr(0,12);
				int* header=(int*)(&headerPacked[0]);
				if (cnt==header[1])
				{
					std::string replyData;
					if (header[2]>12)
						replyData=str.substr(12,header[2]-12);
					return replyData;
				}
				if (header[2]>=int(str.size()))
					str.clear();
				else
					str=str.substr(header[2]);
			}
		}
	}
}

int main(int argc,char* argv[])
{
	int clientID=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
	if (clientID!=-1)
	{
		printf("Connected to remote API server\n");

		// Commands are send via the string signal 'commandsFromRemoteApiClient'.
		// Commands are simply appended to that string signal
		// Each command is coded in following way:
		// 1. Command ID (integer, 4 chars)
		// 2. Command counter (integer, 4 chars). Simply start with 0 and increment for each command you send
		// 3. Command length (integer, includes the command ID, the command counter, the command length, and the additional data (i.e. command data))
		// 4. Command data (chars, can be of any length, depending on the command)
		//
		// Replies are coded in a same way. An executed command should reply with the same command counter.
		// 
		// Above simple protocol is just an example: you could use your own protocol! (but it has to be the same on the V-REP side)
		
		// 1. First send a command to display a specific message in a dialog box:
		int cmdID=1; // this command id means: 'display a text in a message box'
		int cmdCnt=0;
		std::string cmdData("Hello world!");
		std::string stringToSend(getCmdString(cmdID,cmdCnt,cmdData));
		simxWriteStringStream(clientID,"commandsFromRemoteApiClient",(simxUChar*)stringToSend.c_str(),(int)stringToSend.size(),simx_opmode_oneshot);
		printf("Returned message: %s\n",waitForCmdReply(clientID,cmdCnt).c_str()); // display the reply from V-REP (in this case, just a string)

		// 2. Now create a dummy object at coordinate 0.1,0.2,0.3:
		cmdID=2; // this command id means: 'create a dummy at a specific coordinate with a specific name'
		cmdCnt++;
		float coords[3]={0.1f,0.2f,0.3f};
		cmdData=std::string("MyDummyName")+std::string((char*)coords,sizeof(float)*3);
		stringToSend=getCmdString(cmdID,cmdCnt,cmdData);
		simxWriteStringStream(clientID,"commandsFromRemoteApiClient",(simxUChar*)stringToSend.c_str(),(int)stringToSend.size(),simx_opmode_oneshot);
		std::string replyData(waitForCmdReply(clientID,cmdCnt));
		printf("Dummy handle: %i\n",((int*)&replyData[0])[0]); // display the reply from V-REP (in this case, the handle of the created dummy)

		extApi_sleepMs(2000);

		// Now close the connection to V-REP:	
		simxFinish(clientID);
	}
	return(0);
}
