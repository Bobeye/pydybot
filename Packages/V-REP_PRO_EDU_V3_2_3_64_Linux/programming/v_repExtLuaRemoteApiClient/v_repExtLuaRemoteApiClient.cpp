// This file is part of the REMOTE API
// 
// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The REMOTE API is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The REMOTE API is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// THE REMOTE API IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.3 on November 24th 2015

#include "v_repExtLuaRemoteApiClient.h"
#include "luaFunctionData.h"
#include "remoteApiLink.h"
#include <boost/lexical_cast.hpp>
#include "v_repLib.h"
#include <iostream>
#include <sstream>

#ifdef _WIN32
	#ifdef QT_COMPIL
		#include <direct.h>
	#else
		#include <shlwapi.h> // required for PathRemoveFileSpec function
		#pragma comment(lib, "Shlwapi.lib")
	#endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
	#define WIN_AFX_MANAGE_STATE
#endif /* __linux || __APPLE__ */

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

LIBRARY vrepLib;
std::vector<CRemoteApiLink*> allRemoteApiClients;

int getIndexFromPortNb(int portNb)
{
	for (unsigned int i=0;i<allRemoteApiClients.size();i++)
	{
		if (allRemoteApiClients[i]->getPortNb()==portNb)
			return(i);
	}
	return(-1);
}

int getIndexFromClientId(int clientId)
{
	for (unsigned int i=0;i<allRemoteApiClients.size();i++)
	{
		if (allRemoteApiClients[i]->getClientId()==clientId)
			return(i);
	}
	return(-1);
}

bool isScriptThreaded(int scriptID)
{
	int prop,obj;
	if (-1!=simGetScriptProperty(scriptID,&prop,&obj))
		return((prop&sim_scripttype_threaded)!=0);
	return(false);
}

// --------------------------------------------------------------------------------------
// simxStart
// --------------------------------------------------------------------------------------
#define LUA_START_COMMAND "simxStart"

const int inArgs_START[]={
	6,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_bool,0,
    sim_lua_arg_bool,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0, // optional argument
};

void LUA_START_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int clientId=-1;
	if (D.readDataFromLua(p,inArgs_START,inArgs_START[0]-1,LUA_START_COMMAND)) // -1 because the last arg is optional
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		std::string ipAddress(inData->at(0).stringData[0]);
		if (ipAddress.length()==0)
			ipAddress="127.0.0.1";
		int port=inData->at(1).intData[0];
		bool waitUntilConnected=inData->at(2).boolData[0];
		bool doNotReconnect=inData->at(3).boolData[0];
		int timeOut=inData->at(4).intData[0];
		int cycle=5;
		if (inData->size()>=6)
			cycle=inData->at(5).intData[0];
		CRemoteApiLink* link=new CRemoteApiLink(ipAddress.c_str(),port,waitUntilConnected,doNotReconnect,timeOut,cycle);
		link->start();
		while (link->getClientId()==-2) // -2 means: we don't know yet
			CRemoteApiLink::waitXms(1);
		clientId=link->getClientId();
		if (clientId<0)
		{ // we failed at starting the client
			delete link;
			clientId=-1;
		}
		else
		{ // we succeeded at starting the client
			allRemoteApiClients.push_back(link);
			int prop,obj;
			if (-1!=simGetScriptProperty(p->scriptID,&prop,&obj))
			{
				int scriptType=((prop|sim_scripttype_threaded)-sim_scripttype_threaded);
				bool destroyAtSimulationEnd=( (scriptType==sim_scripttype_mainscript)||(scriptType==sim_scripttype_childscript)||(scriptType==sim_scripttype_jointctrlcallback)||(scriptType==sim_scripttype_contactcallback)||(scriptType==sim_scripttype_generalcallback) );
				link->setDestroyAtSimulationEnd(destroyAtSimulationEnd);
			}
		}
	}
	D.pushOutData(CLuaFunctionDataItem(clientId));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxFinish
// --------------------------------------------------------------------------------------
#define LUA_FINISH_COMMAND "simxFinish"

const int inArgs_FINISH[]={
	1,
    sim_lua_arg_int,0,
};

void LUA_FINISH_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_FINISH,inArgs_FINISH[0],LUA_FINISH_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		if (clientId==-1)
		{ // we want to end all clients!
			for (unsigned int i=0;i<allRemoteApiClients.size();i++)
				delete allRemoteApiClients[i]; // this will call simxFinish appropriately
			allRemoteApiClients.clear();
		}
		else
		{
			int index=getIndexFromClientId(clientId);
			if (index!=-1)
			{
				delete allRemoteApiClients[index]; // this will call simxFinish appropriately
				allRemoteApiClients.erase(allRemoteApiClients.begin()+index);
			}
			else
				simSetLastError(LUA_FINISH_COMMAND,"Invalid client ID."); // output an error
		}
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxAddStatusbarMessage
// --------------------------------------------------------------------------------------
#define LUA_ADDSTATUSBARMESSAGE_COMMAND "simxAddStatusbarMessage"

const int inArgs_ADDSTATUSBARMESSAGE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_ADDSTATUSBARMESSAGE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_ADDSTATUSBARMESSAGE,inArgs_ADDSTATUSBARMESSAGE[0],LUA_ADDSTATUSBARMESSAGE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(0,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_ADDSTATUSBARMESSAGE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxAuxiliaryConsoleClose
// --------------------------------------------------------------------------------------
#define LUA_AUXILIARYCONSOLECLOSE_COMMAND "simxAuxiliaryConsoleClose"

const int inArgs_AUXILIARYCONSOLECLOSE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_AUXILIARYCONSOLECLOSE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_AUXILIARYCONSOLECLOSE,inArgs_AUXILIARYCONSOLECLOSE[0],LUA_AUXILIARYCONSOLECLOSE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(1,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_AUXILIARYCONSOLECLOSE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxAuxiliaryConsoleOpen
// --------------------------------------------------------------------------------------
#define LUA_AUXILIARYCONSOLEOPEN_COMMAND "simxAuxiliaryConsoleOpen"

const int inArgs_AUXILIARYCONSOLEOPEN[]={
	9,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,2,
    sim_lua_arg_int|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,2,
    sim_lua_arg_float|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,3,
    sim_lua_arg_float|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,3,
    sim_lua_arg_int,0,
};

void LUA_AUXILIARYCONSOLEOPEN_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_AUXILIARYCONSOLEOPEN,inArgs_AUXILIARYCONSOLEOPEN[0],LUA_AUXILIARYCONSOLEOPEN_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(8).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(2,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_AUXILIARYCONSOLEOPEN_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxAuxiliaryConsolePrint
// --------------------------------------------------------------------------------------
#define LUA_AUXILIARYCONSOLEPRINT_COMMAND "simxAuxiliaryConsolePrint"

const int inArgs_AUXILIARYCONSOLEPRINT[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_string|SIM_LUA_ARG_NIL_ALLOWED,0,
    sim_lua_arg_int,0,
};

void LUA_AUXILIARYCONSOLEPRINT_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_AUXILIARYCONSOLEPRINT,inArgs_AUXILIARYCONSOLEPRINT[0],LUA_AUXILIARYCONSOLEPRINT_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(3,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_AUXILIARYCONSOLEPRINT_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxAuxiliaryConsoleShow
// --------------------------------------------------------------------------------------
#define LUA_AUXILIARYCONSOLESHOW_COMMAND "simxAuxiliaryConsoleShow"

const int inArgs_AUXILIARYCONSOLESHOW[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_bool,0,
    sim_lua_arg_int,0,
};

void LUA_AUXILIARYCONSOLESHOW_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_AUXILIARYCONSOLESHOW,inArgs_AUXILIARYCONSOLESHOW[0],LUA_AUXILIARYCONSOLESHOW_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(4,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_AUXILIARYCONSOLESHOW_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxBreakForceSensor
// --------------------------------------------------------------------------------------
#define LUA_BREAKFORCESENSOR_COMMAND "simxBreakForceSensor"

const int inArgs_BREAKFORCESENSOR[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_BREAKFORCESENSOR_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_BREAKFORCESENSOR,inArgs_BREAKFORCESENSOR[0],LUA_BREAKFORCESENSOR_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(5,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_BREAKFORCESENSOR_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxClearFloatSignal
// --------------------------------------------------------------------------------------
#define LUA_CLEARFLOATSIGNAL_COMMAND "simxClearFloatSignal"

const int inArgs_CLEARFLOATSIGNAL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_CLEARFLOATSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_CLEARFLOATSIGNAL,inArgs_CLEARFLOATSIGNAL[0],LUA_CLEARFLOATSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(6,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_CLEARFLOATSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxClearIntegerSignal
// --------------------------------------------------------------------------------------
#define LUA_CLEARINTEGERSIGNAL_COMMAND "simxClearIntegerSignal"

const int inArgs_CLEARINTEGERSIGNAL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_CLEARINTEGERSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_CLEARINTEGERSIGNAL,inArgs_CLEARINTEGERSIGNAL[0],LUA_CLEARINTEGERSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(7,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_CLEARINTEGERSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxClearStringSignal
// --------------------------------------------------------------------------------------
#define LUA_CLEARSTRINGSIGNAL_COMMAND "simxClearStringSignal"

const int inArgs_CLEARSTRINGSIGNAL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_CLEARSTRINGSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_CLEARSTRINGSIGNAL,inArgs_CLEARSTRINGSIGNAL[0],LUA_CLEARSTRINGSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(8,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_CLEARSTRINGSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectHandle
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTHANDLE_COMMAND "simxGetObjectHandle"

const int inArgs_GETOBJECTHANDLE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTHANDLE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTHANDLE,inArgs_GETOBJECTHANDLE[0],LUA_GETOBJECTHANDLE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(9,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTHANDLE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxCloseScene
// --------------------------------------------------------------------------------------
#define LUA_CLOSESCENE_COMMAND "simxCloseScene"

const int inArgs_CLOSESCENE[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_CLOSESCENE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_CLOSESCENE,inArgs_CLOSESCENE[0],LUA_CLOSESCENE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(1).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(10,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_CLOSESCENE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxCopyPasteObjects
// --------------------------------------------------------------------------------------
#define LUA_COPYPASTEOBJECTS_COMMAND "simxCopyPasteObjects"

const int inArgs_COPYPASTEOBJECTS[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int|sim_lua_arg_table,1, // at least one item in the table
    sim_lua_arg_int,0,
};

void LUA_COPYPASTEOBJECTS_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_COPYPASTEOBJECTS,inArgs_COPYPASTEOBJECTS[0],LUA_COPYPASTEOBJECTS_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(11,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_COPYPASTEOBJECTS_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxLoadScene
// --------------------------------------------------------------------------------------
#define LUA_LOADSCENE_COMMAND "simxLoadScene"

const int inArgs_LOADSCENE[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_LOADSCENE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_LOADSCENE,inArgs_LOADSCENE[0],LUA_LOADSCENE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(12,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_LOADSCENE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxLoadModel
// --------------------------------------------------------------------------------------
#define LUA_LOADMODEL_COMMAND "simxLoadModel"

const int inArgs_LOADMODEL[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_LOADMODEL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_LOADMODEL,inArgs_LOADMODEL[0],LUA_LOADMODEL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(13,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_LOADMODEL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxLoadUI
// --------------------------------------------------------------------------------------
#define LUA_LOADUI_COMMAND "simxLoadUI"

const int inArgs_LOADUI[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_LOADUI_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_LOADUI,inArgs_LOADUI[0],LUA_LOADUI_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(14,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_LOADUI_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetBooleanParameter
// --------------------------------------------------------------------------------------
#define LUA_SETBOOLEANPARAMETER_COMMAND "simxSetBooleanParameter"

const int inArgs_SETBOOLEANPARAMETER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_bool,0,
    sim_lua_arg_int,0,
};

void LUA_SETBOOLEANPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETBOOLEANPARAMETER,inArgs_SETBOOLEANPARAMETER[0],LUA_SETBOOLEANPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(15,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETBOOLEANPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxStartSimulation
// --------------------------------------------------------------------------------------
#define LUA_STARTSIMULATION_COMMAND "simxStartSimulation"

const int inArgs_STARTSIMULATION[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_STARTSIMULATION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_STARTSIMULATION,inArgs_STARTSIMULATION[0],LUA_STARTSIMULATION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(1).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(16,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_STARTSIMULATION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxStopSimulation
// --------------------------------------------------------------------------------------
#define LUA_STOPSIMULATION_COMMAND "simxStopSimulation"

const int inArgs_STOPSIMULATION[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_STOPSIMULATION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_STOPSIMULATION,inArgs_STOPSIMULATION[0],LUA_STOPSIMULATION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(1).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(17,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_STOPSIMULATION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetFloatingParameter
// --------------------------------------------------------------------------------------
#define LUA_SETFLOATINGPARAMETER_COMMAND "simxSetFloatingParameter"

const int inArgs_SETFLOATINGPARAMETER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
};

void LUA_SETFLOATINGPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETFLOATINGPARAMETER,inArgs_SETFLOATINGPARAMETER[0],LUA_SETFLOATINGPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(18,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETFLOATINGPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetIntegerParameter
// --------------------------------------------------------------------------------------
#define LUA_SETINTEGERPARAMETER_COMMAND "simxSetIntegerParameter"

const int inArgs_SETINTEGERPARAMETER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_SETINTEGERPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETINTEGERPARAMETER,inArgs_SETINTEGERPARAMETER[0],LUA_SETINTEGERPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(19,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETINTEGERPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetArrayParameter
// --------------------------------------------------------------------------------------
#define LUA_SETARRAYPARAMETER_COMMAND "simxSetArrayParameter"

const int inArgs_SETARRAYPARAMETER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
	sim_lua_arg_float|sim_lua_arg_table,3,
    sim_lua_arg_int,0,
};

void LUA_SETARRAYPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETARRAYPARAMETER,inArgs_SETARRAYPARAMETER[0],LUA_SETARRAYPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(20,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETARRAYPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetBooleanParameter
// --------------------------------------------------------------------------------------
#define LUA_GETBOOLEANPARAMETER_COMMAND "simxGetBooleanParameter"

const int inArgs_GETBOOLEANPARAMETER[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETBOOLEANPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETBOOLEANPARAMETER,inArgs_GETBOOLEANPARAMETER[0],LUA_GETBOOLEANPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(21,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETBOOLEANPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetFloatingParameter
// --------------------------------------------------------------------------------------
#define LUA_GETFLOATINGPARAMETER_COMMAND "simxGetFloatingParameter"

const int inArgs_GETFLOATINGPARAMETER[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETFLOATINGPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETFLOATINGPARAMETER,inArgs_GETFLOATINGPARAMETER[0],LUA_GETFLOATINGPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(22,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETFLOATINGPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetIntegerParameter
// --------------------------------------------------------------------------------------
#define LUA_GETINTEGERPARAMETER_COMMAND "simxGetIntegerParameter"

const int inArgs_GETINTEGERPARAMETER[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETINTEGERPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETINTEGERPARAMETER,inArgs_GETINTEGERPARAMETER[0],LUA_GETINTEGERPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(23,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETINTEGERPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetArrayParameter
// --------------------------------------------------------------------------------------
#define LUA_GETARRAYPARAMETER_COMMAND "simxGetArrayParameter"

const int inArgs_GETARRAYPARAMETER[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETARRAYPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETARRAYPARAMETER,inArgs_GETARRAYPARAMETER[0],LUA_GETARRAYPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(24,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETARRAYPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetStringParameter
// --------------------------------------------------------------------------------------
#define LUA_GETSTRINGPARAMETER_COMMAND "simxGetStringParameter"

const int inArgs_GETSTRINGPARAMETER[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETSTRINGPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETSTRINGPARAMETER,inArgs_GETSTRINGPARAMETER[0],LUA_GETSTRINGPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(25,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETSTRINGPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetFloatSignal
// --------------------------------------------------------------------------------------
#define LUA_SETFLOATSIGNAL_COMMAND "simxSetFloatSignal"

const int inArgs_SETFLOATSIGNAL[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
};

void LUA_SETFLOATSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETFLOATSIGNAL,inArgs_SETFLOATSIGNAL[0],LUA_SETFLOATSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(26,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETFLOATSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetIntegerSignal
// --------------------------------------------------------------------------------------
#define LUA_SETINTEGERSIGNAL_COMMAND "simxSetIntegerSignal"

const int inArgs_SETINTEGERSIGNAL[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_SETINTEGERSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETINTEGERSIGNAL,inArgs_SETINTEGERSIGNAL[0],LUA_SETINTEGERSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(27,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETINTEGERSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetStringSignal
// --------------------------------------------------------------------------------------
#define LUA_SETSTRINGSIGNAL_COMMAND "simxSetStringSignal"

const int inArgs_SETSTRINGSIGNAL[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_charbuff,0,
    sim_lua_arg_int,0,
};

void LUA_SETSTRINGSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETSTRINGSIGNAL,inArgs_SETSTRINGSIGNAL[0],LUA_SETSTRINGSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(28,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETSTRINGSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetFloatSignal
// --------------------------------------------------------------------------------------
#define LUA_GETFLOATSIGNAL_COMMAND "simxGetFloatSignal"

const int inArgs_GETFLOATSIGNAL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETFLOATSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETFLOATSIGNAL,inArgs_GETFLOATSIGNAL[0],LUA_GETFLOATSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(29,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETFLOATSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetIntegerSignal
// --------------------------------------------------------------------------------------
#define LUA_GETINTEGERSIGNAL_COMMAND "simxGetIntegerSignal"

const int inArgs_GETINTEGERSIGNAL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETINTEGERSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETINTEGERSIGNAL,inArgs_GETINTEGERSIGNAL[0],LUA_GETINTEGERSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(30,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETINTEGERSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetStringSignal
// --------------------------------------------------------------------------------------
#define LUA_GETSTRINGSIGNAL_COMMAND "simxGetStringSignal"

const int inArgs_GETSTRINGSIGNAL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETSTRINGSIGNAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETSTRINGSIGNAL,inArgs_GETSTRINGSIGNAL[0],LUA_GETSTRINGSIGNAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(31,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETSTRINGSIGNAL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxCreateDummy
// --------------------------------------------------------------------------------------
#define LUA_CREATEDUMMY_COMMAND "simxCreateDummy"

const int inArgs_CREATEDUMMY[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,12,
    sim_lua_arg_int,0,
};

void LUA_CREATEDUMMY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_CREATEDUMMY,inArgs_CREATEDUMMY[0],LUA_CREATEDUMMY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(32,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_CREATEDUMMY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxDisplayDialog
// --------------------------------------------------------------------------------------
#define LUA_DISPLAYDIALOG_COMMAND "simxDisplayDialog"

const int inArgs_DISPLAYDIALOG[]={
	8,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_float|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,6,
    sim_lua_arg_float|sim_lua_arg_table|SIM_LUA_ARG_NIL_ALLOWED,6,
    sim_lua_arg_int,0,
};

void LUA_DISPLAYDIALOG_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_DISPLAYDIALOG,inArgs_DISPLAYDIALOG[0],LUA_DISPLAYDIALOG_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(7).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(33,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_DISPLAYDIALOG_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxEndDialog
// --------------------------------------------------------------------------------------
#define LUA_ENDDIALOG_COMMAND "simxEndDialog"

const int inArgs_ENDDIALOG[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_ENDDIALOG_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_ENDDIALOG,inArgs_ENDDIALOG[0],LUA_ENDDIALOG_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(34,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_ENDDIALOG_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetDialogInput
// --------------------------------------------------------------------------------------
#define LUA_GETDIALOGINPUT_COMMAND "simxGetDialogInput"

const int inArgs_GETDIALOGINPUT[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETDIALOGINPUT_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETDIALOGINPUT,inArgs_GETDIALOGINPUT[0],LUA_GETDIALOGINPUT_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(35,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETDIALOGINPUT_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetDialogResult
// --------------------------------------------------------------------------------------
#define LUA_GETDIALOGRESULT_COMMAND "simxGetDialogResult"

const int inArgs_GETDIALOGRESULT[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETDIALOGRESULT_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETDIALOGRESULT,inArgs_GETDIALOGRESULT[0],LUA_GETDIALOGRESULT_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(36,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETDIALOGRESULT_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxEraseFile
// --------------------------------------------------------------------------------------
#define LUA_ERASEFILE_COMMAND "simxEraseFile"

const int inArgs_ERASEFILE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_ERASEFILE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_ERASEFILE,inArgs_ERASEFILE[0],LUA_ERASEFILE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(37,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_ERASEFILE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetCollisionHandle
// --------------------------------------------------------------------------------------
#define LUA_GETCOLLISIONHANDLE_COMMAND "simxGetCollisionHandle"

const int inArgs_GETCOLLISIONHANDLE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETCOLLISIONHANDLE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETCOLLISIONHANDLE,inArgs_GETCOLLISIONHANDLE[0],LUA_GETCOLLISIONHANDLE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(38,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETCOLLISIONHANDLE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetConnectionId
// --------------------------------------------------------------------------------------
#define LUA_GETCONNECTIONID_COMMAND "simxGetConnectionId"

const int inArgs_GETCONNECTIONID[]={
	1,
    sim_lua_arg_int,0,
};

void LUA_GETCONNECTIONID_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETCONNECTIONID,inArgs_GETCONNECTIONID[0],LUA_GETCONNECTIONID_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(39,&D,false,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETCONNECTIONID_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetDistanceHandle
// --------------------------------------------------------------------------------------
#define LUA_GETDISTANCEHANDLE_COMMAND "simxGetDistanceHandle"

const int inArgs_GETDISTANCEHANDLE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETDISTANCEHANDLE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETDISTANCEHANDLE,inArgs_GETDISTANCEHANDLE[0],LUA_GETDISTANCEHANDLE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(40,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETDISTANCEHANDLE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetInMessageInfo
// --------------------------------------------------------------------------------------
#define LUA_GETINMESSAGEINFO_COMMAND "simxGetInMessageInfo"

const int inArgs_GETINMESSAGEINFO[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETINMESSAGEINFO_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETINMESSAGEINFO,inArgs_GETINMESSAGEINFO[0],LUA_GETINMESSAGEINFO_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(41,&D,false,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETINMESSAGEINFO_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetOutMessageInfo
// --------------------------------------------------------------------------------------
#define LUA_GETOUTMESSAGEINFO_COMMAND "simxGetOutMessageInfo"

const int inArgs_GETOUTMESSAGEINFO[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOUTMESSAGEINFO_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOUTMESSAGEINFO,inArgs_GETOUTMESSAGEINFO[0],LUA_GETOUTMESSAGEINFO_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(42,&D,false,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOUTMESSAGEINFO_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetJointForce
// --------------------------------------------------------------------------------------
#define LUA_GETJOINTFORCE_COMMAND "simxGetJointForce"

const int inArgs_GETJOINTFORCE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETJOINTFORCE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETJOINTFORCE,inArgs_GETJOINTFORCE[0],LUA_GETJOINTFORCE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(43,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETJOINTFORCE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetJointMatrix
// --------------------------------------------------------------------------------------
#define LUA_GETJOINTMATRIX_COMMAND "simxGetJointMatrix"

const int inArgs_GETJOINTMATRIX[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETJOINTMATRIX_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETJOINTMATRIX,inArgs_GETJOINTMATRIX[0],LUA_GETJOINTMATRIX_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(44,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETJOINTMATRIX_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetJointPosition
// --------------------------------------------------------------------------------------
#define LUA_GETJOINTPOSITION_COMMAND "simxGetJointPosition"

const int inArgs_GETJOINTPOSITION[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETJOINTPOSITION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETJOINTPOSITION,inArgs_GETJOINTPOSITION[0],LUA_GETJOINTPOSITION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(45,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETJOINTPOSITION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetLastCmdTime
// --------------------------------------------------------------------------------------
#define LUA_GETLASTCMDTIME_COMMAND "simxGetLastCmdTime"

const int inArgs_GETLASTCMDTIME[]={
	1,
    sim_lua_arg_int,0,
};

void LUA_GETLASTCMDTIME_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETLASTCMDTIME,inArgs_GETLASTCMDTIME[0],LUA_GETLASTCMDTIME_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(46,&D,false,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETLASTCMDTIME_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetLastErrors
// --------------------------------------------------------------------------------------
#define LUA_GETLASTERRORS_COMMAND "simxGetLastErrors"

const int inArgs_GETLASTERRORS[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETLASTERRORS_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETLASTERRORS,inArgs_GETLASTERRORS[0],LUA_GETLASTERRORS_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(1).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(47,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETLASTERRORS_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetModelProperty
// --------------------------------------------------------------------------------------
#define LUA_GETMODELPROPERTY_COMMAND "simxGetModelProperty"

const int inArgs_GETMODELPROPERTY[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETMODELPROPERTY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETMODELPROPERTY,inArgs_GETMODELPROPERTY[0],LUA_GETMODELPROPERTY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(48,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETMODELPROPERTY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectChild
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTCHILD_COMMAND "simxGetObjectChild"

const int inArgs_GETOBJECTCHILD[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTCHILD_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTCHILD,inArgs_GETOBJECTCHILD[0],LUA_GETOBJECTCHILD_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(49,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTCHILD_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectFloatParameter
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTFLOATPARAMETER_COMMAND "simxGetObjectFloatParameter"

const int inArgs_GETOBJECTFLOATPARAMETER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTFLOATPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTFLOATPARAMETER,inArgs_GETOBJECTFLOATPARAMETER[0],LUA_GETOBJECTFLOATPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(50,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTFLOATPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectIntParameter
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTINTPARAMETER_COMMAND "simxGetObjectIntParameter"

const int inArgs_GETOBJECTINTPARAMETER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTINTPARAMETER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTINTPARAMETER,inArgs_GETOBJECTINTPARAMETER[0],LUA_GETOBJECTINTPARAMETER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(51,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTINTPARAMETER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectGroupData
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTGROUPDATA_COMMAND "simxGetObjectGroupData"

const int inArgs_GETOBJECTGROUPDATA[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTGROUPDATA_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTGROUPDATA,inArgs_GETOBJECTGROUPDATA[0],LUA_GETOBJECTGROUPDATA_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(52,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTGROUPDATA_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectOrientation
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTORIENTATION_COMMAND "simxGetObjectOrientation"

const int inArgs_GETOBJECTORIENTATION[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTORIENTATION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTORIENTATION,inArgs_GETOBJECTORIENTATION[0],LUA_GETOBJECTORIENTATION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(53,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTORIENTATION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectPosition
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTPOSITION_COMMAND "simxGetObjectPosition"

const int inArgs_GETOBJECTPOSITION[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTPOSITION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTPOSITION,inArgs_GETOBJECTPOSITION[0],LUA_GETOBJECTPOSITION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(54,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTPOSITION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectParent
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTPARENT_COMMAND "simxGetObjectParent"

const int inArgs_GETOBJECTPARENT[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTPARENT_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTPARENT,inArgs_GETOBJECTPARENT[0],LUA_GETOBJECTPARENT_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(55,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTPARENT_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjects
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTS_COMMAND "simxGetObjects"

const int inArgs_GETOBJECTS[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTS_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTS,inArgs_GETOBJECTS[0],LUA_GETOBJECTS_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(56,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTS_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectSelection
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTSELECTION_COMMAND "simxGetObjectSelection"

const int inArgs_GETOBJECTSELECTION[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTSELECTION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTSELECTION,inArgs_GETOBJECTSELECTION[0],LUA_GETOBJECTSELECTION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(1).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(57,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTSELECTION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetObjectVelocity
// --------------------------------------------------------------------------------------
#define LUA_GETOBJECTVELOCITY_COMMAND "simxGetObjectVelocity"

const int inArgs_GETOBJECTVELOCITY[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETOBJECTVELOCITY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETOBJECTVELOCITY,inArgs_GETOBJECTVELOCITY[0],LUA_GETOBJECTVELOCITY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(58,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETOBJECTVELOCITY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetPingTime
// --------------------------------------------------------------------------------------
#define LUA_GETPINGTIME_COMMAND "simxGetPingTime"

const int inArgs_GETPINGTIME[]={
	1,
    sim_lua_arg_int,0,
};

void LUA_GETPINGTIME_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETPINGTIME,inArgs_GETPINGTIME[0],LUA_GETPINGTIME_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(59,&D,true,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETPINGTIME_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetUIButtonProperty
// --------------------------------------------------------------------------------------
#define LUA_GETUIBUTTONPROPERTY_COMMAND "simxGetUIButtonProperty"

const int inArgs_GETUIBUTTONPROPERTY[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETUIBUTTONPROPERTY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETUIBUTTONPROPERTY,inArgs_GETUIBUTTONPROPERTY[0],LUA_GETUIBUTTONPROPERTY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(60,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETUIBUTTONPROPERTY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetUIEventButton
// --------------------------------------------------------------------------------------
#define LUA_GETUIEVENTBUTTON_COMMAND "simxGetUIEventButton"

const int inArgs_GETUIEVENTBUTTON[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETUIEVENTBUTTON_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETUIEVENTBUTTON,inArgs_GETUIEVENTBUTTON[0],LUA_GETUIEVENTBUTTON_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(61,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETUIEVENTBUTTON_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetUIHandle
// --------------------------------------------------------------------------------------
#define LUA_GETUIHANDLE_COMMAND "simxGetUIHandle"

const int inArgs_GETUIHANDLE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_GETUIHANDLE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETUIHANDLE,inArgs_GETUIHANDLE[0],LUA_GETUIHANDLE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(62,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETUIHANDLE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetUISlider
// --------------------------------------------------------------------------------------
#define LUA_GETUISLIDER_COMMAND "simxGetUISlider"

const int inArgs_GETUISLIDER[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETUISLIDER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETUISLIDER,inArgs_GETUISLIDER[0],LUA_GETUISLIDER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(63,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETUISLIDER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetVisionSensorDepthBuffer
// --------------------------------------------------------------------------------------
#define LUA_GETVISIONSENSORDEPTHBUFFER_COMMAND "simxGetVisionSensorDepthBuffer"

const int inArgs_GETVISIONSENSORDEPTHBUFFER[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETVISIONSENSORDEPTHBUFFER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETVISIONSENSORDEPTHBUFFER,inArgs_GETVISIONSENSORDEPTHBUFFER[0],LUA_GETVISIONSENSORDEPTHBUFFER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(64,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETVISIONSENSORDEPTHBUFFER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxGetVisionSensorImage
// --------------------------------------------------------------------------------------
#define LUA_GETVISIONSENSORIMAGE_COMMAND "simxGetVisionSensorImage"

const int inArgs_GETVISIONSENSORIMAGE[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_GETVISIONSENSORIMAGE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_GETVISIONSENSORIMAGE,inArgs_GETVISIONSENSORIMAGE[0],LUA_GETVISIONSENSORIMAGE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(65,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_GETVISIONSENSORIMAGE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetVisionSensorImage
// --------------------------------------------------------------------------------------
#define LUA_SETVISIONSENSORIMAGE_COMMAND "simxSetVisionSensorImage"

const int inArgs_SETVISIONSENSORIMAGE[]={
	5,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_charbuff,1,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_SETVISIONSENSORIMAGE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETVISIONSENSORIMAGE,inArgs_SETVISIONSENSORIMAGE[0],LUA_SETVISIONSENSORIMAGE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(66,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETVISIONSENSORIMAGE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxPauseCommunication
// --------------------------------------------------------------------------------------
#define LUA_PAUSECOMMUNICATION_COMMAND "simxPauseCommunication"

const int inArgs_PAUSECOMMUNICATION[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_bool,0,
};

void LUA_PAUSECOMMUNICATION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_PAUSECOMMUNICATION,inArgs_PAUSECOMMUNICATION[0],LUA_PAUSECOMMUNICATION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(67,&D,false,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_PAUSECOMMUNICATION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxPauseSimulation
// --------------------------------------------------------------------------------------
#define LUA_PAUSESIMULATION_COMMAND "simxPauseSimulation"

const int inArgs_PAUSESIMULATION[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_PAUSESIMULATION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_PAUSESIMULATION,inArgs_PAUSESIMULATION[0],LUA_PAUSESIMULATION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(1).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(68,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_PAUSESIMULATION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxQuery
// --------------------------------------------------------------------------------------
#define LUA_QUERY_COMMAND "simxQuery"

const int inArgs_QUERY[]={
	5,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_charbuff,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_QUERY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_QUERY,inArgs_QUERY[0],LUA_QUERY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(69,&D,true,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_QUERY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxReadCollision
// --------------------------------------------------------------------------------------
#define LUA_READCOLLISION_COMMAND "simxReadCollision"

const int inArgs_READCOLLISION[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_READCOLLISION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_READCOLLISION,inArgs_READCOLLISION[0],LUA_READCOLLISION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(70,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_READCOLLISION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxReadDistance
// --------------------------------------------------------------------------------------
#define LUA_READDISTANCE_COMMAND "simxReadDistance"

const int inArgs_READDISTANCE[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_READDISTANCE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_READDISTANCE,inArgs_READDISTANCE[0],LUA_READDISTANCE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(71,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_READDISTANCE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxReadForceSensor
// --------------------------------------------------------------------------------------
#define LUA_READFORCESENSOR_COMMAND "simxReadForceSensor"

const int inArgs_READFORCESENSOR[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_READFORCESENSOR_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_READFORCESENSOR,inArgs_READFORCESENSOR[0],LUA_READFORCESENSOR_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(72,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_READFORCESENSOR_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxReadProximitySensor
// --------------------------------------------------------------------------------------
#define LUA_READPROXIMITYSENSOR_COMMAND "simxReadProximitySensor"

const int inArgs_READPROXIMITYSENSOR[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_READPROXIMITYSENSOR_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_READPROXIMITYSENSOR,inArgs_READPROXIMITYSENSOR[0],LUA_READPROXIMITYSENSOR_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(73,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_READPROXIMITYSENSOR_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxReadStringStream
// --------------------------------------------------------------------------------------
#define LUA_READSTRINGSTREAM_COMMAND "simxReadStringStream"

const int inArgs_READSTRINGSTREAM[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_int,0,
};

void LUA_READSTRINGSTREAM_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_READSTRINGSTREAM,inArgs_READSTRINGSTREAM[0],LUA_READSTRINGSTREAM_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(74,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_READSTRINGSTREAM_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxWriteStringStream
// --------------------------------------------------------------------------------------
#define LUA_WRITESTRINGSTREAM_COMMAND "simxWriteStringStream"

const int inArgs_WRITESTRINGSTREAM[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_string,0,
    sim_lua_arg_charbuff,0,
    sim_lua_arg_int,0,
};

void LUA_WRITESTRINGSTREAM_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_WRITESTRINGSTREAM,inArgs_WRITESTRINGSTREAM[0],LUA_WRITESTRINGSTREAM_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(75,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_WRITESTRINGSTREAM_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxReadVisionSensor
// --------------------------------------------------------------------------------------
#define LUA_READVISIONSENSOR_COMMAND "simxReadVisionSensor"

const int inArgs_READVISIONSENSOR[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_READVISIONSENSOR_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_READVISIONSENSOR,inArgs_READVISIONSENSOR[0],LUA_READVISIONSENSOR_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(76,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_READVISIONSENSOR_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxRemoveModel
// --------------------------------------------------------------------------------------
#define LUA_REMOVEMODEL_COMMAND "simxRemoveModel"

const int inArgs_REMOVEMODEL[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_REMOVEMODEL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_REMOVEMODEL,inArgs_REMOVEMODEL[0],LUA_REMOVEMODEL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(77,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_REMOVEMODEL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxRemoveObject
// --------------------------------------------------------------------------------------
#define LUA_REMOVEOBJECT_COMMAND "simxRemoveObject"

const int inArgs_REMOVEOBJECT[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_REMOVEOBJECT_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_REMOVEOBJECT,inArgs_REMOVEOBJECT[0],LUA_REMOVEOBJECT_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(78,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_REMOVEOBJECT_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxRemoveUI
// --------------------------------------------------------------------------------------
#define LUA_REMOVEUI_COMMAND "simxRemoveUI"

const int inArgs_REMOVEUI[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
};

void LUA_REMOVEUI_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_REMOVEUI,inArgs_REMOVEUI[0],LUA_REMOVEUI_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(79,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_REMOVEUI_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetJointForce
// --------------------------------------------------------------------------------------
#define LUA_SETJOINTFORCE_COMMAND "simxSetJointForce"

const int inArgs_SETJOINTFORCE[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
};

void LUA_SETJOINTFORCE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETJOINTFORCE,inArgs_SETJOINTFORCE[0],LUA_SETJOINTFORCE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(80,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETJOINTFORCE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetJointPosition
// --------------------------------------------------------------------------------------
#define LUA_SETJOINTPOSITION_COMMAND "simxSetJointPosition"

const int inArgs_SETJOINTPOSITION[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
};

void LUA_SETJOINTPOSITION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETJOINTPOSITION,inArgs_SETJOINTPOSITION[0],LUA_SETJOINTPOSITION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(81,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETJOINTPOSITION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetJointTargetPosition
// --------------------------------------------------------------------------------------
#define LUA_SETJOINTTARGETPOSITION_COMMAND "simxSetJointTargetPosition"

const int inArgs_SETJOINTTARGETPOSITION[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
};

void LUA_SETJOINTTARGETPOSITION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETJOINTTARGETPOSITION,inArgs_SETJOINTTARGETPOSITION[0],LUA_SETJOINTTARGETPOSITION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(82,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETJOINTTARGETPOSITION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetJointTargetVelocity
// --------------------------------------------------------------------------------------
#define LUA_SETJOINTTARGETVELOCITY_COMMAND "simxSetJointTargetVelocity"

const int inArgs_SETJOINTTARGETVELOCITY[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
};

void LUA_SETJOINTTARGETVELOCITY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETJOINTTARGETVELOCITY,inArgs_SETJOINTTARGETVELOCITY[0],LUA_SETJOINTTARGETVELOCITY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(83,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETJOINTTARGETVELOCITY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetModelProperty
// --------------------------------------------------------------------------------------
#define LUA_SETMODELPROPERTY_COMMAND "simxSetModelProperty"

const int inArgs_SETMODELPROPERTY[]={
	4,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
};

void LUA_SETMODELPROPERTY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETMODELPROPERTY,inArgs_SETMODELPROPERTY[0],LUA_SETMODELPROPERTY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(84,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETMODELPROPERTY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetObjectOrientation
// --------------------------------------------------------------------------------------
#define LUA_SETOBJECTORIENTATION_COMMAND "simxSetObjectOrientation"

const int inArgs_SETOBJECTORIENTATION[]={
	5,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_float|sim_lua_arg_table,3,
	sim_lua_arg_int,0,
};

void LUA_SETOBJECTORIENTATION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETOBJECTORIENTATION,inArgs_SETOBJECTORIENTATION[0],LUA_SETOBJECTORIENTATION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(85,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETOBJECTORIENTATION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetObjectPosition
// --------------------------------------------------------------------------------------
#define LUA_SETOBJECTPOSITION_COMMAND "simxSetObjectPosition"

const int inArgs_SETOBJECTPOSITION[]={
	5,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_float|sim_lua_arg_table,3,
	sim_lua_arg_int,0,
};

void LUA_SETOBJECTPOSITION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETOBJECTPOSITION,inArgs_SETOBJECTPOSITION[0],LUA_SETOBJECTPOSITION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(86,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETOBJECTPOSITION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetObjectParent
// --------------------------------------------------------------------------------------
#define LUA_SETOBJECTPARENT_COMMAND "simxSetObjectParent"

const int inArgs_SETOBJECTPARENT[]={
	5,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_bool,0,
	sim_lua_arg_int,0,
};

void LUA_SETOBJECTPARENT_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETOBJECTPARENT,inArgs_SETOBJECTPARENT[0],LUA_SETOBJECTPARENT_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(87,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETOBJECTPARENT_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetObjectSelection
// --------------------------------------------------------------------------------------
#define LUA_SETOBJECTSELECTION_COMMAND "simxSetObjectSelection"

const int inArgs_SETOBJECTSELECTION[]={
	3,
	sim_lua_arg_int,0,
	sim_lua_arg_int|sim_lua_arg_table,0,
	sim_lua_arg_int,0,
};

void LUA_SETOBJECTSELECTION_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETOBJECTSELECTION,inArgs_SETOBJECTSELECTION[0],LUA_SETOBJECTSELECTION_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(2).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(88,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETOBJECTSELECTION_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetSphericalJointMatrix
// --------------------------------------------------------------------------------------
#define LUA_SETSPHERICALJOINTMATRIX_COMMAND "simxSetSphericalJointMatrix"

const int inArgs_SETSPHERICALJOINTMATRIX[]={
	4,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_float|sim_lua_arg_table,12,
	sim_lua_arg_int,0,
};

void LUA_SETSPHERICALJOINTMATRIX_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETSPHERICALJOINTMATRIX,inArgs_SETSPHERICALJOINTMATRIX[0],LUA_SETSPHERICALJOINTMATRIX_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(3).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(89,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETSPHERICALJOINTMATRIX_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetUIButtonLabel
// --------------------------------------------------------------------------------------
#define LUA_SETUIBUTTONLABEL_COMMAND "simxSetUIButtonLabel"

const int inArgs_SETUIBUTTONLABEL[]={
	6,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_string,0,
	sim_lua_arg_string,0,
	sim_lua_arg_int,0,
};

void LUA_SETUIBUTTONLABEL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETUIBUTTONLABEL,inArgs_SETUIBUTTONLABEL[0],LUA_SETUIBUTTONLABEL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(5).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(90,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETUIBUTTONLABEL_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetUIButtonProperty
// --------------------------------------------------------------------------------------
#define LUA_SETUIBUTTONPROPERTY_COMMAND "simxSetUIButtonProperty"

const int inArgs_SETUIBUTTONPROPERTY[]={
	5,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
};

void LUA_SETUIBUTTONPROPERTY_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETUIBUTTONPROPERTY,inArgs_SETUIBUTTONPROPERTY[0],LUA_SETUIBUTTONPROPERTY_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(91,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETUIBUTTONPROPERTY_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSetUISlider
// --------------------------------------------------------------------------------------
#define LUA_SETUISLIDER_COMMAND "simxSetUISlider"

const int inArgs_SETUISLIDER[]={
	5,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
};

void LUA_SETUISLIDER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SETUISLIDER,inArgs_SETUISLIDER[0],LUA_SETUISLIDER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(92,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SETUISLIDER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSynchronous
// --------------------------------------------------------------------------------------
#define LUA_SYNCHRONOUS_COMMAND "simxSynchronous"

const int inArgs_SYNCHRONOUS[]={
	2,
	sim_lua_arg_int,0,
	sim_lua_arg_bool,0,
};

void LUA_SYNCHRONOUS_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SYNCHRONOUS,inArgs_SYNCHRONOUS[0],LUA_SYNCHRONOUS_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(93,&D,true,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SYNCHRONOUS_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxSynchronousTrigger
// --------------------------------------------------------------------------------------
#define LUA_SYNCHRONOUSTRIGGER_COMMAND "simxSynchronousTrigger"

const int inArgs_SYNCHRONOUSTRIGGER[]={
	1,
	sim_lua_arg_int,0,
};

void LUA_SYNCHRONOUSTRIGGER_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_SYNCHRONOUSTRIGGER,inArgs_SYNCHRONOUSTRIGGER[0],LUA_SYNCHRONOUSTRIGGER_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(94,&D,true,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_SYNCHRONOUSTRIGGER_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simxTransferFile
// --------------------------------------------------------------------------------------
#define LUA_TRANSFERFILE_COMMAND "simxTransferFile"

const int inArgs_TRANSFERFILE[]={
	5,
	sim_lua_arg_int,0,
	sim_lua_arg_string,0,
	sim_lua_arg_string,0,
	sim_lua_arg_int,0,
	sim_lua_arg_int,0,
};

void LUA_TRANSFERFILE_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	if (D.readDataFromLua(p,inArgs_TRANSFERFILE,inArgs_TRANSFERFILE[0],LUA_TRANSFERFILE_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int clientId=inData->at(0).intData[0];
		int operationMode=inData->at(4).intData[0];
		int index=getIndexFromClientId(clientId);
		if (index!=-1)
			allRemoteApiClients[index]->handleCommand(95,&D,operationMode==simx_opmode_oneshot_wait,isScriptThreaded(p->scriptID)); // after calling this, index might not be valid anymore!!
		else
			simSetLastError(LUA_TRANSFERFILE_COMMAND,"Invalid client ID."); // output an error
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// This is the plugin start routine:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP
	// Dynamically load and bind V-REP functions:
	char curDirAndFile[1024];
#ifdef _WIN32
	#ifdef QT_COMPIL
		_getcwd(curDirAndFile, sizeof(curDirAndFile));
	#else
		GetModuleFileName(NULL,curDirAndFile,1023);
		PathRemoveFileSpec(curDirAndFile);
	#endif
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);

#ifdef _WIN32
	temp+="\\v_rep.dll";
#elif defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'LuaRemoteApiClient' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'LuaRemoteApiClient' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'LuaRemoteApiClient' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	std::vector<int> inArgs;

	// Register the new Lua commands:

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_START,inArgs);
	simRegisterCustomLuaFunction(LUA_START_COMMAND,strConCat("number clientId=",LUA_START_COMMAND,"(string connectionAddress,number connectionPort,boolean waitUntilConnected,boolean doNotReconnectOnceDisconnected,number timeOutInMs,number commThreadCycleInMs=5)"),&inArgs[0],LUA_START_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_FINISH,inArgs);
	simRegisterCustomLuaFunction(LUA_FINISH_COMMAND,strConCat("",LUA_FINISH_COMMAND,"(number clientId)"),&inArgs[0],LUA_FINISH_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ADDSTATUSBARMESSAGE,inArgs);
	simRegisterCustomLuaFunction(LUA_ADDSTATUSBARMESSAGE_COMMAND,strConCat("number returnCode=",LUA_ADDSTATUSBARMESSAGE_COMMAND,"(number clientId,string message,number operationMode)"),&inArgs[0],LUA_ADDSTATUSBARMESSAGE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_AUXILIARYCONSOLECLOSE,inArgs);
	simRegisterCustomLuaFunction(LUA_AUXILIARYCONSOLECLOSE_COMMAND,strConCat("number returnCode=",LUA_AUXILIARYCONSOLECLOSE_COMMAND,"(number clientId,number consoleHandle,number operationMode)"),&inArgs[0],LUA_AUXILIARYCONSOLECLOSE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_AUXILIARYCONSOLEOPEN,inArgs);
	simRegisterCustomLuaFunction(LUA_AUXILIARYCONSOLEOPEN_COMMAND,strConCat("number returnCode,number consoleHandle=",LUA_AUXILIARYCONSOLEOPEN_COMMAND,"(number clientId,string title,number maxLines,number mode,table_2 position,table_2 size,table_3 textColor,table_3 backgroundColor,number operationMode)"),&inArgs[0],LUA_AUXILIARYCONSOLEOPEN_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_AUXILIARYCONSOLEPRINT,inArgs);
	simRegisterCustomLuaFunction(LUA_AUXILIARYCONSOLEPRINT_COMMAND,strConCat("number returnCode=",LUA_AUXILIARYCONSOLEPRINT_COMMAND,"(number clientId,number consoleHandle,string txt,number operationMode)"),&inArgs[0],LUA_AUXILIARYCONSOLEPRINT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_AUXILIARYCONSOLESHOW,inArgs);
	simRegisterCustomLuaFunction(LUA_AUXILIARYCONSOLESHOW_COMMAND,strConCat("number returnCode=",LUA_AUXILIARYCONSOLESHOW_COMMAND,"(number clientId,number consoleHandle,boolean showState,number operationMode)"),&inArgs[0],LUA_AUXILIARYCONSOLESHOW_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_BREAKFORCESENSOR,inArgs);
	simRegisterCustomLuaFunction(LUA_BREAKFORCESENSOR_COMMAND,strConCat("number returnCode=",LUA_BREAKFORCESENSOR_COMMAND,"(number clientId,number forceSensorHandle,number operationMode)"),&inArgs[0],LUA_BREAKFORCESENSOR_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CLEARFLOATSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_CLEARFLOATSIGNAL_COMMAND,strConCat("number returnCode=",LUA_CLEARFLOATSIGNAL_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_CLEARFLOATSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CLEARINTEGERSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_CLEARINTEGERSIGNAL_COMMAND,strConCat("number returnCode=",LUA_CLEARINTEGERSIGNAL_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_CLEARINTEGERSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CLEARSTRINGSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_CLEARSTRINGSIGNAL_COMMAND,strConCat("number returnCode=",LUA_CLEARSTRINGSIGNAL_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_CLEARSTRINGSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTHANDLE,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTHANDLE_COMMAND,strConCat("number returnCode,number objectHandle=",LUA_GETOBJECTHANDLE_COMMAND,"(number clientId,string objectName,number operationMode)"),&inArgs[0],LUA_GETOBJECTHANDLE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CLOSESCENE,inArgs);
	simRegisterCustomLuaFunction(LUA_CLOSESCENE_COMMAND,strConCat("number returnCode=",LUA_CLOSESCENE_COMMAND,"(number clientId,number operationMode)"),&inArgs[0],LUA_CLOSESCENE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_COPYPASTEOBJECTS,inArgs);
	simRegisterCustomLuaFunction(LUA_COPYPASTEOBJECTS_COMMAND,strConCat("number returnCode,table newObjectHandles=",LUA_COPYPASTEOBJECTS_COMMAND,"(number clientId,table objectHandles,number operationMode)"),&inArgs[0],LUA_COPYPASTEOBJECTS_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LOADSCENE,inArgs);
	simRegisterCustomLuaFunction(LUA_LOADSCENE_COMMAND,strConCat("number returnCode=",LUA_LOADSCENE_COMMAND,"(number clientId,string scenePathAndName,number options,number operationMode)"),&inArgs[0],LUA_LOADSCENE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LOADMODEL,inArgs);
	simRegisterCustomLuaFunction(LUA_LOADMODEL_COMMAND,strConCat("number returnCode,number baseHandle=",LUA_LOADMODEL_COMMAND,"(number clientId,string modelPathAndName,number options,number operationMode)"),&inArgs[0],LUA_LOADMODEL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LOADUI,inArgs);
	simRegisterCustomLuaFunction(LUA_LOADUI_COMMAND,strConCat("number returnCode,table uiHandles=",LUA_LOADUI_COMMAND,"(number clientId,string uiPathAndName,number options,number operationMode)"),&inArgs[0],LUA_LOADUI_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETBOOLEANPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_SETBOOLEANPARAMETER_COMMAND,strConCat("number returnCode=",LUA_SETBOOLEANPARAMETER_COMMAND,"(number clientId,number paramIdentifier,boolean paramValue,number operationMode)"),&inArgs[0],LUA_SETBOOLEANPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_STARTSIMULATION,inArgs);
	simRegisterCustomLuaFunction(LUA_STARTSIMULATION_COMMAND,strConCat("number returnCode=",LUA_STARTSIMULATION_COMMAND,"(number clientId,number operationMode)"),&inArgs[0],LUA_STARTSIMULATION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_STOPSIMULATION,inArgs);
	simRegisterCustomLuaFunction(LUA_STOPSIMULATION_COMMAND,strConCat("number returnCode=",LUA_STOPSIMULATION_COMMAND,"(number clientId,number operationMode)"),&inArgs[0],LUA_STOPSIMULATION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETFLOATINGPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_SETFLOATINGPARAMETER_COMMAND,strConCat("number returnCode=",LUA_SETFLOATINGPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number paramValue,number operationMode)"),&inArgs[0],LUA_SETFLOATINGPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETINTEGERPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_SETINTEGERPARAMETER_COMMAND,strConCat("number returnCode=",LUA_SETINTEGERPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number paramValue,number operationMode)"),&inArgs[0],LUA_SETINTEGERPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETARRAYPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_SETARRAYPARAMETER_COMMAND,strConCat("number returnCode=",LUA_SETARRAYPARAMETER_COMMAND,"(number clientId,number paramIdentifier,table_3 paramValues,number operationMode)"),&inArgs[0],LUA_SETARRAYPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETBOOLEANPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETBOOLEANPARAMETER_COMMAND,strConCat("number returnCode,boolean paramValue=",LUA_GETBOOLEANPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number operationMode)"),&inArgs[0],LUA_GETBOOLEANPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETFLOATINGPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETFLOATINGPARAMETER_COMMAND,strConCat("number returnCode,number paramValue=",LUA_GETFLOATINGPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number operationMode)"),&inArgs[0],LUA_GETFLOATINGPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETINTEGERPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETINTEGERPARAMETER_COMMAND,strConCat("number returnCode,number paramValue=",LUA_GETINTEGERPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number operationMode)"),&inArgs[0],LUA_GETINTEGERPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETARRAYPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETARRAYPARAMETER_COMMAND,strConCat("number returnCode,table_3 paramValues=",LUA_GETARRAYPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number operationMode)"),&inArgs[0],LUA_GETARRAYPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETSTRINGPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETSTRINGPARAMETER_COMMAND,strConCat("number returnCode,string paramValue=",LUA_GETSTRINGPARAMETER_COMMAND,"(number clientId,number paramIdentifier,number operationMode)"),&inArgs[0],LUA_GETSTRINGPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETFLOATSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_SETFLOATSIGNAL_COMMAND,strConCat("number returnCode=",LUA_SETFLOATSIGNAL_COMMAND,"(number clientId,string signalName,number signalValue,number operationMode)"),&inArgs[0],LUA_SETFLOATSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETINTEGERSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_SETINTEGERSIGNAL_COMMAND,strConCat("number returnCode=",LUA_SETINTEGERSIGNAL_COMMAND,"(number clientId,string signalName,number signalValue,number operationMode)"),&inArgs[0],LUA_SETINTEGERSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETSTRINGSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_SETSTRINGSIGNAL_COMMAND,strConCat("number returnCode=",LUA_SETSTRINGSIGNAL_COMMAND,"(number clientId,string signalName,string signalValue,number operationMode)"),&inArgs[0],LUA_SETSTRINGSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETFLOATSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_GETFLOATSIGNAL_COMMAND,strConCat("number returnCode,number signalValue=",LUA_GETFLOATSIGNAL_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_GETFLOATSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETINTEGERSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_GETINTEGERSIGNAL_COMMAND,strConCat("number returnCode,number signalValue=",LUA_GETINTEGERSIGNAL_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_GETINTEGERSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETSTRINGSIGNAL,inArgs);
	simRegisterCustomLuaFunction(LUA_GETSTRINGSIGNAL_COMMAND,strConCat("number returnCode,string signalValue=",LUA_GETSTRINGSIGNAL_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_GETSTRINGSIGNAL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CREATEDUMMY,inArgs);
	simRegisterCustomLuaFunction(LUA_CREATEDUMMY_COMMAND,strConCat("number returnCode,number dummyHandle=",LUA_CREATEDUMMY_COMMAND,"(number clientId,number size,table_12 colors,number operationMode)"),&inArgs[0],LUA_CREATEDUMMY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_DISPLAYDIALOG,inArgs);
	simRegisterCustomLuaFunction(LUA_DISPLAYDIALOG_COMMAND,strConCat("number returnCode,number dialogHandle,number UiHandle=",LUA_DISPLAYDIALOG_COMMAND,"(number clientId,string titleText,string mainText,\nnumber dialogType,string initialText,table_6 titleColors,table_6 dialogColors,number operationMode)"),&inArgs[0],LUA_DISPLAYDIALOG_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ENDDIALOG,inArgs);
	simRegisterCustomLuaFunction(LUA_ENDDIALOG_COMMAND,strConCat("number returnCode=",LUA_ENDDIALOG_COMMAND,"(number clientId,number dialogHandle,number operationMode)"),&inArgs[0],LUA_ENDDIALOG_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETDIALOGINPUT,inArgs);
	simRegisterCustomLuaFunction(LUA_GETDIALOGINPUT_COMMAND,strConCat("number returnCode,string inputText=",LUA_GETDIALOGINPUT_COMMAND,"(number clientId,number dialogHandle,number operationMode)"),&inArgs[0],LUA_GETDIALOGINPUT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETDIALOGRESULT,inArgs);
	simRegisterCustomLuaFunction(LUA_GETDIALOGRESULT_COMMAND,strConCat("number returnCode,number result=",LUA_GETDIALOGRESULT_COMMAND,"(number clientId,number dialogHandle,number operationMode)"),&inArgs[0],LUA_GETDIALOGRESULT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_ERASEFILE,inArgs);
	simRegisterCustomLuaFunction(LUA_ERASEFILE_COMMAND,strConCat("number returnCode=",LUA_ERASEFILE_COMMAND,"(number clientId,string fileName_serverSide,number operationMode)"),&inArgs[0],LUA_ERASEFILE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETCOLLISIONHANDLE,inArgs);
	simRegisterCustomLuaFunction(LUA_GETCOLLISIONHANDLE_COMMAND,strConCat("number returnCode,number collisionHandle=",LUA_GETCOLLISIONHANDLE_COMMAND,"(number clientId,string collisionObjectName,number operationMode)"),&inArgs[0],LUA_GETCOLLISIONHANDLE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETCONNECTIONID,inArgs);
	simRegisterCustomLuaFunction(LUA_GETCONNECTIONID_COMMAND,strConCat("number connectionId=",LUA_GETCONNECTIONID_COMMAND,"(number clientId)"),&inArgs[0],LUA_GETCONNECTIONID_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETDISTANCEHANDLE,inArgs);
	simRegisterCustomLuaFunction(LUA_GETDISTANCEHANDLE_COMMAND,strConCat("number returnCode,number distanceHandle=",LUA_GETDISTANCEHANDLE_COMMAND,"(number clientId,string distanceObjectName,number operationMode)"),&inArgs[0],LUA_GETDISTANCEHANDLE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETINMESSAGEINFO,inArgs);
	simRegisterCustomLuaFunction(LUA_GETINMESSAGEINFO_COMMAND,strConCat("number result,number info=",LUA_GETINMESSAGEINFO_COMMAND,"(number clientId,number infoType)"),&inArgs[0],LUA_GETINMESSAGEINFO_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOUTMESSAGEINFO,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOUTMESSAGEINFO_COMMAND,strConCat("number result,number info=",LUA_GETOUTMESSAGEINFO_COMMAND,"(number clientId,number infoType)"),&inArgs[0],LUA_GETOUTMESSAGEINFO_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETJOINTFORCE,inArgs);
	simRegisterCustomLuaFunction(LUA_GETJOINTFORCE_COMMAND,strConCat("number returnCode,number force=",LUA_GETJOINTFORCE_COMMAND,"(number clientId,number jointHandle,number operationMode)"),&inArgs[0],LUA_GETJOINTFORCE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETJOINTMATRIX,inArgs);
	simRegisterCustomLuaFunction(LUA_GETJOINTMATRIX_COMMAND,strConCat("number returnCode,table_12 matrix=",LUA_GETJOINTMATRIX_COMMAND,"(number clientId,number jointHandle,number operationMode)"),&inArgs[0],LUA_GETJOINTMATRIX_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETJOINTPOSITION,inArgs);
	simRegisterCustomLuaFunction(LUA_GETJOINTPOSITION_COMMAND,strConCat("number returnCode,number position=",LUA_GETJOINTPOSITION_COMMAND,"(number clientId,number jointHandle,number operationMode)"),&inArgs[0],LUA_GETJOINTPOSITION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETLASTCMDTIME,inArgs);
	simRegisterCustomLuaFunction(LUA_GETLASTCMDTIME_COMMAND,strConCat("number simulationTimeInMs=",LUA_GETLASTCMDTIME_COMMAND,"(number clientId)"),&inArgs[0],LUA_GETLASTCMDTIME_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETLASTERRORS,inArgs);
	simRegisterCustomLuaFunction(LUA_GETLASTERRORS_COMMAND,strConCat("number returnCode,table errorStrings=",LUA_GETLASTERRORS_COMMAND,"(number clientId,number operationMode)"),&inArgs[0],LUA_GETLASTERRORS_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETMODELPROPERTY,inArgs);
	simRegisterCustomLuaFunction(LUA_GETMODELPROPERTY_COMMAND,strConCat("number returnCode,number property=",LUA_GETMODELPROPERTY_COMMAND,"(number clientId,number objectHandle,number operationMode)"),&inArgs[0],LUA_GETMODELPROPERTY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTCHILD,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTCHILD_COMMAND,strConCat("number returnCode,number childObjectHandle=",LUA_GETOBJECTCHILD_COMMAND,"(number clientId,number parentObjectHandle,number childIndex,number operationMode)"),&inArgs[0],LUA_GETOBJECTCHILD_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTFLOATPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTFLOATPARAMETER_COMMAND,strConCat("number returnCode,number parameterValue=",LUA_GETOBJECTFLOATPARAMETER_COMMAND,"(number clientId,number objectHandle,number parameterId,number operationMode)"),&inArgs[0],LUA_GETOBJECTFLOATPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTINTPARAMETER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTINTPARAMETER_COMMAND,strConCat("number returnCode,number parameterValue=",LUA_GETOBJECTINTPARAMETER_COMMAND,"(number clientId,number objectHandle,number parameterId,number operationMode)"),&inArgs[0],LUA_GETOBJECTINTPARAMETER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTGROUPDATA,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTGROUPDATA_COMMAND,strConCat("number returnCode,table handles,table intData,table floatData,table stringData=",LUA_GETOBJECTGROUPDATA_COMMAND,"(number clientId,number objectType,number dataType,number operationMode)"),&inArgs[0],LUA_GETOBJECTGROUPDATA_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTORIENTATION,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTORIENTATION_COMMAND,strConCat("number returnCode,table_3 eulerAngles=",LUA_GETOBJECTORIENTATION_COMMAND,"(number clientId,number objectHandle,number relativeToObjectHandle,number operationMode)"),&inArgs[0],LUA_GETOBJECTORIENTATION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTPOSITION,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTPOSITION_COMMAND,strConCat("number returnCode,table_3 position=",LUA_GETOBJECTPOSITION_COMMAND,"(number clientId,number objectHandle,number relativeToObjectHandle,number operationMode)"),&inArgs[0],LUA_GETOBJECTPOSITION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTPARENT,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTPARENT_COMMAND,strConCat("number returnCode,number parentObjectHandle=",LUA_GETOBJECTPARENT_COMMAND,"(number clientId,number objectHandle,number operationMode)"),&inArgs[0],LUA_GETOBJECTPARENT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTS,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTS_COMMAND,strConCat("number returnCode,table objectHandles=",LUA_GETOBJECTS_COMMAND,"(number clientId,number objectType,number operationMode)"),&inArgs[0],LUA_GETOBJECTS_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTSELECTION,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTSELECTION_COMMAND,strConCat("number returnCode,table objectHandles=",LUA_GETOBJECTSELECTION_COMMAND,"(number clientId,number operationMode)"),&inArgs[0],LUA_GETOBJECTSELECTION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETOBJECTVELOCITY,inArgs);
	simRegisterCustomLuaFunction(LUA_GETOBJECTVELOCITY_COMMAND,strConCat("number returnCode,table_3 linearVelocity,table_3 angularVelocity=",LUA_GETOBJECTVELOCITY_COMMAND,"(number clientId,number objectHandle,number operationMode)"),&inArgs[0],LUA_GETOBJECTVELOCITY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETPINGTIME,inArgs);
	simRegisterCustomLuaFunction(LUA_GETPINGTIME_COMMAND,strConCat("number returnCode,number pingTime=",LUA_GETPINGTIME_COMMAND,"(number clientId)"),&inArgs[0],LUA_GETPINGTIME_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETUIBUTTONPROPERTY,inArgs);
	simRegisterCustomLuaFunction(LUA_GETUIBUTTONPROPERTY_COMMAND,strConCat("number returnCode,number property=",LUA_GETUIBUTTONPROPERTY_COMMAND,"(number clientId,number uiHandle,number uiButtonId,number operationMode)"),&inArgs[0],LUA_GETUIBUTTONPROPERTY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETUIEVENTBUTTON,inArgs);
	simRegisterCustomLuaFunction(LUA_GETUIEVENTBUTTON_COMMAND,strConCat("number returnCode,number uiEventButtonId,table_2 auxValues=",LUA_GETUIEVENTBUTTON_COMMAND,"(number clientId,number uiHandle,number operationMode)"),&inArgs[0],LUA_GETUIEVENTBUTTON_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETUIHANDLE,inArgs);
	simRegisterCustomLuaFunction(LUA_GETUIHANDLE_COMMAND,strConCat("number returnCode,number uiHandle=",LUA_GETUIHANDLE_COMMAND,"(number clientId,string uiName,number operationMode)"),&inArgs[0],LUA_GETUIHANDLE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETUISLIDER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETUISLIDER_COMMAND,strConCat("number returnCode,number sliderPosition=",LUA_GETUISLIDER_COMMAND,"(number clientId,number uiHandle,number uiButtonId,number operationMode)"),&inArgs[0],LUA_GETUISLIDER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETVISIONSENSORDEPTHBUFFER,inArgs);
	simRegisterCustomLuaFunction(LUA_GETVISIONSENSORDEPTHBUFFER_COMMAND,strConCat("number returnCode,table buffer,table_2 resolution=",LUA_GETVISIONSENSORDEPTHBUFFER_COMMAND,"(number clientId,number sensorHandle,number operationMode)"),&inArgs[0],LUA_GETVISIONSENSORDEPTHBUFFER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETVISIONSENSORIMAGE,inArgs);
	simRegisterCustomLuaFunction(LUA_GETVISIONSENSORIMAGE_COMMAND,strConCat("number returnCode,string image,table_2 resolution=",LUA_GETVISIONSENSORIMAGE_COMMAND,"(number clientId,number sensorHandle,number options,number operationMode)"),&inArgs[0],LUA_GETVISIONSENSORIMAGE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETVISIONSENSORIMAGE,inArgs);
	simRegisterCustomLuaFunction(LUA_SETVISIONSENSORIMAGE_COMMAND,strConCat("number returnCode=",LUA_SETVISIONSENSORIMAGE_COMMAND,"(number clientId,number sensorHandle,string image,number options,number operationMode)"),&inArgs[0],LUA_SETVISIONSENSORIMAGE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_PAUSECOMMUNICATION,inArgs);
	simRegisterCustomLuaFunction(LUA_PAUSECOMMUNICATION_COMMAND,strConCat("number result=",LUA_PAUSECOMMUNICATION_COMMAND,"(number clientId,boolean pause)"),&inArgs[0],LUA_PAUSECOMMUNICATION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_PAUSESIMULATION,inArgs);
	simRegisterCustomLuaFunction(LUA_PAUSESIMULATION_COMMAND,strConCat("number returnCode=",LUA_PAUSESIMULATION_COMMAND,"(number clientId,number operationMode)"),&inArgs[0],LUA_PAUSESIMULATION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_QUERY,inArgs);
	simRegisterCustomLuaFunction(LUA_QUERY_COMMAND,strConCat("number returnCode,string retSignalValue=",LUA_QUERY_COMMAND,"(number clientId,string signalName,string signalValue,string retSignalName,number timeOutInMs)"),&inArgs[0],LUA_QUERY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READCOLLISION,inArgs);
	simRegisterCustomLuaFunction(LUA_READCOLLISION_COMMAND,strConCat("number returnCode,boolean collisionState=",LUA_READCOLLISION_COMMAND,"(number clientId,number collisionObjectHandle,number operationMode)"),&inArgs[0],LUA_READCOLLISION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READDISTANCE,inArgs);
	simRegisterCustomLuaFunction(LUA_READDISTANCE_COMMAND,strConCat("number returnCode,number minimumDistance=",LUA_READDISTANCE_COMMAND,"(number clientId,number distanceObjectHandle,number operationMode)"),&inArgs[0],LUA_READDISTANCE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READFORCESENSOR,inArgs);
	simRegisterCustomLuaFunction(LUA_READFORCESENSOR_COMMAND,strConCat("number returnCode,number state,table_3 force,table_3 torque=",LUA_READFORCESENSOR_COMMAND,"(number clientId,number forceSensorHandle,number operationMode)"),&inArgs[0],LUA_READFORCESENSOR_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READPROXIMITYSENSOR,inArgs);
	simRegisterCustomLuaFunction(LUA_READPROXIMITYSENSOR_COMMAND,strConCat("number returnCode,boolean detectionState,table_3 detectedPoint,number objectHandle,table_3 normalVector=",LUA_READPROXIMITYSENSOR_COMMAND,"(number clientId,number proximitySensorHandle,number operationMode)"),&inArgs[0],LUA_READPROXIMITYSENSOR_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READSTRINGSTREAM,inArgs);
	simRegisterCustomLuaFunction(LUA_READSTRINGSTREAM_COMMAND,strConCat("number returnCode,string signalValue=",LUA_READSTRINGSTREAM_COMMAND,"(number clientId,string signalName,number operationMode)"),&inArgs[0],LUA_READSTRINGSTREAM_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_WRITESTRINGSTREAM,inArgs);
	simRegisterCustomLuaFunction(LUA_WRITESTRINGSTREAM_COMMAND,strConCat("number returnCode=",LUA_WRITESTRINGSTREAM_COMMAND,"(number clientId,string signalName,string signalValueToAppend,number operationMode)"),&inArgs[0],LUA_WRITESTRINGSTREAM_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_READVISIONSENSOR,inArgs);
	simRegisterCustomLuaFunction(LUA_READVISIONSENSOR_COMMAND,strConCat("number returnCode,table packet1,table packet2,.. =",LUA_READVISIONSENSOR_COMMAND,"(number clientId,number visionSensorHandle,number operationMode)"),&inArgs[0],LUA_READVISIONSENSOR_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_REMOVEMODEL,inArgs);
	simRegisterCustomLuaFunction(LUA_REMOVEMODEL_COMMAND,strConCat("number returnCode=",LUA_REMOVEMODEL_COMMAND,"(number clientId,number objectHandle,number operationMode)"),&inArgs[0],LUA_REMOVEMODEL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_REMOVEOBJECT,inArgs);
	simRegisterCustomLuaFunction(LUA_REMOVEOBJECT_COMMAND,strConCat("number returnCode=",LUA_REMOVEOBJECT_COMMAND,"(number clientId,number objectHandle,number operationMode)"),&inArgs[0],LUA_REMOVEOBJECT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_REMOVEUI,inArgs);
	simRegisterCustomLuaFunction(LUA_REMOVEUI_COMMAND,strConCat("number returnCode=",LUA_REMOVEUI_COMMAND,"(number clientId,number uiHandle,number operationMode)"),&inArgs[0],LUA_REMOVEUI_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETJOINTFORCE,inArgs);
	simRegisterCustomLuaFunction(LUA_SETJOINTFORCE_COMMAND,strConCat("number returnCode=",LUA_SETJOINTFORCE_COMMAND,"(number clientId,number jointHandle,number force,number operationMode)"),&inArgs[0],LUA_SETJOINTFORCE_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETJOINTPOSITION,inArgs);
	simRegisterCustomLuaFunction(LUA_SETJOINTPOSITION_COMMAND,strConCat("number returnCode=",LUA_SETJOINTPOSITION_COMMAND,"(number clientId,number jointHandle,number position,number operationMode)"),&inArgs[0],LUA_SETJOINTPOSITION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETJOINTTARGETPOSITION,inArgs);
	simRegisterCustomLuaFunction(LUA_SETJOINTTARGETPOSITION_COMMAND,strConCat("number returnCode=",LUA_SETJOINTTARGETPOSITION_COMMAND,"(number clientId,number jointHandle,number targetPosition,number operationMode)"),&inArgs[0],LUA_SETJOINTTARGETPOSITION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETJOINTTARGETVELOCITY,inArgs);
	simRegisterCustomLuaFunction(LUA_SETJOINTTARGETVELOCITY_COMMAND,strConCat("number returnCode=",LUA_SETJOINTTARGETVELOCITY_COMMAND,"(number clientId,number jointHandle,number targetVelocity,number operationMode)"),&inArgs[0],LUA_SETJOINTTARGETVELOCITY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETMODELPROPERTY,inArgs);
	simRegisterCustomLuaFunction(LUA_SETMODELPROPERTY_COMMAND,strConCat("number returnCode=",LUA_SETMODELPROPERTY_COMMAND,"(number clientId,number objectHandle,number modelProperty,number operationMode)"),&inArgs[0],LUA_SETMODELPROPERTY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETOBJECTORIENTATION,inArgs);
	simRegisterCustomLuaFunction(LUA_SETOBJECTORIENTATION_COMMAND,strConCat("number returnCode=",LUA_SETOBJECTORIENTATION_COMMAND,"(number clientId,number objectHandle,number relativeToObjectHandle,table_3 eulerAngles,number operationMode)"),&inArgs[0],LUA_SETOBJECTORIENTATION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETOBJECTPOSITION,inArgs);
	simRegisterCustomLuaFunction(LUA_SETOBJECTPOSITION_COMMAND,strConCat("number returnCode=",LUA_SETOBJECTPOSITION_COMMAND,"(number clientId,number objectHandle,number relativeToObjectHandle,table_3 eulerAngles,number operationMode)"),&inArgs[0],LUA_SETOBJECTPOSITION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETOBJECTPARENT,inArgs);
	simRegisterCustomLuaFunction(LUA_SETOBJECTPARENT_COMMAND,strConCat("number returnCode=",LUA_SETOBJECTPARENT_COMMAND,"(number clientId,number objectHandle,number parentObject,boolean keepInPlace,number operationMode)"),&inArgs[0],LUA_SETOBJECTPARENT_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETOBJECTSELECTION,inArgs);
	simRegisterCustomLuaFunction(LUA_SETOBJECTSELECTION_COMMAND,strConCat("number returnCode=",LUA_SETOBJECTSELECTION_COMMAND,"(number clientId,table objectHandles,number operationMode)"),&inArgs[0],LUA_SETOBJECTSELECTION_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETSPHERICALJOINTMATRIX,inArgs);
	simRegisterCustomLuaFunction(LUA_SETSPHERICALJOINTMATRIX_COMMAND,strConCat("number returnCode=",LUA_SETSPHERICALJOINTMATRIX_COMMAND,"(number clientId,number jointHandle,table_12 matrix,number operationMode)"),&inArgs[0],LUA_SETSPHERICALJOINTMATRIX_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETUIBUTTONLABEL,inArgs);
	simRegisterCustomLuaFunction(LUA_SETUIBUTTONLABEL_COMMAND,strConCat("number returnCode=",LUA_SETUIBUTTONLABEL_COMMAND,"(number clientId,number uiHandle,number uiButtonId,string upStateLabel,string downStateLabel,number operationMode)"),&inArgs[0],LUA_SETUIBUTTONLABEL_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETUIBUTTONPROPERTY,inArgs);
	simRegisterCustomLuaFunction(LUA_SETUIBUTTONPROPERTY_COMMAND,strConCat("number returnCode=",LUA_SETUIBUTTONPROPERTY_COMMAND,"(number clientId,number uiHandle,number uiButtonId,number property,number operationMode)"),&inArgs[0],LUA_SETUIBUTTONPROPERTY_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SETUISLIDER,inArgs);
	simRegisterCustomLuaFunction(LUA_SETUISLIDER_COMMAND,strConCat("number returnCode=",LUA_SETUISLIDER_COMMAND,"(number clientId,number uiHandle,number uiButtonId,number position,number operationMode)"),&inArgs[0],LUA_SETUISLIDER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SYNCHRONOUS,inArgs);
	simRegisterCustomLuaFunction(LUA_SYNCHRONOUS_COMMAND,strConCat("number returnCode=",LUA_SYNCHRONOUS_COMMAND,"(number clientId,boolean enable)"),&inArgs[0],LUA_SYNCHRONOUS_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_SYNCHRONOUSTRIGGER,inArgs);
	simRegisterCustomLuaFunction(LUA_SYNCHRONOUSTRIGGER_COMMAND,strConCat("number returnCode=",LUA_SYNCHRONOUSTRIGGER_COMMAND,"(number clientId)"),&inArgs[0],LUA_SYNCHRONOUSTRIGGER_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_TRANSFERFILE,inArgs);
	simRegisterCustomLuaFunction(LUA_TRANSFERFILE_COMMAND,strConCat("number returnCode=",LUA_TRANSFERFILE_COMMAND,"(number clientId,string filePathAndName,string filename_serverSide,number timeOut,number operationMode)"),&inArgs[0],LUA_TRANSFERFILE_CALLBACK);

	simRegisterCustomLuaVariable("simx_return_ok",(boost::lexical_cast<std::string>(int(simx_return_ok))).c_str());
	simRegisterCustomLuaVariable("simx_return_novalue_flag",(boost::lexical_cast<std::string>(int(simx_return_novalue_flag))).c_str());	
	simRegisterCustomLuaVariable("simx_return_timeout_flag",(boost::lexical_cast<std::string>(int(simx_return_timeout_flag))).c_str());	
	simRegisterCustomLuaVariable("simx_return_illegal_opmode_flag",(boost::lexical_cast<std::string>(int(simx_return_illegal_opmode_flag))).c_str());	
	simRegisterCustomLuaVariable("simx_return_remote_error_flag",(boost::lexical_cast<std::string>(int(simx_return_remote_error_flag))).c_str());	
	simRegisterCustomLuaVariable("simx_return_split_progress_flag",(boost::lexical_cast<std::string>(int(simx_return_split_progress_flag))).c_str());	
	simRegisterCustomLuaVariable("simx_return_local_error_flag",(boost::lexical_cast<std::string>(int(simx_return_local_error_flag))).c_str());	
	simRegisterCustomLuaVariable("simx_return_initialize_error_flag",(boost::lexical_cast<std::string>(int(simx_return_initialize_error_flag))).c_str());	

	simRegisterCustomLuaVariable("simx_opmode_oneshot",(boost::lexical_cast<std::string>(int(simx_opmode_oneshot))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_oneshot_wait",(boost::lexical_cast<std::string>(int(simx_opmode_oneshot_wait))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_streaming",(boost::lexical_cast<std::string>(int(simx_opmode_streaming))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_oneshot_split",(boost::lexical_cast<std::string>(int(simx_opmode_oneshot_split))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_streaming_split",(boost::lexical_cast<std::string>(int(simx_opmode_streaming_split))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_discontinue",(boost::lexical_cast<std::string>(int(simx_opmode_discontinue))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_buffer",(boost::lexical_cast<std::string>(int(simx_opmode_buffer))).c_str());	
	simRegisterCustomLuaVariable("simx_opmode_remove",(boost::lexical_cast<std::string>(int(simx_opmode_remove))).c_str());	

	simRegisterCustomLuaVariable("simx_headeroffset_version",(boost::lexical_cast<std::string>(int(simx_headeroffset_version))).c_str());	
	simRegisterCustomLuaVariable("simx_headeroffset_message_id",(boost::lexical_cast<std::string>(int(simx_headeroffset_message_id))).c_str());	
	simRegisterCustomLuaVariable("simx_headeroffset_client_time",(boost::lexical_cast<std::string>(int(simx_headeroffset_client_time))).c_str());	
	simRegisterCustomLuaVariable("simx_headeroffset_server_time",(boost::lexical_cast<std::string>(int(simx_headeroffset_server_time))).c_str());	
	simRegisterCustomLuaVariable("simx_headeroffset_scene_id",(boost::lexical_cast<std::string>(int(simx_headeroffset_scene_id))).c_str());	
	simRegisterCustomLuaVariable("simx_headeroffset_server_state",(boost::lexical_cast<std::string>(int(simx_headeroffset_server_state))).c_str());	

	simRegisterCustomLuaVariable("simx_cmdheaderoffset_mem_size",(boost::lexical_cast<std::string>(int(simx_cmdheaderoffset_mem_size))).c_str());	
	simRegisterCustomLuaVariable("simx_cmdheaderoffset_full_mem_size",(boost::lexical_cast<std::string>(int(simx_cmdheaderoffset_full_mem_size))).c_str());	
	simRegisterCustomLuaVariable("simx_cmdheaderoffset_cmd",(boost::lexical_cast<std::string>(int(simx_cmdheaderoffset_cmd))).c_str());	
	simRegisterCustomLuaVariable("simx_cmdheaderoffset_sim_time",(boost::lexical_cast<std::string>(int(simx_cmdheaderoffset_sim_time))).c_str());	
	simRegisterCustomLuaVariable("simx_cmdheaderoffset_status",(boost::lexical_cast<std::string>(int(simx_cmdheaderoffset_status))).c_str());	

	return(SIMX_VERSION); // initialization went fine, we return the version number of this extension module (can be queried with simGetModuleName)
}

// This is the plugin end routine:
VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP

	for (unsigned int i=0;i<allRemoteApiClients.size();i++)
		delete allRemoteApiClients[i];
	
	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle

	// This function should not generate any error messages:
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

	void* retVal=NULL;

	if (message==sim_message_eventcallback_instancepass)
	{ // This message is sent each time the scene was rendered (well, shortly after) (very often)

	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ // Main script is about to be run

	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended
		for (unsigned int i=0;i<allRemoteApiClients.size();i++)
		{
			if (allRemoteApiClients[i]->getDestroyAtSimulationEnd())
			{
				delete allRemoteApiClients[i];
				allRemoteApiClients.erase(allRemoteApiClients.begin()+i);
				i--; // reprocess this position
			}
		}
	}

	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}

