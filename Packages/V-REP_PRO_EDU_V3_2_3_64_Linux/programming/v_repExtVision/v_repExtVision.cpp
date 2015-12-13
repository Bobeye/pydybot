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

#include "v_repExtVision.h"
#include "luaFunctionData.h"
#include "v_repLib.h"
#include <iostream>
#include "visionTransfCont.h"
#include "visionVelodyneHDL64ECont.h"
#include "visionVelodyneVPL16Cont.h"


#ifdef _WIN32
	#ifdef QT_COMPIL
		#include <direct.h>
	#else
		#include <shlwapi.h>
		#pragma comment(lib, "Shlwapi.lib")
	#endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
#endif /* __linux || __APPLE__ */

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

LIBRARY vrepLib;
CVisionTransfCont* visionTransfContainer;
CVisionVelodyneHDL64ECont* visionVelodyneHDL64EContainer;
CVisionVelodyneVPL16Cont* visionVelodyneVPL16Container;

// --------------------------------------------------------------------------------------
// simExtVision_handleSpherical
// --------------------------------------------------------------------------------------
#define LUA_HANDLESPHERICAL_COMMAND "simExtVision_handleSpherical"

const int inArgs_HANDLESPHERICAL[]={
	4,
    sim_lua_arg_int,0,
    sim_lua_arg_int|sim_lua_arg_table,6,
    sim_lua_arg_float,0,
    sim_lua_arg_float,0,
};

void LUA_HANDLESPHERICAL_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int result=-1;
	if (D.readDataFromLua(p,inArgs_HANDLESPHERICAL,inArgs_HANDLESPHERICAL[0],LUA_HANDLESPHERICAL_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int passiveVisionSensorHande=inData->at(0).intData[0];
		int activeVisionSensorHandes[6];
		for (int i=0;i<6;i++)
			activeVisionSensorHandes[i]=inData->at(1).intData[i];
		float horizontalAngle=inData->at(2).floatData[0];
		float verticalAngle=inData->at(3).floatData[0];
		CVisionTransf* obj=visionTransfContainer->getVisionTransfFromPassiveVisionSensor(passiveVisionSensorHande);
		if (obj!=NULL)
		{
			if (!obj->isSame(activeVisionSensorHandes,horizontalAngle,verticalAngle))
			{
				visionTransfContainer->removeObject(passiveVisionSensorHande);
				obj=NULL;
			}
		}
		if (obj==NULL)
		{
			obj=new CVisionTransf(passiveVisionSensorHande,activeVisionSensorHandes,horizontalAngle,verticalAngle);
			visionTransfContainer->addObject(obj);
		}

		if (obj->doAllObjectsExistAndAreVisionSensors())
		{
			if (obj->isActiveVisionSensorResolutionCorrect())
			{
				if (obj->areActiveVisionSensorsExplicitelyHandled())
				{
					obj->handleObject();
					result=1; // success
				}
				else
					simSetLastError(LUA_HANDLESPHERICAL_COMMAND,"Active vision sensors should be explicitely handled."); // output an error
			}
			else
				simSetLastError(LUA_HANDLESPHERICAL_COMMAND,"Invalid vision sensor resolutions."); // output an error
		}
		else
			simSetLastError(LUA_HANDLESPHERICAL_COMMAND,"Invalid handles, or handles are not vision sensor handles."); // output an error

		if (result==-1)
			visionTransfContainer->removeObject(passiveVisionSensorHande);

	}
	D.pushOutData(CLuaFunctionDataItem(result));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_handleAnaglyphStereo
// --------------------------------------------------------------------------------------
#define LUA_HANDLEANAGLYPHSTEREO_COMMAND "simExtVision_handleAnaglyphStereo"

const int inArgs_HANDLEANAGLYPHSTEREO[]={
	3,
    sim_lua_arg_int,0,
    sim_lua_arg_int|sim_lua_arg_table,2,
    sim_lua_arg_float|sim_lua_arg_table,6,
};

void LUA_HANDLEANAGLYPHSTEREO_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int result=-1;
	if (D.readDataFromLua(p,inArgs_HANDLEANAGLYPHSTEREO,inArgs_HANDLEANAGLYPHSTEREO[0]-1,LUA_HANDLEANAGLYPHSTEREO_COMMAND)) // -1 because last arg is optional
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int passiveVisionSensorHande=inData->at(0).intData[0];
		int leftSensorHandle=inData->at(1).intData[0];
		int rightSensorHandle=inData->at(1).intData[1];
		// Check the object types:
		bool existAndAreVisionSensors=true;
		if (simGetObjectType(passiveVisionSensorHande)!=sim_object_visionsensor_type)
			existAndAreVisionSensors=false;
		if (simGetObjectType(leftSensorHandle)!=sim_object_visionsensor_type)
			existAndAreVisionSensors=false;
		if (simGetObjectType(rightSensorHandle)!=sim_object_visionsensor_type)
			existAndAreVisionSensors=false;
		if (existAndAreVisionSensors)
		{ // check the sensor resolutions:
			int r[2];
			simGetVisionSensorResolution(passiveVisionSensorHande,r);
			int rl[2];
			simGetVisionSensorResolution(leftSensorHandle,rl);
			int rr[2];
			simGetVisionSensorResolution(rightSensorHandle,rr);
			if ((r[0]==rl[0])&&(r[0]==rr[0])&&(r[1]==rl[1])&&(r[1]==rr[1]))
			{ // check if the sensors are explicitely handled:
				int e=simGetExplicitHandling(passiveVisionSensorHande);
				int el=simGetExplicitHandling(leftSensorHandle);
				int er=simGetExplicitHandling(rightSensorHandle);
				if ((e&el&er&1)==1)
				{
					float leftAndRightColors[6]={1.0f,0.0f,0.0f,0.0f,1.0f,1.0f}; // default
					if (inData->size()>2)
					{ // we have the optional argument
						for (int i=0;i<6;i++)
							leftAndRightColors[i]=inData->at(2).floatData[i];
					}
					simHandleVisionSensor(leftSensorHandle,NULL,NULL);
					float* leftImage=simGetVisionSensorImage(leftSensorHandle);
					simHandleVisionSensor(rightSensorHandle,NULL,NULL);
					float* rightImage=simGetVisionSensorImage(rightSensorHandle);
					for (int i=0;i<r[0]*r[1];i++)
					{
						float il=(leftImage[3*i+0]+leftImage[3*i+1]+leftImage[3*i+2])/3.0f;
						float ir=(rightImage[3*i+0]+rightImage[3*i+1]+rightImage[3*i+2])/3.0f;
						leftImage[3*i+0]=il*leftAndRightColors[0]+ir*leftAndRightColors[3];
						leftImage[3*i+1]=il*leftAndRightColors[1]+ir*leftAndRightColors[4];
						leftImage[3*i+2]=il*leftAndRightColors[2]+ir*leftAndRightColors[5];
					}
					simSetVisionSensorImage(passiveVisionSensorHande,leftImage);
					simReleaseBuffer((char*)leftImage);
					simReleaseBuffer((char*)rightImage);
					result=1;
				}
				else
					simSetLastError(LUA_HANDLEANAGLYPHSTEREO_COMMAND,"Vision sensors should be explicitely handled."); // output an error
			}
			else
				simSetLastError(LUA_HANDLEANAGLYPHSTEREO_COMMAND,"Invalid vision sensor resolutions."); // output an error
		}
		else
			simSetLastError(LUA_HANDLEANAGLYPHSTEREO_COMMAND,"Invalid handles, or handles are not vision sensor handles."); // output an error
	}
	D.pushOutData(CLuaFunctionDataItem(result));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_createVelodyneHDL64E
// --------------------------------------------------------------------------------------
#define LUA_CREATEVELODYNEHDL64E_COMMAND "simExtVision_createVelodyneHDL64E"

const int inArgs_CREATEVELODYNEHDL64E[]={
	6,
    sim_lua_arg_int|sim_lua_arg_table,4,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_float|sim_lua_arg_table,2,
    sim_lua_arg_float,0,
};

void LUA_CREATEVELODYNEHDL64E_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int velodyneHandle=-1;
	if (D.readDataFromLua(p,inArgs_CREATEVELODYNEHDL64E,inArgs_CREATEVELODYNEHDL64E[0]-4,LUA_CREATEVELODYNEHDL64E_COMMAND)) // -4 because the last 4 arguments are optional
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int options=0;
		float pointSize=2.0f;
		float scalingFactor=1.0f;
		float coloringDistances[2]={1,5};
		int visionSensorHandes[4];
		for (int i=0;i<4;i++)
			visionSensorHandes[i]=inData->at(0).intData[i];
		float frequency=inData->at(1).floatData[0];
		if (inData->size()>2)
		{ // we have the optional 'options' argument:
			options=inData->at(2).intData[0];
		}
		if (inData->size()>3)
		{ // we have the optional 'pointSize' argument:
			pointSize=inData->at(3).floatData[0];	
		}
		if (inData->size()>4)
		{ // we have the optional 'coloringDistance' argument:
			coloringDistances[0]=inData->at(4).floatData[0];
			coloringDistances[1]=inData->at(4).floatData[1];
		}
		if (inData->size()>5)
		{ // we have the optional 'displayScalingFactor' argument:
			scalingFactor=inData->at(5).floatData[0];
		}
		CVisionVelodyneHDL64E* obj=new CVisionVelodyneHDL64E(visionSensorHandes,frequency,options,pointSize,coloringDistances,scalingFactor);
		visionVelodyneHDL64EContainer->addObject(obj);
		if (obj->doAllObjectsExistAndAreVisionSensors())
		{
			if (obj->areVisionSensorsExplicitelyHandled())
				velodyneHandle=obj->getVelodyneHandle(); // success
			else
				simSetLastError(LUA_CREATEVELODYNEHDL64E_COMMAND,"Vision sensors should be explicitely handled."); // output an error
		}
		else
			simSetLastError(LUA_CREATEVELODYNEHDL64E_COMMAND,"Invalid handles, or handles are not vision sensor handles."); // output an error

		if (velodyneHandle==-1)
			visionVelodyneHDL64EContainer->removeObject(obj->getVelodyneHandle());
	}
	D.pushOutData(CLuaFunctionDataItem(velodyneHandle));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_destroyVelodyneHDL64E
// --------------------------------------------------------------------------------------
#define LUA_DESTROYVELODYNEHDL64E_COMMAND "simExtVision_destroyVelodyneHDL64E"

const int inArgs_DESTROYVELODYNEHDL64E[]={
	1,
    sim_lua_arg_int,0,
};

void LUA_DESTROYVELODYNEHDL64E_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int result=-1;
	if (D.readDataFromLua(p,inArgs_DESTROYVELODYNEHDL64E,inArgs_DESTROYVELODYNEHDL64E[0],LUA_DESTROYVELODYNEHDL64E_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int handle=inData->at(0).intData[0];
		CVisionVelodyneHDL64E* obj=visionVelodyneHDL64EContainer->getObject(handle);
		if (obj!=NULL)
		{
			visionVelodyneHDL64EContainer->removeObject(obj->getVelodyneHandle());
			result=1;
		}
		else
			simSetLastError(LUA_DESTROYVELODYNEHDL64E_COMMAND,"Invalid handle."); // output an error
	}
	D.pushOutData(CLuaFunctionDataItem(result));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_handleVelodyneHDL64E
// --------------------------------------------------------------------------------------
#define LUA_HANDLEVELODYNEHDL64E_COMMAND "simExtVision_handleVelodyneHDL64E"

const int inArgs_HANDLEVELODYNEHDL64E[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
};

void LUA_HANDLEVELODYNEHDL64E_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	std::vector<float> pts;
	bool result=false;
	if (D.readDataFromLua(p,inArgs_HANDLEVELODYNEHDL64E,inArgs_HANDLEVELODYNEHDL64E[0],LUA_HANDLEVELODYNEHDL64E_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int handle=inData->at(0).intData[0];
		float dt=inData->at(1).floatData[0];
		CVisionVelodyneHDL64E* obj=visionVelodyneHDL64EContainer->getObject(handle);
		if (obj!=NULL)
			result=obj->handle(dt,pts);
		else
			simSetLastError(LUA_HANDLEVELODYNEHDL64E_COMMAND,"Invalid handle."); // output an error
	}
	if (result)
		D.pushOutData(CLuaFunctionDataItem(pts));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_createVelodyneVPL16
// --------------------------------------------------------------------------------------
#define LUA_CREATEVELODYNEVPL16_COMMAND "simExtVision_createVelodyneVPL16"

const int inArgs_CREATEVELODYNEVPL16[]={
	6,
    sim_lua_arg_int|sim_lua_arg_table,4,
    sim_lua_arg_float,0,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
    sim_lua_arg_float|sim_lua_arg_table,2,
    sim_lua_arg_float,0,
};

void LUA_CREATEVELODYNEVPL16_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int velodyneHandle=-1;
	if (D.readDataFromLua(p,inArgs_CREATEVELODYNEVPL16,inArgs_CREATEVELODYNEVPL16[0]-4,LUA_CREATEVELODYNEVPL16_COMMAND)) // -4 because the last 4 arguments are optional
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int options=0;
		float pointSize=2.0f;
		float scalingFactor=1.0f;
		float coloringDistances[2]={1,5};
		int visionSensorHandes[4];
		for (int i=0;i<4;i++)
			visionSensorHandes[i]=inData->at(0).intData[i];
		float frequency=inData->at(1).floatData[0];
		if (inData->size()>2)
		{ // we have the optional 'options' argument:
			options=inData->at(2).intData[0];
		}
		if (inData->size()>3)
		{ // we have the optional 'pointSize' argument:
			pointSize=inData->at(3).floatData[0];	
		}
		if (inData->size()>4)
		{ // we have the optional 'coloringDistance' argument:
			coloringDistances[0]=inData->at(4).floatData[0];
			coloringDistances[1]=inData->at(4).floatData[1];
		}
		if (inData->size()>5)
		{ // we have the optional 'displayScalingFactor' argument:
			scalingFactor=inData->at(5).floatData[0];
		}
		CVisionVelodyneVPL16* obj=new CVisionVelodyneVPL16(visionSensorHandes,frequency,options,pointSize,coloringDistances,scalingFactor);
		visionVelodyneVPL16Container->addObject(obj);
		if (obj->doAllObjectsExistAndAreVisionSensors())
		{
			if (obj->areVisionSensorsExplicitelyHandled())
				velodyneHandle=obj->getVelodyneHandle(); // success
			else
				simSetLastError(LUA_CREATEVELODYNEVPL16_COMMAND,"Vision sensors should be explicitely handled."); // output an error
		}
		else
			simSetLastError(LUA_CREATEVELODYNEVPL16_COMMAND,"Invalid handles, or handles are not vision sensor handles."); // output an error

		if (velodyneHandle==-1)
			visionVelodyneVPL16Container->removeObject(obj->getVelodyneHandle());
	}
	D.pushOutData(CLuaFunctionDataItem(velodyneHandle));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_destroyVelodyneVPL16
// --------------------------------------------------------------------------------------
#define LUA_DESTROYVELODYNEVPL16_COMMAND "simExtVision_destroyVelodyneVPL16"

const int inArgs_DESTROYVELODYNEVPL16[]={
	1,
    sim_lua_arg_int,0,
};

void LUA_DESTROYVELODYNEVPL16_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	int result=-1;
	if (D.readDataFromLua(p,inArgs_DESTROYVELODYNEVPL16,inArgs_DESTROYVELODYNEVPL16[0],LUA_DESTROYVELODYNEVPL16_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int handle=inData->at(0).intData[0];
		CVisionVelodyneVPL16* obj=visionVelodyneVPL16Container->getObject(handle);
		if (obj!=NULL)
		{
			visionVelodyneVPL16Container->removeObject(obj->getVelodyneHandle());
			result=1;
		}
		else
			simSetLastError(LUA_DESTROYVELODYNEVPL16_COMMAND,"Invalid handle."); // output an error
	}
	D.pushOutData(CLuaFunctionDataItem(result));
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------
// simExtVision_handleVelodyneVPL16
// --------------------------------------------------------------------------------------
#define LUA_HANDLEVELODYNEVPL16_COMMAND "simExtVision_handleVelodyneVPL16"

const int inArgs_HANDLEVELODYNEVPL16[]={
	2,
    sim_lua_arg_int,0,
    sim_lua_arg_float,0,
};

void LUA_HANDLEVELODYNEVPL16_CALLBACK(SLuaCallBack* p)
{
	p->outputArgCount=0;
	CLuaFunctionData D;
	std::vector<float> pts;
	bool result=false;
	if (D.readDataFromLua(p,inArgs_HANDLEVELODYNEVPL16,inArgs_HANDLEVELODYNEVPL16[0],LUA_HANDLEVELODYNEVPL16_COMMAND))
	{
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
		int handle=inData->at(0).intData[0];
		float dt=inData->at(1).floatData[0];
		CVisionVelodyneVPL16* obj=visionVelodyneVPL16Container->getObject(handle);
		if (obj!=NULL)
			result=obj->handle(dt,pts);
		else
			simSetLastError(LUA_HANDLEVELODYNEVPL16_COMMAND,"Invalid handle."); // output an error
	}
	if (result)
		D.pushOutData(CLuaFunctionDataItem(pts));
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
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'Vision' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'Vision' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'Vision' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	std::vector<int> inArgs;

	// Register the new Lua commands:

	// Spherical vision sensor:
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_HANDLESPHERICAL,inArgs);
	simRegisterCustomLuaFunction(LUA_HANDLESPHERICAL_COMMAND,strConCat("number result=",LUA_HANDLESPHERICAL_COMMAND,"(number passiveVisionSensorHandle,table_6 activeVisionSensorHandles,number horizontalAngle,number verticalAngle)"),&inArgs[0],LUA_HANDLESPHERICAL_CALLBACK);

	// Anaglyph sensor:
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_HANDLEANAGLYPHSTEREO,inArgs);
	simRegisterCustomLuaFunction(LUA_HANDLEANAGLYPHSTEREO_COMMAND,strConCat("number result=",LUA_HANDLEANAGLYPHSTEREO_COMMAND,"(number passiveVisionSensorHandle,table_2 activeVisionSensorHandles,table_6 leftAndRightColors=nil)"),&inArgs[0],LUA_HANDLEANAGLYPHSTEREO_CALLBACK);

	// Following 6 lines for backward compatibility:
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CREATEVELODYNEHDL64E,inArgs);
	simRegisterCustomLuaFunction("simExtVision_createVelodyne",strConCat("number velodyneHandle=","simExtVision_createVelodyne","(table_4 visionSensorHandles,number frequency,number options=0,number pointSize=2,table_2 coloring_closeFarDist={1,5},number displayScalingFactor=1)"),&inArgs[0],LUA_CREATEVELODYNEHDL64E_CALLBACK);
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_DESTROYVELODYNEHDL64E,inArgs);
	simRegisterCustomLuaFunction("simExtVision_destroyVelodyne",strConCat("number result=","simExtVision_destroyVelodyne","(number velodyneHandle)"),&inArgs[0],LUA_DESTROYVELODYNEHDL64E_CALLBACK);
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_HANDLEVELODYNEHDL64E,inArgs);
	simRegisterCustomLuaFunction("simExtVision_handleVelodyne",strConCat("table points=","simExtVision_handleVelodyne","(number velodyneHandle,number dt)"),&inArgs[0],LUA_HANDLEVELODYNEHDL64E_CALLBACK);

	// HDL-64E:
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CREATEVELODYNEHDL64E,inArgs);
	simRegisterCustomLuaFunction(LUA_CREATEVELODYNEHDL64E_COMMAND,strConCat("number velodyneHandle=",LUA_CREATEVELODYNEHDL64E_COMMAND,"(table_4 visionSensorHandles,number frequency,number options=0,number pointSize=2,table_2 coloring_closeFarDist={1,5},number displayScalingFactor=1)"),&inArgs[0],LUA_CREATEVELODYNEHDL64E_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_DESTROYVELODYNEHDL64E,inArgs);
	simRegisterCustomLuaFunction(LUA_DESTROYVELODYNEHDL64E_COMMAND,strConCat("number result=",LUA_DESTROYVELODYNEHDL64E_COMMAND,"(number velodyneHandle)"),&inArgs[0],LUA_DESTROYVELODYNEHDL64E_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_HANDLEVELODYNEHDL64E,inArgs);
	simRegisterCustomLuaFunction(LUA_HANDLEVELODYNEHDL64E_COMMAND,strConCat("table points=",LUA_HANDLEVELODYNEHDL64E_COMMAND,"(number velodyneHandle,number dt)"),&inArgs[0],LUA_HANDLEVELODYNEHDL64E_CALLBACK);

	// VPL-16:
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_CREATEVELODYNEVPL16,inArgs);
	simRegisterCustomLuaFunction(LUA_CREATEVELODYNEVPL16_COMMAND,strConCat("number velodyneHandle=",LUA_CREATEVELODYNEVPL16_COMMAND,"(table_4 visionSensorHandles,number frequency,number options=0,number pointSize=2,table_2 coloring_closeFarDist={1,5},number displayScalingFactor=1)"),&inArgs[0],LUA_CREATEVELODYNEVPL16_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_DESTROYVELODYNEVPL16,inArgs);
	simRegisterCustomLuaFunction(LUA_DESTROYVELODYNEVPL16_COMMAND,strConCat("number result=",LUA_DESTROYVELODYNEVPL16_COMMAND,"(number velodyneHandle)"),&inArgs[0],LUA_DESTROYVELODYNEVPL16_CALLBACK);

	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_HANDLEVELODYNEVPL16,inArgs);
	simRegisterCustomLuaFunction(LUA_HANDLEVELODYNEVPL16_COMMAND,strConCat("table points=",LUA_HANDLEVELODYNEVPL16_COMMAND,"(number velodyneHandle,number dt)"),&inArgs[0],LUA_HANDLEVELODYNEVPL16_CALLBACK);



	visionTransfContainer = new CVisionTransfCont();
	visionVelodyneHDL64EContainer = new CVisionVelodyneHDL64ECont();
	visionVelodyneVPL16Container = new CVisionVelodyneVPL16Cont();

	return(2);	// initialization went fine, we return the version number of this extension module (can be queried with simGetModuleName)
				// Version 2 since 3.2.1
}

// This is the plugin end routine:
VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP

	delete visionTransfContainer;
	delete visionVelodyneHDL64EContainer;
	delete visionVelodyneVPL16Container;

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
	{
		if (auxiliaryData[0]&1)
			visionTransfContainer->removeInvalidObjects();
	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended
		visionTransfContainer->removeAll();
		visionVelodyneHDL64EContainer->removeAll();
		visionVelodyneVPL16Container->removeAll();
	}

	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}

