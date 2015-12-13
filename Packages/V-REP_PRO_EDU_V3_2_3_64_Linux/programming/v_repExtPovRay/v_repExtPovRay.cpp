// This file is part of the POV RAY PLUGIN for V-REP
// 
// The POV RAY PLUGIN was written by Andreu Vidal on behalf of Coppelia Robotics GmbH.
// 
// The POV RAY PLUGIN is licensed under the same terms as the POV Ray
// library: GNU Affero General Public License. You should have received
// a copy of the GNU Affero General Public License along with the
// POV RAY PLUGIN.  If not, see <http://www.gnu.org/licenses/agpl-3.0.en.html>.
// 

//
// This file was automatically created for V-REP release V3.2.3 on November 24th 2015

#include "v_repExtPovRay.h"
#include "v_repLib.h"
#include "4X4Matrix.h"
#include <iostream>
#include <QTextStream>
#include <QFile>
#include <QDir>
#include <QMap>
#include "povray/povray.h"

#ifdef _WIN32
	#include <direct.h>
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
	#include <string.h>
	#define _stricmp(x,y) strcasecmp(x,y)
#endif

LIBRARY vrepLib;

int resolutionX, resolutionY;
bool perspectiveOperation;
int light_count, mesh_count;
QString file_name (QDir::tempPath() + "/scene.pov");
QFile scene (file_name);
char paragraph[65535];

struct MeshObject
{
    char* data; int size; bool used;

    MeshObject ()                           { data = 0, size = 0, used = 0; }
    ~MeshObject ()                          { delete[] data; }
    char* alloc (int n)                     { data = new char[n]; return data; }
    void append (const void* src, int cnt)  { memcpy (data + size, src, cnt); size += cnt; }
};
QMap<int, MeshObject> objects;

static const char* make_patterned_texture (int material);


VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.

	// Dynamically load and bind V-REP functions:
	 // ******************************************
	 // 1. Figure out this plugin's directory:
	 char curDirAndFile[1024];
 #ifdef _WIN32
	 _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
	 getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
	 std::string currentDirAndPath(curDirAndFile);
	 // 2. Append the V-REP library's name:
	 std::string temp(currentDirAndPath);
 #ifdef _WIN32
	 temp+="/v_rep.dll";
 #elif defined (__linux)
	 temp+="/libv_rep.so";
 #elif defined (__APPLE__)
	 temp+="/libv_rep.dylib";
 #endif /* __linux || __APPLE__ */
	 // 3. Load the V-REP library:
	 vrepLib=loadVrepLibrary(temp.c_str());
	 if (vrepLib==NULL)
	 {
		 std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'RayTracer' plugin.\n";
		 return(0); // Means error, V-REP will unload this plugin
	 }
	 if (getVrepProcAddresses(vrepLib)==0)
	 {
		 std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'RayTracer' plugin.\n";
		 unloadVrepLibrary(vrepLib);
		 return(0); // Means error, V-REP will unload this plugin
	 }
	 // ******************************************

	 // Check the version of V-REP:
	 // ******************************************
	 int vrepVer;
	 simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	 if (vrepVer<30201) // if V-REP version is smaller than 3.02.01
	 {
		 std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'RayTracer' plugin.\n";
		 unloadVrepLibrary(vrepLib);
		 return(0); // Means error, V-REP will unload this plugin
	 }
	 // ******************************************

    return(1);	// initialization went fine, return the version number of this plugin!
}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
	unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
	return(NULL);
}

VREP_DLLEXPORT void v_repPovRay(int message,void* data)
{
	if (message==sim_message_eventcallback_extrenderer_start)
	{
		// Collect camera and environment data from V-REP:
		void** valPtr=(void**)data;
		resolutionX=((int*)valPtr[0])[0];
		resolutionY=((int*)valPtr[1])[0];
		float* backgroundColor=((float*)valPtr[2]);
		float viewAngle=((float*)valPtr[8])[0];
		perspectiveOperation=(((int*)valPtr[5])[0]==0);
		float nearClippingPlane=((float*)valPtr[9])[0];
		float farClippingPlane=((float*)valPtr[10])[0];
		float* amb=(float*)valPtr[11];
		C7Vector cameraTranformation(C4Vector((float*)valPtr[4]),C3Vector((float*)valPtr[3]));
		float* fogBackgroundColor=(float*)valPtr[12];
		bool fogEnabled=((bool*)valPtr[17])[0];
		float orthoViewSize=((float*)valPtr[18])[0];
		float fogDistance=((float*)valPtr[22])[0];
		float fogTransp=((float*)valPtr[23])[0];
		bool povFocalBlurEnabled=((bool*)valPtr[24])[0];
		float povFocalDistance=((float*)valPtr[25])[0];
		float povAperture=((float*)valPtr[26])[0];
		int povBlurSamples=((int*)valPtr[27])[0];

		// Open output file
		scene.open (QIODevice::WriteOnly);
		light_count = mesh_count = 0;

		// Camera transform
		C4X4Matrix m4(cameraTranformation.getMatrix());
		C3Vector& pos = m4.X;
		C3Vector& dir = m4.M.axis[2];
		C3Vector  foc = pos + dir;
		C3Vector& sky = m4.M.axis[1];
		float ratio = (float) resolutionX / (float) resolutionY;
		char* p = paragraph;

		p += sprintf (p, "global_settings {ambient_light rgb <%f,%f,%f> "
						 "max_trace_level 15}\nbackground {rgb <%f,%f,%f>}\ncamera {",
					  amb[0], amb[1], amb[2],
					  backgroundColor[0], backgroundColor[1], backgroundColor[2]);

		// Set camera options according to projection type
		if (perspectiveOperation)
		{
			float fld = 2 * pos.getLength () * tan (viewAngle / 2);
			float fx, fy;
			if (resolutionX > resolutionY)
				fx = fld, fy = fld / ratio;
			else
				fx = fld * ratio, fy = fld;

			p += sprintf (p, "perspective location <%f,%f,%f> direction <%f,%f,%f>"
							 " right <%f,0,0> up <0,%f,0> sky <%f,%f,%f> look_at <%f,%f,%f>",
						  pos(0), pos(1), pos(2), pos(0), pos(1), pos(2),
						  fx, fy, -sky(0), -sky(1), -sky(2), foc(0), foc(1), foc(2));
			if (povFocalBlurEnabled)
			{
				C3Vector  focD(pos + dir*povFocalDistance);
				p += sprintf (p, " focal_point <%f,%f,%f> aperture %f blur_samples %i}\n",
							  focD(0),focD(1),focD(2),povAperture,povBlurSamples);
			}
			else
				p += sprintf (p, "}\n");
		}
		else
		{
			float fx, fy;
			if (resolutionX > resolutionY)
				fx = orthoViewSize, fy = orthoViewSize / ratio;
			else
				fx = orthoViewSize * ratio, fy = orthoViewSize;

			p += sprintf (p, "orthographic location <%f,%f,%f>"
							 " right x * %f up y * %f sky <%f,%f,%f> look_at <%f,%f,%f>}\n",
						  pos(0), pos(1), pos(2),
						  fx, fy, -sky(0), -sky(1), -sky(2), foc(0), foc(1), foc(2));
		}

		// Set fog parameters
		if (fogEnabled)
		{
			p += sprintf (p, "fog {distance %f rgbt <%f,%f,%f,%f>}\n",
						  fogDistance, fogBackgroundColor[0],
						  fogBackgroundColor[1], fogBackgroundColor[2], fogTransp);
		}

		// Set scene boundaries in camera view direction
		C3Vector hyp (-pos(0), -pos(1), -pos(2));
		double f1 = hyp.getLength ();
		double f2 = f1 * dir.getLength ();
		double f3 = (f2 > 0.0 ? f1 * (hyp * dir) / f2 : f1);
		p += sprintf (p, "#declare NearPlane = plane {<%f,%f,%f>, %f};\n"
						 "#declare FarPlane = plane {<%f,%f,%f>, %f};\n",
					  -dir(0), -dir(1), -dir(2), f3 - nearClippingPlane,
					  dir(0), dir(1), dir(2), farClippingPlane - f3);

		scene.write (paragraph, p - paragraph);

		// Initialize object pool usage
		QMap<int, MeshObject>::iterator it;
		for (it = objects.begin(); it != objects.end(); it++)
			it->used = 0;
	}

	else if (message==sim_message_eventcallback_extrenderer_light)
	{
		// Collect light data from V-REP (one light at a time):
		void** valPtr=(void**)data;
		int lightType=((int*)valPtr[0])[0];
		float cutoffAngle=((float*)valPtr[1])[0];
		int spotExponent=((int*)valPtr[2])[0];
		float* colors=((float*)valPtr[3]);
		//float constAttenuation=((float*)valPtr[4])[0];
		//float linAttenuation=((float*)valPtr[5])[0];
		//float quadAttenuation=((float*)valPtr[6])[0];
		C7Vector lightTranformation(C4Vector((float*)valPtr[8]),C3Vector((float*)valPtr[7]));
		float lightSize=((float*)valPtr[9])[0];
		float FadeXDistance=((float*)valPtr[10])[0];
		bool lightIsVisible=((bool*)valPtr[11])[0];
		bool noShadow=((bool*)valPtr[12])[0];

		// Light transform
		C4X4Matrix m4(lightTranformation.getMatrix());
		C3Vector pos = m4.X;
		C3Vector dir = m4.M.axis[2];
		if (lightType == sim_light_directional_subtype)
			pos(0) = -dir(0) * 100, pos(1) = -dir(1) * 100, pos(2) = -dir(2) * 100;
		else
			dir += pos;
		char* p = paragraph;

		// Set light source
		p += sprintf (p, "light_source {<%f,%f,%f> rgb <%f,%f,%f>",
					  pos(0), pos(1), pos(2), colors[3], colors[4], colors[5]);

		// Assign surface and draw a white disc to signal it
		if (lightSize > 0 && lightType != sim_light_directional_subtype)
		{
			p += sprintf (p, " area_light <%f,0,0>, <0,%f,0>, 3, 3 adaptive 1 circular orient jitter",
						  lightSize, lightSize);

			if (lightIsVisible)
			{
				p += sprintf (p, " looks_like {sphere {<0,0,0>, %f double_illuminate texture"
								 " {pigment {color <1,1,1>} finish {ambient 1 diffuse 1 phong 1}}}}",
							  lightSize / 2);
			}
		}

		// Add parameters according to light type
		if (lightType == sim_light_omnidirectional_subtype)
		{
		}
		else if (lightType == sim_light_directional_subtype)
		{
			p += sprintf (p, " parallel point_at <%f,%f,%f>",
						  dir(0), dir(1), dir(2));
		}
		else if (lightType == sim_light_spot_subtype)
		{
			float ang = cutoffAngle * radToDeg;
			float exp = (float) spotExponent / 2.0f;
			p += sprintf (p, " spotlight radius %f falloff %f tightness %f point_at <%f,%f,%f>",
						  ang, ang, exp, dir(0), dir(1), dir(2));
		}

		// Set light attenuation
		if (FadeXDistance > 0)
		{
			p += sprintf (p, " fade_distance %f fade_power 1", FadeXDistance);
		}

		if (noShadow)
			p += sprintf (p, " shadowless");

		p += sprintf (p, "}\n");

		scene.write (paragraph, p - paragraph);

		light_count++;
	}

	else if (message==sim_message_eventcallback_extrenderer_mesh)
	{
		// Collect mesh data from V-REP:
		void** valPtr=(void**)data;
		float* vertices=((float*)valPtr[0]);
		//int verticesCnt=((int*)valPtr[1])[0];
		int* indices=((int*)valPtr[2]);
		int triangleCnt=((int*)valPtr[3])[0];
		float* normals=((float*)valPtr[4]);
		int normalsCnt=((int*)valPtr[5])[0];
		float* colors=((float*)valPtr[8]);
		C7Vector tr(C4Vector((float*)valPtr[7]),C3Vector((float*)valPtr[6]));
		bool textured=((bool*)valPtr[18])[0];
		//float shadingAngle=((float*)valPtr[19])[0];
		unsigned int meshId=((unsigned int*)valPtr[20])[0];
		bool translucid=((bool*)valPtr[21])[0];
		float opacityFactor=((float*)valPtr[22])[0];
		int povRayMaterial=((int*)valPtr[29])[0];
		int displayAttrib=((int*)valPtr[30])[0];
		const char* colorName=((char*)valPtr[31]);
		float povCol[3]={colors[0]+colors[9],colors[1]+colors[10],colors[2]+colors[11]};

		float* texCoords=0;
		int texCoordCnt=0;
		char* textureBuff=0;
		int textureSizeX=0;
		int textureSizeY=0;
		bool repeatU=0;
		bool repeatV=0;
		bool interpolateColors=0;
		int applyMode=0;

		if (textured)
		{
			// Read some additional data from V-REP (i.e. texture data):
			texCoords=((float*)valPtr[9]);
			texCoordCnt=((int*)valPtr[10])[0];
			textureBuff=((char*)valPtr[11]); // RGBA
			textureSizeX=((int*)valPtr[12])[0];
			textureSizeY=((int*)valPtr[13])[0];
			repeatU=((bool*)valPtr[14])[0];
			repeatV=((bool*)valPtr[15])[0];
			interpolateColors=((bool*)valPtr[16])[0];
			applyMode=((int*)valPtr[17])[0];
		}

		// Create or reuse mesh object (only vertices)
		MeshObject& obj = objects[meshId];
		obj.used = true;

		if (! obj.data && obj.alloc (triangleCnt * 200))
		{
			obj.append ("mesh {", 6);
			for (int i = 0, vrt = 0; i < triangleCnt; ++i)
			{
				if (vrt < normalsCnt)
				{
					obj.append ("smooth_triangle {", 17);
					for (int j = 3; j > 0; --j, ++vrt)
					{
						obj.append (vertices + 3 * indices[vrt], sizeof (float) * 3);
						obj.append (normals + 3 * vrt, sizeof (float) * 3);
					}
				}
				else
				{
					obj.append ("triangle {", 10);
					for (int j = 3; j > 0; --j, ++vrt)
						obj.append (vertices + 3 * indices[vrt], sizeof (float) * 3);
				}

				if (textured && i * 3 < texCoordCnt)
				{
					obj.append (" uv_vectors ", 12);
					obj.append (texCoords + 6 * i, sizeof (float) * 6);
				}

				obj.append ("}\n", 2);
			}
		}

		scene.write (obj.data, obj.size);

		// Object transform
		C4X4Matrix m4(tr.getMatrix());
		float tp = (translucid ? 1.0f - opacityFactor : 0.0f);
		const char* str;
		char* p = paragraph;

		// Build base texture
		if (povRayMaterial > 0 && (str = make_patterned_texture (povRayMaterial)))
		{
			p += sprintf (p, "#declare ObjectColor = color rgbft <%f,%f,%f,1,%f>;\n%s",
						  povCol[0], povCol[1], povCol[2], tp, str);
		}
		else
		{
			p += sprintf (p, "texture {pigment {rgbt <%f,%f,%f,%f>}"
							 " finish {ambient 1 diffuse 1 specular 0.5 roughness 0.01}}",
						  povCol[0], povCol[1], povCol[2], tp);
		}

		// Build bitmap texture
		if (textured)
		{
			p += sprintf (p, " texture {uv_mapping pigment {image_map {sys ");
			scene.write (paragraph, p - paragraph);
			p = paragraph;

			char b[4];
			b[0] = (textureSizeX >> 8) & 0xFF; b[1] = textureSizeX & 0xFF;
			b[2] = (textureSizeY >> 8) & 0xFF; b[3] = textureSizeY & 0xFF;
			scene.write (b, 4);
			scene.write (textureBuff, textureSizeX * textureSizeY * 4);

			p += sprintf (p, " %s}} finish {ambient rgb <%f,%f,%f>"
							 " diffuse 1 specular 0.5 roughness 0.01}}",
						  (repeatU || repeatV ? "" : "once"), povCol[0], povCol[1], povCol[2]);
		}

		// Add object modifiers
		p += sprintf (p, " matrix <%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f>",
					  m4.M.axis[0].data[0], m4.M.axis[0].data[1], m4.M.axis[0].data[2],
					  m4.M.axis[1].data[0], m4.M.axis[1].data[1], m4.M.axis[1].data[2],
					  m4.M.axis[2].data[0], m4.M.axis[2].data[1], m4.M.axis[2].data[2],
					  m4.X.data[0], m4.X.data[1], m4.X.data[2]);

		p += sprintf (p, " clipped_by {plane {NearPlane}} clipped_by {plane {FarPlane}}}\n");

		scene.write (paragraph, p - paragraph);

		mesh_count++;
	}

	else if (message==sim_message_eventcallback_extrenderer_triangles)
	{
		// Collect mesh data from V-REP:
		void** valPtr=(void**)data;
		float* vertices=((float*)valPtr[0]);
		int verticesCnt=((int*)valPtr[1])[0];
		float* normals=((float*)valPtr[2]);
		float* colors=((float*)valPtr[3]);
		bool translucid=((bool*)valPtr[4])[0];
		float opacityFactor=((float*)valPtr[5])[0];
		int povRayMaterial=((int*)valPtr[6])[0];
		bool isMirror=((bool*)valPtr[7])[0];
		if (isMirror)
			povRayMaterial=sim_pov_mirror;

		int triangleCnt = verticesCnt / 3;
		float tp = (translucid ? 1.0f - opacityFactor : 0.0f);
		const char* str;
		char* p = paragraph;

		// Write object vertices
		scene.write ("mesh {", 6);
		for (int i = 0, vrt = 0; i < triangleCnt; ++i)
		{
			scene.write ("smooth_triangle {", 17);
			for (int j = 3; j > 0; --j, ++vrt)
			{
				scene.write ((char*) (vertices + 3 * vrt), sizeof (float) * 3);
				scene.write ((char*) (normals + 3 * i), sizeof (float) * 3);
			}

			scene.write ("}\n", 2);
		}

		std::string w;
		// Build base texture
		if (povRayMaterial > 0 && (str = make_patterned_texture (povRayMaterial)))
		{
			p += sprintf (p, "#declare ObjectColor = color rgbft <%f,%f,%f,1,%f>;\n%s",
						  colors[0], colors[1], colors[2], tp, str);
		}
		else
		{
			p += sprintf (p, "texture {pigment {rgbt <%f,%f,%f,%f>}"
							 " finish {ambient 1 diffuse 1 specular 0.5 roughness 0.01}}",
						  colors[0], colors[1], colors[2], tp);
		}

		// Add object modifiers
		p += sprintf (p, " clipped_by {plane {NearPlane}} clipped_by {plane {FarPlane}}}\n");

		scene.write (paragraph, p - paragraph);
	}

	else if (message==sim_message_eventcallback_extrenderer_stop)
	{
		// Collect image buffer data from V-REP:
		void** valPtr=(void**)data;
		unsigned char* rgbBuffer=(unsigned char*)valPtr[0];
		//float* depthBuffer=(float*)valPtr[1];

		// Close output file
		scene.close();

		// Call POV-Ray to render scene
		povray_render_scene (file_name.toLatin1(), rgbBuffer, resolutionX, resolutionY, 0);

		// Check object usage
		QMap<int, MeshObject>::iterator it;
		it = objects.begin();
		while (it != objects.end())
			if (! it->used)
				it = objects.erase (it);
			else
				it++;
	}
}

const char* make_patterned_texture (int material)
{
    switch (material)
    {
    case sim_pov_white_marble:
        return "texture {pigment {marble turbulence 1 color_map {"
               "[0.0 rgbft <0.9, 0.9, 0.9, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.5, 0.5, 0.5, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.2, 0.2, 0.2, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_blood_marble:
        return "texture {pigment {marble turbulence 2.3 color_map {"
               "[0.0 rgbft <0, 0, 0, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.0, 0.6, 0.6, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.6, 0.0, 0.0, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0, 0, 0, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_blue_agate:
        return "texture {pigment {agate color_map {"
               "[0.5  rgbft <0.30, 0.30, 0.50, 0, 1> * ObjectColor]"
               "[0.55 rgbft <0.20, 0.20, 0.30, 0, 1> * ObjectColor]"
               "[0.6  rgbft <0.25, 0.25, 0.35, 0, 1> * ObjectColor]"
               "[0.7  rgbft <0.15, 0.15, 0.26, 0, 1> * ObjectColor]"
               "[0.8  rgbft <0.10, 0.10, 0.20, 0, 1> * ObjectColor]"
               "[0.9  rgbft <0.30, 0.30, 0.50, 0, 1> * ObjectColor]"
               "[1.0  rgbft <0.10, 0.10, 0.20, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_sapphire_agate:
        return "texture {pigment {agate color_map {"
               "[0.0  rgbft <0.0, 0.0, 0.9, 0, 1> * ObjectColor]"
               "[0.3  rgbft <0.0, 0.0, 0.8, 0, 1> * ObjectColor]"
               "[1.0  rgbft <0.0, 0.0, 0.4, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_brown_agate:
        return "texture {pigment {agate color_map {"
               "[0.0 rgbft <0, 0, 0, 0, 1> * ObjectColor]"
               "[0.5 rgbft <0.9, 0.7, 0.6, 0, 1> * ObjectColor]"
               "[0.6 rgbft <0.9, 0.7, 0.4, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.7, 0.4, 0.2, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_pink_granite:
        return "texture {pigment {granite color_map {"
               "[0.4  rgbft <0, 0, 0, 0, 1> * ObjectColor]"
               "[0.4  rgbft <0.85, 0.85, 0.95, 0, 1> * ObjectColor]"
               "[0.45 rgbft <0.85, 0.85, 0.95, 0, 1> * ObjectColor]"
               "[0.5  rgbft <0.75, 0.75, 0.75, 0, 1> * ObjectColor]"
               "[0.55 rgbft <0.82, 0.57, 0.46, 0, 1> * ObjectColor]"
               "[0.8  rgbft <0.82, 0.57, 0.46, 0, 1> * ObjectColor]"
               "[1.0  rgbft <1.00, 0.50, 0.00, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_pink_alabaster:
        return "texture {pigment {bozo turbulence 0.25 color_map {"
               "[0 rgbft <0.9, 0.75, 0.75, 0, 1> * ObjectColor]"
               "[1 rgbft <0.6, 0.6,  0.6 , 0, 1> * ObjectColor]"
               "} scale 0.1} finish {ambient 0.99 diffuse 0.99}}"
               "texture {pigment {granite color_map {"
               "[0.0 rgbft <0.52, 0.39, 0.39, 0, 1.0>]"
               "[0.9 rgbft <0.52, 0.39, 0.39, 0, 0.5>]"
               "[0.9 rgbft <0.42, 0.14, 0.55, 0, 0.0>]"
               "} scale 0.5} finish {ambient 0.99 diffuse 0.99 specular 1 roughness 0.0001 "
               "phong 0.25 phong_size 75 brilliance 4}}";

    case sim_pov_cherry_wood:
        return "texture {pigment {wood turbulence 0.3 scale <.025, .025, 1> color_map {"
               "[0.8 rgbft <0.96, 0.51, 0.30, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.70, 0.33, 0.16, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.40, 0.26, 0.13, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_pine_wood:
        return "texture {pigment {wood turbulence 0.2 scale <.025, .025, 1> color_map {"
               "[0.8 rgbft <1.0, 0.72, 0.25, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.7, 0.60, 0.16, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.6, 0.50, 0.23, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.65}}";

    case sim_pov_dark_wood:
        return "texture {pigment {wood turbulence 0.2 scale 0.025 color_map {"
               "[0.8 rgbft <0.63, 0.34, 0.15, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.60, 0.43, 0.16, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.30, 0.13, 0.13, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_tan_wood:
        return "texture {pigment {wood turbulence 0.1 scale 0.025 color_map {"
               "[0.8 rgbft <0.88, 0.60, 0.30, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.60, 0.40, 0.20, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.40, 0.30, 0.20, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_white_wood:
        return "texture {pigment {wood turbulence 0.6 scale 0.025 color_map {"
               "[0.0 rgbft <0.93, 0.71, 0.53, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.98, 0.81, 0.60, 0, 1> * ObjectColor]"
               "[0.8 rgbft <0.60, 0.33, 0.27, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.70, 0.60, 0.23, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_tom_wood:
        return "texture {pigment {wood turbulence 0.31 scale 0.025 color_map {"
               "[0.8 rgbft < 0.7, 0.3, 0.0, 0, 1> * ObjectColor]"
               "[0.8 rgbft < 0.5, 0.2, 0.0, 0, 1> * ObjectColor]"
               "[1.0 rgbft < 0.4, 0.1, 0.0, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_wood1:
        return "texture {pigment {wood turbulence 0.04 octaves 3 scale <.025, .025, 1> color_map {"
               "[0.1 rgbft <0.90, 0.40, 0.28, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.40, 0.25, 0.19, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_wood2:
        return "texture {pigment {wood turbulence 0.03 octaves 4 scale <.025, .025, 1> color_map {"
               "[0.1 rgbft <0.72, 0.47, 0.36, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.62, 0.16, 0.25, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_wood3:
        return "texture {pigment {wood turbulence 0.05 octaves 2 scale <.025, .025, 1> color_map {"
               "[0.1 rgbft <0.6, 0.233, 0.166, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.3, 0.165, 0.133, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_wood4:
        return "texture {pigment {wood turbulence 0.04 octaves 3 scale <.025, .025, 1> color_map {"
               "[0.1 rgbft <0.888, 0.600, 0.3, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.600, 0.400, 0.2, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_wood5:
        return "texture {pigment {wood turbulence 0.05 octaves 6 scale <.025, .025, 1> color_map {"
               "[0.1 rgbft <0.60, 0.20, 0.150, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.45, 0.17, 0.138, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_wood6:
        return "texture {pigment {wood turbulence 0.04 octaves 3 scale <.025, .025, 1> color_map {"
               "[0.1 rgbft <0.88, 0.60, 0.4, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.60, 0.40, 0.3, 0, 1> * ObjectColor]}}"
               "finish {ambient 0.99 diffuse 0.65 specular 0.25 roughness 0.05 reflection 0.15}}"
               "texture {pigment {wood  turbulence <0.1, 0.5, 1> octaves 5 lambda 3.25 "
               "scale <0.15, .5, 1> color_map {"
               "[0.0 rgbft <0.7, 0.6, 0.4, 0, 0.100>]"
               "[0.1 rgbft <0.8, 0.6, 0.3, 0, 0.500>]"
               "[0.1 rgbft <0.8, 0.6, 0.3, 0, 0.650>]"
               "[0.9 rgbft <0.6, 0.4, 0.2, 0, 0.975>]"
               "[1.0 rgbft <0.6, 0.4, 0.2, 0, 1.000>]"
               "} rotate <5, 10, 5> translate -x * 0.2}"
               "finish {specular 0.25 roughness 0.0005 ambient .99 diffuse 0.65}}"
               "texture {pigment {rgbft <0.75, 0.15, 0.0, 0, 0.95>}"
               "finish {specular 0.25 roughness 0.01 ambient 0.99 diffuse 0.65}}";

    case sim_pov_dmf_light_oak:
        return "texture {pigment {wood turbulence 0.05 scale <0.05, 0.05, 1> color_map {"
               "[0.1 rgbft <0.82, 0.66, 0.45, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.92, 0.77, 0.56, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_dmf_dark_oak:
        return "texture {pigment {wood turbulence 0.04 octaves 3 scale <0.05, 0.05, 1> color_map {"
               "[0.1 rgbft <0.90, 0.40, 0.28, 0, 1> * ObjectColor]"
               "[0.9 rgbft <0.40, 0.25, 0.19, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_emb_wood1:
        return "texture {pigment {wood turbulence 0.05 color_map {"
               "[0.00 rgbft <0.58, 0.45, 0.23, 0, 1> * ObjectColor]"
               "[0.34 rgbft <0.65, 0.45, 0.25, 0, 1> * ObjectColor]"
               "[0.40 rgbft <0.33, 0.23, 0.13, 0, 1> * ObjectColor]"
               "[0.47 rgbft <0.60, 0.40, 0.20, 0, 1> * ObjectColor]"
               "[1.00 rgbft <0.25, 0.15, 0.05, 0, 1> * ObjectColor]"
               "}} finish {crand 0.02 ambient 0.99 diffuse 0.63 phong 0.2 phong_size 10}"
               "normal {bumps 0.05} scale 0.01}"
               "texture {pigment {bozo color_map {"
               "[0.0 rgbft <1.00, 1.00, 1.00, 0, 1.00>]"
               "[0.8 rgbft <1.00, 0.90, 0.80, 0, 0.80>]"
               "[1.0 rgbft <0.30, 0.20, 0.10, 0, 0.40>]"
               "} scale 0.05} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_yellow_pine:
        return "texture {pigment {wood turbulence 0.02 color_map {"
               "[0.222 rgbft <0.808, 0.671, 0.251, 0, 1> * ObjectColor]"
               "[0.342 rgbft <0.600, 0.349, 0.043, 0, 1> * ObjectColor]"
               "[0.393 rgbft <0.808, 0.671, 0.251, 0, 1> * ObjectColor]"
               "[0.709 rgbft <0.808, 0.671, 0.251, 0, 1> * ObjectColor]"
               "[0.821 rgbft <0.533, 0.298, 0.027, 0, 1> * ObjectColor]"
               "[1.000 rgbft <0.808, 0.671, 0.251, 0, 1> * ObjectColor]"
               "} scale 0.01 translate <10, 0, 0>} finish {ambient 0.99 diffuse 0.99}}"
               "texture {pigment {wood turbulence 0.01 color_map {"
               "[0.000 rgbft <1.000, 1.000, 1.000, 0, 1.000>]"
               "[0.120 rgbft <0.702, 0.467, 0.118, 0, 0.608>]"
               "[0.496 rgbft <1.000, 1.000, 1.000, 0, 1.000>]"
               "[0.701 rgbft <1.000, 1.000, 1.000, 0, 1.000>]"
               "[0.829 rgbft <0.702, 0.467, 0.118, 0, 0.608>]"
               "[1.000 rgbft <1.000, 1.000, 1.000, 0, 1.000>]"
               "} scale 0.05 translate <10, 0, 0>} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_rose_wood:
        return "texture {pigment {bozo turbulence 0.04 color_map {"
               "[0.000 rgbft <0.204, 0.110, 0.078, 0, 1> * ObjectColor]"
               "[0.256 rgbft <0.231, 0.125, 0.090, 0, 1> * ObjectColor]"
               "[0.393 rgbft <0.247, 0.133, 0.090, 0, 1> * ObjectColor]"
               "[0.581 rgbft <0.204, 0.110, 0.075, 0, 1> * ObjectColor]"
               "[0.726 rgbft <0.259, 0.122, 0.102, 0, 1> * ObjectColor]"
               "[0.983 rgbft <0.231, 0.125, 0.086, 0, 1> * ObjectColor]"
               "[1.000 rgbft <0.204, 0.110, 0.078, 0, 1> * ObjectColor]"
               "} scale <0.01, 0.01, 1> translate <10, 0, 0>}"
               "finish {ambient 0.99 diffuse 0.8}}"
               "texture {pigment {wood turbulence 0.04 color_map {"
               "[0.000 rgbft <0.545, 0.349, 0.247, 0, 1.000>]"
               "[0.139 rgbft <0.000, 0.000, 0.000, 0, 0.004>]"
               "[0.148 rgbft <0.000, 0.000, 0.000, 0, 0.004>]"
               "[0.287 rgbft <0.545, 0.349, 0.247, 0, 1.000>]"
               "[0.443 rgbft <0.545, 0.349, 0.247, 0, 1.000>]"
               "[0.626 rgbft <0.000, 0.000, 0.000, 0, 0.004>]"
               "[0.635 rgbft <0.000, 0.000, 0.000, 0, 0.004>]"
               "[0.843 rgbft <0.545, 0.349, 0.247, 0, 1.000>]"
               "} scale <0.05, 0.05, 1> translate <10, 0, 0>}"
               "finish {ambient 0.99 diffuse 0.8}}";

    case sim_pov_sandal_wood:
        return "texture {pigment {bozo turbulence 0.2 color_map {"
               "[0.000 rgbft <0.725, 0.659, 0.455, 0, 1> * ObjectColor]"
               "[0.171 rgbft <0.682, 0.549, 0.420, 0, 1> * ObjectColor]"
               "[0.274 rgbft <0.557, 0.451, 0.322, 0, 1> * ObjectColor]"
               "[0.393 rgbft <0.725, 0.659, 0.455, 0, 1> * ObjectColor]"
               "[0.564 rgbft <0.682, 0.549, 0.420, 0, 1> * ObjectColor]"
               "[0.701 rgbft <0.482, 0.392, 0.278, 0, 1> * ObjectColor]"
               "[1.000 rgbft <0.725, 0.659, 0.455, 0, 1> * ObjectColor]"
               "} scale <0.02, 0.02, 1> scale 2}}"
               "texture {pigment {bozo turbulence 0.8 color_map {"
               "[0.000 rgbft <0.682, 0.604, 0.380, 0, 1.000>]"
               "[0.087 rgbft <0.761, 0.694, 0.600, 0, 0.020>]"
               "[0.226 rgbft <0.635, 0.553, 0.325, 0, 1.000>]"
               "[0.348 rgbft <0.761, 0.694, 0.600, 0, 0.020>]"
               "[0.496 rgbft <0.682, 0.604, 0.380, 0, 1.000>]"
               "[0.565 rgbft <0.761, 0.694, 0.600, 0, 0.020>]"
               "[0.661 rgbft <0.682, 0.604, 0.380, 0, 1.000>]"
               "[0.835 rgbft <0.761, 0.694, 0.600, 0, 0.020>]"
               "[1.000 rgbft <0.682, 0.604, 0.380, 0, 1.000>]"
               "} scale 0.05} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_glass:
        return "texture {pigment {rgbft <1.0, 1.0, 1.0, 0.7, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.1 specular 1 roughness 0.001}}";

    case sim_pov_glass2:
        return "texture {pigment {rgbft <1.0, 1.0, 1.0, 1.0, 1> * ObjectColor} finish {"
               "ambient 0 diffuse 0 reflection 0.5 phong 0.3 phong_size 60}}";

    case sim_pov_glass3:
        return "texture {pigment {rgbft <0.98, 0.98, 0.98, 0.9, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.1 "
               "specular 0.8 roughness 0.0003 phong 1 phong_size 400}}";

    case sim_pov_green_glass:
        return "texture {pigment {rgbft <0.98, 0.98, 0.98, 0.9, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.1 "
               "specular 0.8 roughness 0.0003 phong 1 phong_size 400}"
               "pigment {rgbft <0.8, 1, 0.95, 0.9, 0>}}";

    case sim_pov_light_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}}";

	case sim_pov_bold_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.8, 0.9, 0.85, 0.85, 0>}}";

    case sim_pov_wine_bottle:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.4, 0.72, 0.4, 0.6, 0>}}";

    case sim_pov_beer_bottle:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.7, 0.5, 0.1, 0.6, 0>}}";

    case sim_pov_ruby_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.9, 0.1, 0.2, 0.8, 0>}}";

    case sim_pov_blue_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.1, 0.7, 0.8, 0.8, 0>}}";

    case sim_pov_yellow_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.8, 0.8, 0.2, 0.8, 0>}}";

    case sim_pov_orange_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <1.0, 0.5, 0.0, 0.8, 0>}}";

	case sim_pov_vicks_bottle_glass:
        return "texture {pigment {rgbft <0.98, 1.0, 0.99, 0.75, 1> * ObjectColor} finish {"
               "ambient 0.5 diffuse 0.5 reflection 0.25 specular 1 roughness 0.001}"
               "pigment {rgbft <0.1, 0.15, 0.5, 0.9, 0>}}";

    case sim_pov_chrome_metal:
        return "texture {pigment {rgbft <0.658824, 0.658824, 0.658824, 0, 1> * ObjectColor} finish {"
               "ambient 0.65 diffuse 0.7 reflection 0.15 brilliance 8 "
               "specular 0.8 roughness 0.1}}";

    case sim_pov_brass_metal:
        return "texture {pigment {rgbft <0.71, 0.65, 0.26, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.7 brilliance 6 reflection 0.25 "
               "phong 0.75 phong_size 80}}";

    case sim_pov_copper_metal:
        return "texture {pigment {rgbft <0.72, 0.45, 0.20, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.7 brilliance 6 reflection 0.25 "
               "phong 0.75 phong_size 80}}";

    case sim_pov_bronze_metal:
        return "texture {pigment {rgbft <0.55, 0.47, 0.14, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.7 brilliance 6 reflection 0.25 "
               "phong 0.75 phong_size 80}}";

    case sim_pov_silver_metal:
        return "texture {pigment {rgbft <0.90, 0.91, 0.98, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.7 brilliance 6 reflection 0.25 "
               "phong 0.75 phong_size 80}}";

    case sim_pov_gold_metal:
        return "texture {pigment {rgbft <0.85, 0.85, 0.10, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.7 brilliance 6 reflection 0.25 "
               "phong 0.75 phong_size 80}}";

    case sim_pov_polished_chrome:
        return "texture {pigment {rgbft <0.4, 0.4, 0.4, 0, 1> * ObjectColor} finish {"
               "ambient 0.65 diffuse 0.99 brilliance 6 reflection 0.6 "
               "phong 0.8 phong_size 120}}";

    case sim_pov_polished_brass:
        return "texture {pigment {rgbft <0.578, 0.422, 0.195, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.99 brilliance 6 reflection 0.4 "
               "phong 0.8 phong_size 120}}";

    case sim_pov_new_brass:
        return "texture {pigment {rgbft <0.70, 0.56, 0.37, 0, 1> * ObjectColor} finish {"
               "ambient 0.65 diffuse 1.0 brilliance 15 "
               "phong 0.41 phong_size 5}}";

    case sim_pov_spun_brass:
        return "texture {pigment {rgbft <0.70, 0.56, 0.37, 0, 1> * ObjectColor} finish {"
               "ambient 0.65 diffuse 1.0 brilliance 15 "
               "phong 0.41 phong_size 5} normal {waves 0.35 frequency 2 scale 0.01}}";

    case sim_pov_brushed_aluminum:
        return "texture {pigment {rgbft <0.658, 0.658, 0.658, 0, 1> * ObjectColor} finish {"
               "ambient 0.65 diffuse 0.7 reflection 0.15 brilliance 8 "
               "specular 0.8 roughness 0.1} normal {bumps -0.25 scale <0.1, 0.001, 0.001>}}";

    case sim_pov_silver1:
        return "texture {pigment {rgbft <0.94, 0.93, 0.83, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.65 brilliance 6 reflection 0.45 "
               "phong 1 phong_size 100}}";

    case sim_pov_silver2:
        return "texture {pigment {rgbft <0.94, 0.93, 0.86, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.65 brilliance 6 reflection 0.45 "
               "phong 1 phong_size 100}}";

    case sim_pov_silver3:
        return "texture {pigment {rgbft <0.94, 0.93, 0.90, 0, 1> * ObjectColor} finish {"
               "metallic ambient 0.65 diffuse 0.65 brilliance 6 reflection 0.45 "
               "phong 1 phong_size 100}}";

    case sim_pov_brass_valley:
        return "texture {pigment {granite color_map {"
               "[0.3 rgbft <0.82, 0.57, 0.46, 0, 1> * ObjectColor]"
               "[0.3 rgbft <0.00, 0.00, 0.00, 0, 1> * ObjectColor]"
               "[0.6 rgbft <0.85, 0.85, 0.95, 0, 1> * ObjectColor]"
               "[0.6 rgbft <0.82, 0.57, 0.46, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.85, 0.85, 0.95, 0, 1> * ObjectColor]"
               "}} finish {metallic ambient 0.65 brilliance 6.0 reflection 0.75 phong 0.75}}";

    case sim_pov_rust:
        return "texture {pigment {granite color_map {"
               "[0.0 rgbft <0.89, 0.51, 0.28, 0, 1> * ObjectColor]"
               "[0.4 rgbft <0.70, 0.13, 0.00, 0, 1> * ObjectColor]"
               "[0.5 rgbft <0.69, 0.41, 0.08, 0, 1> * ObjectColor]"
               "[0.6 rgbft <0.49, 0.31, 0.28, 0, 1> * ObjectColor]"
               "[1.0 rgbft <0.89, 0.51, 0.28, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99}}";

    case sim_pov_rusty_iron:
        return "texture {pigment {granite color_map {"
               "[0.0 rgbft <0.42, 0.20, 0.20, 0, 1> * ObjectColor]"
               "[0.5 rgbft <0.50, 0.50, 0.02, 0, 1> * ObjectColor]"
               "[0.6 rgbft <0.60, 0.20, 0.20, 0, 1> * ObjectColor]"
               "[0.6 rgbft <0.30, 0.20, 0.20, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.99 diffuse 0.99} normal {wrinkles 1 scale 0.1}}";

    case sim_pov_soft_silver:
        return "texture {pigment {rgbft <0.94, 0.93, 0.83, 0, 1> * ObjectColor}"
               "finish {metallic ambient 0.65 diffuse 0.35 specular 0.85 "
               "roughness 0.01 reflection 0.45 brilliance 1.5}}";

    case sim_pov_new_penny:
        return "texture {pigment {rgbft <0.6, 0.45, 0.4, 0, 1> * ObjectColor}"
               "finish {metallic ambient 0.65 diffuse 0.65 specular 0.85 "
               "roughness 0.01 reflection 0.45 brilliance 1.5}}";

    case sim_pov_tinny_brass:
        return "texture {pigment {rgbft <0.70, 0.56, 0.37, 0, 1> * ObjectColor}"
               "finish {metallic ambient 0.5 diffuse 0.65 specular 0.85 "
               "roughness 0.01 reflection 0.45 brilliance 1.5}}";

    case sim_pov_gold_nugget:
        return "texture {pigment {rgbft <0.5, 0.35, 0.25, 0, 1> * ObjectColor}"
               "finish {metallic ambient 0.65 diffuse 0.65 specular 0.85 "
               "roughness 0.01 reflection 0.45 brilliance 1.5}}";

    case sim_pov_aluminum:
        return "texture {pigment {rgbft <0.55, 0.5, 0.45, 0, 1> * ObjectColor}"
               "finish {metallic ambient 0.65 diffuse 0.65 specular 0.85 "
               "roughness 0.01 reflection 0.45 brilliance 1.5}}";

    case sim_pov_bright_bronze:
        return "texture {pigment {rgbft <0.28, 0.40, 0.32, 0, 1> * ObjectColor}"
               "finish {metallic ambient 0.65 diffuse 0.65 specular 0.85 "
               "roughness 0.01 reflection 0.45 brilliance 1.5}}";

    case sim_pov_water:
        return "texture {pigment {rgbft <0.0, 0.0, 1.0, 0.9, 1> * ObjectColor}"
               "normal {ripples 0.75 frequency 10}"
               "finish {ambient 0.99 reflection {0.3, 1 fresnel} conserve_energy}}";

    case sim_pov_cork:
        return "texture {pigment {granite color_map {"
               "[0.00 rgbft <0.93, 0.71, 0.532, 0, 1> * ObjectColor]"
               "[0.60 rgbft <0.98, 0.81, 0.60, 0, 1> * ObjectColor]"
               "[0.60 rgbft <0.50, 0.30, 0.20, 0, 1> * ObjectColor]"
               "[0.65 rgbft <0.50, 0.30, 0.20, 0, 1> * ObjectColor]"
               "[0.65 rgbft <0.80, 0.53, 0.46, 0, 1> * ObjectColor]"
               "[1.00 rgbft <0.85, 0.75, 0.35, 0, 1> * ObjectColor]"
               "}} finish {ambient 0.65 diffuse 0.99 specular 0.1 roughness 0.5} scale 0.25}";

    case sim_pov_lightning:
        return "texture {pigment {marble color_map {"
               "[0.00 rgbft <1,1,1,0,1> * ObjectColor]"
               "[0.15 rgbft <0.94, 0.81, 0.99, 0.65, 1> * ObjectColor]"
               "[0.25 rgbft <0.94, 0.81, 0.99, 0.65, 1> * ObjectColor]"
               "[0.30 rgbft <0.87, 0.58, 0.98, 0.85, 1> * ObjectColor]"
               "[0.40 rgbft <0.87, 0.58, 0.98, 0.85, 1> * ObjectColor]"
               "[0.45 rgbft <0.73, 0.16, 0.96, 0.95, 1> * ObjectColor]"
                "} turbulence 0.5} finish {ambient 1}}";

    case sim_pov_mirror:
        return "texture {pigment {rgbft <0, 0, 0, 0, 0>}"
               "finish {ambient 0  diffuse 0 reflection 1}}";
    }

    return 0;
}
