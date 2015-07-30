#include <jni.h>
#include <android/log.h>

#include <string.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>

#include "pba.h"
#include "util.h"

#ifndef JNIEXPORT
#define JNIEXPORT
#endif

#ifndef JNICALL
#define JNICALL
#endif


using namespace std;

extern "C"{
	void test() {
		//////////////////////////////////////////////////////
        int argc = 2;
        char** argv = new char*[argc];
		argv[0] = "driver_no_gpu";
		argv[1] = "/storage/sdcard/Bust2.nvm";            //first argument must be filename
		char* input_filename  = argv[1];            //first argument must be filename
		char* driver_argument = argv[argc - 1];    //last argument  for the driver

		/////////////////////////////////////////////////////////////////
		//CameraT, Point3D, Point2D are defined in src/pba/DataInterface.h
		vector<CameraT>        camera_data;    //camera (input/ouput)
		vector<Point3D>        point_data;     //3D point(iput/output)
		vector<Point2D>        measurements;   //measurment/projection vector
		vector<int>            camidx, ptidx;  //index of camera/point for each projection

		/////////////////////////////////////
		vector<string>         photo_names;        //from NVM file, not used in bundle adjustment
		vector<int>            point_color;        //from NVM file, not used in bundle adjustment

		/////////////////////////////////////////////////////////////
		///load the data. You will need to replace the loader for your own data
		if(argc < 2 || !LoadModelFile(input_filename, camera_data, point_data, measurements,
							ptidx, camidx, photo_names, point_color))
		{
			LOGI("Start load model file ...\n");
			std::cout << "==== Multicore Bundle Adjustment ---- Demo Driver Syntax ====\n"
	#ifdef WIN32
				"driver(_x64)(_debug) "
	#else
				"driver "
	#endif
				"input [pba parameters][-out output_nvm][driver argument]\n"
				"	input:            file must be NVM or bundler format\n"
				"[driver argument] must be the last one. It can be one of the following\n"
				"	--noise:          add 5% random noise to all parameters\n"
				"	--float:          use CPU-float implementation instead of GPU\n"
				"	--double:         use CPU-double implementation instead of GPU\n"
				"[pba parameters]: -lmi <#>, -cgi <#>, -cgn <f>, -budget <#>...\n"
				"	-lmi <#>:         specify the number of LM iterations\n"
				"	-profile:         run profiling experiments and see the speeds\n"
				"	                  check documentation for more details.\n";
			return;
		}else
		{
			LOGI("Start load model file2 ...\n");
			//if(strstr(driver_argument, "--checkv")) ExamineVisiblity(input_filename);

			//remove file extension for conversion/saving purpose
			//char* dotpos = strrchr(input_filename, '.');
			//if(dotpos && strchr(dotpos, '/') == NULL && strchr(dotpos, '\\') == NULL) *dotpos = 0;

			LOGI("Start load model file3 ...\n");
			//////////////////////////////////////////////////////////
			//use the last parameter for special purpose.
			string surfix = "-converted";
			if(strstr(driver_argument, "--fix_visibility"))
			{
				if(RemoveInvisiblePoints(camera_data, point_data, ptidx, camidx, measurements, photo_names, point_color))
					surfix = "-fixed";
			}else
			{
				if(strstr(driver_argument, "--noise") != NULL)     //add noise for experimentation
				{
					//AddNoise(camera_data, point_data, 0.05f);    //add 5% noise for experiments
					AddStableNoise(camera_data, point_data, ptidx, camidx, 0.05f);
					surfix = "-noised";
				}
			}
			LOGI("Start load model file4 ...\n");
			//file format conversion for experimentation
			if(strstr(driver_argument, "--convert_to_nvm"))
				SaveNVM(    (string(input_filename) + surfix + ".nvm").c_str(), camera_data,
							point_data, measurements, ptidx, camidx, photo_names, point_color);
			if(strstr(driver_argument, "--convert_to_bm"))
				SaveBundlerModel(    (string(input_filename) + surfix + ".txt").c_str(),
									camera_data, point_data, measurements, ptidx, camidx);
		}
		
		LOGI("End load model file ...\n");

		/////////////////////////////////////////////////////////////////////////////////////////
		ParallelBA::DeviceT device = ParallelBA::PBA_CUDA_DEVICE_DEFAULT;
		if(strstr(driver_argument, "--float"))          device = ParallelBA::PBA_CPU_FLOAT;
		else if(strstr(driver_argument, "--double"))    device = ParallelBA::PBA_CPU_DOUBLE;

		/////////////////////////////////////////////////////////////////////
		ParallelBA pba(device);          //You should reusing the same object for all new data

		/////////////////////////////////////////////////////////
		//Parameters can be changed before every call of RunBundleAdjustment
		//But do not change them from another thread when it is running BA.
		pba.ParseParam(argc, argv);      //indirect parameter tuning from commandline
		//pba.SetFixedIntrinsics(true); //if your focal lengths are calibrated.
		//           equivalent to pba.GetInternalConfig()->__fixed_focallength = true;
		//pba.EnableRadialDistortion(*); // if you want to enable radial distortion
		//           equivalent to pba.GetInternalConfig()->__use_radial_distortion = *;
		//check src/pba/ConfigBA.h for more details on the parameter system

		////////////////////////////////////
		//Tweaks before bundle adjustment
		//1. For each camera, you can call CameraT::SetConstantCamera() to keep it unchanged.
		//2. pba.SetNextBundleMode(ParallelBA::BUNDLE_ONLY_MOTION) //chose a truncated mode?

		////////////////////////////////////////////////////////////////
		pba.SetCameraData(camera_data.size(),  &camera_data[0]);                        //set camera parameters
		pba.SetPointData(point_data.size(), &point_data[0]);                            //set 3D point data
		pba.SetProjection(measurements.size(), &measurements[0], &ptidx[0], &camidx[0]);//set the projections

		vector<int> cmask;
		if(strstr(driver_argument, "--common"))
		{
			cmask.resize(camera_data.size(), 0);
			pba.SetFocalMask(&cmask[0]);
		}
		//WARNING: measumrents must be stored in correct order
		//all measutments (in different images) for a single 3D point must be stored continously,
		//Currently, there is no order verification internally.
		//Basically, ptidx should be non-decreasing

		//////////////////////////////////////////////////////
		//pba.SetTimeBudget(10);      //use at most 10 seconds?
		pba.RunBundleAdjustment();    //run bundle adjustment, and camera_data/point_data will be modified


		//Write the optimized system to file
		const char*  outpath = pba.GetInternalConfig()->GetOutputParam();
		SaveModelFile(outpath, camera_data, point_data, measurements, ptidx, camidx, photo_names, point_color);
		//It is easy to visualize the camera/points yourself,
	}

	void Java_com_example_pbatest_MainActivity_test(JNIEnv* env, jobject thiz )
	{
		test();
	}
}
