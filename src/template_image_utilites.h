#ifndef TEMPLATE_IMAGE_UTILITIES_H
#define TEMPLATE_IMAGE_UTILITIES_H

#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include "CameraCalibration.hpp"
#include "Marker.hpp"
#include "MarkerDetector.hpp"
#include "glm.h"


// struct for passing mouse call parameters for opencv highgui
typedef struct MouseFuncParams_s
{
	MouseFuncParams_s(void):winName_(0), frameBuff_(0), frameDrawing_(0), drawingBox_(0), isDrawing_(0), done_(0)
	{
	};

	MouseFuncParams_s( char *winName, cv::Mat *frameBuff, cv::Mat *frameDrawing, cv::Rect *drawingBox, bool *isDrawing, bool *done )
	{
		winName_ = winName;
		frameBuff_ = frameBuff;
		frameDrawing_ = frameDrawing;
		drawingBox_ = drawingBox; 
		isDrawing_ = isDrawing;
		done_ = done;
	};

	~MouseFuncParams_s(void)
	{
	};

	char *winName_;
	cv::Mat *frameBuff_;
	cv::Mat *frameDrawing_;
	cv::Rect *drawingBox_;
	bool *isDrawing_;
	bool *done_;
} MouseFuncParams;

// call back function for segment out the object by using rectangle
void cvMouseDrawRectFunc( int event,int x,int y,int flags,void *param )
{
	MouseFuncParams *mfParams = (MouseFuncParams *)param;
	char *winName = mfParams->winName_;
	cv::Mat *frame = mfParams->frameBuff_;
	cv::Mat *frameDrawing = mfParams->frameDrawing_;
	cv::Rect *drawingBox = mfParams->drawingBox_;
	bool *isDrawing = mfParams->isDrawing_;
	bool *done = mfParams->done_;

	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		{
			*isDrawing = true;
			*done = false;
			drawingBox->x = x;
			drawingBox->y = y;
		}
		break;
	case cv::EVENT_MOUSEMOVE:
		if( *isDrawing )
		{
			drawingBox->width = x - drawingBox->x + 1;
			drawingBox->height = y - drawingBox->y + 1;

			frame->copyTo( *frameDrawing );
			cv::rectangle( *frameDrawing, *drawingBox, cv::Scalar(0,255,0), 2);
			cv::circle( *frameDrawing, cv::Point(drawingBox->x+drawingBox->width/2, drawingBox->y+drawingBox->height/2), 
				3, cv::Scalar(0,255,0), CV_FILLED);


			cv::imshow(winName, *frameDrawing);
		}
		break;
	case cv::EVENT_LBUTTONUP:
		if(isDrawing)
		{
			drawingBox->width = x - drawingBox->x + 1;
			drawingBox->height = y - drawingBox->y + 1;

			frame->copyTo( *frameDrawing );
			cv::rectangle( *frameDrawing, *drawingBox, cv::Scalar(0,0,255), 2);	
			cv::circle( *frameDrawing, cv::Point(drawingBox->x+drawingBox->width/2, drawingBox->y+drawingBox->height/2), 
				3, cv::Scalar(0,0,255), CV_FILLED);

			imshow(winName, *frameDrawing);

			*isDrawing = false;
			*done = true;
		}
		break;
	}
}


//==============================================================================================

void projectPoints(const std::vector<cv::Point3f> &points3d, const CameraCalibration &camCalib,
	const Transformation &trans, std::vector<cv::Point2f> &points2d)
{
	cv::Mat intrinsic = cv::Mat::eye(3,3,CV_32FC1);
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			intrinsic.at<float>(i,j) = camCalib.getIntrinsic()(i,j);
		}
	}

	cv::Mat extrinsic(3,4,CV_32FC1);
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			extrinsic.at<float>(i,j) = trans.r().mat[i][j];
		}

		extrinsic.at<float>(i,3) = trans.t().data[i];
	}

	int pNum = (int)points3d.size();
	cv::Mat ps3d(4, pNum, CV_32FC1);
	for(int j=0; j<pNum; ++j)
	{
		ps3d.at<float>(0,j) = points3d[j].x;
		ps3d.at<float>(1,j) = points3d[j].y;
		ps3d.at<float>(2,j) = points3d[j].z;
		ps3d.at<float>(3,j) = 1;
	}

	cv::Mat ps2d(3, pNum, CV_32FC1);
	ps2d = intrinsic*extrinsic*ps3d;
	for(int j=0; j<pNum; ++j)
	{
		float u = ps2d.at<float>(0,j)/ps2d.at<float>(2,j);
		float v = ps2d.at<float>(1,j)/ps2d.at<float>(2,j);
		points2d.push_back(cv::Point2f(u,v));
	}
}

void calculateObjTransformation(const std::vector<Transformation> &markerTransformation, 
	float *marker2objTranslation, std::vector<Transformation> &objTransformation)
{
	if(markerTransformation.size()==0)
		return ;

	// just consider one marker here
	const Matrix33 &rMat = markerTransformation[0].r();
	const Vector3 &tVec = markerTransformation[0].t();

	Matrix33 objRMat = Matrix33::identity();
	Vector3 objTVec = Vector3::zero();

	// the rotation mat is same to marker's
	/*float cos60 = std::cos(3.1415926/3);
	float sin60 = std::sin(3.1415926/3);
	Matrix33 rotz60 = Matrix33::identity();
	rotz60.mat[0][0] = cos60; rotz60.mat[0][1] = -sin60; rotz60.mat[0][2] = 0;
	rotz60.mat[1][0] = sin60; rotz60.mat[1][1] = cos60; rotz60.mat[1][2] = 0;
	rotz60.mat[2][0] = 0; rotz60.mat[2][1] = 0; rotz60.mat[2][2] = 1;

	for( int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			float val = 0;
			for(int k=0; k<3; ++k)
			{
				val +=(rMat.mat[i][k]*rotz60.mat[k][j]);
			}

			objRMat.mat[i][j] = val;
		}
	}*/

	for( int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			objRMat.mat[i][j] = rMat.mat[i][j];
		}
	}

	//the translation =  rMat*marker2objTranslation + tVec
	for(int i=0; i<3; ++i)
	{
		float val = 0;
		for(int j=0; j<3; ++j)
		{
			val += (rMat.mat[i][j] * marker2objTranslation[j]);
		}

		objTVec.data[i] = val + tVec.data[i];
	}

	objTransformation.push_back( Transformation(objRMat, objTVec));
}

void drawCoordinates(const CameraCalibration &camCalib,
	const std::vector<Transformation> &trans, cv::Mat &img)
{
	// camera parameters and distortion coefficients
	const cv::Matx33f camIntrinsic = camCalib.getIntrinsic();
	const cv::Matx<float,5,1> camDistortion = camCalib.getDistorsion();

	// coordiantes points in 3d
	std::vector<cv::Point3f> coor_points_3d;
	coor_points_3d.push_back(cv::Point3f(0.f,0.f,0.f));
	coor_points_3d.push_back(cv::Point3f(4.f,0.f,0.f));
	coor_points_3d.push_back(cv::Point3f(0.f,4.f,0.f));
	coor_points_3d.push_back(cv::Point3f(0.f,0.f,4.f));

	// project above points to 2d
	std::vector<cv::Point2f> coor_points_2d;
	for( size_t i=0; i<trans.size(); ++i)
	{
		if(i>0)
			break;
		cv::Matx33f rotMat(trans[i].r().data);
		std::vector<float> rVec;
		cv::Rodrigues(rotMat, rVec);

		std::vector<float> tVec(trans[i].t().data, trans[i].t().data+3);

		coor_points_2d.clear();
		cv::projectPoints(coor_points_3d, rVec, tVec, camIntrinsic, camDistortion, coor_points_2d);
		//projectPoints(coor_points_3d, camCalib, trans[0], coor_points_2d);

		cv::line(img, cv::Point((int)coor_points_2d[0].x, (int)coor_points_2d[0].y),
			cv::Point((int)coor_points_2d[1].x, (int)coor_points_2d[1].y), CV_RGB(255,0,0),2);
		cv::line(img, cv::Point((int)coor_points_2d[1].x, (int)coor_points_2d[1].y),
			cv::Point((int)coor_points_2d[1].x, (int)coor_points_2d[1].y), CV_RGB(255,0,0),5);

		cv::line(img, cv::Point((int)coor_points_2d[0].x, (int)coor_points_2d[0].y),
			cv::Point((int)coor_points_2d[2].x, (int)coor_points_2d[2].y), CV_RGB(0,255,0),2);
		cv::line(img, cv::Point((int)coor_points_2d[2].x, (int)coor_points_2d[2].y),
			cv::Point((int)coor_points_2d[2].x, (int)coor_points_2d[2].y), CV_RGB(0,255,0),5);

		cv::line(img, cv::Point((int)coor_points_2d[0].x, (int)coor_points_2d[0].y),
			cv::Point((int)coor_points_2d[3].x, (int)coor_points_2d[3].y), CV_RGB(0,0,255),2);
		cv::line(img, cv::Point((int)coor_points_2d[3].x, (int)coor_points_2d[3].y),
			cv::Point((int)coor_points_2d[3].x, (int)coor_points_2d[3].y), CV_RGB(0,0,255),5);
	}
}

void drawObject( const CameraCalibration &camCalib,const std::vector<Transformation> &trans,
	GLMmodel *objModel, cv::Mat &img)
{
	// camera parameters and distortion coefficients
	const cv::Matx33f camIntrinsic = camCalib.getIntrinsic();
	const cv::Matx<float,5,1> camDistortion = camCalib.getDistorsion();

	// get obj vertices data
	unsigned int verticesNum = objModel->numvertices;
	GLfloat *vertices = objModel->vertices;
	std::vector<cv::Point3f> points3d(verticesNum);
	for(size_t i=1; i<=verticesNum; ++i)
	{
		points3d[i-1] = cv::Point3f(vertices[3 * i + 0],
									vertices[3 * i + 1],
									vertices[3 * i + 2]);
	}

	//get obj edges date
	unsigned int edgesNum = objModel->numLines;
	GLMLine *edges = objModel->lines;

	// project above points to 2d
	std::vector<cv::Point2f> points2d;
	for( size_t i=0; i<trans.size(); ++i)
	{
		if(i>0)
			break;

		cv::Matx33f rotMat(trans[i].r().data);
		std::vector<float> rVec;
		cv::Rodrigues(rotMat, rVec);

		std::vector<float> tVec(trans[i].t().data, trans[i].t().data+3);

		points2d.clear();
		cv::projectPoints(points3d, rVec, tVec, camIntrinsic, camDistortion, points2d);
		//projectPoints(points3d, camCalib, trans[0], points2d);

		//draw the 2d points
		for(size_t j=0; j<verticesNum; ++j)
		{
			cv::line(img, cv::Point((int)points2d[j].x, (int)points2d[j].y),
			cv::Point((int)points2d[j].x, (int)points2d[j].y), CV_RGB(0,0,0),2);
		}

		// and draw the edges
		for(size_t j=0; j<edgesNum; ++j)
		{
			unsigned int p0= edges[j].vindices[0]-1;
			unsigned int p1 = edges[j].vindices[1]-1;
			cv::line(img, cv::Point((int)points2d[p0].x, (int)points2d[p0].y),
			cv::Point((int)points2d[p1].x, (int)points2d[p1].y), CV_RGB(255,255,255),1);
		}
	}
}

//NOTE: before capturing template images
//      MUST MODIFY the configurations
// marker size configuration

// ----- for object -----
// the z coordinates value of object origin
// MUST change when new object model coming

//float objOriginZ = 0.95f; //this is for model duplo small square
//float objOriginZ = 0.4f; //this is for model slot and noslot
float objOriginZ = 1.5f; //this is for model box 

// ----- for marker 1 -----//the location of object w.r.t the marker origin point
cv::Size2f marker9x9 = cv::Size2f(9.0f, 9.0f);
float obj2marker9x9Translation[3] = {10.5f, 4.5f, objOriginZ};


// ----- for marker 2 -----//the location of object w.r.t the marker origin point
cv::Size2f marker5x5 = cv::Size2f(5.3f, 5.3f);
float obj2marker5x5Translation[3] = {9.35f, 5.65f, objOriginZ};

// obtain template from camera capturing or video files
int TemplateImagesFromReal( const std::string &objFilePath, const std::string &imgStoreFolder, 
	std::string videoPath= std::string())
{
	//the location of object w.r.t the marker
	float *obj2markerTranslation = obj2marker5x5Translation;

	//load camera intrinsic parameters
	float fx = 840.83909392801706;
	float fy = 840.83909392801706;
	float cx = 319.5;
	float cy = 239.5;
	float distortionCoeff[5] = {-0.0073528715923502968, 1.3968282968421968, 
								0., 0.,-9.3220236317679284};
	CameraCalibration camCalib(fx,fy,cx,cy,distortionCoeff);

	//float distortionCoeff[5] = {-0.040817887112457034, 0.42918200942843993, 
	//							0., 0.,-1.1326434091563919};
	//CameraCalibration camCalib(727.55,727.55,479.5,269.5,distortionCoeff);

	//load obj file
	GLMmodel *objModel = glmReadOBJ(const_cast<char*>(objFilePath.c_str()));

	//initial marker detector
	MarkerDetector markerDetector(camCalib, marker5x5);

	// read video or camera
	cv::VideoCapture vc;
	if( videoPath.empty() )
	{
		vc.open(0);
		printf("Open camera\n");
	}else
	{
		vc.open(videoPath);
		printf("Open video file: %s\n", videoPath.c_str());
	}

	if( !vc.isOpened() )
	{
		printf("Cannot find the video source\n");
		return -1;
	}

	{
		printf("Help:\n");
		printf("\t -- Left Click to draw the bounding box of target objects\n"
		         "\t z --  Undo\n"
		         "\t n --  Next\n"
				 "\t q --  Quit\n");
	}

	// control variables and settings
	bool isDrawing = false;
	bool done = true;
	cv::Rect drawingBox(0,0,0,0);

	int counter = 1;
	char *imgFileNameFormat = "%04d.png"; 
	std::string poseFileName = imgStoreFolder + "/poses.txt"; // store the pose of each template image
	std::ofstream poseOutFile( poseFileName.c_str() );
	if(!poseOutFile.is_open())
	{
		printf("Cannot create file for recording poses for template images\n");
		return -1;
	}

	const int frameWidth = (int) vc.get(CV_CAP_PROP_FRAME_WIDTH);
	const int frameHeight = (int) vc.get(CV_CAP_PROP_FRAME_HEIGHT);
	cv::Mat frame, frameDrawing, arDrawing;

	// initial GUI
	char *winNameSource = "TemplateSource";
	char *winNameDisplay = "ARScene";
	char * winNameTemplate = "Template";
	cv::namedWindow(winNameSource);
	cv::namedWindow(winNameDisplay);
	cv::namedWindow(winNameTemplate);
	MouseFuncParams params(winNameSource, &frame, &frameDrawing, &drawingBox, &isDrawing, &done);
	cv::setMouseCallback(winNameSource, cvMouseDrawRectFunc, &params);

	for(;;)
	{
		vc >> frame;
		if( frame.empty() ) break;
		frame.copyTo( frameDrawing );
		frame.copyTo( arDrawing );

		markerDetector.processFrame(arDrawing);
		const std::vector<Transformation> &markerTrans = markerDetector.getTransformations();
		drawCoordinates(camCalib, markerTrans, arDrawing);

		std::vector<Transformation> objTrans;
		calculateObjTransformation(markerTrans, obj2markerTranslation, objTrans);
		drawObject(camCalib, objTrans, objModel, arDrawing);

		cv::imshow(winNameDisplay, arDrawing);
		cv::imshow(winNameSource, frameDrawing);

		//listen the keyboard and manipulate the keyboard event
		int key = 0;
		if(isDrawing && !done)
		{
			key=cv::waitKey(0);

			switch ((char)key)
			{
				//for z 122, undo
			case 'z':
				if(done)
				{
					done = false;
					frame.copyTo(frameDrawing);
					imshow(winNameSource,frameDrawing);
				}
				break;
				//for n 110, finish
			case 'n':
				if(done)
				{
					// cut out the template image that contains the object
					if((int)objTrans.size()>0)
					{
						cv::Mat objTemplImg = frame(drawingBox);
						std::string objFileName = imgStoreFolder + "/" + cv::format(imgFileNameFormat,counter);
						cv::imwrite(objFileName, objTemplImg);

						cv::Mat rotMat = cv::Mat::eye(3,3,CV_32FC1);
						for(int i=0; i<3; ++i)
						{
							for(int j=0; j<3; ++j)
							{
								rotMat.at<float>(i,j) = objTrans[0].r().mat[i][j];
							}
						}
						std::vector<float> rotVec;
						cv::Rodrigues(rotMat, rotVec);

						// rotation vector and translation vector
						poseOutFile << rotVec[0] <<" " << rotVec[1] <<" " << rotVec[2] <<" "
									<< objTrans[0].t().data[0] <<" "
									<< objTrans[0].t().data[1] <<" "
									<< objTrans[0].t().data[2] << std::endl;

						++counter;
					}else
					{
						printf("Cannot estiamte pose, so this snapshot is canceled\n");
					}
				}
				break;
			}
		}
		else
		{
			key=cv::waitKey(60);
		}

		if(key==27) break;

	}


	cv::destroyWindow(winNameSource);
	cv::destroyWindow(winNameDisplay);
	cv::destroyWindow(winNameTemplate);
	vc.release();

	poseOutFile.close();
	glmDelete(objModel);

	return 1;
}

#endif //TEMPLATE_IMAGE_UTILITIES_H