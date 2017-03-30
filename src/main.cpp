#include "template_image_utilites.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>
#include "filex.h"
#include <math.h>

#if 1
int main(int argc, char *argv[])
{


	union U{
	float f;
	int i;};
	union U u;
	u.f  = 1.25;
	int d = pow(2.0f,100);
	d  = d%7;
	printf("%x\n",u.i);

	printf("%d\n",d);


	const std::string objFilePath = "data/bunny.obj";
	const std::string tempImageStorePath = "data/bunny";

	TemplateImagesFromReal(objFilePath, tempImageStorePath);

	return 0;

}
#endif 

#if 0 // 将3dsmax 渲染时输出的姿态（以旋转矩阵形式表示）转化成opencv中的罗德里格斯形式
int main(int argc, char *argv[])
{
	//.exe 3dsmax 渲染时输出的姿态参数文件路径 文件中姿态条目的个数

	if(argc < 3)
		return 0;
	else
	{
		const std::string poseInRotationFilePath = std::string(argv[1]);
		const int poseNum = atoi(argv[2]);
		const std::string poseInRodrigusFilePath = "poseInRodrigus.txt";

		std::ifstream poseInFile(poseInRotationFilePath.c_str());
		if(!poseInFile.is_open())
		{
			printf("Cannot open pose file\n");
			return -1;
		}

		std::ofstream poseOutFile( poseInRodrigusFilePath.c_str() );
		if(!poseOutFile.is_open())
		{
			printf("Cannot create file for recording poses for template images\n");
			return -1;
		}

		for(int i=0;i<poseNum;++i)
		{
			cv::Mat rotMat = cv::Mat::eye(3,3,CV_32FC1);
			float tVec[3] = {0.,0.,0.};
			poseInFile >> rotMat.at<float>(0,0) >> rotMat.at<float>(0,1) >> rotMat.at<float>(0,2)
				>> rotMat.at<float>(1,0) >> rotMat.at<float>(1,1) >> rotMat.at<float>(1,2)
				>> rotMat.at<float>(2,0) >> rotMat.at<float>(2,1) >> rotMat.at<float>(2,2)
				>> tVec[0] >> tVec[1] >> tVec[2];

			//rodrigus transformation
			std::vector<float> rotVec;
			cv::Rodrigues(rotMat, rotVec);

			poseOutFile << rotVec[0] << " " << rotVec[1] << " " << rotVec[2] << " "
				<< tVec[0] << " " << tVec[1] << " " << tVec[2] << std::endl;
		}

		poseOutFile.close();
		poseInFile.close();
	}

	return 0;

}
#endif 

#if 0 // 从3dsmax 渲染时输出的图片（和相机分辨率大小一致）中截取只包含物体的中心区域，作为模板
int main(int argc, char *argv[])
{
	// .exe 3dsmax 渲染时输出的图片文件夹路径 截取后的图片文件夹路径 截取区域的宽 截取区域的高

	if(argc < 5)
		return 0;
	else
	{
		const std::string maxImgFilePath = std::string(argv[1]);
		const std::string newImgFilePath = std::string(argv[2]);
		const int regionWidth = atoi(argv[3]);
		const int regionHeight = atoi(argv[4]);

		// load images
		std::vector<std::string> fileName;
		if( GetFileNames(maxImgFilePath,fileName) )
		{
			for(size_t k=0; k<fileName.size(); ++k)
			{
				std::string file = maxImgFilePath + "/" + fileName[k];
				cv::Mat img=cv::imread( file );

				int imgHeight = img.rows;
				int imgWidth = img.cols;
				int imgCenterY = imgHeight/2;
				int imgCenterX = imgWidth/2;

				// region area
				cv::Rect regionRect = cv::Rect(imgCenterX-regionWidth/2,imgCenterY-regionHeight/2,regionWidth,regionHeight);
				cv::Mat newImg = img(regionRect);

				std::string outFile = newImgFilePath + "/" + fileName[k];
				cv::imshow("img",newImg);
				cv::imwrite(outFile,newImg);
				cv::waitKey(30);
			}

		}

	}

	return 0;

}
#endif 

#if 0
int main(int argc, char *argv[])
{
	const std::string videoFilePath = "data/v_record.avi";

	// read video or camera
	cv::VideoCapture vc;
	vc.open(0);
	printf("Open camera\n");

	if( !vc.isOpened() )
	{
		printf("Cannot find the video source\n");
		return -1;
	}

	// write video
	int width = (int)vc.get(CV_CAP_PROP_FRAME_WIDTH);
	int height = (int)vc.get(CV_CAP_PROP_FRAME_HEIGHT);
	double fps = 30;
	cv::VideoWriter vw(videoFilePath,CV_FOURCC('D','I','V','X'),fps,cv::Size(width,height),1);

	if( !vw.isOpened() )
	{
		printf("Cannot write the video\n");
		return -1;
	}

	cv::Mat frame;
	char key = 'c';
	while(1)
	{
		vc>>frame;
		if(frame.empty()) break;

		cv::imshow("img",frame);
		char lkey= (char)cv::waitKey(10);
		
		if(lkey=='s')
		{
			printf("start...\n");
			key = lkey;
		}
		if(lkey=='f')
		{
			printf("end...\n");
			break;
		}

		if(key=='s')
		vw<<frame;
	}

	vc.release();
	vw.release();
	return 0;

}
#endif 

#if 0
int main(int argc, char *argv[])
{
	const std::string videoFilePath = "./data/squares40.avi";

	// read video or camera
	cv::VideoCapture vc;
	vc.open("./data/squares.avi");
	printf("Open camera\n");

	if( !vc.isOpened() )
	{
		printf("Cannot find the video source\n");
		return -1;
	}

	// write video
	int width = (int)vc.get(CV_CAP_PROP_FRAME_WIDTH);
	int height = (int)vc.get(CV_CAP_PROP_FRAME_HEIGHT);
	double fps = 40;
	cv::VideoWriter vw(videoFilePath,CV_FOURCC('D','I','V','X'),fps,cv::Size(width,height),1);

	if( !vw.isOpened() )
	{
		printf("Cannot write the video\n");
		return -1;
	}

	cv::Mat frame;
	char key = 'c';
	while(1)
	{
		vc>>frame;
		if(frame.empty()) break;

		cv::imshow("img",frame);
		char lkey= (char)cv::waitKey(10);

		vw<<frame;
	}

	vc.release();
	vw.release();
	return 0;

}
#endif 