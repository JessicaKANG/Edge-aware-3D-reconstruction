#include<opencv2/opencv.hpp>  
#include<iostream> 
#include<vector>
using namespace std;
using namespace cv;

void Converter(char* in_file, char* out_file, char* out_type) {
    //cv::String pattern = (string)in_file + (string)in_type;
    cv::String pattern = (string)in_file;

    vector<cv::String> fileNames;
    glob(pattern, fileNames, false);
    int i;
    for (i = 0; i < fileNames.size(); i++)
    {
        //get image name  
        string fileName = fileNames[i];
        string fileFullName = fileName;
	//find subname
	int pos = fileName.find(in_file);
	fileName.erase(pos, strlen(in_file));
	pos = fileName.find('.');
	fileName.erase(pos, 4);
	
	string out_fileFullName = (string)out_file + fileName + (string)out_type;
	Mat img = imread(fileFullName);
	Mat dst;

	cvtColor(img,dst,CV_BGR2GRAY);
	imwrite(out_fileFullName, dst);
	
        cout << "File: " << fileName <<" converted."<< endl;

    }
}
int main(int argc, char** argv) {
    if (argc!=4){
      cout << endl
      << "-----------------------------------------------------------\n"
      << "Image Converte:\n"
      << "-----------------------------------------------------------\n"
      << "-input_file: e.g. dir_path\n"
      << "-output_file: e.g. dir_path\n"
      << "-output_image_type: e.g. .ppm"<< endl
      << "------------------------------------------------------------"
      << endl;     
    }
    else
      Converter(argv[1], argv[2], argv[3]);
    return 0;
}