#include "util.h"
using namespace std;


int main() {
	vector<string> fileVec;
	vector<string> fileNam;

	//current path is ./build
	//model path: eg.data is http://irc.cs.sdu.edu.cn/~yunhai/public_html/ssl/data/Large-Chairs/shapes.zip
	string pathModel = "../Data/shapes/";
	//label path: eg.data is http://irc.cs.sdu.edu.cn/~yunhai/public_html/ssl/data/Large-Chairs/gt.zip
	string pathLable = "../Data/gt/";
	//output path
	string pathOutput = "../Data/offOutput/";

	getFiles(pathModel, fileVec, fileNam);
	solvePro(fileNam, pathModel, pathLable, pathOutput);
	return 0;
}



