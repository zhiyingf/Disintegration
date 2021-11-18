#pragma once
#include <Eigen/core>
#include <iostream>
#include <vector>
#include <igl/writeOFF.h>
#include <igl/writeOBJ.h>
#include <regex>
#include <queue>
#include <fstream>
#include <igl/boundary_loop.h>
#include <igl/slice.h>
#include <igl/cat.h>
#include <igl/colon.h>
#include <igl/upsample.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/harmonic.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <filesystem>
#include <io.h> 

using namespace std;

void GetValidMeshAndInvalidVertex(const Eigen::MatrixXd& trimeshVertex,
    const Eigen::MatrixXi& trimeshFace,
    vector<Eigen::RowVector3d>& validVertex,
    vector<Eigen::RowVector3i>& validFace);

void GetValidMeshAndInvalidVertex(const vector<Eigen::RowVector3d>& trimeshVertex,
    const vector<Eigen::RowVector3i>& trimeshFace,
    vector<Eigen::RowVector3d>& validVertex,
    vector<Eigen::RowVector3i>& validFace);

void WriteOFFtoVector(string filename,const vector<Eigen::RowVector3d>& validVertex,const vector<Eigen::RowVector3i>& validFace,string suffix);

void WriteObjFromVF(string filename, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

//输入g fileName，以便再unity为网格命名
//https://www.jianshu.com/p/f7f3e7b6ebf5
void WriteObjByOfs(string pathOutput,string filename, const vector<Eigen::RowVector3d>& validVertex, const vector<Eigen::RowVector3i>& validFace);

//vector<Eigen::RowVectorXd> --> Eigen::MatrixXd or vector<Eigen::RowVectorXi> --> Eigen::MatrixXi; size = X
template<typename T1,typename T2>
void VectorToMatrix(const vector<T1>& vec, T2& matrix, size_t size=3) {
    matrix.resize(vec.size(), size);
    size_t i = 0;
    for (auto& v : vec) {
        matrix.block(i++, 0, 1, size) = v;
    }
}

//eg. Eigen::MatrixXi (n*3) --> vector<Eigen::RowVector3i> (n*3)
template<typename T1, typename T2>
void MatrixToVector(const T1& matrix, vector<T2>& vec, size_t size=3) {
    for (size_t t = 0; t < matrix.rows(); t++) {
        vec.push_back(matrix.row(t));
    }
}

//eg. vector<int> --> Eigen::VectorXi
template<typename T1,typename T2>
void VectorToVectorXX(const vector<T1>& vec, T2& vectorXX) {
    vectorXX.resize(vec.size());
    size_t i = 0;
    for (auto& v : vec) {
        vectorXX.coeffRef(i++) = v;
    }
}

//eg. Eigen::VectorXi --> vector<int>
template<typename T1, typename T2>
void VectorXXToVector(const T1& vectorXX, vector<T2>& vec) {
    for (size_t t = 0; t < vectorXX.size(); t++) {
        vec.push_back(vectorXX.coeffRef(t));
    }
}


void MatchStr(string str, string& matStrRes, string regSearch);

void DividedMesh(string name,const vector<Eigen::RowVector3d>& validVertex,const vector<Eigen::RowVector3i>& validFace, string pathOutput);

void findConnectedRegion(size_t& index, vector<Eigen::RowVector3i>& regionFace, vector<vector<int>>& pointToFace, vector<bool>& visPoint, const vector<Eigen::RowVector3d>& validVertex, const vector<Eigen::RowVector3i>& validFace);

void holeFilling(string fileName, vector<Eigen::RowVector3d>& validVertex, vector<Eigen::RowVector3i>& validFace);

void OFF2OBJ(string pathOld, string pathNew);

void OBJ2OFF(string pathOld, string pathNew);

void getFiles(std::string path, std::vector<std::string>& files, std::vector<std::string>& filenames);

void OnlyDiviedMesh(string filename, string pathOutput);

void solvePro(const vector<string>& fileVec, string pathModel, string pathLable, string pathOutput);
