#include "util.h"

using namespace std;

//去除三角网格多余点（保留有效的点和面）
void GetValidMeshAndInvalidVertex(const Eigen::MatrixXd& trimeshVertex,
    const Eigen::MatrixXi& trimeshFace,
    vector<Eigen::RowVector3d>& validVertex,
    vector<Eigen::RowVector3i>& validFace)
{
    //分配给无效和有效的点label
    vector<bool> trimeshVertexValidLable(trimeshVertex.rows(), false);
    for (size_t i = 0; i < trimeshFace.rows(); ++i)
    {
        trimeshVertexValidLable[trimeshFace.coeffRef(i, 0)] = true;
        trimeshVertexValidLable[trimeshFace.coeffRef(i, 1)] = true;
        trimeshVertexValidLable[trimeshFace.coeffRef(i, 2)] = true;
    }
    //获取有效的点 and 获取无效的点
    validVertex.clear();
    for (size_t i = 0; i < trimeshVertexValidLable.size(); ++i)
    {
        if (trimeshVertexValidLable[i])
        {
            validVertex.push_back(trimeshVertex.row(i));
        }
    }
    //获取有效的面
    int invalidcount = 0;
    vector<int> vertexbeforenumofiv(trimeshVertex.rows(), -1);
    for (size_t i = 0; i < trimeshVertexValidLable.size(); ++i)
    {
        if (trimeshVertexValidLable[i])
        {
            vertexbeforenumofiv[i] = invalidcount;
        }
        else
        {
            invalidcount++;
            vertexbeforenumofiv[i] = invalidcount;
        }
    }
    validFace.clear();
    for (size_t i = 0; i < trimeshFace.rows(); ++i)
    {
        Eigen::Vector3i temf = trimeshFace.row(i);
        temf[0] -= vertexbeforenumofiv[temf[0]];
        temf[1] -= vertexbeforenumofiv[temf[1]];
        temf[2] -= vertexbeforenumofiv[temf[2]];
        validFace.push_back(temf);
    }
}

void GetValidMeshAndInvalidVertex(const vector<Eigen::RowVector3d>& trimeshVertex,
    const vector<Eigen::RowVector3i>& trimeshFace,
    vector<Eigen::RowVector3d>& validVertex,
    vector<Eigen::RowVector3i>& validFace)
{
    //分配给无效和有效的点label
    vector<bool> trimeshVertexValidLable(trimeshVertex.size(), false);
    for (unsigned int i = 0; i < trimeshFace.size(); ++i)
    {
        trimeshVertexValidLable[trimeshFace[i][0]] = true;
        trimeshVertexValidLable[trimeshFace[i][1]] = true;
        trimeshVertexValidLable[trimeshFace[i][2]] = true;
    }
    //获取有效的点 and 获取无效的点
    //invalidVertex.clear();
    validVertex.clear();
    for (unsigned int i = 0; i < trimeshVertexValidLable.size(); ++i)
    {
        if (trimeshVertexValidLable[i])
        {
            validVertex.push_back(trimeshVertex[i]);
        }
    }
    //获取有效的面
    int invalidcount = 0;
    vector<int> vertexbeforenumofiv(trimeshVertex.size(), -1);
    for (unsigned int i = 0; i < trimeshVertexValidLable.size(); ++i)
    {
        if (trimeshVertexValidLable[i])
        {
            vertexbeforenumofiv[i] = invalidcount;
        }
        else
        {
            invalidcount++;
            vertexbeforenumofiv[i] = invalidcount;
        }
    }
    validFace.clear();
    for (unsigned int i = 0; i < trimeshFace.size(); ++i)
    {
        Eigen::RowVector3i temf = trimeshFace[i];
        temf[0] -= vertexbeforenumofiv[temf[0]];
        temf[1] -= vertexbeforenumofiv[temf[1]];
        temf[2] -= vertexbeforenumofiv[temf[2]];
        validFace.push_back(temf);
    }
}

void WriteOFFtoVector(string filename,const vector<Eigen::RowVector3d>& validVertex,const vector<Eigen::RowVector3i>& validFace,string suffix) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    /*VectorToMatrix<Eigen::RowVector3d, Eigen::MatrixXd>(validVertex, V);
    VectorToMatrix<Eigen::RowVector3i, Eigen::MatrixXi>(validFace, F);*/
    VectorToMatrix(validVertex, V);
    VectorToMatrix(validFace, F);

    if (suffix == ".off") {

        igl::writeOFF(filename + suffix, V, F);
    }
    else if (suffix == ".obj") {
        igl::writeOBJ(filename + ".obj", V, F);
    }
}
//add g filename string filepath不含后缀 
void WriteObjFromVF(string filename, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    ofstream out(filename + ".obj");
    vector<Eigen::RowVector3d> validVertex;
    vector<Eigen::RowVector3i> validFace;
    MatrixToVector(V, validVertex);
    MatrixToVector(F, validFace);

    out << "g " << filename << endl;
    for (auto v : validVertex) {
        out << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }
    for (auto f : validFace) {
        f = f + Eigen::RowVector3i(1, 1, 1);
        out << "f " << f[0] << " " << f[1] << " " << f[2] << endl;
    }
    out.close();

}

void WriteObjByOfs(string pathOutput, string filename, const vector<Eigen::RowVector3d>& validVertex, const vector<Eigen::RowVector3i>& validFace) {
    ofstream out(pathOutput + filename+".obj");
    out << "g " << filename << endl;
    for (auto v : validVertex) {
        out << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }
    for (auto f: validFace) {
        f = f + Eigen::RowVector3i(1, 1, 1);
        out << "f " << f[0] << " " << f[1] << " " << f[2] << endl;
    }
    out.close();
}

//regex_search 搜索匹配
void MatchStr(string str, string& matStrRes, string regSearch) {
    regex name_regex(regSearch);
    smatch result;
    string name_model;
    if (regex_search(str, result, name_regex))
    {
        matStrRes = result.str();
    }
    //cout << matStrRes << endl;
}

///思路：
/// 1)找到每个顶点直接相连的面 vector<vector<int>> pointToFace
/// 2)新建一个标记是否访问过顶点 vector<bool> visPoint(pointSize,false);
/// 3)新建一个容器记录面相邻面 vector<Eigen::RowVector3i> regionFace
/// 使用广度优先遍历queue，从第一个顶点开始遍历，将相邻面加入容器，将相邻面的另两个顶点入队... ...
void DividedMesh(string name,const vector<Eigen::RowVector3d>& validVertex,const vector<Eigen::RowVector3i>& validFace,string pathOutput) {
    size_t vN = validVertex.size();
    size_t fN = validFace.size();

    vector<vector<int>> pointToFace(vN);
    vector<bool> visPoint(vN, false);
    vector<Eigen::RowVector3i> regionFace;

    for (auto it = validFace.begin(); it != validFace.end(); it++) {
        int t = it - validFace.begin();
        pointToFace[(*it)[0]].push_back(t);
        pointToFace[(*it)[1]].push_back(t);
        pointToFace[(*it)[2]].push_back(t);
    }

    int regionNum = 0;
    for (size_t i = 0; i < vN ; i++) {
        if (!visPoint[i]) {
            regionNum++;
            regionFace.clear();
            findConnectedRegion(i, regionFace, pointToFace, visPoint, validVertex, validFace);
            
            vector<Eigen::RowVector3d> regVertex;
            vector<Eigen::RowVector3i> regFace;
            GetValidMeshAndInvalidVertex(validVertex, regionFace, regVertex, regFace);

            //命名规则mesh_X1_X2_X3.off
            //X1.off ; X2 to seg label ; X3 to regionNum
            string fileName = name + "_" + to_string(regionNum);// +string(".obj");//off
            if (regFace.size() == fN) {
                fileName = name;// +string(".obj");
            }

            holeFilling(fileName, regVertex, regFace);

            //WriteOFFtoVector(fileName, regVertex, regFace,string(".off"));
            WriteObjByOfs(pathOutput, fileName, regVertex, regFace);
            
        }
    }

      

}

//给定一个point 查找连通区域
void findConnectedRegion(size_t& index, vector<Eigen::RowVector3i>& regionFace, vector<vector<int>>& pointToFace, vector<bool>& visPoint, const vector<Eigen::RowVector3d>& validVertex, const vector<Eigen::RowVector3i>& validFace) {
    queue<size_t> que;
    
    //记录一个面是否已记录
    vector<bool> visFace(validFace.size(), false);

    que.push(index);
    visPoint[index] = true;
    while (!que.empty()) {
        size_t top = que.front();
        que.pop();
        
        vector<int>& ve = pointToFace[top];
        for (auto& v : ve) {
            const Eigen::RowVector3i& face = validFace[v];
            if (!visFace[v]) {
                regionFace.push_back(face);
                visFace[v] = true;
            }
            for (int i = 0; i < 3; i++) {
                if (face[i] != top && !visPoint[face[i]]) {
                    que.push(face[i]);
                    visPoint[face[i]] = true;
                }
            }
        }
    }
}

void holeFilling(string fileName,vector<Eigen::RowVector3d>& validVertex,vector<Eigen::RowVector3i>& validFace) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    VectorToMatrix(validVertex, V);
    VectorToMatrix(validFace, F);

    vector<vector<int>> boundaryLoop;
    
    
    igl::boundary_loop(F, boundaryLoop);//indices of the boundary of the hole.

    if (boundaryLoop.size() == 0)return;
    
    for (auto& originalLoop : boundaryLoop) {
        // compute boundary center.
        Eigen::VectorXi R;
        VectorToVectorXX(originalLoop, R);
        Eigen::RowVectorXd bcenter(3);
        {
            Eigen::VectorXi C(3); C << 0, 1, 2;
            Eigen::MatrixXd B;
            Eigen::MatrixXd A = V;
            igl::slice(A, R, C, B);
            bcenter = (1.0f / originalLoop.size()) * B.colwise().sum();
        }

        validVertex.push_back(bcenter);
        for (int i = 0; i < originalLoop.size(); ++i) {
            Eigen::RowVector3i pt;
            int x = validVertex.size() - 1;
            int y = originalLoop[i];
            int z = originalLoop[(1 + i) % originalLoop.size()];
            //pt << x, y, z;
            pt << x, z, y;
            validFace.push_back(pt);
        }

        //Eigen::MatrixXd patchV = Eigen::MatrixXd(originalLoop.size() + 1, 3);
        //Eigen::MatrixXi patchF = Eigen::MatrixXi(originalLoop.size(), 3);
        //{
        //    Eigen::VectorXi C(3); C << 0, 1, 2;
        //    Eigen::MatrixXd temp1;
        //    igl::slice(V, R, C, temp1);

        //    Eigen::MatrixXd temp2(1, 3);
        //    temp2 << bcenter(0), bcenter(1), bcenter(2);

        //    // patch vertices will be the boundary vertices, plus a center vertex. concat these together.
        //    igl::cat(1, temp1, temp2, patchV);

        //    // create triangles that connect boundary vertices and the center vertex.
        //    for (int i = 0; i < originalLoop.size(); ++i) {
        //        patchF(i, 2) = (int)originalLoop.size();
        //        patchF(i, 1) = i;
        //        patchF(i, 0) = (1 + i) % originalLoop.size();
        //    }

        //    // also upsample patch. patch and original mesh will have the same upsampling level now
        //    // making it trivial to fuse them together.
        //    //igl::upsample(Eigen::MatrixXd(patchV), Eigen::MatrixXi(patchF), patchV, patchF, upsampleN);
        //}

    }


}

//path: /user/to/path
void OBJ2OFF(string pathOld, string pathNew) {
    vector<string> fileVec;
    vector<string> fileNam;
    getFiles(pathOld, fileVec, fileNam);

    int i = 0;
    for (auto ve : fileVec) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        //igl::readOFF(ve, V, F);
        igl::readOBJ(ve, V, F);

        string name_model = fileNam[i++];
        name_model = name_model.substr(0, name_model.size() - 4);
        name_model = pathNew + name_model + ".off";
        igl::writeOFF(name_model, V, F);
    }
}

//文件夹操作
void OFF2OBJ(string pathOld, string pathNew) {
    vector<string> fileVec;
    vector<string> fileNam;
    //需要读取的文件夹路径，使用单右斜杠“/”  
    //string filePath = "F:/xinCode/Disintegration/obj/add";
    //string filePath = "./datas"; //相对路径也可以  
    getFiles(pathOld, fileVec, fileNam);

    int i = 0;
    for (auto ve : fileVec) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        //igl::readOFF(ve, V, F);
        igl::readOBJ(ve, V, F);

        string name_model = fileNam[i++];
        name_model = name_model.substr(0, name_model.size() - 4);
        WriteObjFromVF(name_model, V, F);

        //name_model = pathNew + name_model;// +".obj";

        //WriteObjFromVF(name_model, V, F);
        /*name_model += ".obj";
        igl::writeOBJ(name_model , V, F);*/
    }    
}


void getFiles(std::string path, std::vector<std::string>& files, std::vector<std::string>& filenames){
    intptr_t   hFile = 0;//intptr_t和uintptr_t的类型:typedef long int； typedef unsigned long int  
    struct _finddata_t fileinfo;
    std::string p;
    if ((hFile = _findfirst(p.assign(path).append("/*").c_str(), &fileinfo)) != -1)//assign方法：先将原字符串清空，然后赋予新的值作替换。  
    {
        do
        {
            if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
            {
                files.push_back(p.assign(path).append("/").append(fileinfo.name));
                filenames.push_back(fileinfo.name);
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
}

//Input one Mesh , Output many Mesh
void OnlyDiviedMesh(string filename, string pathOutput) {

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOFF(filename, V, F);

    vector<Eigen::RowVector3d> validVertex;
    vector<Eigen::RowVector3i> validFace;

    VectorToMatrix(validVertex, V);
    VectorToMatrix(validFace, F);

    DividedMesh(filename, validVertex, validFace, pathOutput);

}


void solvePro(const vector<string>& fileVec, string pathModel, string pathLable, string pathOutput) {

    for (auto v : fileVec) {

        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        map<int, vector<int>> Seg;

        string name_model = v;
        string filename = pathModel + name_model;// +".off";
        igl::readOFF(filename, V, F);

        name_model = name_model.substr(0, name_model.size() - 4);
        cout << name_model << endl;

        string segname = pathLable + name_model + ".seg";
        ifstream seg(segname);

        for (size_t t = 0; t < F.rows(); t++) {
            int x;
            seg >> x;
            Seg[x].push_back(t);
        }

        int all = 0;
        for (auto mp : Seg) {
            Eigen::MatrixXi f1;
            {
                Eigen::VectorXi R;
                VectorToVectorXX(mp.second, R);
                Eigen::VectorXi C(3); C << 0, 1, 2;
                igl::slice(F, R, C, f1);
            }

            vector<Eigen::RowVector3d> validVertex;
            vector<Eigen::RowVector3i> validFace;
            GetValidMeshAndInvalidVertex(V, f1, validVertex, validFace);

            //不划分网格
            //string meshname = string("mesh_") + name_model + "_" + to_string(mp.first);// + ".off";
            //WriteOFFtoVector(meshname, validVertex, validFace);

            //划分网格
            string name1 = name_model + +"_" + to_string(mp.first);//string("mesh_") + 
            DividedMesh(name1, validVertex, validFace, pathOutput);

        }
    }
}


