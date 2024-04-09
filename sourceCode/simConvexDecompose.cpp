#include "simConvexDecompose.h"
#include <simLib/simLib.h>
#include <hacdHACD.h>
#include <hacdMicroAllocator.h>
#include "VHACD.h"
#include <iostream>
#include <string>
#include <simStack/stackArray.h>
#include <simStack/stackMap.h>

#define PLUGIN_VERSION 5

static LIBRARY simLib;

int computeHACD(const double* vertices,int verticesLength,const int* indices,int indicesLength,std::vector<std::vector<double>*>& verticesList,std::vector<std::vector<int>*>& indicesList,size_t nClusters,double concavity,bool addExtraDistPoints,bool addFacesPoints,double ccConnectDist,size_t targetNTrianglesDecimatedMesh,size_t maxHullVertices,double smallestClusterThreshold)
{
    int retVal=0;

    simAddLog("ConvexDecompose",sim_verbosity_infos,"computing the convex decomposition (HACD)...");
    std::vector< HACD::Vec3<HACD::Real> > points;
    std::vector< HACD::Vec3<long> > triangles;

    for (int i=0;i<verticesLength/3;i++)
    {
        HACD::Vec3<HACD::Real> v(vertices[3*i+0],vertices[3*i+1],vertices[3*i+2]);
        points.push_back(v);
    }
    for (int i=0;i<indicesLength/3;i++)
    {
        HACD::Vec3<long> t(indices[3*i+0],indices[3*i+1],indices[3*i+2]);
        triangles.push_back(t);
    }

    HACD::HeapManager * heapManager = HACD::createHeapManager(65536*(1000));

    HACD::HACD * const myHACD = HACD::CreateHACD(heapManager);
    myHACD->SetPoints(&points[0]);
    myHACD->SetNPoints(points.size());
    myHACD->SetTriangles(&triangles[0]);
    myHACD->SetNTriangles(triangles.size());
    myHACD->SetCompacityWeight(0.0001);
    myHACD->SetVolumeWeight(0.0);
    myHACD->SetConnectDist(ccConnectDist);               // if two connected components are seperated by distance < ccConnectDist
                                                        // then create a virtual edge between them so the can be merged during
                                                        // the simplification process

    myHACD->SetNClusters(nClusters);                     // minimum number of clusters
    myHACD->SetNVerticesPerCH(maxHullVertices);                      // max of 100 vertices per convex-hull
    myHACD->SetConcavity(concavity);                     // maximum concavity
    myHACD->SetSmallClusterThreshold(smallestClusterThreshold);              // threshold to detect small clusters
    myHACD->SetNTargetTrianglesDecimatedMesh(targetNTrianglesDecimatedMesh); // # triangles in the decimated mesh
//  myHACD->SetCallBack(&CallBack);
    myHACD->SetAddExtraDistPoints(addExtraDistPoints);
    myHACD->SetAddFacesPoints(addFacesPoints);

    {
        myHACD->Compute();
    }

    nClusters = myHACD->GetNClusters();

    std::string txt("done (");
    txt+=std::to_string(nClusters);
    txt+=" clusters generated).";
    simAddLog("ConvexDecompose",sim_verbosity_infos,txt.c_str());
    retVal=int(nClusters);

    for(size_t c = 0; c < nClusters; ++c)
    {
        size_t nPoints = myHACD->GetNPointsCH(c);
        size_t nTriangles = myHACD->GetNTrianglesCH(c);
        HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
        HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
        myHACD->GetCH(c, pointsCH, trianglesCH);
        std::vector<double>* _vert=new std::vector<double>;
        std::vector<int>* _ind=new std::vector<int>;
        for (int i=0;i<int(nPoints);i++)
        {
            _vert->push_back(pointsCH[i].X());
            _vert->push_back(pointsCH[i].Y());
            _vert->push_back(pointsCH[i].Z());
        }
        for (int i=0;i<int(nTriangles);i++)
        {
            _ind->push_back(trianglesCH[i].X());
            _ind->push_back(trianglesCH[i].Y());
            _ind->push_back(trianglesCH[i].Z());
        }
        verticesList.push_back(_vert);
        indicesList.push_back(_ind);

        delete [] pointsCH;
        delete [] trianglesCH;
    }

    HACD::DestroyHACD(myHACD);
    HACD::releaseHeapManager(heapManager);

    return(retVal);
}

void LUA_HACD_CALLBACK(SScriptCallBack* p)
{
    int stack=p->stackID;

    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    CStackArray* retHandles = new CStackArray();

    if ( (inArguments.getSize()>=1) && inArguments.isNumber(0) )
    {
        int shape = inArguments.getInt(0);
        CStackMap* map = nullptr;
        if ( (inArguments.getSize()>=2) && inArguments.isMap(1) )
            map = inArguments.getMap(1);

        int min_cluster_cnt = 1;
        if (map && map->isKeyPresent("min_cluster_cnt"))
            min_cluster_cnt = map->getInt("min_cluster_cnt");
        double max_concavity = 100.0;
        if (map && map->isKeyPresent("max_concavity"))
            max_concavity = map->getDouble("max_concavity");
        double max_connection_dist = 30.0;
        if (map && map->isKeyPresent("max_connection_dist"))
            max_connection_dist = map->getDouble("max_connection_dist");
        int triangle_cnt_decimated_mesh = 500;
        if (map && map->isKeyPresent("triangle_cnt_decimated_mesh"))
            triangle_cnt_decimated_mesh = map->getInt("triangle_cnt_decimated_mesh");
        int max_vertices_cnt = 200;
        if (map && map->isKeyPresent("max_vertices_cnt"))
            max_vertices_cnt = map->getInt("max_vertices_cnt");
        double small_cluster_detect_threshold = 0.25;
        if (map && map->isKeyPresent("small_cluster_detect_threshold"))
            small_cluster_detect_threshold = map->getDouble("small_cluster_detect_threshold");
        bool add_extra_pts = true;
        if (map && map->isKeyPresent("add_extra_pts"))
            add_extra_pts = map->getBool("add_extra_pts");
        bool add_extra_face_pts = true;
        if (map && map->isKeyPresent("add_extra_face_pts"))
            add_extra_face_pts = map->getBool("add_extra_face_pts");

        double* vert;
        int vertL;
        int* ind;
        int indL;
        int res = simGetShapeMesh(shape, &vert, &vertL, &ind, &indL, nullptr);
        if (res < 0)
            simSetLastError(nullptr, "Invalid shape handle.");
        else
        {
            double m[12];
            simGetObjectMatrix(shape, -1, m);
            for (int i = 0; i < vertL/3; i++)
                simTransformVector(m, vert + 3 * i);
            std::vector<std::vector<double>*> verticesList;
            std::vector<std::vector<int>*> indicesList;
            computeHACD(vert, vertL, ind, indL, verticesList, indicesList, min_cluster_cnt, max_concavity, add_extra_pts, add_extra_face_pts, max_connection_dist, triangle_cnt_decimated_mesh, max_vertices_cnt, small_cluster_detect_threshold);
            simReleaseBuffer(vert);
            simReleaseBuffer(ind);

            std::vector<int> newShapes;
            for (size_t i = 0; i < verticesList.size(); i++)
            {
                int h = simCreateShape(0, 0.0, verticesList[i]->data(), verticesList[i]->size(), indicesList[i]->data(), indicesList[i]->size(), nullptr, nullptr, nullptr, nullptr);
                newShapes.push_back(h);
                delete verticesList[i];
                delete indicesList[i];
            }
            retHandles->setIntArray(newShapes.data(), newShapes.size());
        }
    }
    else
        simSetLastError(nullptr,"Not enough arguments or wrong arguments.");

    // Return generated shape handles:
    CStackArray outArguments;
    outArguments.pushArray(retHandles);
    outArguments.buildOntoStack(stack);
}

int computeVHACD(const double* vertices,int verticesLength,const int* indices,int indicesLength,std::vector<std::vector<double>*>& verticesList,std::vector<std::vector<int>*>& indicesList,int resolution,int depth,double concavity,int planeDownsampling,int convexHullDownsampling,double alpha,double beta,double gamma,bool pca,bool voxelBased,int maxVerticesPerCH,double minVolumePerCH)
{
    simAddLog("ConvexDecompose",sim_verbosity_infos,"computing the convex decomposition (VHACD)...");
    VHACD::IVHACD::Parameters   params;
    params.m_resolution=uint32_t(resolution);
//    params.m_depth=depth;
    params.m_concavity=double(concavity);
    params.m_planeDownsampling=uint32_t(planeDownsampling);
    params.m_convexhullDownsampling=uint32_t(convexHullDownsampling);
    params.m_alpha=double(alpha);
    params.m_beta=double(beta);
//    params.m_gamma=gamma;
    params.m_pca=pca;
    params.m_mode=!voxelBased;
    params.m_maxNumVerticesPerCH=uint32_t(maxVerticesPerCH);
    params.m_minVolumePerCH=double(minVolumePerCH);
    VHACD::IVHACD* interfaceVHACD=VHACD::CreateVHACD();
//    interfaceVHACD->Compute(vertices,3,verticesLength/3,indices,3,indicesLength/3,params);
    uint32_t* tris=new uint32_t[size_t(indicesLength)];
    for (int i=0;i<indicesLength;i++)
        tris[i]=uint32_t(indices[i]);
    interfaceVHACD->Compute(vertices,uint32_t(verticesLength/3),tris,uint32_t(indicesLength/3),params);
    delete[] tris;

    unsigned int nConvexHulls=interfaceVHACD->GetNConvexHulls();
    
    std::string txt("done (");
    txt+=std::to_string(nConvexHulls);
    txt+=" clusters generated).";
    simAddLog("ConvexDecompose",sim_verbosity_infos,txt.c_str());

    VHACD::IVHACD::ConvexHull ch;
    for (unsigned int p=0;p<nConvexHulls;++p)
    {
        interfaceVHACD->GetConvexHull(p,ch);
        std::vector<double>* _vert=new std::vector<double>;
        std::vector<int>* _ind=new std::vector<int>;
        for (unsigned int v=0,idx=0;v<ch.m_nPoints;++v,idx+=3)
        {
            _vert->push_back(ch.m_points[idx+0]);
            _vert->push_back(ch.m_points[idx+1]);
            _vert->push_back(ch.m_points[idx+2]);
        }
        for (unsigned int t=0,idx=0;t<ch.m_nTriangles;++t,idx+=3)
        {
            _ind->push_back(int(ch.m_triangles[idx+0]));
            _ind->push_back(int(ch.m_triangles[idx+1]));
            _ind->push_back(int(ch.m_triangles[idx+2]));
        }
        verticesList.push_back(_vert);
        indicesList.push_back(_ind);
    }

    interfaceVHACD->Clean();
    interfaceVHACD->Release();
    return(int(nConvexHulls));
}

void LUA_VHACD_CALLBACK(SScriptCallBack* p)
{
    int stack=p->stackID;

    CStackArray inArguments;
    inArguments.buildFromStack(stack);

    CStackArray* retHandles = new CStackArray();

    if ( (inArguments.getSize()>=1) && inArguments.isNumber(0) )
    {
        int shape = inArguments.getInt(0);
        CStackMap* map = nullptr;
        if ( (inArguments.getSize()>=2) && inArguments.isMap(1) )
            map = inArguments.getMap(1);

        int resolution = 100000;
        if (map && map->isKeyPresent("res"))
            resolution = map->getInt("res");
        double concavity = 0.0025;
        if (map && map->isKeyPresent("concavity"))
            concavity = map->getDouble("concavity");
        int plane_downsampling = 4;
        if (map && map->isKeyPresent("plane_downsampling"))
            plane_downsampling = map->getInt("plane_downsampling");
        int hull_downsampling = 4;
        if (map && map->isKeyPresent("hull_downsampling"))
            hull_downsampling = map->getInt("hull_downsampling");
        double alpha = 0.05;
        if (map && map->isKeyPresent("alpha"))
            alpha = map->getDouble("alpha");
        double beta = 0.05;
        if (map && map->isKeyPresent("beta"))
            beta = map->getDouble("beta");
        int max_vertices = 64;
        if (map && map->isKeyPresent("max_vertices"))
            max_vertices = map->getInt("max_vertices");
        double min_volume = 0.0001;
        if (map && map->isKeyPresent("min_volume"))
            min_volume = map->getDouble("min_volume");
        bool pca = false;
        if (map && map->isKeyPresent("pca"))
            pca = map->getBool("pca");
        bool voxels = true;
        if (map && map->isKeyPresent("voxels"))
            voxels = map->getBool("voxels");

        double* vert;
        int vertL;
        int* ind;
        int indL;
        int res = simGetShapeMesh(shape, &vert, &vertL, &ind, &indL, nullptr);
        if (res < 0)
            simSetLastError(nullptr, "Invalid shape handle.");
        else
        {
            double m[12];
            simGetObjectMatrix(shape, -1, m);
            for (int i = 0; i < vertL/3; i++)
                simTransformVector(m, vert + 3 * i);
            std::vector<std::vector<double>*> verticesList;
            std::vector<std::vector<int>*> indicesList;
            computeVHACD(vert, vertL, ind, indL, verticesList, indicesList, resolution, 20, concavity, plane_downsampling, hull_downsampling, alpha, beta, 0.00125, pca, voxels, max_vertices, min_volume);

            simReleaseBuffer(vert);
            simReleaseBuffer(ind);

            std::vector<int> newShapes;
            for (size_t i = 0; i < verticesList.size(); i++)
            {
                int h = simCreateShape(0, 0.0, verticesList[i]->data(), verticesList[i]->size(), indicesList[i]->data(), indicesList[i]->size(), nullptr, nullptr, nullptr, nullptr);
                newShapes.push_back(h);
                delete verticesList[i];
                delete indicesList[i];
            }
            retHandles->setIntArray(newShapes.data(), newShapes.size());
        }
    }
    else
        simSetLastError(nullptr,"Not enough arguments or wrong arguments.");

    // Return generated shape handles:
    CStackArray outArguments;
    outArguments.pushArray(retHandles);
    outArguments.buildOntoStack(stack);
}

SIM_DLLEXPORT int simInit(SSimInit* info)
{
    simLib=loadSimLibrary(info->coppeliaSimLibPath);
    if (simLib==NULL)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
        return(0); 
    }
    if (getSimProcAddresses(simLib)==0)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0);
    }

    simRegisterScriptCallbackFunction("hacd", nullptr, LUA_HACD_CALLBACK);
    simRegisterScriptCallbackFunction("vhacd", nullptr, LUA_VHACD_CALLBACK);

    return(PLUGIN_VERSION);
}

SIM_DLLEXPORT void simCleanup()
{
    unloadSimLibrary(simLib);
}

SIM_DLLEXPORT void simMsg(SSimMsg*)
{
}

SIM_DLLEXPORT void simHACD(void* data)
{
    // Collect info from CoppeliaSim:
    void** valPtr=(void**)data;
    double* vertices=((double*)valPtr[0]);
    int verticesLength=((int*)valPtr[1])[0];
    int* indices=((int*)valPtr[2]);
    int indicesLength=((int*)valPtr[3])[0];
    size_t nClusters=((size_t*)valPtr[4])[0];
    double concavity=((double*)valPtr[5])[0];
    bool addExtraDistPoints=((bool*)valPtr[6])[0];
    bool addFacesPoints=((bool*)valPtr[7])[0];
    double ccConnectDist=((double*)valPtr[8])[0];
    size_t targetNTrianglesDecimatedMesh=((size_t*)valPtr[9])[0];
    size_t maxHullVertices=((size_t*)valPtr[10])[0];
    double smallestClusterThreshold=((double*)valPtr[11])[0];

    std::vector<std::vector<double>*> verticesList;
    std::vector<std::vector<int>*> indicesList;
    computeHACD(vertices,verticesLength,indices,indicesLength,verticesList,indicesList,nClusters,concavity,addExtraDistPoints,addFacesPoints,ccConnectDist,targetNTrianglesDecimatedMesh,maxHullVertices,smallestClusterThreshold);
    int el=int(verticesList.size());
    ((int*)valPtr[12])[0]=el;
    if (el>0)
    {
        double** vertList=(double**)simCreateBuffer(el*sizeof(double*));
        ((double***)valPtr[13])[0]=vertList;
        int** indList=(int**)simCreateBuffer(el*sizeof(int*));
        ((int***)valPtr[14])[0]=indList;
        int* vertCountList=(int*)simCreateBuffer(el*sizeof(int));
        ((int**)valPtr[15])[0]=vertCountList;
        int* indCountList=(int*)simCreateBuffer(el*sizeof(int));
        ((int**)valPtr[16])[0]=indCountList;

        for (int mesh=0;mesh<el;mesh++)
        {
            double* vert=(double*)simCreateBuffer(verticesList[mesh]->size()*sizeof(double));
            for (size_t i=0;i<verticesList[mesh]->size();i++)
                vert[i]=verticesList[mesh]->at(i);
            vertList[mesh]=vert;
            vertCountList[mesh]=verticesList[mesh]->size();
            int* ind=(int*)simCreateBuffer(indicesList[mesh]->size()*sizeof(int));
            for (size_t i=0;i<indicesList[mesh]->size();i++)
                ind[i]=indicesList[mesh]->at(i);
            indList[mesh]=ind;
            indCountList[mesh]=indicesList[mesh]->size();
        }
    }
}

SIM_DLLEXPORT void simVHACD(void* data)
{
    // Collect info from CoppeliaSim:
    void** valPtr=(void**)data;
    double* vertices=((double*)valPtr[0]);
    int verticesLength=((int*)valPtr[1])[0];
    int* indices=((int*)valPtr[2]);
    int indicesLength=((int*)valPtr[3])[0];

    int resolution=((int*)valPtr[4])[0];
    int depth=((int*)valPtr[5])[0];
    double concavity=((double*)valPtr[6])[0];
    int planeDownsampling=((int*)valPtr[7])[0];
    int convexHullDownsampling=((int*)valPtr[8])[0];
    double alpha=((double*)valPtr[9])[0];
    double beta=((double*)valPtr[10])[0];
    double gamma=((double*)valPtr[11])[0];
    bool pca=((bool*)valPtr[12])[0];
    bool voxelBased=((bool*)valPtr[13])[0];
    int maxVerticesPerCH=((int*)valPtr[14])[0];
    double minVolumePerCH=((double*)valPtr[15])[0];

    std::vector<std::vector<double>*> verticesList;
    std::vector<std::vector<int>*> indicesList;
    computeVHACD(vertices,verticesLength,indices,indicesLength,verticesList,indicesList,resolution,depth,concavity,planeDownsampling,convexHullDownsampling,alpha,beta,gamma,pca,voxelBased,maxVerticesPerCH,minVolumePerCH);
    int el=int(verticesList.size());
    ((int*)valPtr[16])[0]=el;
    if (el>0)
    {
        double** vertList=(double**)simCreateBuffer(el*sizeof(double*));
        ((double***)valPtr[17])[0]=vertList;
        int** indList=(int**)simCreateBuffer(el*sizeof(int*));
        ((int***)valPtr[18])[0]=indList;
        int* vertCountList=(int*)simCreateBuffer(el*sizeof(int));
        ((int**)valPtr[19])[0]=vertCountList;
        int* indCountList=(int*)simCreateBuffer(el*sizeof(int));
        ((int**)valPtr[20])[0]=indCountList;

        for (int mesh=0;mesh<el;mesh++)
        {
            double* vert=(double*)simCreateBuffer(verticesList[mesh]->size()*sizeof(double));
            for (size_t i=0;i<verticesList[mesh]->size();i++)
                vert[i]=verticesList[mesh]->at(i);
            vertList[mesh]=vert;
            vertCountList[mesh]=verticesList[mesh]->size();
            int* ind=(int*)simCreateBuffer(indicesList[mesh]->size()*sizeof(int));
            for (size_t i=0;i<indicesList[mesh]->size();i++)
                ind[i]=indicesList[mesh]->at(i);
            indList[mesh]=ind;
            indCountList[mesh]=indicesList[mesh]->size();
        }
    }
}
