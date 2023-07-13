#include "simConvexDecompose.h"
#include <simLib/simLib.h>
#include <hacdHACD.h>
#include <hacdMicroAllocator.h>
#include "VHACD.h"
#include <iostream>
#include <string>

#define PLUGIN_VERSION 4

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
