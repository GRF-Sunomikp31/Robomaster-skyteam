#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "../grid/map.h"
#include "../utils/macro_params.h"

#define LASER_MAXBEAMS 2048

namespace GMapping {

class ScanMatcher{
public:
    ScanMatcher();
    ~ScanMatcher();

    void setLaserParameters(unsigned int beams, double* angles);
    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0 );

    double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    inline void likelihoodAndScore(double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    //分配内存、计算地图
    void computeMap(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    static const double nullLikelihood;
private:

protected:

    unsigned int  m_laserBeams;														//激光束的数量
    double        m_laserAngles[LASER_MAXBEAMS];								    //各个激光束的角度
    IntPoint*     m_linePoints;                                                     //存放临时击中点
    
    //定义一大堆参数以及其set和get函数
    PARAM_SET_GET(double,       laserMaxRange,              protected, public, public)	//激光的最大测距范围
    PARAM_SET_GET(double,       usableRange,                protected, public, public)	//使用的激光的最大范围
    PARAM_SET_GET(double,       gaussianSigma,              protected, public, public)
    PARAM_SET_GET(double,       likelihoodSigma,            protected, public, public)
    PARAM_SET_GET(int,          kernelSize,                 protected, public, public)
    PARAM_SET_GET(double,       optAngularDelta,            protected, public, public)	//优化时的角度增量
    PARAM_SET_GET(double,       optLinearDelta,             protected, public, public)	//优化时的长度增量
    PARAM_SET_GET(unsigned int, optRecursiveIterations,     protected, public, public)	//优化时的迭代次数
    PARAM_SET_GET(unsigned int, likelihoodSkip,             protected, public, public)
    PARAM_SET_GET(bool,         generateMap,                protected, public, public)
    PARAM_SET_GET(double,       enlargeStep,                protected, public, public)
    PARAM_SET_GET(double,       fullnessThreshold,          protected, public, public)	//被认为是占用的阈值
    PARAM_SET_GET(double,       angularOdometryReliability, protected, public, public)	//里程计的角度可靠性
    PARAM_SET_GET(double,       linearOdometryReliability,  protected, public, public)	//里程计的长度可靠性
    PARAM_SET_GET(double,       freeCellRatio,              protected, public, public)	//击中点旁边的空闲点的距离差
    
};

//输入当前地图和当前经过运动模型更新的激光雷达位姿p，以及激光雷达数据
//遍历若干激光束，求当前激光雷达位姿p与地图的匹配得分，越匹配，位姿得分越高
//匹配的方式就是找到，击中点和九宫格中的点的差距最小的点，差距越小，得分越高，越匹配，并且遍历若干束激光数据，计算累计得分
inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const
{
    double s=0;
    //获得当前帧激光数据的每束激光角度
    const double * angle=m_laserAngles;
    //lp 表示此刻激光雷达坐标系在地图坐标系下的坐标
    OrientedPoint lp=p;
    
    //如果激光束击中了某个点那么沿着激光方向的freeDelta距离的地方要是空闲才可以
    unsigned int skip=0;
    double freeDelta=map.getDelta()*m_freeCellRatio;//默认0.05×1.414的距离，表示斜着一个栅格的差距

    //枚举所有的激光束，有时为了提高计算速度，不需要对所有的激光数据进行计算
    for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
    {
        skip++;//m_likelihoodSkip默认为0,这里若设置1,每两束激光计算1束
        skip=skip>m_likelihoodSkip?0:skip;
        if (skip || *r>m_usableRange || *r==0.0) continue;

        //被激光雷达击中的点 在地图坐标系中的坐标phit
        Point phit=lp;
        phit.x+=*r*cos(lp.theta+*angle);
        phit.y+=*r*sin(lp.theta+*angle);
        //击中点在栅格地图中的栅格坐标iphit
        IntPoint iphit=map.world2map(phit);

        //假设phit是被激光击中的点，这样的话沿着激光方向的前面一个点必定是空闲的
        Point pfree=lp;
        pfree.x+=(*r - freeDelta)*cos(lp.theta+*angle);
        pfree.y+=(*r - freeDelta)*sin(lp.theta+*angle);

        //phit 和 pfree的栅格坐标的差距ipfree
        pfree=pfree-phit;
        IntPoint ipfree=map.world2map(pfree);

        //在kernelSize大小的窗口中搜索出最优最可能被这个激光束击中的点 kernelSize默认为1
        //这里形象化描述就是以击中点为中心的九宫格和相邻未击中点的九宫格
        bool found=false;
        Point bestMu(0.,0.);//击中点与九宫格中最小的位置差（具体体现两点距离公式）
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
            for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
            {
                IntPoint pr=iphit+IntPoint(xx,yy); 
                IntPoint pf=pr+ipfree;             

                //得到各自对应的Cell
                const PointAccumulator& cell=map.cell(pr);
                const PointAccumulator& fcell=map.cell(pf);
                
                //这束激光要合法必须要满足cell是被占用的，而fcell是空闲的
                //(double)cell 使用操作符重载，表示被占用概率
                if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold)
                {
                    //这里要理解一个事情，world2map会将物理坐标小于地图分辨率容差的物理坐标分到一个栅格
                    //所以才累计栅格被击中是的物理位置求平均，也就是cell.mean()，也就是历史栅格物理位姿的平均值
                    //通过mu表示击中点和九宫格中的点的差距
                    //我们寻找最小差距的那一个点
                    Point mu=phit-cell.mean();
                    if (!found)
                    {
                        bestMu=mu;
                        found=true;
                    }
                    else
                    {
                        //遍历九宫格中所有符合条件的，寻找两点差距最小的距离
                        bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                    }
                }
            }
        /*
            socre的计算公式exp(-d^2 / sigma)) 这里的sigma表示方差
            m_gaussianSigma默认0.05
        */
        if (found)
        {
            //exp单增函数，bestMu越小，tmp_score越大，该点与地图越匹配
            double tmp_score = exp(-1.0/m_gaussianSigma*bestMu*bestMu);//每一个找到bestMu的激光束计算得分
            s += tmp_score;                                            //对若干激光束操作的累计得分
        }
    }
    return s;
}

//和score函数几乎一致，相同的地方就不做注释了
inline void ScanMatcher::likelihoodAndScore(double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const
{
    using namespace std;
    l=0;
    const double * angle=m_laserAngles;
    OrientedPoint lp=p;

    //如果没有击中的时候的似然值 nullLikehood = -0.5
    double noHit=nullLikelihood/(m_likelihoodSigma);

    double freeDelta=map.getDelta()*m_freeCellRatio;

    for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
    {
        if (*r>m_usableRange || *r==0.0) continue;

        Point phit=lp;
        phit.x+=*r*cos(lp.theta+*angle);
        phit.y+=*r*sin(lp.theta+*angle);
        IntPoint iphit=map.world2map(phit);

        Point pfree=lp;
        pfree.x+=(*r - freeDelta)*cos(lp.theta+*angle);
        pfree.y+=(*r - freeDelta)*sin(lp.theta+*angle);
        pfree=pfree-phit;
        IntPoint ipfree=map.world2map(pfree);

        bool found=false;
        Point bestMu(0.,0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
        {
            for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
            {
                IntPoint pr=iphit+IntPoint(xx,yy);
                IntPoint pf=pr+ipfree;
                const PointAccumulator& cell=map.cell(pr);
                const PointAccumulator& fcell=map.cell(pf);

                if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold)
                {
                    Point mu=phit-cell.mean();
                    if (!found)
                    {
                        bestMu=mu;
                        found=true;
                    }
                    else
                    {
                        bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                    }
                }
            }
        }

        //似然不是指数 似然只是指数的上标，误差越小，似然越大，似然大小，代表权重大小
        double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);//参数设置m_likelihoodSigma=0.075
        l+=(found)?f:noHit;
    }
}

};

#endif
