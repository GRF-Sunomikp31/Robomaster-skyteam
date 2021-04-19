#include "scanmatcher.h"

#include <cstring>
#include <limits>
#include <list>
#include <vector>
#include <iostream>
#include <omp.h>
#include "ros/ros.h"
#include "tf/LinearMath/Scalar.h"
#include "gridlinetraversal.h"

namespace GMapping {

using namespace std;

const double ScanMatcher::nullLikelihood=-.5;

//构造函数
ScanMatcher::ScanMatcher()
{
	m_laserBeams=0;
	//爬山算法减少搜索步长的次数
    m_optRecursiveIterations=9;
    //地图进行拓展的大小
	m_enlargeStep=10.;
	//判断栅格是否被占据的概率阈值
	m_fullnessThreshold=0.1;

    //指示里程计和陀螺仪是否可靠
    //如果可靠的话，那么进行score计算的时候，就需要对离里程计数据比较远的位姿增加惩罚
    //对于我们的应用来说 陀螺仪在短期内还是很可靠的。
	m_angularOdometryReliability=0.3;
    m_linearOdometryReliability=0.3;

	//用来表示和击中栅格前面的栅格
    m_freeCellRatio=sqrt(2.);//1.414

    m_linePoints = new IntPoint[20000];
}

//析构函数
ScanMatcher::~ScanMatcher()
{
	delete [] m_linePoints;
}

//设置laser的参数
void ScanMatcher::setLaserParameters(unsigned int beams, double* angles)
{
    assert(beams<LASER_MAXBEAMS);
    m_laserBeams=beams;
    //把angles数据拷贝到m_laserAngles中
    memcpy(m_laserAngles, angles, sizeof(double)*m_laserBeams);
}

//设置match的参数
void ScanMatcher::setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations,  double likelihoodSigma, unsigned int likelihoodSkip)
{
	m_usableRange=urange;						//传感器的使用范围
	m_laserMaxRange=range;						//传感器的最大范围
	m_kernelSize=kernsize;						//kernsize主要用在计算score时搜索框的大小
	m_optLinearDelta=lopt;						//优化时的线性步长
	m_optAngularDelta=aopt;						//优化时的角度步长，搜索改变步长
	m_optRecursiveIterations=iterations;		//优化时的迭代次数，搜索步长改变的次数
	m_gaussianSigma=sigma;						//计算socre时的方差，计算匹配得分
	m_likelihoodSigma=likelihoodSigma;			//计算似然时的方差，计算似然得分，当作粒子的权重	
	m_likelihoodSkip=likelihoodSkip;			//计算似然时，跳过的激光束，默认为0
}

//每个粒子调用爬山算法函数，optimize函数对位姿进行适当修改，分为四个方向和两个转向，就是为了得到一个得分更好的位姿，直到得分开始下降才认为，此时的位姿点为，最优位姿点
double ScanMatcher::optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const
{
    double bestScore=-1;
    //计算当前位置的得分
    OrientedPoint currentPose=init;
	//score函数求当前激光雷达位姿p，在当前地图的匹配得分，越匹配，位姿得分越高，说明位姿越准确
    double currentScore=score(map, currentPose, readings);
	
    //角度和线性位移步进增量大小
    double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
	
    //减小搜索步长的次数
    unsigned int refinement=0;
	
    //搜索的方向
    enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
	//就这样在运动模型的更新的激光雷达位姿基础之上，不断的超不同的方向，一层一层的向上爬，同时进行位姿修改，直到位姿得分出现下降，或者迭代结束
    do
    {
        //如果某一次(currentScore)算出来比上一次(bestScore)差，则有可能是走太多了，要减少搜索步长 这个策略跟LM有点像
        if (bestScore>=currentScore)
        {
            refinement++;//减小搜索步长的次数++
            adelta*=.5;
            ldelta*=.5;
        }
		//初始化bestScore，前面在运动模型的更新的激光雷达位姿基础上计算的得分
        bestScore=currentScore;
        OrientedPoint bestLocalPose=currentPose;
        OrientedPoint localPose=currentPose;

        //把6个方向都搜索一次
        Move move=Front;
        do
        {
			//注意currentPose在这一层循环中并没有改变
            localPose=currentPose;
            switch(move)
            {
                case Front:
                    localPose.x+=ldelta;
                    move=Back;
                    break;
                case Back:
                    localPose.x-=ldelta;
                    move=Left;
                    break;
                case Left:
                    localPose.y-=ldelta;
                    move=Right;
                    break;
                case Right:
                    localPose.y+=ldelta;
                    move=TurnLeft;
                    break;
                case TurnLeft:
                    localPose.theta+=adelta;
                    move=TurnRight;
                    break;
                case TurnRight:
                    localPose.theta-=adelta;
                    move=Done;
                    break;
                default:;
            }
			
            //表示当前的位姿和初始位姿的区别 区别越大增益越小
            double odo_gain=1;

            //如果里程计比较可靠的话
            //那么进行匹配的时候就需要对离初始位姿比较远的位姿施加惩罚
			//差距越大，odo_gain越小，小于1也就产生了惩罚
            if (m_angularOdometryReliability>0.)
            {
                double dth=init.theta-localPose.theta; 	
				dth=atan2(sin(dth), cos(dth)); 	
				dth*=dth;
                odo_gain*=exp(-m_angularOdometryReliability*dth);
            }
            //同样差距越大，odo_gain越小
            if (m_linearOdometryReliability>0.)
            {
                double dx=init.x-localPose.x;
                double dy=init.y-localPose.y;
                double drho=dx*dx+dy*dy;
                odo_gain*=exp(-m_linearOdometryReliability*drho);
            }
            /*计算得分=增益*score*/
            double localScore=odo_gain*score(map, localPose, readings);
			
            /*如果得分更好，则更新得分，和位姿*/
            if (localScore>currentScore)
            {
                currentScore=localScore;
                bestLocalPose=localPose;
            }
        } while(move!=Done);//结束条件：走完6步
		
        /* 把当前位置设置为目前最优的位置*/
        currentPose=bestLocalPose;
    }while (currentScore>bestScore || refinement<m_optRecursiveIterations);//两者有一个满足条件，都不会结束循环
	
    /*返回最优位置和得分*/
    pnew=currentPose;
    return bestScore;
}

//拓展地图大小、找到地图的有效区域，单位patch,申请内存、更新每个栅格的内容
void ScanMatcher::computeMap(ScanMatcherMap& map, const OrientedPoint& p, const double* readings)
{
	//lp为地图坐标系下的激光雷达坐标系的位姿
	OrientedPoint lp=p;

    //此时的p0为栅格地图上，该激光雷达坐标系的位置
	IntPoint p0=map.world2map(lp);

	//本函数中多次使用该变量的值，故通过此法，减少正弦和余弦计算量
	std::vector<double> cos_;
	std::vector<double> sin_;

	//之前设置的地图的物理坐标的最小、最大尺寸
	Point min(map.map2world(0,0));
	Point max(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));

	/*根据激光数据扩展地图的范围到激光的击中点*/
	const double * angle = m_laserAngles;
    //遍历所有激光点找到地图需要扩张的大小,同时计算cos_、sin_
	for (const double* r = readings; r<readings+m_laserBeams; r++,angle++)
	{
		if (*r>m_laserMaxRange||*r==0.0||isnan(*r))
		{
			cos_.push_back(0.0);
			sin_.push_back(0.0);
			continue;
		}
		double d=*r>m_usableRange?m_usableRange:*r;

		/*被激光击中的位置*/
		//此时的phit为激光雷达在地图坐标系下的物理位置
		Point phit=lp;
		cos_.push_back(cos(lp.theta+*angle));
		sin_.push_back(sin(lp.theta+*angle));
        //得到phit，击中点在地图坐标系下的物理位置
		phit.x+=d*cos_.back();
		phit.y+=d*sin_.back();

		/*更新扩充范围*/
		if (phit.x<min.x) min.x=phit.x;
		if (phit.y<min.y) min.y=phit.y;
		if (phit.x>max.x) max.x=phit.x;
		if (phit.y>max.y) max.y=phit.y;
	}
	
    //判断将要修改的地图的物理最小值和最大值是否还在原来的地图中
	//如果不在，当然要进行地图扩展，此次扩展在击中点之外m_enlargeStep米，默认10米
	if ( !map.isInside(min)	|| !map.isInside(max))
	{
		/*得到目前地图的大小*/
		Point lmin(map.map2world(0,0));
		Point lmax(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));

		/*在激光雷达击中的基础之上，进行左右拓展m_enlargeStep，默认10m*/
		min.x=( min.x >= lmin.x )? lmin.x: min.x-m_enlargeStep;
		max.x=( max.x <= lmax.x )? lmax.x: max.x+m_enlargeStep;
		min.y=( min.y >= lmin.y )? lmin.y: min.y-m_enlargeStep;
		max.y=( max.y <= lmax.y )? lmax.y: max.y+m_enlargeStep;
		map.resize(min.x, min.y, max.x, max.y);
		//此时地图已经扩展到需要的大小，只操作了patch 
	}
	
	/*地图的有效区域(地图坐标系)*/
	HierarchicalArray2D<PointAccumulator>::PointSet activeArea;

    //通过激光雷达的数据，找出地图的有效区域
	size_t i = 0;
	for (const double* r=readings; r<readings+m_laserBeams; r++, i++)
	{
		/*如果需要生成地图*/
		if (m_generateMap)
		{
            //排除错误的激光点
			double d=*r;
			if (d>m_laserMaxRange||d==0.0||isnan(d)) continue;
			if (d>m_usableRange) d=m_usableRange;
			
            //p1击中栅格的栅格坐标
			Point phit=lp;
			phit.x+=d*cos_[i];
			phit.y+=d*sin_[i];
			IntPoint p1=map.world2map(phit);
			
            /*bresenham算法来计算激光起点到终点要经过的栅格*/
			GridLineTraversalLine line;
            //m_linePoints指针指向line.points指针，此时两个指针指向一致
			line.points=m_linePoints;
			GridLineTraversal::gridLine(p0, p1, &line);
			
            /*更新地图 把画线算法计算出来的值都算进去*/
			for (int i=0; i<line.num_points-1; i++)
			{
				assert(map.isInside(m_linePoints[i]));
				activeArea.insert(map.storage().patchIndexes(m_linePoints[i]));//通过这种方式，找到有效patch
			}
            //如果d<m_usableRange则需要把击中点也算进去 说明这个值是好的。
            //同时如果d==m_usableRange　那么说明这个值只用来进行标记空闲区域　不用来进行标记障碍物
			if (d<m_usableRange)
			{
				IntPoint cp=map.storage().patchIndexes(p1);
				activeArea.insert(cp);
			}
		} 
		else 
		{
			if (*r>m_laserMaxRange||*r>m_usableRange||*r==0.0||isnan(*r)) continue;
            //得到击中栅格的物理坐标phit
			Point phit=lp;
			phit.x+=*r*cos_[i];
			phit.y+=*r*sin_[i];
            //得到击中栅格的栅格坐标p1
			IntPoint p1=map.world2map(phit);
            //得到击中栅格p1坐落在patch（栅格补丁）的栅格坐标cp
			IntPoint cp=map.storage().patchIndexes(p1);
            //将patch的栅格坐标cp插入activeArea
			activeArea.insert(cp);
		}
	}

	map.storage().setActiveArea(activeArea, true);
	//============================================================================================
    //为activeArea里面的没有分配内存的区域分配内存
	map.storage().allocActiveArea();
	//更新栅格，经过的，击中的栅格
	i=0;
	for (const double* r=readings; r<readings+m_laserBeams; r++, i++)
	{
		if (m_generateMap)
		{
			/*去除非法的激光束*/
			double d=*r;
			if (d>m_laserMaxRange||d==0.0||isnan(d))
				continue;
			if (d>m_usableRange) d=m_usableRange;
			
			/*被该激光束击中的点的栅格坐标p1*/
			Point phit=lp;
			phit.x+=d*cos_[i];
			phit.y+=d*sin_[i];
			IntPoint p1=map.world2map(phit);
			
			/*bresenham画线算法来计算 激光位置和被激光击中的位置之间的空闲位置*/
			GridLineTraversalLine line;
			line.points=m_linePoints;
			GridLineTraversal::gridLine(p0, p1, &line);
			
			/*更新空闲位置*/
			for (int i=0; i<line.num_points-1; i++)
			{   //未击中，就不记录击中的位置了，所以传入参数Point(0,0)
				map.cell(line.points[i]).update(false, Point(0,0));
			}
			
            /*更新被击中的位置　只有小于m_usableRange的栅格来用来标记障碍物*/
			if (d<m_usableRange)
			{
				//击中，记录击中的位置，所以传入参数phit，击中点的物理位置
				map.cell(p1).update(true, phit);
			}
		} 
		else 
		{
			//排除异常的数据
			if (*r>m_laserMaxRange||*r>m_usableRange||*r==0.0||isnan(*r)) continue;
			/*被击中的点的物理坐标phit*/
			Point phit=lp;
			phit.x+=*r*cos_[i];
			phit.y+=*r*sin_[i];
            //击中栅格的栅格坐标p1
			IntPoint p1=map.world2map(phit);
			/*更新对应的cell的值*/
			map.cell(p1).update(true,phit);
		}
	}
}

};
