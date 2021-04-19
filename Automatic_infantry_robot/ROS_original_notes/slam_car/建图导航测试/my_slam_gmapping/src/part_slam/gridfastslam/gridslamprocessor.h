#ifndef GRIDSLAMPROCESSOR_H
#define GRIDSLAMPROCESSOR_H

#include <string>
#include <list>
#include <map>
#include <set>
#include <iomanip>
#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <omp.h>

#include "../particlefilter/particlefilter.h"
#include "../utils/point.h"
#include "../sensor_range/rangereading.h"
#include "../scanmatcher/scanmatcher.h"
#include "../motionmodel/motionmodel.h"

namespace GMapping {

// 这个类实现了一个GridFastSLAM算法。实现了一个RBPF，每个粒子都拥有自己的地图和激光雷达位姿。
/*
	 工作流程如下：
	 每当收到里程计数据和激光雷达的数据之后，每个粒子的位姿根据运动模型来更新。
	 根据运动模型更新得到的新的位置随后被用来初始化scan-match算法。
	 scan-matcher为每个粒子执行了一个局部优化算法。
	 scan-matcher被用运动模型得到的位置来初始化，然后根据自己的地图来优化位置。
*/
class GridSlamProcessor
{
public:
  /*
    树的节点，一个树储存了一整条轨迹，一个节点表示这条轨迹中的其中一个点。存储激光雷达的整条轨迹
    一个节点（TNode）包含了：
      该节点粒子的激光雷达位姿  pose
      指向父节点的指针       parent
      该节点激光雷达的读数   reading
  */
  struct TNode
  {
    TNode(const OrientedPoint& pose, TNode* parent=0);
    ~TNode();

    OrientedPoint pose; 
    const RangeReading* reading;
    TNode* parent;
  };
  
  //用来定义一个节点数组，存储多条轨迹
  typedef std::vector<GridSlamProcessor::TNode*> TNodeVector;

  /*
    粒子滤波器中的粒子结构体,
    每个粒子有自己的地图、位姿、权重、轨迹
    轨迹是按照时间顺序排列的，叶子节点表示最近的节点
  */

  struct Particle
  {
    //构造函数，初始化地图以及其他成员变量
    Particle(const ScanMatcherMap& map);
    //重载括号运算符
    inline operator double() const {return weight;}
    inline operator OrientedPoint() const {return pose;}
    //设置粒子权重
    inline void setWeight(double w) {weight = w;}
    //记录粒子当前地图
    ScanMatcherMap map;
    //记录粒子当前时刻激光雷达位姿
    OrientedPoint pose;
    //该粒子的当前的权重
    double weight;
    //该粒子的累计权重
    double weightSum;
    //该粒子的节点，指向父节点
    TNode* node; 
  };

  //重命名粒子元素的vector动态数组
  typedef std::vector<Particle> ParticleVector;

  //构造函数：初始化一些参数
  GridSlamProcessor();
  //销毁每一个粒子的轨迹树
  virtual ~GridSlamProcessor();
  //设置扫描匹配的参数
  void setMatchingParameters(double urange, double range, double sigma, int kernsize, 
                             double lopt, double aopt, int iterations, double likelihoodSigma=1, 
                             double likelihoodGain=1, unsigned int likelihoodSkip=0);
  //设置运动模型的参数
  void setMotionModelParameters(double srr, double srt, double str, double stt);
  //设置更新距离的参数
  void setUpdateDistances(double linear, double angular, double resampleThreshold);
  //设置更新频率
  void setUpdatePeriod(double p) {period_=p;}

  /*
    设置地图尺寸，分辨率，初始位姿,清空每个粒子
    new一个轨迹树的根节点，初始化，初始位姿
    定义一个地图，为size个粒子，初始化，地图、位姿、权重、轨迹
  */
  void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, 
            OrientedPoint initialPose=OrientedPoint(0,0,0));
  //SLAM处理程序
  bool processScan(const RangeReading & reading, int adaptParticles=0);
  //获得粒子的数量
  inline const ParticleVector& getParticles() const {return m_particles; }
  //获得最优位姿粒子的序号
  int getBestParticleIndex() const;
  //扫描匹配对象
  ScanMatcher m_matcher;

  //定义大量的成员的set和get函数，成员本身有get和set函数
  //这些成员在自己的类中，已经有了自己的get和set函数
  MEMBER_PARAM_SET_GET(m_matcher, double,       laserMaxRange,          protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, double,       usableRange,            protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, double,       gaussianSigma,          protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, double,       likelihoodSigma,        protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, int,          kernelSize,             protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, double,       optAngularDelta,        protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, double,       optLinearDelta,         protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, unsigned int, optRecursiveIterations, protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, unsigned int, likelihoodSkip,         protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, bool,         generateMap,            protected, public, public);
  MEMBER_PARAM_SET_GET(m_matcher, bool,         enlargeStep,            protected, public, public);

  //定义大量的成员的set和get函数，成员本身没有有get和set函数，直接读取
  STRUCT_PARAM_SET_GET(m_motionModel, double, srr, protected, public, public);
  STRUCT_PARAM_SET_GET(m_motionModel, double, srt, protected, public, public);
  STRUCT_PARAM_SET_GET(m_motionModel, double, str, protected, public, public);
  STRUCT_PARAM_SET_GET(m_motionModel, double, stt, protected, public, public);

  //认为匹配成功的最小得分阈值
  PARAM_SET_GET(double, minimumScore,              protected, public, public);

protected:
  //记录上一次更新时间
  double                    last_update_time_;
  //两次更新时间的最小间隔
  double                    period_;
  //激光束的数量
  unsigned int              m_beams;
  //粒子数组
  ParticleVector            m_particles;
  //重采样之后，剩余的粒子的下标，这个下标的是会重复的
  std::vector<unsigned int> m_indexes;
  //所有粒子的当前的权重
  std::vector<double>       m_weights;
  //粒子运动模型
  MotionModel               m_motionModel;
  //上一次的位姿  
  OrientedPoint             m_odoPose;
  //激光雷达的位姿
  OrientedPoint             m_pose;
  //被处理过的激光雷达帧数
  int                       m_count;
  //记录两帧之间激光雷达直线位移
  double                    m_linearDistance;
  //记录两帧之间激光雷达角度位移
  double                    m_angularDistance;
  //定义一大堆变量以及其set和get函数
  //地图尺寸，精度
  PARAM_GET(double, xmin, protected, public);
  PARAM_GET(double, ymin, protected, public);
  PARAM_GET(double, xmax, protected, public);
  PARAM_GET(double, ymax, protected, public);
  PARAM_GET(double, delta, protected, public);
  //记录粒子的离散程度，论文中有计算公式
  PARAM_GET(double, neff, protected, public);
  //两帧之间激光雷达直线、角度位移阈值
  PARAM_SET_GET(double, linearThresholdDistance,  protected, public, public);
  PARAM_SET_GET(double, angularThresholdDistance, protected, public, public);
  //
  PARAM_SET_GET(double, obsSigmaGain,             protected, public, public);
  //粒子选择性重采样的阈值
  PARAM_SET_GET(double, resampleThreshold,        protected, public, public);

private:
  /*
    在当前位姿、激光雷达的数据下，对每个粒子与当前的地图进行扫描匹配
    通过爬山函数，每一层向前后左右、左转、右转，6个方向，为每个粒子寻找最优位姿，直到最优位姿分数开始下降，或者超过迭代次数，停止爬山
    关于位姿得分，由score函数计算，在击中栅格周围的9个栅格，寻找最优得分
  */
  inline void scanMatch(const double *plainReading);
  
  //更新每个粒子的权重，计算重采样neff
  inline void normalize();

  //粒子重采样  根据neff的大小来进行重采样  不但进行了重采样，也对地图进行更新
  inline bool resample(const double* plainReading, int adaptParticles, const RangeReading* rr=0);

};

inline void GridSlamProcessor::scanMatch(const double* plainReading)
{
  //每个粒子都要进行scan-match
  int particle_number = m_particles.size();
  for (int i = 0; i < particle_number;i++)
  {
    OrientedPoint corrected;
    double score, l;
    //爬山算法，score最优位姿的最大的匹配得分
    score=m_matcher.optimize(corrected, m_particles[i].map, m_particles[i].pose, plainReading);

    //更新该粒子计算出的激光雷达最优位姿（地图坐标系）
    if (score > m_minimumScore)
    {
      m_particles[i].pose = corrected;
    }
    //输入当前地图和当前最优位姿，遍历激光束累计似然，把累计似然当作该粒子的权重，误差越小，似然越大
    //似然大小，代表权重大小
    m_matcher.likelihoodAndScore(l, m_particles[i].map, m_particles[i].pose, plainReading);
    //为每个粒子更新权重和累计权重
    m_particles[i].weight+=l;    //该粒子的权重
    m_particles[i].weightSum+=l; //该粒子的权重累计，寻找最优粒子使用

  }
}

//归一化粒子权重：找到所有粒子中最大的权重，更新每个粒子的权重、累积所有粒子的权重之和，归一化每个粒子的权重、更新每个粒子权重，计算neff：粒子离散程度。
inline void GridSlamProcessor::normalize()
{
  /*第一次计算
    it->weight = 0;              （粒子的权重，通过累计权重寻找最优粒子的中间变量）
    m_weights通过公式计算都为1      （为了计算粒子离散程度，而产生的中间变量）
    归一化之后，m_weights得到均分概率 
    通过公式计算m_neff             （m_neff用来描述粒子的离散程度）
  */
  double gain=1./(m_obsSigmaGain*m_particles.size());//m_obsSigmaGain默认为3
  
  /*求所有粒子中的最大的权重*/
  double lmax= -std::numeric_limits<double>::max();//返回编译器允许的 double 型数 最大值
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
  {
    lmax=it->weight>lmax?it->weight:lmax;
  }
  
  /*权重以最大权重为中心的高斯分布*/
  //粒子的权重，粒子的离散程度
  //权重更新、累积所有粒子的权重

  m_weights.clear();
  double wcum=0;
  for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
  {
    m_weights.push_back(exp(gain*(it->weight - lmax)));
    wcum+=m_weights.back();
  }
  
  /*
  计算有效粒子数 和 归一化权重
  权重=wi/w
  neff = 1/w*w
  */
  //归一化权重，除以所有粒子权重之和

  m_neff=0;
  for (std::vector<double>::iterator it=m_weights.begin(); it!=m_weights.end(); it++)
  {
    *it=*it/wcum;//对每一个粒子的权重进行归一化处理
    double w=*it;
    m_neff+=w*w;
  }
  m_neff=1./m_neff;
  
}
 
inline bool GridSlamProcessor::resample(const double* plainReading, int adaptSize, const RangeReading* reading)
{
  //是否进行了重采样的标志
  bool hasResampled = false;
  
  /*备份老的粒子的轨迹*/
  TNodeVector oldGeneration;
  for (unsigned int i=0; i<m_particles.size(); i++)
  {
    oldGeneration.push_back(m_particles[i].node);
  }
  
  /*如果需要进行重采样*/
  if (m_neff < m_resampleThreshold*m_particles.size())
  {	
    //采取重采样方法决定，哪些粒子会保留  保留的粒子会返回下标.里面的下标可能会重复，因为有些粒子会重复采样
    uniform_resampler<double, double> resampler;
    //重采样之后新粒子的重采样之前的序号
    m_indexes=resampler.resampleIndexes(m_weights, adaptSize);
    
    //临时存储重采样之后的粒子
    ParticleVector temp;

    //枚举每一个要被保留的粒子，m_indexes[i] 会重复
    for (unsigned int i=0; i<m_indexes.size(); i++)
    {
      //得到当前的保留的粒子
      Particle & p=m_particles[m_indexes[i]];

      //每一个需要保留下来的粒子都需要在路径中增加一个新的节点
      TNode* node=0;
      TNode* oldNode=oldGeneration[m_indexes[i]];

      //创建一个新的节点 改节点的父节点为oldNode
      node=new TNode(p.pose, oldNode);
      node->reading=reading;

      //这个要保留下来的粒子
      temp.push_back(p);
      temp.back().node=node;
    }

    //清除之前全部的粒子 然后从temp中读取重采样的粒子
    m_particles.clear();
  
    //对于保留下来的粒子进行更新
    int tmp_size = temp.size();
    for(int i = 0; i<tmp_size;i++)
    {
      //每个粒子的权重都设置为相同的值
      temp[i].setWeight(0);

      //拓展地图大小、找到地图的有效区域，单位patch,申请内存、更新每个栅格的内容
      m_matcher.computeMap(temp[i].map,temp[i].pose,plainReading);

      //从temp中读取重采样的粒子
      m_particles.push_back(temp[i]);
    }

    hasResampled = true;
  } 
  /*否则的话*/
  else 
  {
    //不进行重采样的话，粒子的权重不变。只为轨迹创建一个新的节点
    int particle_size = m_particles.size();
    //为每个粒子更新地图
    for(int i = 0; i < particle_size;i++)
    {
      //创建一个新的树节点
      TNode* node = 0;
      node = new TNode(m_particles[i].pose,oldGeneration[i]);

      //把这个节点接入到树中
      node->reading = reading;
      m_particles[i].node = node;//每一个粒子都指向一个最新的节点

      //拓展地图大小、找到地图的有效区域，单位patch,申请内存、更新每个栅格的内容
      m_matcher.computeMap(m_particles[i].map, m_particles[i].pose, plainReading);
    }

  }
  return hasResampled;
}
  
};
#endif
