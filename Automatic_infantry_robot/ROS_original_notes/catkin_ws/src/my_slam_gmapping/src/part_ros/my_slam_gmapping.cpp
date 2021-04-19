#include "my_slam_gmapping.h"
#include <iostream>
#include <time.h>
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/String.h"

/***********************************************************************************************************
  基于滤波器的SLAM算法-《gmapping算法的删减版》+《加入激光雷达运动畸变去除》
  在原来gmapping源码的基础之上，公众号：小白学移动机器人的作者对其进行了大刀阔斧的更改。
  （1）删除几乎所有不需要的代码，对代码的运行结构也进行了调整
  （2）对该代码进行详细中文注释，以及对核心代码进行更改
  （3）将激光雷达运动畸变去除算法，直接加入删减版的gmapping算法中，算法文件在part_data文件夹
  =============公众号：小白学移动机器人========================================================================
  欢迎关注公众号，从此学习的路上变得不再孤单，加油！奥利给！！！
  修改时间：2020年11月29日
***********************************************************************************************************/

//构造函数-初始化相关变量，比如指针的初始化
MySlamGMapping::MySlamGMapping():
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),//默认两个坐标系重合
    private_nh_("~"), 
    scan_filter_sub_(NULL), 
    scan_filter_(NULL), 
    transform_thread_(NULL)
{
    seed_ = time(NULL);
    init();
}

//析构函数
MySlamGMapping::~MySlamGMapping()
{
    if(transform_thread_)
    {
        transform_thread_->join();
        delete transform_thread_;
    }

    delete gsp_;
    delete lmc_;

    if (scan_filter_)
        delete scan_filter_;
    if (scan_filter_sub_)
        delete scan_filter_sub_;
}

//slamgmapping的初始化，主要用来读取配置文件中写入的参数以及初始化一些对象
void MySlamGMapping::init()
{
    if(!private_nh_.getParam("map_frame", map_frame_))
        map_frame_ = "map";
    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!private_nh_.getParam("scan_topic", scan_topic_))
        scan_topic_ = "scan";
    if(!private_nh_.getParam("laser_frame", laser_frame_))
        laser_frame_ = "laser_link";
    
    //new一个激光雷达运动畸变的对象
    lmc_ = new LidarMotionCalibrator(laser_frame_,odom_frame_);
    //new一个GridSlamProcessor对象，也是ros和gridslam的连接
    gsp_ = new GMapping::GridSlamProcessor();              //这里需要跳进去看，第一次不要看
    //new一个TransformBroadcaster对象，用来发布map和odom的关系
    tfB_ = new tf::TransformBroadcaster();

    got_first_scan_ = false;
    got_map_ = false;

    private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

    double tmp;
    if(!private_nh_.getParam("map_update_interval", tmp))//地图更新的秒数间隔
        tmp = 5.0;
    map_update_interval_.fromSec(tmp);

    //GMapping算法本身使用的参数
    maxUrange_ = 0.0;  maxRange_ = 0.0; 
    if(!private_nh_.getParam("particles", particles_))
        particles_ = 30;
    if(!private_nh_.getParam("xmin", xmin_))
        xmin_ = -10.0;
    if(!private_nh_.getParam("ymin", ymin_))
        ymin_ = -10.0;
    if(!private_nh_.getParam("xmax", xmax_))
        xmax_ = 10.0;
    if(!private_nh_.getParam("ymax", ymax_))
        ymax_ = 10.0;
    if(!private_nh_.getParam("delta", delta_))
        delta_ = 0.05;
    if(!private_nh_.getParam("occ_thresh", occ_thresh_))
        occ_thresh_ = 0.25;

    if(!private_nh_.getParam("minimumScore", minimum_score_))
        minimum_score_ = 0;
    if(!private_nh_.getParam("sigma", sigma_))
        sigma_ = 0.05;
    if(!private_nh_.getParam("kernelSize", kernelSize_))
        kernelSize_ = 1;
    if(!private_nh_.getParam("lstep", lstep_))//默认一个栅格距离大小变化
        lstep_ = delta_;                          
    if(!private_nh_.getParam("astep", astep_))
        astep_ = delta_;
    if(!private_nh_.getParam("iterations", iterations_))
        iterations_ = 5;
    if(!private_nh_.getParam("lsigma", lsigma_))
        lsigma_ = 0.075;
    if(!private_nh_.getParam("ogain", ogain_))
        ogain_ = 3.0;
    if(!private_nh_.getParam("lskip", lskip_))//计算scan与地图匹配得分时，跳过的部分，默认为0
        lskip_ = 0;
    if(!private_nh_.getParam("srr", srr_))
        srr_ = 0.1;
    if(!private_nh_.getParam("srt", srt_))
        srt_ = 0.2;
    if(!private_nh_.getParam("str", str_))
        str_ = 0.1;
    if(!private_nh_.getParam("stt", stt_))
        stt_ = 0.2;
    if(!private_nh_.getParam("linearUpdate", linearUpdate_))
        linearUpdate_ = 1.0;
    if(!private_nh_.getParam("angularUpdate", angularUpdate_))
        angularUpdate_ = 0.5;
    if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
        temporalUpdate_ = -1.0;
    if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
        resampleThreshold_ = 0.5;

    if(!private_nh_.getParam("tf_delay", tf_delay_))
        tf_delay_ = transform_publish_period_;
        
    ROS_DEBUG("MySlamGMapping::init finish");
}

//开始实时SLAM
void MySlamGMapping::startLiveSlam()
{
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    ss_ = node_.advertiseService("dynamic_map", &MySlamGMapping::mapCallback, this);

    {
        //用message_filters来订阅scan_topic_，进而初始化scan_filter_，
        scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, scan_topic_, 5);
        //tf::MessageFilter，订阅激光数据同时和odom_frame之间转换时间同步
        scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
        //scan_filter_注册回调函数laserCallback
        scan_filter_->registerCallback(boost::bind(&MySlamGMapping::laserCallback, this, _1));

        ROS_DEBUG("Start Subscribe LaserScan & odom!!!");
    }

    /*发布map到odom的转换关系的线程*/
    transform_thread_ = new boost::thread(boost::bind(&MySlamGMapping::publishLoop, this, transform_publish_period_));

    ROS_DEBUG("Start transform_thread ");
    // int aaaaa=40>>5;
    // std::cout<<aaaaa<<std::endl;
}

//每当到达一帧scan数据，就将调用laserCallback函数
void MySlamGMapping::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //===========================激光雷达数据运动畸变处理部分===============================
    ros::Time startTime, endTime;
    //一帧scan的时间戳就代表一帧数据的开始时间
    startTime = scan->header.stamp;
    sensor_msgs::LaserScan laserScanMsg = *scan;
    int beamNum = laserScanMsg.ranges.size();
    //根据激光时间分割和激光束个数的乘积+startTime得到endTime（最后一束激光束的时间）
    endTime = startTime + ros::Duration(laserScanMsg.time_increment * beamNum);
    laser_ranges_.clear();
    laser_angles_.clear();
    //拷贝scan数据到laser_ranges_,laser_angles_
    double lidar_dist,lidar_angle;
    for(int i = 0; i < beamNum;i++)
    {
        lidar_dist  = laserScanMsg.ranges[i];//单位米
        lidar_angle = laserScanMsg.angle_min + laserScanMsg.angle_increment * i;//单位弧度
        laser_ranges_.push_back(lidar_dist);
        laser_angles_.push_back(lidar_angle);
    }
    //激光雷达运动畸变去除
    lmc_->lidarCalibration(laser_ranges_,laser_angles_,startTime,endTime,&tf_);
    //因为运动畸变去除之后，激光束的角度就不是均匀的了，所以就需要更新激光束的角度值
    gsp_->m_matcher.setLaserParameters(beamNum,&(laser_angles_[0]));

    //===================================================================================    
    static ros::Time last_map_update(0,0);     //存储上一次地图更新的时间

    if(!got_first_scan_)                       //如果是第一次接收scan
    {
        if(!initMapper(*scan))                 //初始化地图
            return;
        got_first_scan_ = true;                //改变第一帧的标志位
    }

    GMapping::OrientedPoint odom_pose;         //当前里程计坐标系下的激光雷达位姿的临时变量    
    if(addScan(*scan, odom_pose))
    {
        ROS_DEBUG("addScan finish");
        //最优粒子，地图坐标系下的激光雷达位姿
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        //在激光雷达当前位姿下，用当前的里程计坐标系下的激光雷达位姿和map坐标系下的激光雷达位姿，来表述里程计坐标系和map坐标系的差距
        //因为两个坐标系描述的是同一个激光雷达，所以才能用位姿的不同描述坐标系变换关系（差异）
        tf::Transform map_to_lidar  = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), 
                                                    tf::Vector3(mpose.x, mpose.y, 0.0));
        tf::Transform odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), 
                                                    tf::Vector3(odom_pose.x, odom_pose.y, 0.0));
        //多个线程访问同一资源时，为了保证数据的一致性，最简单的方式就是使用 mutex（互斥锁）
        //阻止了同一时刻有多个线程并发访问共享资源
        map_to_odom_mutex_.lock();
        map_to_odom_ = map_to_lidar * (odom_to_lidar.inverse()); //表述里程计坐标系和map坐标系的差距 
        map_to_odom_mutex_.unlock();

        //多久更新一次地图
        if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
            updateMap(*scan);
            last_map_update = scan->header.stamp;
            ROS_INFO("Updated the map");    
        }
    }
    else
    {
        ROS_DEBUG("cannot process scan");
    }
}

//第一帧激光数据来临，需要做的事情，默认雷达水平朝上安装
bool MySlamGMapping::initMapper(const sensor_msgs::LaserScan& scan)
{
    //通过真实的激光雷达数据，设置gmapping算法中激光的最大距离和最大使用距离，这样是为什么前面不一起初始化的原因
    ros::NodeHandle private_nh_("~");
    if(!private_nh_.getParam("maxRange", maxRange_))
        maxRange_ = scan.range_max - 0.01;
    if(!private_nh_.getParam("maxUrange", maxUrange_))
        maxUrange_ = maxRange_;

    //得到激光雷达在里程计的初始位姿，如果没有 则把初始位姿设置为(0,0,0)，也是建立地图的起始位置
    GMapping::OrientedPoint initialPose;
    if(!getLidarPose(initialPose, scan.header.stamp))
    {
        ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }
    //为gmapping算法设置各种参数
    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,kernelSize_, lstep_, astep_, iterations_,lsigma_, ogain_, lskip_);
    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    //初始化 m_generateMap = false（是scanmatch中的成员变量）
    gsp_->setgenerateMap(false);
    //初始化粒子个数，地图尺寸，分辨率，建图初始位姿
    gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,delta_, initialPose);
    gsp_->setminimumScore(minimum_score_);
    //高斯噪声的随机数种子
    GMapping::sampleGaussian(1,seed_);

    ROS_DEBUG("initMapper complete");

    return true;
}

//每一帧激光数据，都要通过该函数封装gmapping算法需要的数据格式，并调用核心算法
bool MySlamGMapping::addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose)
{
    //得到该scan时刻，激光雷达在里程计下的位姿
    if(!getLidarPose(gmap_pose, scan.header.stamp))
        return false;
   
    //把ROS的激光雷达数据信息 转换为 GMapping算法看得懂的形式,这里加入运动畸变去除之后的激光点的角度数据
    GMapping::RangeReading reading(scan.ranges.size(),&(laser_ranges_[0]),&(laser_angles_[0]));
    
    //为每一个reading设置激光雷达的里程计位姿
    reading.setPose(gmap_pose);

    //调用gmapping算法进行处理,传入算法看的懂的数据结构
    return gsp_->processScan(reading);
}

//通过每一帧scan的时间戳，计算出当前scan时刻里程计坐标系下激光雷达坐标系的位姿
bool MySlamGMapping::getLidarPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),tf::Vector3(0,0,0)), t, laser_frame_);
    //odom_pose存储输出的里程计坐标系下激光雷达的tf::Stamped<tf::Pose>格式的位姿
    tf::Stamped<tf::Pose> odom_pose;//激光雷达的里程计位姿
    try
    {
        //odom_frame_ 目标坐标系
        //ident，带时间戳的激光雷达坐标系tf::Stamped<tf::Pose>格式的数据
        //odom_pose 得到该时间戳下的位姿数据
        tf_.transformPose(odom_frame_, ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    //通过转化得到OrientedPoint格式的里程计坐标系下的位姿
    double yaw = tf::getYaw(odom_pose.getRotation());
    gmap_pose = GMapping::OrientedPoint(odom_pose.getOrigin().x(),
                                        odom_pose.getOrigin().y(),
                                        yaw);
    return true;
}

//地图更新，第一次没有地图直接更新，之后按照周期更新地图
void MySlamGMapping::updateMap(const sensor_msgs::LaserScan& scan)
{
    ROS_DEBUG("Update map start");
    //scope_lock:严格基于作用域的锁管理类模板
    //构造时是否加锁是可选的(不加锁时假定当前线程已经获得锁的所有权)，析构时自动释放锁，所有权不可转移
    //对象生存期内不允许手动加锁和释放锁
    boost::mutex::scoped_lock map_lock (map_mutex_);
    GMapping::ScanMatcher matcher;

    /*设置scanmatcher的各个参数*/
    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    /*得到权重最高的粒子*/
    GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

    //如果没有地图 则初始化一个地图，连地图map_.map的长宽都没初始化
    if(!got_map_)
    {
        map_.map.info.resolution = delta_;
        map_.map.info.origin.position.x = 0.0;
        map_.map.info.origin.position.y = 0.0;
        map_.map.info.origin.position.z = 0.0;
        map_.map.info.origin.orientation.x = 0.0;
        map_.map.info.origin.orientation.y = 0.0;
        map_.map.info.origin.orientation.z = 0.0;
        map_.map.info.origin.orientation.w = 1.0;
    }

    /*地图的中点*/
    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    /*初始化一个scanmatcherMap 创建一个地图*/
    //这里的地图一定要区分part_slam算法部分的地图，这里可视化用，和part_slam不一样
    //区别smap与map_.map这个非常重要
    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,delta_);

    //遍历粒子的整条轨迹 按照轨迹上各个节点存储的信息来重新绘制一个地图
    for(GMapping::GridSlamProcessor::TNode* n = best.node;n;n = n->parent)
    {
        if(!n->reading)
        {
            ROS_DEBUG("Reading is NULL");
            continue;
        }
        //每一个节点的激光雷达角度数据都不同，所以每个节点都要重新设置
        std::vector<double> laser_angles_for_map;
        for(int i = 0; i < n->reading->m_angles.size();i++)
            laser_angles_for_map.push_back(n->reading->m_angles[i]);
        matcher.setLaserParameters(scan.ranges.size(), &(laser_angles_for_map[0]));

        //拓展地图大小、找到地图的有效区域，单位patch,申请内存、更新每个栅格的内容
        matcher.computeMap(smap, n->pose, &(n->reading->m_dists[0]));
    }

    // 根据smap地图更改可视化地图的内容
    if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY())
    {
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; 
        ymin_ = wmin.y;
        xmax_ = wmax.x; 
        ymax_ = wmax.y;

        ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
                  xmin_, ymin_, xmax_, ymax_);

        map_.map.info.width = smap.getMapSizeX();
        map_.map.info.height = smap.getMapSizeY();
        map_.map.info.origin.position.x = xmin_;
        map_.map.info.origin.position.y = ymin_;
        map_.map.data.resize(map_.map.info.width * map_.map.info.height);//存储栅格数据的数组大小，可视化用的

        ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
    }

    //根据smap地图中存储的栅格数据，修改map_.map.data[]的数据,这里为一维数组
    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            //从smap中得到栅格点p(x, y)被占用的概率
            GMapping::IntPoint p(x, y);
            double occ = double(smap.cell(p)); // -1、0-1，到达一定的occ_thresh_认为被占用
            assert(occ <= 1.0);

            //未知
            if(occ < 0)
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_UNKNOWN;
            //占用
            else if(occ > occ_thresh_)//默认0.25
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_OCC;
            //空闲
            else
                map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_FREE;
        }
    }

    //到了这一步，肯定是有地图了。
    got_map_ = true;

    //把计算出来的地图发布出去
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = map_frame_;

    //发布map和map_metadata
    sst_.publish(map_.map);      
    sstm_.publish(map_.map.info);
}

//地图服务的回调函数
bool MySlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                               nav_msgs::GetMap::Response &res)
{
    boost::mutex::scoped_lock map_lock (map_mutex_);
    if(got_map_ && map_.map.info.width && map_.map.info.height)
    {
        res = map_;
        return true;
    }
    else
        return false;
}

//发布map->odom的转换关系
void MySlamGMapping::publishLoop(double transform_publish_period)
{
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
    while(ros::ok())
    {
        publishTransform();
        r.sleep();
    }
}

//发布map到odom的转换关系
void MySlamGMapping::publishTransform()
{
    map_to_odom_mutex_.lock();
    //默认情况下 tf_delay_ = transform_publish_period_;
    //默认情况下ros::Duration(tf_delay_)时间长度，等于 r.sleep();的时间长度
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);//这个没搞明白为啥要加这一点时间，感觉没有必要
    tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
    map_to_odom_mutex_.unlock();
}
