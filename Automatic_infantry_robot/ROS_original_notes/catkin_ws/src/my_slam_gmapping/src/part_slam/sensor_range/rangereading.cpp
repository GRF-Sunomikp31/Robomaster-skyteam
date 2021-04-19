#include "rangereading.h"

namespace GMapping{

using namespace std;

/*构造函数
n_beams  指定了激光束的数量
d        表示各个激光的距离
anle     每束激光的角度
*/

//不均匀角度的激光雷达数据
RangeReading::RangeReading(unsigned int n_beams, const double* d,const double* angle)
{
    m_beams = n_beams;
    m_dists.resize(n_beams);
    m_angles.resize(n_beams);
    for (unsigned int i=0; i<n_beams; i++)
    {
        m_dists[i]=d[i];
        m_angles[i]=angle[i];
    }
}

RangeReading::~RangeReading(){}

};