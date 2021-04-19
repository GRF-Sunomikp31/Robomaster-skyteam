#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <stdlib.h>
#include <sys/types.h>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>

template <class Particle, class Numeric>
struct uniform_resampler
{
	std::vector<unsigned int> resampleIndexes(const std::vector<Particle> & particles, int nparticles=0) const;
};

template <class Particle, class Numeric>
std::vector<unsigned int> uniform_resampler<Particle, Numeric>:: resampleIndexes(const std::vector<Particle>& particles, int nparticles) const
{
	Numeric cweight=0;

	/*计算总的权重*/
	unsigned int n=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it)
	{
		cweight+=(Numeric)*it;
		n++;
	}
	//自定义部分n
	if (nparticles>0)
		n=nparticles;
	
	//interval表示权重的区间，平均每份权重的大小
	Numeric interval=cweight/n;

	//target这里在一份权重大小区间内，随机采取一个目标权重
	//drand48 返回服从均匀分布的·[0.0, 1.0) 之间的 double 型随机数
	Numeric target=interval*::drand48();

	/*根据权值进行采样*/
	//采用度盘轮转算法，target每次加interval。如果某个粒子的权重比较大的话，那么他肯定会被采样到很多次。
    //比如说如果某个粒子的区间为4*interval。那么它至少被采样3次。
	cweight=0;
	//序列数组的大小，根据粒子数目而定，默认粒子序号为0号粒子
	std::vector<unsigned int> indexes(n);
	n=0;
	unsigned int i=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, ++i)
	{
		cweight+=(Numeric)* it;
		while(cweight>target)
		{
			indexes[n++]=i;
			target+=interval;
		}
	}
	return indexes;
}

#endif
