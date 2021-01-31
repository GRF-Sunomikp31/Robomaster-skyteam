# Python课外阅读——taichi  计算机图形学编程语言

## 前言

2020年5月份，知乎上一篇关于“99行代码的《冰雪奇缘》”的文章引起了很多人的关注，作者胡渊鸣为MIT博士生，清华大学本科生开发出一种名为“taichi”（太极）的编程的语言，在此基础上利用计算机图像学的方法实现**物理引擎**的效果。

以下是作者胡渊鸣知乎文章：https://zhuanlan.zhihu.com/p/97700605

## 简介

什么是taichi（太极）？

**Taichi** (太极) is a programming language designed for *high-performance computer graphics*. It is deeply embedded in **Python**, and its **just-in-time compiler** offloads compute-intensive tasks to multi-core CPUs and massively parallel GPUs.

简单来说，目前的taichi已经嵌入Python中了，您可以在下载完成之后，通过 import taichi as ti 来使用它；

~~记得有位大佬说，python的精髓在于import；~~  

p.s.这位大佬一看就不是业内人士；

什么是物理引擎（Physics engine）？

wiki:

A **physics engine** is [computer software](https://en.wikipedia.org/wiki/Computer_software) that provides an approximate [simulation](https://en.wikipedia.org/wiki/Computer_simulation) of certain [physical systems](https://en.wikipedia.org/wiki/Physical_system), such as [rigid body dynamics](https://en.wikipedia.org/wiki/Rigid_body_dynamics) (including [collision detection](https://en.wikipedia.org/wiki/Collision_detection)), [soft body dynamics](https://en.wikipedia.org/wiki/Soft_body_dynamics), and [fluid dynamics](https://en.wikipedia.org/wiki/Fluid_simulation), of use in the domains of [computer graphics](https://en.wikipedia.org/wiki/Computer_graphics), [video games](https://en.wikipedia.org/wiki/Video_game) and film ([CGI](https://en.wikipedia.org/wiki/Computer-generated_imagery)). Their main uses are in video games (typically as [middleware](https://en.wikipedia.org/wiki/Game_middleware)), in which case the simulations are in [real-time](https://en.wikipedia.org/wiki/Real-time_simulation). The term is sometimes used more generally to describe any [software system](https://en.wikipedia.org/wiki/Software_system) for simulating physical phenomena, such as [high-performance scientific simulation](https://en.wikipedia.org/wiki/High-performance_computing).

简单来说，就是在计算机上实现现实世界中的一些物理效果；

## 安装使用

### 安装过程

管理员运行cmd，输出：

```
 pip install taichi
```

![20210131133157](C:\Users\lenovo\Desktop\taichi\20210131133157.png)

即可完成安装，然后就可以 import taichi as ti ；

### Demo

这里关于taichi的语法就不在加累述，具体查看 其github文档或者B站视频课程；

Demo1

​		效果图：（taichi的demo基本上都是动态的，这里我偷懒了:D）

![image-20210131133751202](C:\Users\lenovo\AppData\Roaming\Typora\typora-user-images\image-20210131133751202.png)

Code：

```python
# fractal.py

import taichi as ti

ti.init(arch=ti.gpu)

n = 320
pixels = ti.var(dt=ti.f32, shape=(n * 2, n))

@ti.func
def complex_sqr(z):
  return ti.Vector([z[0] ** 2 - z[1] ** 2, z[1] * z[0] * 2])

@ti.kernel
def paint(t: ti.f32):
  for i, j in pixels: # 对于所有像素，并行执行
    c = ti.Vector([-0.8, ti.sin(t) * 0.2])
    z = ti.Vector([float(i) / n - 1, float(j) / n - 0.5]) * 2
    iterations = 0
    while z.norm() < 20 and iterations < 50:
      z = complex_sqr(z) + c
      iterations += 1
    pixels[i, j] = 1 - iterations * 0.02

gui = ti.GUI("Fractal", (n * 2, n))

for i in range(1000000):
  paint(i * 0.03)
  gui.set_image(pixels)
  gui.show()
```

Demo2

​		效果图：

![image-20210131134412416](C:\Users\lenovo\AppData\Roaming\Typora\typora-user-images\image-20210131134412416.png)

​	Code：

```
import taichi as ti
import numpy as np
ti.init(arch=ti.gpu) # Try to run on GPU
quality = 1 # Use a larger value for higher-res simulations
n_particles, n_grid = 9000 * quality ** 2, 128 * quality
dx, inv_dx = 1 / n_grid, float(n_grid)
dt = 1e-4 / quality
p_vol, p_rho = (dx * 0.5)**2, 1
p_mass = p_vol * p_rho
E, nu = 0.1e4, 0.2 # Young's modulus and Poisson's ratio
mu_0, lambda_0 = E / (2 * (1 + nu)), E * nu / ((1+nu) * (1 - 2 * nu)) # Lame parameters
x = ti.Vector.field(2, dtype=float, shape=n_particles) # position
v = ti.Vector.field(2, dtype=float, shape=n_particles) # velocity
C = ti.Matrix.field(2, 2, dtype=float, shape=n_particles) # affine velocity field
F = ti.Matrix.field(2, 2, dtype=float, shape=n_particles) # deformation gradient
material = ti.field(dtype=int, shape=n_particles) # material id
Jp = ti.field(dtype=float, shape=n_particles) # plastic deformation
grid_v = ti.Vector.field(2, dtype=float, shape=(n_grid, n_grid)) # grid node momentum/velocity
grid_m = ti.field(dtype=float, shape=(n_grid, n_grid)) # grid node mass

@ti.kernel
def substep():
  for i, j in grid_m:
    grid_v[i, j] = [0, 0]
    grid_m[i, j] = 0
  for p in x: # Particle state update and scatter to grid (P2G)
    base = (x[p] * inv_dx - 0.5).cast(int)
    fx = x[p] * inv_dx - base.cast(float)
    # Quadratic kernels  [http://mpm.graphics   Eqn. 123, with x=fx, fx-1,fx-2]
    w = [0.5 * (1.5 - fx) ** 2, 0.75 - (fx - 1) ** 2, 0.5 * (fx - 0.5) ** 2]
    F[p] = (ti.Matrix.identity(float, 2) + dt * C[p]) @ F[p] # deformation gradient update
    h = ti.exp(10 * (1.0 - Jp[p])) # Hardening coefficient: snow gets harder when compressed
    if material[p] == 1: # jelly, make it softer
      h = 0.3
    mu, la = mu_0 * h, lambda_0 * h
    if material[p] == 0: # liquid
      mu = 0.0
    U, sig, V = ti.svd(F[p])
    J = 1.0
    for d in ti.static(range(2)):
      new_sig = sig[d, d]
      if material[p] == 2:  # Snow
        new_sig = min(max(sig[d, d], 1 - 2.5e-2), 1 + 4.5e-3)  # Plasticity
      Jp[p] *= sig[d, d] / new_sig
      sig[d, d] = new_sig
      J *= new_sig
    if material[p] == 0:  # Reset deformation gradient to avoid numerical instability
      F[p] = ti.Matrix.identity(float, 2) * ti.sqrt(J)
    elif material[p] == 2:
      F[p] = U @ sig @ V.transpose() # Reconstruct elastic deformation gradient after plasticity
    stress = 2 * mu * (F[p] - U @ V.transpose()) @ F[p].transpose() + ti.Matrix.identity(float, 2) * la * J * (J - 1)
    stress = (-dt * p_vol * 4 * inv_dx * inv_dx) * stress
    affine = stress + p_mass * C[p]
    for i, j in ti.static(ti.ndrange(3, 3)): # Loop over 3x3 grid node neighborhood
      offset = ti.Vector([i, j])
      dpos = (offset.cast(float) - fx) * dx
      weight = w[i][0] * w[j][1]
      grid_v[base + offset] += weight * (p_mass * v[p] + affine @ dpos)
      grid_m[base + offset] += weight * p_mass
  for i, j in grid_m:
    if grid_m[i, j] > 0: # No need for epsilon here
      grid_v[i, j] = (1 / grid_m[i, j]) * grid_v[i, j] # Momentum to velocity
      grid_v[i, j][1] -= dt * 50 # gravity
      if i < 3 and grid_v[i, j][0] < 0:          grid_v[i, j][0] = 0 # Boundary conditions
      if i > n_grid - 3 and grid_v[i, j][0] > 0: grid_v[i, j][0] = 0
      if j < 3 and grid_v[i, j][1] < 0:          grid_v[i, j][1] = 0
      if j > n_grid - 3 and grid_v[i, j][1] > 0: grid_v[i, j][1] = 0
  for p in x: # grid to particle (G2P)
    base = (x[p] * inv_dx - 0.5).cast(int)
    fx = x[p] * inv_dx - base.cast(float)
    w = [0.5 * (1.5 - fx) ** 2, 0.75 - (fx - 1.0) ** 2, 0.5 * (fx - 0.5) ** 2]
    new_v = ti.Vector.zero(float, 2)
    new_C = ti.Matrix.zero(float, 2, 2)
    for i, j in ti.static(ti.ndrange(3, 3)): # loop over 3x3 grid node neighborhood
      dpos = ti.Vector([i, j]).cast(float) - fx
      g_v = grid_v[base + ti.Vector([i, j])]
      weight = w[i][0] * w[j][1]
      new_v += weight * g_v
      new_C += 4 * inv_dx * weight * g_v.outer_product(dpos)
    v[p], C[p] = new_v, new_C
    x[p] += dt * v[p] # advection

group_size = n_particles // 3
@ti.kernel
def initialize():
  for i in range(n_particles):
    x[i] = [ti.random() * 0.2 + 0.3 + 0.10 * (i // group_size), ti.random() * 0.2 + 0.05 + 0.32 * (i // group_size)]
    material[i] = i // group_size # 0: fluid 1: jelly 2: snow
    v[i] = ti.Matrix([0, 0])
    F[i] = ti.Matrix([[1, 0], [0, 1]])
    Jp[i] = 1
initialize()
gui = ti.GUI("Taichi MLS-MPM-99", res=512, background_color=0x112F41)
while not gui.get_event(ti.GUI.ESCAPE, ti.GUI.EXIT):
  for s in range(int(2e-3 // dt)):
    substep()
  colors = np.array([0x068587, 0xED553B, 0xEEEEF0], dtype=np.uint32)
  gui.circles(x.to_numpy(), radius=1.5, color=colors[material.to_numpy()])
  gui.show() # Change to gui.show(f'{frame:06d}.png') to write images to disk
```

Demo3

​		效果图：

![image-20210131134634348](C:\Users\lenovo\AppData\Roaming\Typora\typora-user-images\image-20210131134634348.png)

​		Code：

```
import taichi as ti
import math

ti.init(arch=ti.gpu)

N = 1600
r0 = 0.05
dt = 1e-5
steps = 160
eps = 1e-3
G = -1e1

pos = ti.Vector.var(2, ti.f32, N)
vel = ti.Vector.var(2, ti.f32, N)


@ti.kernel
def initialize():
    for i in range(N):
        a = ti.random() * math.tau
        r = ti.sqrt(ti.random()) * 0.3
        pos[i] = 0.5 + ti.Vector([ti.cos(a), ti.sin(a)]) * r


@ti.kernel
def substep():
    for i in range(N):
        acc = ti.Vector([0.0, 0.0])

        p = pos[i]
        for j in range(N):
            if i != j:
                r = p - pos[j]
                x = r0 / r.norm(1e-4)
                # Molecular force: https://www.zhihu.com/question/38966526
                acc += eps * (x**13 - x**7) * r
                # Long-distance gravity force:
                acc += G * (x**3) * r

        vel[i] += acc * dt

    for i in range(N):
        pos[i] += vel[i] * dt


gui = ti.GUI('N-body Star')

initialize()
while gui.running and not gui.get_event(ti.GUI.ESCAPE):
    gui.circles(pos.to_numpy(), radius=2, color=0xfbfcbf)
    gui.show()
    for i in range(steps):
        substep()
```

更多demo，参考[1];

最后，如果您想了解这方面的更多内容以及taichi的更多使用方法，可以阅读 参考资料 [2]中作者的taichi课程链接以及[1]中的官方github。

## 参考资料

1、taichi项目官方github

地址链接：https://github.com/taichi-dev/taichi

2、作者胡渊鸣B站账号（包括其在2020年中国计算机图形学大会关于taichi编程语言的介绍课程）

地址链接：https://space.bilibili.com/490448800

