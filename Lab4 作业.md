# Lab4 作业

在最后两次课程中, 讲解了物理仿真和人体动画的内容. 本次作业要求实现弹簧质点和逆向运动学(Inverse Kinematics)算法. 作业预期结果如视频所示. 

本次作业代码基于C++实现, 编译方式与Lab1, Lab2相同. 程序入口在src/main.cpp中. 在src/renderer/renderer.h中定义了一个用于渲染的类Renderer. 其中DynamicSystem是弹簧质点的求解与渲染, SimpleArm提供了IK算法的求解与渲染.

## 弹簧质点(15')

实现基于隐式积分的弹簧质点算法. 将质点通过弹簧进行连接, 对质点系统进行模拟. 对于弹簧质点系统模拟, 当数值积分步长, 弹簧劲度系数等发生变化时, 弹簧质点仿真结果会有所不同. 需要的代码在src/massspring/MassSpring.cpp中

### 作业要求:

- (2')实现对单根弹簧受力的计算, 即完成函数void Edge::compute_spring_force(Vec3f& force)

- (5')实现对单根弹簧受力的Hessian矩阵的计算, 即完成函数void Edge::compute_hessian_matrix(Mat3f& hessian) 

  （即，计算并实现$\frac{\partial f^{(i)}}{\partial x^{(i)}}$ ，则根据对称性即可得到$\frac{\partial f^{(j)}}{\partial x^{(j)}}=\frac{\partial f^{(j)}}{\partial x^{(j)}},\,\frac{\partial f^{(j)}}{\partial x^{(i)}}=\frac{\partial f^{(i)}}{\partial x^{(j)}}=-\frac{\partial f^{(j)}}{\partial x^{(j)}}$)

- (3')利用显式欧拉法对系统进行更新（加速度、速度、位置）, 即完成函数void DynamicSystem::explicit_euler_step(float dt)中homework部分

- (5')理解隐式欧拉法的求解过程，并利用牛顿迭代的计算结果对系统进行更新, 即完成函数void DynamicSystem::implicit_euler_step(float dt)中homework部分

### 代码说明

- struct Node: 质点,  保存了质量, 位置, 速度, 加速度, 外力等物理属性. 其中is_fixed属性表示该质点在仿真过程中, 是否应该被固定.

- struct Edge: 弹簧, 其中init_length表示弹簧的初始长度, k表示弹簧的劲度系数, nodes表示与弹簧相连的两个质点.

- struct DynamicSystem: 对弹簧质点系统进行仿真和渲染
  - void create_line: 创建一条用弹簧连接的绳子
  -  void create_mesh_grad: 创建一个弹簧网格. 
  - void add_matrix_block: 将单个$3\times3$矩阵组装到$3n\times3n$的Hessian矩阵的对应位置
  - void explicit_euler_step: 基于显式欧拉法进行单步仿真
  - void implicit_euler_step: 基于隐式欧拉法进行单步仿真（注：此处与课程slides上的伪代码略有差异，实现的是不带backtracking line search的Newton solver）
- struct SparseSolver: 简单封装了一个基于CG的稀疏矩阵求解器.

## 逆向运动学(15')

逆向运动学代码实现在src/ik/SimpleArm.cpp中. 给定一个机械臂末端位置, 求解出每个关节的旋转角度. 代码框架中, 使用四元数进行旋转操作, 可以通过Eigen库进行数学运算(旋转,向量运算等)

作业要求:

1. 实现CCD IK(4')和fabr IK(7')算法, 比较这两种算法收敛需要的迭代次数, 即修改void SimpleArm::ccd_ik和void SimpleArm::fabr_ik函数
2. (2')增加关节节数, 观察IK结果 (对构造函数创建关节部分进行修改)
3. (2')create_trajectory函数给定了机械臂末端在每一时刻的位置, 修改这个函数, 自定义绘制曲线(比如数字, 姓名首字母缩写等) 

代码说明: 对于SimpleArm类, 

- joint_offset表示一个关节相对于父关节的平移量. 

- joint_offset表示根关节(root)的全局位置; 
- joint_offset_len表示每个关节的长度; 
- joint_rotation表示每个关节的局部旋转; 
- joint_orientation表示每个关节的全局旋转;
-  joint_position表示每个关节的全局位置;
-  end_position_hist表示末端关节的位置序列; 
- trajectory表示末端关节应当做出的位置序列; 
- forward_kinematics函数: 通过关节局部旋转, 计算出关节全局旋转, 以及关节全局位置



