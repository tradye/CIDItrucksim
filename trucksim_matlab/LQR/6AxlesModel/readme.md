
算法实现步骤主要来自Apollo：

ComputeCOMPosition.m：根据车辆实际位置计算实时质心位置

computeFeedForward.m：计算前馈角度

ComputeLateralErrors.m：计算横向误差

QueryNearestPointByPosition.m：计算离当前位置最近的规划点

readPlanPoints.m：读取规划点

SolveLQRProblem.m：LQR求解

Interpolate8.m：速度和Q矩阵的一维插值

plot_plot8.m：画轨迹和误差图

generateTrailerPoints.m：根据运动学模型和卡车规划点得到挂车的理想路径

truck8_LQRController.m：这是s-function的主要文件
