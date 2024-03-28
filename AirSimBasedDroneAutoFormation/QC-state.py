import random
import time
import math
import airsim
import os
import json

# 参数
# agent_num = 10
initial_position = [[0, 0, 0], [-5, 5, 0], [12, 9, 0], [6, -5, 0], [5, 9, 0], [-2, -6, 0], [7, -9, 0], [11, -8, 0],
                    [-8, 4, 0], [13, 5, 0]]
client = airsim.MultirotorClient()
k_sep = 7
k_coh = 1
k_mig = 1


# 计算距离函数
def distance(i, j):
    a = "UAV" + str(i)
    b = "UAV" + str(j)
    x = (client.getMultirotorState(vehicle_name=a).kinematics_estimated.position.x_val + initial_position[i][0]) - (
            client.getMultirotorState(
                vehicle_name=b).kinematics_estimated.position.x_val + initial_position[j][0])
    y = (client.getMultirotorState(vehicle_name=a).kinematics_estimated.position.y_val + initial_position[i][1]) - (
            client.getMultirotorState(
                vehicle_name=b).kinematics_estimated.position.y_val + initial_position[j][1])
    z = (client.getMultirotorState(vehicle_name=a).kinematics_estimated.position.z_val + initial_position[i][2]) - (
            client.getMultirotorState(
                vehicle_name=b).kinematics_estimated.position.z_val + initial_position[j][2])
    return math.sqrt(x * x + y * y + z * z)

def adjust_velocity(i, Topology, pos_x, pos_y, pos_z, max_velocity):
    # i 是当前无人机的索引
    # Topology 是无人机的连接拓扑
    # pos_x, pos_y, pos_z 是无人机的位置列表
    # max_velocity 是最大速度
    some_threshold = 15
    collision_threshold=2
    some_factor = 0.5
    # 计算群体的平均位置
    avg_x = sum(pos_x) / len(pos_x)
    avg_y = sum(pos_y) / len(pos_y)
    avg_z = sum(pos_z) / len(pos_z)

    # 计算当前无人机到群体平均位置的距离
    distance_to_avg = math.sqrt((pos_x[i] - avg_x)**2 + (pos_y[i] - avg_y)**2 + (pos_z[i] - avg_z)**2)

    # 根据距离调整速度
    if distance_to_avg > some_threshold:
        # 如果远离群体中心，增加速度
        velocity = min(max_velocity, some_factor * distance_to_avg)
    else:
        # 如果靠近群体中心，减小速度
        velocity = max_velocity * (distance_to_avg / some_threshold)

    # 考虑与最近的无人机的距离，进一步调整速度
    for j in range(len(Topology[i])):
        if Topology[i][j] == 1 and i != j:  # 如果与无人机 j 相连
            distance_to_j = math.sqrt((pos_x[i] - pos_x[j])**2 + (pos_y[i] - pos_y[j])**2 + (pos_z[i] - pos_z[j])**2)
            if distance_to_j < collision_threshold:
                # 如果距离太近，降低速度以避免碰撞
                velocity = min(velocity, distance_to_j / collision_threshold * max_velocity)

    return velocity
def distance1(i,j):
    a = "UAV" + str(i)
    b = "UAV" + str(j)
    x = (client.simGetGroundTruthKinematics(vehicle_name=a).position.x_val + initial_position[i][0]) - (
            client.simGetGroundTruthKinematics(
                vehicle_name=b).position.x_val + initial_position[j][0])
    y = (client.simGetGroundTruthKinematics(vehicle_name=a).position.y_val + initial_position[i][1]) - (
            client.simGetGroundTruthKinematics(
                vehicle_name=b).position.y_val + initial_position[j][1])
    z = (client.simGetGroundTruthKinematics(vehicle_name=a).position.z_val + initial_position[i][2]) - (
            client.simGetGroundTruthKinematics(
                vehicle_name=b).position.z_val + initial_position[j][2])
    return math.sqrt(x * x + y * y + z * z)

# 位置函数返回无人机位置参数
def getpos(a):
    a = "UAV" + str(a)
    client.getMultirotorState()
    return client.getMultirotorState(vehicle_name=a).kinematics_estimated.position



# 常值错误
def consterr(i,pos_x, pos_y, pos_z):
    pos_x[i] = 0
    pos_y[i] = 0
    pos_z[i] = 0


# 随机错误
def randerr(i, pos_x, pos_y, pos_z, randmax=10):
    randmax = randmax
    pos_x[i] = random.uniform(-randmax, randmax)
    pos_y[i] = random.uniform(-randmax, randmax)
    pos_z[i] = random.uniform(-randmax, randmax)


# 相反值错误
def reverseerr(i, pos_x, pos_y, pos_z):
    pos_x[i] = -pos_x[i]
    pos_y[i] = -pos_y[i]
    pos_z[i] = -pos_z[i]


def fly(err_agent, T_limit, randmax, T_switch, dis, Beta, Eta):
    err_agent = err_agent
    T_switch = T_switch
    dis = dis
    T_limit = T_limit
    randmax = randmax
    Beta = Beta
    Eta = Eta
    futures = []
    result = [[[] for _ in range(10)] for _ in range(T_limit)]
    Topology = [[1, 0, 0, 1, 0, 0, 0, 1, 0, 0],
                [0, 1, 0, 1, 0, 0, 1, 0, 1, 0],
                [0, 0, 1, 0, 1, 1, 0, 0, 0, 1],
                [1, 1, 0, 1, 1, 1, 0, 1, 1, 1],
                [0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
                [0, 0, 1, 1, 1, 1, 1, 0, 1, 0],
                [0, 1, 0, 0, 1, 1, 1, 1, 0, 1],
                [1, 0, 0, 1, 1, 0, 1, 1, 0, 0],
                [0, 1, 0, 1, 1, 1, 0, 0, 1, 0],
                [0, 0, 1, 1, 0, 0, 1, 0, 0, 1]]

    Topology = [[1, 1, 1],
                [1, 1, 1],
                [1, 1, 1]]
    agent_num = len(Topology)

    # 1为正常连接，0为原本就不连接，2为距离过大掉线
    Dist = [[0] * agent_num for i in range(agent_num)]  # 距离
    # 初始化 Cre 矩阵为二维列表
    Cre = [[1 for _ in range(agent_num)] for _ in range(agent_num)]
    Reward = [[[] for _ in range(agent_num)] for _ in range(agent_num)]
    Weight = [[[] * T_limit for i in range(agent_num)] for j in range(agent_num)]


    # 初始化位置列表
    pos_x = [random.randint(-10, 10) for _ in range(agent_num)]
    pos_y = [random.randint(-10, 10) for _ in range(agent_num)]
    pos_z = [random.randint(-5, -2) for _ in range(agent_num)]
    v =[0 for _ in range(agent_num)]
    # pos_x = [2, -9, -7]
    # pos_y = [8, -8, -3]
    # pos_z = [-2, -3, -4]
    for agent in err_agent:
        if agent[1] == 1:
            index = agent[0]
            pos_x[index] = 0
            pos_y[index] = 0
            pos_z[index] = 0
    # 连接无人机，解除锁，起飞
    for i in range(agent_num):
        name = "UAV" + str(i)
        client.enableApiControl(True, name)
        client.armDisarm(True, name)
    for i in range(agent_num):
        name = "UAV" + str(i)
        client.takeoffAsync(vehicle_name=name)

    time.sleep(3)

    flag2 = False  # 周期错误状态
    for t in range(T_limit):
        for index1, value1 in enumerate(Topology):
            for index2, value2 in enumerate(value1):
                if value2 == 0 or index1 == index2:  # 原本就不连接的和自己本身跳过
                    Dist[index1][index2] = 0
                    continue
                # if distance(index1, index2) > dis:
                #     Topology[index1][index2] = 2  # 掉线
                # else:
                #     Topology[index1][index2] = 1  # 重连接
                Dist[index1][index2] = distance1(index1, index2)
        for i in range(agent_num):
            result[t][i].append(round(pos_x[i], 2))
            result[t][i].append(round(pos_y[i], 2))
            result[t][i].append(round(pos_z[i], 2))

        # 执行飞行命令
        for i in range(agent_num):
            name = "UAV" + str(i)
            # print(i, Cre[i], pos_x[i], pos_y[i], pos_z[i])
            # print(f'{i}无人机,置信度表{Cre[i]},距离表{Dist[i]}, 位置信息{pos_x[i]}, {pos_y[i]}, {pos_z[i]}')
            # future = client.moveToPositionAsync(pos_x[i]+initial_position[i][0], pos_y[i]+initial_position[i][1], pos_z[i]+initial_position[i][2],2, vehicle_name=name)
            future = client.moveToPositionAsync(pos_x[i], pos_y[i], pos_z[i],2, vehicle_name=name)
            futures.append(future)
        print(Topology)
        # print(v)
        time.sleep(2)

        for i, value1 in enumerate(Topology):
            name_i = "UAV" + str(i)
            N_agent = value1.count(1)
            # sum_cre_i = 0
            sum_weight = 0
            for j, value2 in enumerate(value1):
                if i == j:
                    continue
                if value2 == 0 or value2 == 2:
                    continue
                name_j = "UAV" + str(j)
                # print("无人机", i, "与无人机", , "之间的距离：", distance(i, j))
                rx_ij = (getpos(i).x_val + initial_position[i][0]) - (getpos(j).x_val + initial_position[j][0])
                ry_ij = (getpos(i).y_val + initial_position[i][1]) - (getpos(j).y_val + initial_position[j][1])
                rz_ij = (getpos(i).z_val + initial_position[i][2]) - (getpos(j).z_val + initial_position[j][2])
                pos_x[i] = getpos(i).x_val + initial_position[i][0]
                pos_y[i] = getpos(i).y_val + initial_position[i][1]
                pos_z[i] = getpos(i).z_val + initial_position[i][2]


                # Reward[i][j].append(math.exp(-distance(i, j) * Beta))
                Reward[i][j].append(math.exp(-distance1(i, j) * Beta))
                # print(i,j,Reward[i][j])
                Cre[i][j] += Eta * (Reward[i][j][-1] - Cre[i][j])
            v[i] = adjust_velocity(i, Topology, pos_x, pos_y, pos_z,8)
        # 执行飞行命令
        for i in range(agent_num):
            name = "UAV"+str(i)
            pos = client.simGetGroundTruthKinematics(vehicle_name=name).position
            print(f'{i}无人机,置信度表{Cre[i]},距离表{Dist[i]}, 位置信息{pos.x_val+initial_position[i][0]}, {pos.y_val+ initial_position[i][1]}, {pos.z_val+ initial_position[i][2]}')

        for i, value1 in enumerate(Topology):
            sum_weight_i = 0
            for j, value2 in enumerate(value1):
                if i == j:
                    continue
                if value2 == 2 or value2 == 0:
                    continue
                Weight[i][j].append(
                    Cre[i][j] / sum([Cre[i][k] for k in range(len(Topology[i]))]) * (1 - 1 / Topology[i].count(1)))
                sum_weight_i += Weight[i][j][-1]
            # Weight_sum[i].append(sum_weight_i)

        # 初始化速度变化量列表
        delta_x = [0 for _ in range(agent_num)]
        delta_y = [0 for _ in range(agent_num)]
        delta_z = [0 for _ in range(agent_num)]
        for i in range(agent_num):
            for j, value in enumerate(Topology[i]):
                if i != j and Topology[i][j] == 1:
                    delta_x[i] += Weight[i][j][-1] * (pos_x[j] - pos_x[i])
                    delta_y[i] += Weight[i][j][-1] * (pos_y[j] - pos_y[i])
                    delta_z[i] += Weight[i][j][-1] * (pos_z[j] - pos_z[i])
        # 一次性更新所有无人机的速度
        for i in range(agent_num):
            pos_x[i] += delta_x[i]
            pos_y[i] += delta_y[i]
            pos_z[i] += delta_z[i]
        if err_agent:
            for erragent in err_agent:
                index = erragent[0]  # 无人机编号
                errtype = erragent[1]  # 错误类型
                errperiod = erragent[2]  # 错误周期
                if errperiod == 0:
                    if errtype == 1:
                        consterr(index,pos_x, pos_y, pos_z)
                    elif errtype == 2:
                        randerr(index, pos_x, pos_y, pos_z, randmax)
                    elif errtype == 3:
                        reverseerr(index,pos_x, pos_y, pos_z)
                elif errperiod == 1:  # 拜占庭错误
                    iserror = bool(random.getrandbits(1))
                    if not iserror:
                        pass
                    else:
                        if errtype == 1:
                            consterr(index, pos_x, pos_y, pos_z)
                        elif errtype == 2:
                            randerr(index, pos_x, pos_y, pos_z, randmax)
                        elif errtype == 3:
                            reverseerr(index, pos_x, pos_y, pos_z)
                elif errperiod == 2:  # 周期错误 周期为5
                    if t % T_switch == 0:
                        if not flag2:
                            flag2 = True
                        else:
                            flag2 = False
                    if flag2:
                        if errtype == 1:
                            consterr(index, pos_x, pos_y, pos_z)
                        elif errtype == 2:
                            randerr(index, pos_x, pos_y, pos_z, randmax)
                        elif errtype == 3:
                            reverseerr(index, pos_x, pos_y, pos_z)
                    else:
                        pass

        # 循环结束
        # 计算中心点centerpoints
    centerpoint_list = []
    for index1, value1 in enumerate(Topology):
        centerpoint_x = 0
        centerpoint_y = 0
        centerpoint_z = 0
        num = 0
        for index2, value2 in enumerate(value1):
            if value2 == 1:
                num += 1
                centerpoint_x += client.getMultirotorState(
                    vehicle_name="UAV" + str(index2)).kinematics_estimated.position.x_val + \
                                 initial_position[index2][0]
                centerpoint_y += client.getMultirotorState(
                    vehicle_name="UAV" + str(index2)).kinematics_estimated.position.y_val + \
                                 initial_position[index2][1]
                centerpoint_z += client.getMultirotorState(
                    vehicle_name="UAV" + str(index2)).kinematics_estimated.position.z_val + \
                                 initial_position[index2][2]
        centerpoint = [round(centerpoint_x / num, 2), round(centerpoint_y / num, 2), round(centerpoint_z / num, 2)]
        centerpoint_list.append(centerpoint)
    # 计算所有无人机的state
    for i in range(agent_num):
        state_x = client.getMultirotorState(vehicle_name="UAV" + str(i)).kinematics_estimated.position.x_val + \
                  initial_position[i][0] - centerpoint_list[i][0]
        state_y = client.getMultirotorState(vehicle_name="UAV" + str(i)).kinematics_estimated.position.y_val + \
                  initial_position[i][1] - centerpoint_list[i][1]
        state_z = client.getMultirotorState(vehicle_name="UAV" + str(i)).kinematics_estimated.position.z_val + \
                  initial_position[i][2] - centerpoint_list[i][2]
        state = round(math.sqrt(state_x * state_x + state_y * state_y + state_z * state_z), 2)
        result[-1][i].append(state)
    return result


"""
记录不同条件下（3*3*3)，以下属性与T（15，30，50）的关系
条件：错误类型3种*错误周期3种*错误无人机个数（1，4，7）
计算出四个值：state的平均值和方差、速度的平均值和方差
对比所有的情况下，qc算法和不使用qc算法的四个值
在以上基础上，调整参数（学习率，随机错误中速度变化范围，周期变化频率，拓扑结构变化范围）
"""


def main(err_agent=[], T_limit=30, randmax=10, T_switch=10, dis=25, Beta=0.1, Eta=0.1):
    client.reset()
    time.sleep(1)
    res = fly(err_agent, T_limit, randmax, T_switch, dis, Beta, Eta)
    return res


if __name__ == "__main__":
    main()
