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


# 计算三维距离函数
def distance(i, j):
    a = "UAV" + str(i)
    b = "UAV" + str(j)
    x = (client.getMultirotorState(vehicle_name=a).kinematics_estimated.position.x_val + initial_position[i - 1][0]) - (
            client.getMultirotorState(
                vehicle_name=b).kinematics_estimated.position.x_val + initial_position[j - 1][0])
    y = (client.getMultirotorState(vehicle_name=a).kinematics_estimated.position.y_val + initial_position[i - 1][1]) - (
            client.getMultirotorState(
                vehicle_name=b).kinematics_estimated.position.y_val + initial_position[j - 1][1])
    z = (client.getMultirotorState(vehicle_name=a).kinematics_estimated.position.z_val + initial_position[i - 1][2]) - (
            client.getMultirotorState(
                vehicle_name=b).kinematics_estimated.position.z_val + initial_position[j - 1][2])
    return math.sqrt(x * x + y * y + z * z)


# 位置函数返回无人机位置参数
def getpos(a):
    a = "UAV" + str(a)
    return client.simGetGroundTruthKinematics(vehicle_name=a).position

    # 错误类型：
    # 1 错误：常值 避碰速度，聚集速度，同步速度都为0
    # 2 错误：随机值 随机生成避碰 聚集 同步速度
    # 3 错误：相反值 避碰速度 聚集速度，同步速度取反
    # 错误周期类型：
    # 0 一直发生错误
    # 1 拜占庭错误
    # 2 周期错误
    # err_agentL列表记录错误无人机编号，错误类型，错误周期类型


# 常值错误
def consterr(i, v_sep, v_coh, vx, vy, vz):
    vx[i] = 0
    vy[i] = 0
    vz[i] = 0
    v_sep[i][0] = 0
    v_sep[i][1] = 0
    v_sep[i][2] = 0
    v_coh[i][0] = 0
    v_coh[i][1] = 0
    v_coh[i][2] = 0


# 随机错误
def randerr(i, v_sep, v_coh, vx, vy, vz, randmax=10):
    randmax = randmax
    vx[i] = random.uniform(-randmax, randmax)
    vy[i] = random.uniform(-randmax, randmax)
    vz[i] = random.uniform(-randmax, randmax)
    v_sep[i][0] = random.uniform(-randmax, randmax)
    v_sep[i][1] = random.uniform(-randmax, randmax)
    v_sep[i][2] = random.uniform(-randmax, randmax)
    v_coh[i][0] = random.uniform(-randmax, randmax)
    v_coh[i][1] = random.uniform(-randmax, randmax)
    v_coh[i][2] = random.uniform(-randmax, randmax)


# 相反值错误
def reverseerr(i, v_sep, v_coh, vx, vy, vz):
    vx[i] = -vx[i]
    vy[i] = -vy[i]
    vz[i] = -vz[i]
    v_sep[i][0] = -v_sep[i][0]
    v_sep[i][1] = -v_sep[i][1]
    v_sep[i][2] = -v_sep[i][2]
    v_coh[i][0] = -v_coh[i][0]
    v_coh[i][1] = -v_coh[i][1]
    v_coh[i][2] = -v_coh[i][2]


def fly(err_agent, T_limit, randmax, T_switch, dis, Beta, Eta):
    err_agent = err_agent
    T_switch = T_switch
    dis = dis
    T_limit = T_limit
    randmax = randmax
    Beta = Beta
    Eta = Eta
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
    Reward = [[[] * T_limit for i in range(agent_num)] for j in range(agent_num)]
    Weight = [[[] * T_limit for i in range(agent_num)] for j in range(agent_num)]

    v_sep = [[0, 0, 0] for _ in range(agent_num)]
    v_coh = [[0, 0, 0] for _ in range(agent_num)]

    # 初始化速度列表
    vx = [random.randint(-10, 10) for _ in range(agent_num)]
    vy = [random.randint(-10, 10) for _ in range(agent_num)]
    vz = [random.randint(-3, 0) for _ in range(agent_num)]
    vx = [2, 2, 7]
    vy = [2, 8, -2]
    vz = [-2, -3, -3]

    for erragent in err_agent:
        if erragent[1] == 1:
            temp = erragent[0]
            vx[temp] = 0
            vy[temp] = 0
            vz[temp] = 0
    # 连接无人机，解除锁，起飞
    for i in range(agent_num):
        name = "UAV" + str(i)
        client.enableApiControl(True, name)
        client.armDisarm(True, name)
    for i in range(agent_num):
        name = "UAV" + str(i)
        client.takeoffAsync(vehicle_name=name)

    time.sleep(2)
    # 避碰速度：( -k * r_ij / np.linalg.norm(r_ij) ) / N_i + 1
    # 聚集速度：( k * r_ij ) / N_i + 1
    # 速度一致：在拓扑结构里做平均
    flag2 = False  # 周期错误状态
    for t in range(T_limit):

        # 动态调整拓扑结构
        for index1, value1 in enumerate(Topology):
            for index2, value2 in enumerate(value1):
                if value2 == 0 or index1 == index2:  # 原本就不连接的和自己本身跳过
                    Dist[index1][index2] = 0
                    continue
                if distance(index1, index2) > dis:
                    Topology[index1][index2] = 2  # 掉线
                else:
                    Topology[index1][index2] = 1  # 重连接
                Dist[index1][index2] = distance(index1, index2)
        for i in range(agent_num):
            # 记录vx，vy, vz
            result[t][i].append(round(vx[i], 2))
            result[t][i].append(round(vy[i], 2))
            result[t][i].append(round(vz[i], 2))
        # 执行飞行命令
        for i in range(agent_num):
            name = "UAV" + str(i)
            print(f'第{t}时刻')
            print(f'无人机{i},置信度表{Cre[i]},速度值[{vx[i]},{vy[i]},{vz[i]}]')
            print(f'无人机{i}的距离表{Dist[i]}')
            client.moveByVelocityZAsync(vx[i], vy[i], vz[i],
                                        2, vehicle_name=name)
        time.sleep(1)
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
                r_ij = distance(i, j)
                # print("无人机", i, "与无人机", , "之间的距离：", distance(i, j))
                rx_ij = (getpos(i).x_val + initial_position[i][0]) - (getpos(j).x_val + initial_position[j][0])
                ry_ij = (getpos(i).y_val + initial_position[i][1]) - (getpos(j).y_val + initial_position[j][1])
                rz_ij = (getpos(i).z_val + initial_position[i][2]) - (getpos(j).z_val + initial_position[j][2])
                v_sep[i][0] += k_sep * rx_ij / r_ij
                v_sep[i][1] += k_sep * ry_ij / r_ij
                v_sep[i][2] += k_sep * rz_ij / r_ij
                v_coh[i][0] += -k_coh * rx_ij
                v_coh[i][1] += -k_coh * ry_ij
                v_coh[i][2] += -k_coh * rz_ij
                v_i = client.simGetGroundTruthKinematics(vehicle_name="UAV" + str(i)).linear_velocity
                # v_i = math.sqrt(v_i.x_val**2+v_i.y_val**2)
                v_j = client.simGetGroundTruthKinematics(vehicle_name="UAV" + str(j)).linear_velocity
                # v_j = math.sqrt(v_j.x_val**2+v_j.y_val**2)
                d_v = (abs(vx[i] - vx[j]) + abs(vy[j] - vy[i]) + abs(vz[j] - vz[i]))
                Reward[i][j].append(math.exp(-d_v * Beta))
                #
                # Reward[i][j].append(Beta / (1 + abs(v_i.x_val - v_j.x_val - v_i.y_val + v_j.y_val)))
                # Reward[i][j].append(-abs(v_j-v_i))
                Cre[i][j] += Eta * (Reward[i][j][-1] - Cre[i][j])

                # sum_cre_i += Cre[i][j][-1]
            # Cre_sum[i].append(sum_cre_i)
            v_sep[i][0] = v_sep[i][0] / N_agent
            v_sep[i][1] = v_sep[i][1] / N_agent
            v_sep[i][2] = v_sep[i][2] / N_agent
            v_coh[i][0] = v_coh[i][0] / N_agent
            v_coh[i][1] = v_coh[i][1] / N_agent
            v_coh[i][2] = v_coh[i][2] / N_agent

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
        delta_vx = [0 for _ in range(agent_num)]
        delta_vy = [0 for _ in range(agent_num)]
        delta_vz = [0 for _ in range(agent_num)]
        for i in range(agent_num):
            for j, value in enumerate(Topology[i]):
                if i != j and Topology[i][j] == 1:
                    delta_vx[i] += Weight[i][j][-1] * (vx[j] - vx[i])
                    delta_vy[i] += Weight[i][j][-1] * (vy[j] - vy[i])
                    delta_vz[i] += Weight[i][j][-1] * (vz[j] - vz[i])

        # 一次性更新所有无人机的速度
        for i in range(agent_num):
            vx[i] += delta_vx[i]
            vy[i] += delta_vy[i]
            vz[i] += delta_vz[i]
            # vx[i] = round(vx[i], 2)
            # vy[i] = round(vy[i], 2)
        if err_agent:
            for erragent in err_agent:
                index = erragent[0]  # 无人机编号
                errtype = erragent[1]  # 错误类型
                errperiod = erragent[2]  # 错误周期
                if errperiod == 0:
                    if errtype == 1:
                        consterr(index, v_sep, v_coh, vx, vy, vz)
                    elif errtype == 2:
                        randerr(index, v_sep, v_coh, vx, vy, vz, randmax)
                    elif errtype == 3:
                        reverseerr(index, v_sep, v_coh, vx, vy, vz)
                elif errperiod == 1:  # 拜占庭错误
                    iserror = bool(random.getrandbits(1))
                    if not iserror:
                        pass
                    else:
                        if errtype == 1:
                            consterr(index, v_sep, v_coh, vx, vy, vz)
                        elif errtype == 2:
                            randerr(index, v_sep, v_coh, vx, vy, vz, randmax)
                        elif errtype == 3:
                            reverseerr(index, v_sep, v_coh, vx, vy, vz)
                elif errperiod == 2:  # 周期错误 周期为5
                    if t % T_switch == 0:
                        if not flag2:
                            flag2 = True
                        else:
                            flag2 = False
                    if flag2:
                        if errtype == 1:
                            consterr(index, v_sep, v_coh, vx, vy, vz)
                        elif errtype == 2:
                            randerr(index, v_sep, v_coh, vx, vy, vz, randmax)
                        elif errtype == 3:
                            reverseerr(index, v_sep, v_coh, vx, vy, vz)
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


def main(err_agent=[[1, 3, 0]], T_limit=30, randmax=10, T_switch=3, dis=25, Beta=0.1, Eta=0.05):
    client.reset()
    time.sleep(1)
    res = fly(err_agent, T_limit, randmax, T_switch, dis, Beta, Eta)
    return res


if __name__ == "__main__":
    main()
