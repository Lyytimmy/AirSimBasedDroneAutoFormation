import statistics
import os
from enum import Enum
from tqdm import tqdm
import QcTest
import NotQcTest
import csv


def save_to_folder(result_qc, result_noqc, err_agent):
    # 输入文件夹名
    folder_name = input("Please input the folder name: ")
    # 创建文件夹
    os.mkdir(folder_name)
    # 获取错误无人机indexList
    errlist = []
    for err in err_agent:
        errlist.append(err[0]-1)
    writeQCPos(result_qc, errlist)
    writePos(result_noqc, errlist)
    writeQCState(result_qc, errlist)
    writeState(result_noqc, errlist)

    # 移动CSV文件到文件夹
    os.rename("QCPos.csv", os.path.join(folder_name, "QCPos.csv"))
    os.rename("Pos.csv", os.path.join(folder_name, "Pos.csv"))
    os.rename("QCState.csv", os.path.join(folder_name, "QCState.csv"))
    os.rename("State.csv", os.path.join(folder_name, "State.csv"))


def writeQCPos(result, errList):
    with open('QCPos.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        headers = ['t', 'vx1', 'vy1', 'vx2', 'vy2', 'vx3', 'vy3', 'vx4', 'vy4', 'vx5', 'vy5', 'vx6', 'vy6', 'vx7',
                   'vy7', 'vx8', 'vy8', 'vx9', 'vy9', 'vx10', 'vy10']
        writer.writerow(headers)
        for t, agents in enumerate(result):
            row = [t]
            for agent in agents:
                row.append(agent[0])  # vx
                row.append(agent[1])  # vy
            writer.writerow(row)
        headers = ['VX均值', 'VX方差', 'VY均值', 'VY方差', 'VX均值无错误', 'VX方差无错误', 'VY均值无错误', 'VY方差无错误']
        writer.writerow(headers)
        row = []
        VX = [v[0] for v in result[-1]]
        VX_mean = round(statistics.mean(VX), 3)
        VX_variance = round(statistics.variance(VX), 3)
        VY = [v[1] for v in result[-1]]
        VY_mean = round(statistics.mean(VY), 3)
        VY_variance = round(statistics.variance(VY), 3)
        VX2 = []
        VY2 = []
        for i, vx in enumerate(VX):
            if i not in errList:
                VX2.append(vx)
        for i, vy in enumerate(VY):
            if i not in errList:
                VY2.append(vy)
        VX2_mean = round(statistics.mean(VX2), 3)
        VX2_variance = round(statistics.variance(VX2), 3)
        VY2_mean = round(statistics.mean(VY2), 3)
        VY2_variance = round(statistics.variance(VY2), 3)
        row.append(VX_mean)
        row.append(VX_variance)
        row.append(VY_mean)
        row.append(VY_variance)
        row.append(VX2_mean)
        row.append(VX2_variance)
        row.append(VY2_mean)
        row.append(VY2_variance)
        writer.writerow(row)


def writePos(result, errList):
    with open('Pos.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        headers = ['t', 'vx1', 'vy1', 'vx2', 'vy2', 'vx3', 'vy3', 'vx4', 'vy4', 'vx5', 'vy5', 'vx6', 'vy6', 'vx7',
                   'vy7', 'vx8', 'vy8', 'vx9', 'vy9', 'vx10', 'vy10']
        writer.writerow(headers)
        for t, agents in enumerate(result):
            row = [t]
            for agent in agents:
                row.append(agent[0])  # vx
                row.append(agent[1])  # vy
            writer.writerow(row)
        headers = ['VX均值', 'VX方差', 'VY均值', 'VY方差', 'VX均值无错误', 'VX方差无错误', 'VY均值无错误', 'VY方差无错误']
        writer.writerow(headers)
        row = []
        VX = [v[0] for v in result[-1]]
        VX_mean = round(statistics.mean(VX), 3)
        VX_variance = round(statistics.variance(VX), 3)
        VY = [v[1] for v in result[-1]]
        VY_mean = round(statistics.mean(VY), 3)
        VY_variance = round(statistics.variance(VY), 3)
        VX2 = []
        VY2 = []
        for i, vx in enumerate(VX):
            if i not in errList:
                VX2.append(vx)
        for i, vy in enumerate(VY):
            if i not in errList:
                VY2.append(vy)
        VX2_mean = round(statistics.mean(VX2), 3)
        VX2_variance = round(statistics.variance(VX2), 3)
        VY2_mean = round(statistics.mean(VY2), 3)
        VY2_variance = round(statistics.variance(VY2), 3)
        row.append(VX_mean)
        row.append(VX_variance)
        row.append(VY_mean)
        row.append(VY_variance)
        row.append(VX2_mean)
        row.append(VX2_variance)
        row.append(VY2_mean)
        row.append(VY2_variance)
        writer.writerow(row)


def writeQCState(result, errList):
    with open('QCState.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        headers = ['state1', 'state2', 'state3', 'state4', 'state5', 'state6', 'state7', 'state8', 'state9', 'state10',
                   '均值', '方差', '均值无错误', '方差无错误']
        writer.writerow(headers)
        row = []
        for value in result[-1]:
            row.append(value[-1])
        states = [v[-1] for v in result[-1]]
        mean = round(statistics.mean(states), 3)
        variance = round(statistics.variance(states), 3)
        states2 = []
        for i, value in enumerate(result[-1]):
            if i not in errList:
                states2.append(value[-1])
        mean2 = round(statistics.mean(states2), 3)
        variance2 = round(statistics.variance(states2), 3)
        row.append(mean)
        row.append(variance)
        row.append(mean2)
        row.append(variance2)
        writer.writerow(row)


def writeState(result, errList):
    with open('State.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        headers = ['state1', 'state2', 'state3', 'state4', 'state5', 'state6', 'state7', 'state8', 'state9', 'state10',
                   '均值', '方差', '均值无错误', '方差无错误']
        writer.writerow(headers)
        row = []
        for value in result[-1]:
            row.append(value[-1])
        states = [v[-1] for v in result[-1]]
        mean = round(statistics.mean(states), 3)
        variance = round(statistics.variance(states), 3)
        states2 = []
        for i, value in enumerate(result[-1]):
            if i not in errList:
                states2.append(value[-1])
        mean2 = round(statistics.mean(states2), 3)
        variance2 = round(statistics.variance(states2), 3)
        row.append(mean)
        row.append(variance)
        row.append(mean2)
        row.append(variance2)
        writer.writerow(row)


"""
标准：
err_agent = [[1, 1, 0]]
T_limit = 30
randmax = 10
T_switch = 3
dis = 25
Beta = 0.01
Eta = 5
"""


def Setting():
    err_agent = [[1, 1, 1], [4, 2, 1],[7, 3, 1]]
    T_limit = 100
    randmax = 10
    T_switch = 3
    dis = 25
    Beta = 0.01
    Eta = 5
    return err_agent, T_limit, randmax, T_switch, dis, Beta, Eta


class ErrorType(Enum):  # 错误类型
    常值错误 = 1
    #随机错误= 2
    #相反值错误 = 3


class ErrorPeriod(Enum):  # 错误周期
    恒定 = 0
    #拜占庭变化 = 1
    #周期变化 = 2

# 错误类型：
    # 1 错误：常值 避碰速度，聚集速度，同步速度都为0
    # 2 错误：随机值 随机生成避碰 聚集 同步速度
    # 3 错误：相反值 避碰速度 聚集速度，同步速度取反
    # 错误周期类型：
    # 0 一直发生错误
    # 1 拜占庭错误
    # 2 周期错误
    # err_agentL列表记录错误无人机编号，错误类型，错误周期类型


def circulateTest():
    T_limit = 15
    randmax = 10
    T_switch = 5
    dis = 25
    Beta = 0.01
    Eta = 5
    for error_type in tqdm(ErrorType):
        for error_period in ErrorPeriod:
            for i in [1]:
                if i == 1:
                    err_agent = [[1, 1, 0]]
                else:
                    err_agent.append([i, error_type, error_period])
                errList = []
                for err in err_agent:
                    errList.append(err[0] - 1)
                folder_name = f'{error_type}_{error_period}_{i}——yest'
                if not os.path.exists(folder_name):
                    os.makedirs(folder_name)
                qc_result = QcTest.main(err_agent, T_limit, randmax, T_switch, dis, Beta, Eta)
                print(qc_result)
                noqc_result = NotQcTest.main(err_agent, T_limit, randmax, T_switch, dis, Beta, Eta)
                writeQCPos(qc_result, errList)
                writePos(noqc_result, errList)
                writeQCState(qc_result, errList)
                writeState(noqc_result, errList)
                os.rename("QCPos.csv", os.path.join(folder_name, "QCPos.csv"))
                os.rename("Pos.csv", os.path.join(folder_name, "Pos.csv"))
                os.rename("QCState.csv", os.path.join(folder_name, "QCState.csv"))
                os.rename("State.csv", os.path.join(folder_name, "State.csv"))


def main():
    mode = 0
    if mode == 1:
        # 参数测试
        err_agent, T_limit, randmax, T_switch, dis, Beta, Eta = Setting()
        qc_result = QcTest.main(*Setting())
        noqc_result = NotQcTest.main(*Setting())
        save_to_folder(qc_result, noqc_result, err_agent)
    else:
        # 循环测试
        circulateTest()


if __name__ == "__main__":
    main()
