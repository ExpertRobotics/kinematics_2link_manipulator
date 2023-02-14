import json
import random
from scipy.integrate import odeint

# import sympy as sp
from sympy.physics.vector import init_vprinting
from numpy import *
from threading import Timer
from threading import Event
from scipy import interpolate

init_vprinting(use_latex='mathjax', pretty_print=False)
from sympy.physics.mechanics import dynamicsymbols
import time
from time import perf_counter

params = {
  "link_lengths": {
    "l1": 0.8,
    "l2": 0.4
  },
  "initial_positions": {
    "phi1": 30,
    "phi2": 15
  },
  "link_value_rp": {
    "l": 0.75,
    "h": 0.25
  },
  "initial_positions_rp": {
    "s": 0.5,
    "phi": 15
  },
  "init_conditions": {
    "w": 0.1,
    "time_move": 5,
    "vs": 0.2
  }
}
"""служебный класс для рассчета реального времени работы"""


class MyTimer:
    def __init__(self, func):
        self.fn = func

    def __call__(self, *args, **kwargs):
        start = perf_counter()
        print(f'Вызывается функция {self.fn.__name__}')
        result = self.fn(*args, **kwargs)
        finish = perf_counter()
        print(f'функция {self.fn.__name__} работала {finish - start} сек')
        return result


class Robot:
    p = list()  # хранит вычисленные параметры движения, для повторного использования

    def __init__(self, robot_conf):
        self.work_space = None  # не реализовано
        self.robot_conf = robot_conf
        #  инициализация значений, общих для всех схем
        self.w = pi * robot_conf['init_conditions']['w']
        self.tm = robot_conf['init_conditions']['time_move']
        self.vs = robot_conf['init_conditions']['vs']

    def fk(self, **args):
        raise NotImplementedError

    def ik(self, x, y):
        raise NotImplementedError

    #  простой аксессор
    def get_init(self):
        w = self.w  # pi / 10  # rad / c
        s = self.vs  # m/s
        return [w, s]

    def set_configuration(self, config):
        raise NotImplementedError

    #  можно посмотреть перед началом работы что в конфигурации json
    def specification(self):
        for i, section in enumerate([self.robot_conf]):  # [0:2:]
            print(section.keys())
            print(section.values())
            # print(section.items())

    def set_start_position(self):
        raise NotImplementedError

    def set_end_position(self, x, y):
        self.x_end = x
        self.y_end = y
        raise NotImplementedError

    # задает ветор направления движения, единажды перед началом для каждого из изменяемых параметров
    def set_dir(self):
        return 1 if random.choice([True, False]) else -1

    # пока просто дискретизация интервала времени
    def trajectory_generation(self, task):
        t = linspace(0, self.tm, 100)
        # t = linspace(0, self.tm * 2, 200)
        dt = t[1] - t[0]

        # def dphidt(t):
        #     return 30 + 5.4 * t ** 2 - 0.72 * t ** 3
        # v0 = 0
        # phi_dot = odeint(dphidt, y0=v0, t=t, tfirst=True)
        # sol_m2 = solve_ivp(dphidt, t_span=(0, max(t)), y0=[v0], t_eval=t)
        # xy = interpolate.InterpolatedUnivariateSpline(x, y)
        # xnew = xy(x)
        # ynew = xy(y)
        # v_xy = sqrt(v_x ** 2 + v_y ** 2)
        return t, dt

    def move_robot(self, **args):
        raise NotImplementedError

    def run_simulation(self):
        raise NotImplementedError

    def update_position(self, **args):
        raise NotImplementedError

    def calc_next_point_of_move(self):
        raise NotImplementedError

    def get_position(self):
        raise NotImplementedError

    def check_end_position(self):
        raise NotImplementedError

    def draw_state(self):
        raise NotImplementedError

    #  вывод на экран результатов расчета схемы
    def display(self):
        raise NotImplementedError


class RR2linkRobot(Robot):
    __slots__ = 'l1', 'l2', 'phi1', 'phi2'

    def __init__(self, robot_conf):
        super().__init__(robot_conf)
        self.l1 = robot_conf['link_lengths']['l1']
        self.l2 = robot_conf['link_lengths']['l2']
        self.phi1 = deg2rad(robot_conf['initial_positions']['phi1'])
        self.phi2 = deg2rad(robot_conf['initial_positions']['phi2'])
        self.work_space = None
        self.values = self.l1, self.l2, self.phi1, self.phi2
        print(f'Robot: {self.work_space} реализция {self.__class__.__name__}')

    def position_end_effector(self, l1, l2, phi1, phi2):
        R0_1 = [
            [cos(phi1), -sin(phi1), 0],
            [sin(phi1), cos(phi1), 0],
            [0, 0, 1]
        ]

        R1_2 = [
            [cos(phi2), -sin(phi2), 0],
            [sin(phi2), cos(phi2), 0],
            [0, 0, 1]
        ]
        d0_1 = [
            [l1 * cos(phi1)],
            [l1 * sin(phi1)],
            [0]
        ]
        d1_2 = [
            [l2 * cos(phi2)],
            [l2 * sin(phi2)],
            [0]
        ]

        T0_1 = vstack((hstack((R0_1, d0_1)), [0, 0, 0, 1]))
        T1_2 = vstack((hstack((R1_2, d1_2)), [0, 0, 0, 1]))
        T0_2 = dot(T0_1, T1_2)
        x = T0_2[0][3]
        y = T0_2[1][3]

        return x, y

    def lin_velocity_projections(self, l1, l2, phi1, phi2, phi1_dot=1.0, phi2_dot=1.0):
        J = [
            [-l1 * sin(phi1) - l2 * sin(phi1 + phi2), - l2 * sin(phi1 + phi2)],
            [l1 * cos(phi1) + l2 * cos(phi1 + phi2), l2 * cos(phi1 + phi2)],
            # [1, 1]
        ]

        phi_dot = [
            [phi1_dot],
            [phi2_dot]
        ]
        xy_dot = dot(J, phi_dot)

        return *xy_dot[0], *xy_dot[1]

    def angular_velocity_end_effector(self, l1, l2, phi1, phi2, x_dot=1.0, y_dot=1.0):
        # phi1 = (phi1 / 180) * pi
        # phi2 = (phi2 / 180) * pi
        # detJ = l1 * l2 * sin(phi2)  ## phi2 [0, pi]
        # multiple = 1 / detJ
        # J_INV2 = [
        #     [multiple * l2 * cos(phi1 + phi2), multiple * (-(- l2 * sin(phi1 + phi2)))],
        #     [multiple * (-(l1 * cos(phi1) + l2 * cos(phi1 + phi2))),
        #      multiple * (-l1 * sin(phi1) - l2 * sin(phi1 + phi2))]
        # ]
        J = [
            [-l1 * sin(phi1) - l2 * sin(phi1 + phi2), - l2 * sin(phi1 + phi2)],
            [l1 * cos(phi1) + l2 * cos(phi1 + phi2), l2 * cos(phi1 + phi2)],
            # [1, 1]
        ]
        # J_INV = matrix(J).I
        J_INV = linalg.inv(J)

        xy_dot = [
            [x_dot],
            [y_dot]
        ]
        # xy_dot = squeeze(xy_dot)
        phi_dot = dot(J_INV, xy_dot)
        # phi_dot = J_INV @ xy_dot
        return *phi_dot[0], *phi_dot[1]

    def update_position(self, i, t_len, dphi, phi1, phi2, vphi1=1, vphi2=-1):
        # максимальная скорость на участке уход - подход
        k = 1 if int(t_len * 0.2) < i < int(t_len * 0.8) else 0.5
        # исходя из: φ˙ = w * dt + φ
        phi1 += vphi1 * dphi
        phi2 += vphi2 * dphi

        phi1_dot = dphi * k + phi1
        phi2_dot = dphi * k + phi2

        return phi1, phi2, phi1_dot, phi2_dot

    def move_robot(self, l1, l2, phi1, phi2, dphi1, dphi2):
        x, y = self.position_end_effector(l1, l2, phi1, phi2)
        x_dot, y_dot = self.lin_velocity_projections(l1, l2, phi1, phi2, dphi1, dphi2)
        phi1_dot, phi2_dot = self.angular_velocity_end_effector(l1, l2, phi1, phi2, x_dot, y_dot)
        return [x, y, 'm |', rad2deg(phi1), rad2deg(phi2), 'degree|', x_dot, y_dot, 'm/s |', phi1_dot, phi2_dot,
                'rad/sec |']

    def fk(self):
        l1, l2, phi1_0, phi2_0 = self.values
        t, dt = self.trajectory_generation('fk')
        w = self.get_init()[0]
        phi1, phi2 = phi1_0, phi2_0
        t_len = len(t)
        dphi = w * dt
        vphi1 = self.set_dir()
        vphi2 = self.set_dir()
        for i in t:
            phi1, phi2, dphi1, dphi2 = self.update_position(i, t_len, dphi, phi1, phi2, vphi1, vphi2)
            # print(self.move_robot(l1, l2, phi1, phi2, dphi1, dphi2), '|', phi1, phi2)
            self.p.append(self.move_robot(l1, l2, phi1, phi2, dphi1, dphi2))

    def dh_method(self):
        phi1, phi2, l1, l2, phi, alpha, a, d, = dynamicsymbols('phi1 phi2 l1 l2 phi alpha a d')

    def ik(self, x, y):
        l1, l2, phi1, phi2 = self.values
        phi2 = arccos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2))
        phi1 = arctan2(y, x) - arctan2(l2 * sin(phi2), l1 + l2 * cos(phi2))

        print('1 solution')
        print(f'phi1:{phi1} rad, {rad2deg(phi1)} deg')
        print(f'phi2:{phi2} rad, {rad2deg(phi2)} deg')
        print('x = ', l1 * cos(phi1) + l2 * cos(phi1 + phi2))
        print('y = ', l1 * sin(phi1) + l2 * sin(phi1 + phi2))

        phi1_0 = phi1

        phi2 = - arccos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2))
        phi1 = - arctan2(x, y) + arctan2((l1 + l2 * cos(phi2)), (l2 * sin(phi2)))
        # phi1 = arctan2(y, x) + arctan2((l2 * sin(phi2)), (l1 + l2 * cos(phi2)))

        print('2 solution')
        print(f'phi1:{phi1} rad, {rad2deg(phi1)} deg')
        print(f'phi2:{phi2} rad, {rad2deg(phi2)} deg')
        print('x = ', l1 * cos(phi1) + l2 * cos(phi1 + phi2))
        print('y = ', l1 * sin(phi1) + l2 * sin(phi1 + phi2))

        phil = arctan2(y, x)
        print('check solutions')
        print(
            f'phil: {phil} rad, {rad2deg(phil)} deg, angle between l1 and L: {phil - phi1_0} rad, {rad2deg(phil - phi1_0)} deg')
        print(
            f'phil + angle: {phil + (phil - phi1_0)} rad, {rad2deg(phil + (phil - phi1_0))} deg, must be equals phi1: {phi1} '
            f'rad, {rad2deg(phi1)} deg.')

        return phi1, phi2

    def display(self):
        print('RR - manipulator move parameters')
        print(self.values)
        print(*self.p, sep='\n')

    def draw_state(self):
        pass

    def run_simulation(self):
        self.fk()
        t = Timer(0.5, self.display)
        t.start()
        print(self.p.__len__())


class RP2linkRobot(Robot):
    #  только собственная конфигурация
    __slots__ = 'l', 'h', 'phi', 's'

    def __init__(self, robot_conf):
        super().__init__(robot_conf)
        self.l = robot_conf['link_value_rp']['l']
        self.h = robot_conf['link_value_rp']['h']
        self.phi = deg2rad(robot_conf['initial_positions_rp']['phi'])
        self.s = robot_conf['initial_positions_rp']['s']
        self.work_space = None
        self.values = self.l, self.h, self.phi, self.s
        print(f'Robot: {self.work_space} реализция {self.__class__.__name__}')

    def position_end_effector(self, phi, s, l, h):
        A0_1 = [
            [sin(phi), 0, cos(phi), h * sin(phi)],
            [cos(phi), 0, sin(phi), -h * cos(phi)],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ]
        A1_2 = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, (l + s)],
            [0, 0, 0, 1]
        ]

        T0_2 = dot(A0_1, A1_2)
        x = T0_2[0][3]
        y = T0_2[1][3]
        return x, y

    def linear_velocity(self, phi, s, l, h, dphi=1.0, ds=1.0):
        J = [
            [(-s * sin(phi) - l * sin(phi) + h * cos(phi)), cos(phi)],
            [s * cos(phi) + l * cos(phi) + h * sin(phi), sin(phi)],
            # [1, 0]
        ]
        dphids = [
            [dphi],
            [ds]
        ]
        xy_dot = dot(J, dphids)

        return *xy_dot[0], *xy_dot[1]

    def update_position(self, i, t_len, dphi, ds, phi, s, vphi=1, vds=1):
        # максимальная скорость на участке уход - подход, простейшая имитация коэффициентом
        k = 1 if int(t_len * 0.2) < i < int(t_len * 0.8) else 0.5

        # исходя из: φ˙ = w * dt + φ
        phi += vphi * dphi
        s += vds * ds

        phi_dot = dphi * k + phi
        s_dot = ds * k + s

        return phi, s, phi_dot, s_dot

    def move_robot(self, phi, s, l, h, dphi, ds):
        x, y = self.position_end_effector(phi, s, l, h)
        x_dot, y_dot = self.linear_velocity(phi, s, l, h, dphi, ds)
        return [x, y, 'm', rad2deg(phi), 'degree', s, 'm', x_dot, y_dot, 'm/s']

    def fk(self):
        l, h, phi_0, s_0 = self.values
        t, dt = self.trajectory_generation('fk')
        w = self.get_init()[0]
        v = self.get_init()[1]
        phi, s = phi_0, s_0
        t_len = len(t)
        dphi = w * dt
        ds = v * dt
        vphi = self.set_dir()
        vds = self.set_dir()
        for i in t:
            phi, s, phi_dot, s_dot = self.update_position(i, t_len, dphi, ds, phi, s, vphi, vds)
            # print(self.move_robot(phi, s, l, h, phi_dot, s_dot), '|', phi, s)
            self.p.append(self.move_robot(phi, s, l, h, dphi, ds))

    def dh_method(self):
        phi, s, l, h, theta, alpha, a, d = dynamicsymbols('phi1 phi2 l1 l2 phi alpha a d')

    def display(self):
        print('RP - manipulator move parameters')
        print(self.values)
        print(*self.p, sep='\n')

    def draw_state(self):
        pass

    def run_simulation(self):
        self.fk()
        t = Timer(0.5, self.display)
        t.start()
        print(self.p.__len__())


class RobotLaboratory:
    def __init__(self, name, r_config):
        self.name = name
        self.__robot_config = r_config
        self.__robot = None

    @property
    def robot_config(self):
        return self.__robot_config

    @robot_config.setter
    def robot_config(self, new_config):
        if isinstance(self.__robot, Robot):
            self.__robot.set_configuration(new_config)

    @staticmethod
    def show_intro():
        print('Инструкция')
        print('Optional: Ввод значений в сиситеме СИ осуществляется из файла robot_configuration.json')
        print('Optional: в main: rr - only round joints, rp - round and prismatic joints')
        print('Optional: направление движения задается случайно, можно задать своё в ref параметрах метода')
        print('Optional: рабочая область не задана')
        print('-' * 30)

    # factory method
    def robot(self, type):
        if not isinstance(type, str):
            raise TypeError('Тип рообота должен быть строкой!')
        if type == 'rr':
            self.__robot = RR2linkRobot(self.robot_config)
        if type == 'rp':
            self.__robot = RP2linkRobot(self.robot_config)
        return self.__robot

    @MyTimer
    def simulation(self, working_time):
        Event().wait(3)
        robot.run_simulation()
        # замедленная симуляция
        time.sleep(2.0)
        startTime = time.time()
        j = 0
        while time.time() - startTime < working_time and j < 100:  # len(Robot.p)
            print(f'{j + 1}: {str(Robot.p[j])}')
            j += 1
            time.sleep(0.25)  # 0.05 dt

    def animate(self):
        pass


if __name__ == "__main__":
    RobotLaboratory.show_intro()
    # robot_config = json.load(open("robot_configuration.json", "r"))
    robot_config = params
    robotLab = RobotLaboratory('my_lab', robot_config)
    robot = robotLab.robot('rr')
    # robot = robotLab.robot('rp')
    robot.specification()
    robotLab.simulation('fk', 26.375)  # 52.75)
