import math as m
from math import sin, cos

def fill_Robot_params(params):
    listValue = []
    while True:
            for i, p in enumerate(params):
                print('Введите значение ', params[i], ': ')
                try:
                    listValue.append(int(input()))
                except ValueError:
                    print('Неверный формат')
                    break
            if i >= (len(params) - 1):
                break
    
    tupleValue = tuple(j for j in listValue)
    return tupleValue


def runFirstScheme():
    print('введите параметры для вычисления 1 - го робота')
    params1 = ['l1', 'l2', 'φ1_0', 'φ2_0', 'range_φ1', 'range_φ2', 'work_mode', 'direction']
    print(params1)
    print('work_mode: 1(only l1), 2(only l2) or 3(both), direction: 1(up) or 2(down), 12 (up/down) 21(down/up)')
    tuple1 = fill_Robot_params(params1)
    move_first_robot(tuple1)


def move_first_robot(tuple1):
    l1, l2, φ1_0, φ2_0, range_φ1, range_φ2, work_mode, direction = tuple1
    φ1 = φ1_0 
    φ2 = φ2_0
    count_step = 10
    dφ1 = range_φ1 / count_step  ##20/10 2.0 ang_step_φ1, slope_φ1
    dφ2 = range_φ2 / count_step  ##10/10 1.0 ang_step_φ2, slope_φ2
    while ((φ1_0 - range_φ1) <= φ1 <= (φ1_0 + range_φ1)) and ((φ2_0 - range_φ2) <= φ2 <= (φ2_0 + range_φ2)):
        x, y, dx, dy = calc_position_firstRobot(l1, l2, φ1, φ2, dφ1, dφ2)
        print('ответ ', x, y, dx, dy, 'params:', l1, l2, φ1, φ2)
        φ1, φ2 = setWorkMode_firstRobot(work_mode, direction, φ1, φ2, dφ1, dφ2)

def calc_position_firstRobot(l1, l2, φ1, φ2, dφ1, dφ2):
    x = l1 * cos(m.radians(φ1)) + l2 * cos(m.radians(φ1 + φ2))
    y = l1 * sin(m.radians(φ1)) + l2 * sin(m.radians(φ1 + φ2))
    dx = -l1 * dφ1 * sin(m.radians(φ1) - l2 * (dφ1 + dφ2) * sin(m.radians(φ1 + φ2)))
    dy = l1 * dφ1 * cos(m.radians(φ1)) + l2 * (dφ1 + dφ2) * cos(m.radians(φ1 + φ2))

    return x, y, dx, dy

def setWorkMode_firstRobot(work_mode, direction, φ1, φ2, dφ1, dφ2):
    if(work_mode == 1): #move: l1, l2 без движения  
        if direction == 1: 
            φ1 += dφ1 #dir: l1 1 up
        elif direction == 2:  
            φ1 -= dφ1 #dir: l1 2 down
    elif(work_mode == 2): #move: l2, l1 без движения  
        if direction == 1: 
            φ2 += dφ2 #dir: l2 1 up
        elif direction == 2: 
            φ2 -= dφ2 #dir: l2 2 down 
    elif(work_mode == 3): #move: l1, l2
        if direction == 11: 
            φ1 += dφ1 #dir: l1  1 up
            φ2 += dφ2 #dir: l2  1 up
        elif direction == 22: 
            φ1 -= dφ1 #dir: l1  2 down
            φ2 -= dφ2 #dir: l2  2 down
        elif direction == 12: 
            φ1 += dφ1 #dir: l1  1 up
            φ2 -= dφ2 #dir: l2  2 down
        elif direction == 21:  
            φ1 -= dφ1 #dir: l1  2 down
            φ2 += dφ2 #dir: l2  1 up

    return φ1, φ2


def runSecondScheme():
    print('введите параметры для вычисления 2 - го робота')
    params2 = ['l', 'h', 's_0', 'φ_0', 'range_s', 'range_φ', 'work_mode', 'direction']#'direction_s', 'direction_φ']
    print(params2)
    print('work_mode: 1(only s(OA)), 2(only rotation) or 3(both), direction_s:1(right выдвижение плеча) or 2 (left втягивание), direction_φ: 1(up) or 2(down)')
    tuple2 = fill_Robot_params(params2)
    move_second_robot(tuple2)


def move_second_robot(tuple2):
    l, h, s_0, φ_0, range_s, range_φ, work_mode, direction = tuple2  
    s = s_0 
    φ = φ_0
    count_step = 10
    ds = range_s / count_step  ##2.0 ang_step_φ2
    dφ = range_φ / count_step  ##4.0 ang_step_φ1
    #φ = m.radians(φ)
    while ((s_0 - range_s) <= s <= (s_0 + range_s)) and ((φ_0 - range_φ) <= φ <= (φ_0 + range_φ)):
        x, y, dx, dy = calc_position_secondRobot(l, h, s, φ, ds, dφ)
        print('ответ ', x, y, dx, dy, 'params:', l, h, s, φ, work_mode, direction)
        s, φ = setWorkMode_secondRobot(work_mode, direction, s, φ, ds, dφ)

def calc_position_secondRobot(l, h, s, φ, ds, dφ):
    x = s * cos(φ) + l * cos(φ) + h * sin(φ)  
    y = s * sin(φ) + l * sin(φ) - h * cos(φ)  
    dx = (-s * sin(φ) - l * sin(φ) + h * cos(φ)) * dφ + cos(φ) * ds
    dy = (s * cos(φ) + l * cos(φ) + h * sin(φ)) * dφ + sin(φ) * ds
                    
    return x, y, dx, dy

def setWorkMode_secondRobot(work_mode, direction, s, φ, ds, dφ):
    if(work_mode == 1): #move: s, φ без движения  
        if direction == 1: # dir: 1 выдвижение
            s += ds
        elif direction == 2: # dir: 2 задвижение 
            s -= ds
    elif(work_mode == 2): #rotation: φ, s без движения  
        if direction == 1: # dir: φ 1 up
            φ += dφ
        elif direction == 2: # dir: φ 2 down
            φ -= dφ
    elif(work_mode == 3): #move: s,rotation: φ
        if direction == 11: # dir: s 1 выдвижение, rotation: φ 1 up
            s += ds
            φ += dφ
        elif direction == 22: # dir: s 2 задвижение, rotation: φ 2 down
            s -= ds
            φ -= dφ
        if direction == 12: # dir: s 1 выдвижение, rotation: φ 2 down
            s += ds
            φ -= dφ
        elif direction == 21: # dir: s 2 задвижение, rotation: φ 1 up
            s -= ds
            φ += dφ
    
    return s, φ


print('Выберите схему, 1 или 2')
continue_ = True
while continue_:
    scheme = input('нажмите 1 или 2 ')
    if scheme.isdigit():
        ##if scheme.isdigit() and (scheme == 1 or scheme == 2):
        scheme = int(scheme)
        if (scheme == 1 or scheme == 2):
            if scheme == 1:
                runFirstScheme()

            elif scheme == 2:
                runSecondScheme()
                       
            
            q = input('Продолжить?: y/n ')
            if q == 'n':
                continue_ = False
                #flag = False
                break

            print('---------------------------------------------')
    else:
        print('Некорректный ввод, только 1 или 2')
print('выход... ')
