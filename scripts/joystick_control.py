#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import numpy as np # Используется для некоторых числовых операций, хотя для базовой логики IMU не строго обязателен
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu # Импорт Imu для получения данных с инерционного измерительного блока
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager # Импорт MotionManager для выполнения сложных действий

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState():
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)
        self.board = Board()
        self.motion_manager = MotionManager() # Инициализация MotionManager для доступа к действиям робота

        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        self.speed_mode = 1 # Робот начинается с нулевой скорости (или заданной по умолчанию)
        self.speed_params = {
            1: { # Скорость 1
                'period_time': [400, 0.2, 0.022],
                'x_amp_base': 0.01,
                'init_z_offset': 0.025, # Начальная высота (по умолчанию)
                'z_move_amplitude': 0.016
            },
            2: { # Скорость 2
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025, # Начальная высота (по умолчанию)
                'z_move_amplitude': 0.016
            },
            3: { # Скорость 3
                'period_time': [500.0, 0.22, 0.020],
                'x_amp_base': 0.02,
                'init_z_offset': 0.025, # Начальная высота (по умолчанию)
                'z_move_amplitude': 0.018
            }
        }

        # Инициализация текущих параметров движения.
        self.period_time = list(self.speed_params[1]['period_time'])
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        self.init_z_offset = self.speed_params[1]['init_z_offset']
        self.time_stamp_ry = 0
        self.count_stop = 0
        self.status = 'stop' # Текущий статус движения робота: 'stop' или 'move'
        self.update_height = False # Флаг для обновления высоты тела
        self.update_param = False  # Флаг для обновления параметров походки

        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0

        time.sleep(0.2) # Небольшая задержка для инициализации

        self.gait_manager = GaitManager() # Инициализация менеджера походки

        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # ====== Переменные и подписка для функции Get Up ======
        self.robot_state = 'stand' # Исходное состояние робота (предполагаем, что стоит)
        self.lie_to_stand_action_name = 'lie_to_stand' # Имя действия для подъема лицом вниз (из app_controller.py)
        self.recline_to_stand_action_name = 'recline_to_stand' # Имя действия для подъема лицом вверх (из app_controller.py)
        
        # ВАЖНО: Код УЖЕ подписывается на /sensor/imu.
        # Топик /sensor/imu должен быть запущен и публиковать данные IMU вашего робота.
        self.imu_sub = rospy.Subscriber('/sensor/imu', Imu, self.imu_callback) 

        # Добавим начальную остановку робота при запуске
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Starting in Speed Mode: 0 (STOP)")

    def imu_callback(self, msg):
        # Эта функция вызывается каждый раз, когда приходят новые данные с IMU.
        # Она определяет, лежит ли робот лицом вниз, лицом вверх или стоит.
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # --- ВАЖНО: НАСТРОЙТЕ ЭТИ ЗНАЧЕНИЯ ДЛЯ ВАШЕГО РОБОТА! ---
        # Вам нужно получить реальные показания IMU вашего робота
        # в каждом из этих положений, используя команду 'rostopic echo /sensor/imu' в терминале.
        #
        # Примерные значения, которые вы можете увидеть:
        # GRAVITY_VALUE = 9.81  # Стандартное значение гравитации. Возможно, для вашего IMU это 9.7 или 9.9.
        # tolerance = 0.5     # Допуск. Чем меньше, тем точнее, но может быть нестабильно. Чем больше, тем менее точно, но стабильнее.
        #                     # Начните с 0.5-1.0 и регулируйте, наблюдая за поведением робота.

        GRAVITY_VALUE = 9.8 # Замените на фактическое значение гравитации, которое измеряет ВАШ IMU
        tolerance = 1.0     # Настройте этот допуск, если робот некорректно определяет положение

        # Логика определения положения робота на основе показаний акселерометра:

        # Робот лежит лицом вниз (prone)
        # Если ось X акселерометра показывает значение, близкое к +GRAVITY_VALUE,
        # а Y и Z близки к 0.0 (робот лежит ровно на животе).
        if abs(ax - GRAVITY_VALUE) < tolerance and \
           abs(ay) < tolerance and \
           abs(az) < tolerance:
            if self.robot_state != 'lie':
                rospy.loginfo(f"Robot state: Lying Face Down (prone) - IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            self.robot_state = 'lie'
        # Робот лежит лицом вверх (supine)
        # Если ось X акселерометра показывает значение, близкое к -GRAVITY_VALUE,
        # а Y и Z близки к 0.0 (робот лежит ровно на спине).
        elif abs(ax + GRAVITY_VALUE) < tolerance and \
             abs(ay) < tolerance and \
             abs(az) < tolerance:
            if self.robot_state != 'recline':
                rospy.loginfo(f"Robot state: Lying Face Up (supine) - IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            self.robot_state = 'recline'
        # Робот стоит
        # Если оси X и Y акселерометра близки к 0.0,
        # а Z близка к +GRAVITY_VALUE (робот стоит вертикально).
        elif abs(ax) < tolerance and \
             abs(ay) < tolerance and \
             abs(az - GRAVITY_VALUE) < tolerance:
            if self.robot_state != 'stand':
                rospy.loginfo(f"Robot state: Standing - IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            self.robot_state = 'stand'
        else:
            # Если ни одно из известных положений не определено, например, робот лежит на боку
            # или находится в процессе падения/подъема.
            if self.robot_state not in ['lie', 'recline', 'stand', 'unknown']: # Логируем только при изменении в неизвестное
                rospy.logwarn(f"Robot state: Unknown (IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f})")
            self.robot_state = 'unknown'


    def axes_callback(self, axes):
        # Если робот находится в режиме остановки (speed_mode = 0),
        # сбрасываем все амплитуды движения в ноль.
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            # Если робот был в движении и переключились на 0 скорость, останавливаем его.
            if self.status == 'move':
                self.gait_manager.stop()
            self.status = 'stop'
            self.update_param = False
        else: # Если робот в режиме скорости 1, 2 или 3
            current_speed_settings = self.speed_params[self.speed_mode]

            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0

            # Обновляем period_time из текущих настроек скорости
            self.period_time = list(current_speed_settings['period_time'])

            # Управление движением вперед/назад по оси LY
            if axes['ly'] > 0.3: # Движение вперед
                self.update_param = True
                self.x_move_amplitude = current_speed_settings['x_amp_base']
            elif axes['ly'] < -0.3: # Движение назад
                self.update_param = True
                self.x_move_amplitude = -current_speed_settings['x_amp_base']

            # Управление движением влево/вправо по оси LX
            if axes['lx'] > 0.3: # Движение вправо
                self.period_time[2] = 0.025 # Параметр для движения вбок
                self.update_param = True
                self.y_move_amplitude = 0.015
            elif axes['lx'] < -0.3: # Движение влево
                self.period_time[2] = 0.025 # Параметр для движения вбок
                self.update_param = True
                self.y_move_amplitude = -0.015

            # Управление поворотом по оси RX
            if axes['rx'] > 0.3: # Поворот вправо
                self.update_param = True
                self.angle_move_amplitude = 8
            elif axes['rx'] < -0.3: # Поворот влево
                self.update_param = True
                self.angle_move_amplitude = -8

        # === Логика запуска движения ===
        # Запускаем движение, если:
        # 1. Есть команда обновления параметров (изменился стик) ИЛИ
        # 2. Робот стоял, перешел в режим движения (speed_mode > 0) и получает команду движения
        if self.update_param or (self.speed_mode > 0 and self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0)):
            self.gait_param = self.gait_manager.get_gait_param()
            self.gait_param['init_z_offset'] = self.init_z_offset

            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']
            else:
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)

        # Логика для перехода между состоянием 'stop' и 'move'
        if self.status == 'stop' and (self.update_param or (self.speed_mode > 0 and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0))):
            self.status = 'move'
        elif self.status == 'move' and not self.update_param and self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0 and self.speed_mode > 0:
            # Если робот был в движении, но все амплитуды стали 0 (и не режим 0) - останавливаем.
            self.status = 'stop'
            self.gait_manager.stop()
        elif self.speed_mode == 0 and self.status == 'move':
            # Если переключились на 0 скорость, и он двигался
            self.gait_manager.stop()
            self.status = 'stop'

        self.update_param = False # Сбрасываем флаг обновления параметров

    def callback(self, axes):
        # Проверяем временную метку, чтобы не обновлять высоту слишком часто
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            # Управление высотой тела робота (ось RY)
            if axes['ry'] < -0.5: # Поднять робота
                self.update_height = True
                self.init_z_offset += 0.005
                if self.init_z_offset > 0.06: # Максимальная высота
                    self.update_height = False
                    self.init_z_offset = 0.06
            elif axes['ry'] > 0.5: # Опустить робота
                self.update_height = True
                self.init_z_offset += -0.005
                if self.init_z_offset < 0.025: # Минимальная высота
                    self.update_height = False
                    self.init_z_offset = 0.025

            # Если высота тела изменилась
            if self.update_height:
                self.gait_param = self.gait_manager.get_gait_param()
                self.gait_param['body_height'] = self.init_z_offset

                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                if self.speed_mode != 0:
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 # Задержка для плавности

    def select_callback(self, new_state):
        pass # Не используется

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Переключение скорости по возрастанию
            self.speed_mode = min(self.speed_mode + 1, 3) # Ограничиваем максимумом 3 (Скорость 3)
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1) # Звуковое подтверждение
            self.gait_manager.stop() # Остановка при смене скорости
            self.status = 'stop'

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Переключение скорости по убыванию
            self.speed_mode = max(self.speed_mode - 1, 0) # Ограничиваем минимумом 0 (СТОП)
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1) # Звуковое подтверждение
            self.gait_manager.stop() # Остановка при смене скорости
            self.status = 'stop'

    def l2_callback(self, new_state):
        pass # Не используется

    def r2_callback(self, new_state):
        pass # Не используется

    def square_callback(self, new_state):
        pass # Не используется

    # ====== Функции Get Up, вызываемые кнопкой Circle ======
    def circle_callback(self, new_state):
        # Активация функции "Get up" по кнопке Circle (соответствует кнопке B на некоторых джойстиках)
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle button pressed. Current Robot state: {self.robot_state}")
            self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звуковое подтверждение

            # Останавливаем любое текущее движение перед выполнением действия подъема
            self.gait_manager.stop()
            self.status = 'stop'
            time.sleep(0.5) # Небольшая задержка для стабилизации перед запуском действия подъема

            if self.robot_state == 'lie':
                # Робот лежит лицом вниз, выполняем действие 'lie_to_stand'
                rospy.loginfo("Executing 'lie_to_stand' action.")
                self.motion_manager.run_action(self.lie_to_stand_action_name)
                time.sleep(1.0) # Даем время роботу завершить действие подъема
            elif self.robot_state == 'recline':
                # Робот лежит лицом вверх, выполняем действие 'recline_to_stand'
                rospy.loginfo("Executing 'recline_to_stand' action.")
                self.motion_manager.run_action(self.recline_to_stand_action_name)
                time.sleep(1.0) # Даем время роботу завершить действие подъема
            elif self.robot_state == 'stand':
                rospy.loginfo("Robot is already standing. No 'Get up' action needed.")
            else:
                rospy.logwarn(f"Cannot perform 'Get up' action. Robot state is unknown: {self.robot_state}. Please ensure IMU is calibrated.")
            
            # После выполнения действия 'run_action' предполагается, что робот уже находится
            # в стоячем положении, поэтому нет необходимости вызывать gait_manager.stand()
            # Если робот не встал полностью, это может указывать на проблему с действием
            # или с калибровкой IMU.


    def triangle_callback(self, new_state):
        pass # Не используется

    def cross_callback(self, new_state):
        pass # Не используется

    def start_callback(self, new_state):
        # Сброс высоты тела к начальной (как в старом коде)
        if new_state == ButtonState.Pressed:
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            self.gait_param = self.gait_manager.get_gait_param()
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                for i in range(t):
                    self.init_z_offset += 0.005 * abs(0.025 - self.init_z_offset) / (0.025 - self.init_z_offset)
                    self.gait_param['body_height'] = self.init_z_offset

                    current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                    if self.speed_mode != 0:
                        self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                    else:
                        self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                    self.gait_manager.update_param(current_period_time_for_update, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
                    time.sleep(0.05)

    def hat_xl_callback(self, new_state):
        pass # Не используется

    def hat_xr_callback(self, new_state):
        pass # Не используется

    def hat_yd_callback(self, new_state):
        pass # Не используется

    def hat_yu_callback(self, new_state):
        pass # Не используется

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        # Обработка высоты тела (независимо от режима скорости)
        self.callback(axes)

        # Проверяем, изменились ли значения осей (кроме 'ry', которая используется для высоты)
        for key, value in axes.items():
            if key != 'ry':
                if self.last_axes[key] != value:
                    axes_changed = True

        # Если оси изменились, вызываем axes_callback для обработки движения
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))

        # Обработка состояний кнопок
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal

            callback = "".join([key, '_callback'])

            if new_state != ButtonState.Normal:
                if hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin() # Запускает основной цикл ROS, который обрабатывает колбэки
    except Exception as e:
        rospy.logerr(str(e))