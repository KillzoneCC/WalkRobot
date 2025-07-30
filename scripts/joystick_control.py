#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import numpy as np
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

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
        self.motion_manager = MotionManager() # Инициализация MotionManager

        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        self.speed_mode = 1 # Робот начинается с нулевой скорости
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
        self.status = 'stop'
        self.update_height = False
        self.update_param = False

        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0

        time.sleep(0.2)

        self.gait_manager = GaitManager()

        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # ====== Переменные и подписка для функции Get Up ======
        self.robot_state = 'stand' # Исходное состояние робота
        self.lie_to_stand_action_name = 'lie_to_stand' # Действие для подъема лицом вниз
        self.recline_to_stand_action_name = 'recline_to_stand' # Действие для подъема лицом вверх
        self.imu_sub = rospy.Subscriber('/sensor/imu', Imu, self.imu_callback) # Подписка на данные IMU

        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Starting in Speed Mode: 0 (STOP)")

    def imu_callback(self, msg):
        # Эта функция определяет, лежит ли робот лицом вниз или вверх
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Проверяем положение робота на основе данных акселерометра
        # Важно: эти значения (9.8, -9.8, 0.0) являются идеальными.
        # Вам, возможно, потребуется немного скорректировать их и/или допуск (tolerance)
        # на основе фактических показаний IMU вашего робота в разных положениях.
        # Используйте 'rostopic echo /sensor/imu' для получения реальных значений.
        
        tolerance = 1.5 # Допуск. Можно увеличить, если детектирование нестабильно, или уменьшить для большей точности.

        # Робот лежит лицом вниз (prone)
        # В этом положении ось X (вперед/назад) направлена вверх, и акселерометр должен показывать +9.8 по X
        # Оси Y и Z должны быть близки к 0.
        if abs(ax - 9.8) < tolerance and abs(ay) < tolerance and abs(az) < tolerance:
            if self.robot_state != 'lie':
                rospy.loginfo(f"Robot state: Lying Face Down (prone) - IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            self.robot_state = 'lie'
        # Робот лежит лицом вверх (supine)
        # В этом положении ось X (вперед/назад) направлена вниз, и акселерометр должен показывать -9.8 по X
        # Оси Y и Z должны быть близки к 0.
        elif abs(ax + 9.8) < tolerance and abs(ay) < tolerance and abs(az) < tolerance:
            if self.robot_state != 'recline':
                rospy.loginfo(f"Robot state: Lying Face Up (supine) - IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            self.robot_state = 'recline'
        else:
            # Робот стоит
            # В этом положении ось Z (вверх/вниз) направлена вверх, и акселерометр должен показывать +9.8 по Z
            # Оси X и Y должны быть близки к 0.
            # Добавлено условие для стоячего положения, чтобы избежать ложных срабатываний
            if abs(ax) < tolerance and abs(ay) < tolerance and abs(az - 9.8) < tolerance:
                if self.robot_state != 'stand':
                    rospy.loginfo(f"Robot state: Standing - IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
                self.robot_state = 'stand'
            else:
                # Если ни одно из известных положений не определено, считаем, что состояние неизвестно
                if self.robot_state not in ['lie', 'recline', 'stand']:
                    rospy.loginfo(f"Robot state: Unknown (IMU: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f})")
                self.robot_state = 'unknown' # Добавлено для лучшей обработки неопределенных состояний


    def axes_callback(self, axes):
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            if self.status == 'move':
                self.gait_manager.stop()
            self.status = 'stop'
            self.update_param = False
        else:
            current_speed_settings = self.speed_params[self.speed_mode]

            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0

            self.period_time = list(current_speed_settings['period_time'])

            if axes['ly'] > 0.3:
                self.update_param = True
                self.x_move_amplitude = current_speed_settings['x_amp_base']
            elif axes['ly'] < -0.3:
                self.update_param = True
                self.x_move_amplitude = -current_speed_settings['x_amp_base']

            if axes['lx'] > 0.3:
                self.period_time[2] = 0.025
                self.update_param = True
                self.y_move_amplitude = 0.015
            elif axes['lx'] < -0.3:
                self.period_time[2] = 0.025
                self.update_param = True
                self.y_move_amplitude = -0.015

            if axes['rx'] > 0.3:
                self.update_param = True
                self.angle_move_amplitude = 8
            elif axes['rx'] < -0.3:
                self.update_param = True
                self.angle_move_amplitude = -8

        if self.update_param or (self.speed_mode > 0 and self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0)):
            self.gait_param = self.gait_manager.get_gait_param()
            
            self.gait_param['init_z_offset'] = self.init_z_offset

            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']
            else:
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)

        if self.status == 'stop' and (self.update_param or (self.speed_mode > 0 and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0))):
            self.status = 'move'
        elif self.status == 'move' and not self.update_param and self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0 and self.speed_mode > 0:
            self.status = 'stop'
            self.gait_manager.stop()
        elif self.speed_mode == 0 and self.status == 'move':
            self.gait_manager.stop()
            self.status = 'stop'

        self.update_param = False

    def callback(self, axes):
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            if axes['ry'] < -0.5:
                self.update_height = True
                self.init_z_offset += 0.005
                if self.init_z_offset > 0.06:
                    self.update_height = False
                    self.init_z_offset = 0.06
            elif axes['ry'] > 0.5:
                self.update_height = True
                self.init_z_offset += -0.005
                if self.init_z_offset < 0.025:
                    self.update_height = False
                    self.init_z_offset = 0.025

            if self.update_height:
                self.gait_param = self.gait_manager.get_gait_param()
                
                self.gait_param['body_height'] = self.init_z_offset

                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                if self.speed_mode != 0:
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = min(self.speed_mode + 1, 3)
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = max(self.speed_mode - 1, 0)
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'

    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        # Активация функции "Get up" по кнопке Circle (B)
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle button pressed. Current Robot state: {self.robot_state}")
            self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звуковое подтверждение

            # Останавливаем текущее движение перед выполнением действия подъема
            self.gait_manager.stop()
            self.status = 'stop'
            time.sleep(0.5) # Небольшая задержка перед запуском действия

            if self.robot_state == 'lie':
                # Робот лежит лицом вниз, выполняем действие 'lie_to_stand'
                rospy.loginfo("Executing 'lie_to_stand' action.")
                self.motion_manager.run_action(self.lie_to_stand_action_name)
                time.sleep(1.0) # Даем время роботу завершить действие
            elif self.robot_state == 'recline':
                # Робот лежит лицом вверх, выполняем действие 'recline_to_stand'
                rospy.loginfo("Executing 'recline_to_stand' action.")
                self.motion_manager.run_action(self.recline_to_stand_action_name)
                time.sleep(1.0) # Даем время роботу завершить действие
            elif self.robot_state == 'stand':
                rospy.loginfo("Robot is already standing. No 'Get up' action needed.")
            else:
                rospy.logwarn(f"Cannot perform 'Get up' action. Robot state is unknown: {self.robot_state}")
            
            # После подъема робот должен быть в стоячем положении, нет необходимости вызывать gait_manager.stand()
            # Если робот не встал полностью, возможно, понадобится ручное управление или повторный вызов.


    def triangle_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def start_callback(self, new_state):
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
        pass

    def hat_xr_callback(self, new_state):
        pass

    def hat_yd_callback(self, new_state):
        pass

    def hat_yu_callback(self, new_state):
        pass

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        self.callback(axes)

        for key, value in axes.items():
            if key != 'ry':
                if self.last_axes[key] != value:
                    axes_changed = True

        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))

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
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))