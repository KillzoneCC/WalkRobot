#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy
from ainex_kinematics.gait_manager import GaitManager

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
        
        # ====== НОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        self.speed_mode = 0 # 0 - остановка, 1 - скорость 1, 2 - скорость 2, 3 - скорость 3

        # Параметры для каждой скорости
        self.speed_params = {
            1: { # Скорость 1 (Начальные значения)
                'period_time': [400, 0.2, 0.022],
                'x_amp_base': 0.01,
                # УДАЛЕНО: 'y_amp_base' и 'angle_amp_base'
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            2: { # Скорость 2 (По фото 2.png)
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                # УДАЛЕНО: 'y_amp_base' и 'angle_amp_base'
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            3: { # Скорость 3 (По фото 3.png)
                'period_time': [500, 0.22, 0.022],
                'x_amp_base': 0.028,
                # УДАЛЕНО: 'y_amp_base' и 'angle_amp_base'
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018 
            }
        }
        # ===================================================

        # Инициализация текущих параметров (начинаем с остановки)
        self.period_time = self.speed_params[1]['period_time'] # По умолчанию берем параметры Скорости 1
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        self.init_z_offset = self.speed_params[1]['init_z_offset'] # По умолчанию берем параметры Скорости 1

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
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def axes_callback(self, axes):
        # Если режим скорости 0 (остановка), то робот не должен двигаться
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            self.period_time = list(self.speed_params[1]['period_time']) # Сброс к базовым для скорости 1
            if self.status == 'move': # Если робот двигался и переключились на остановку
                self.gait_manager.stop()
                self.status = 'stop'
            self.update_param = False # Убедиться, что не обновляем параметры, если в режиме остановки
            return # Выходим из функции, если в режиме остановки

        # Получаем параметры для текущего режима скорости
        current_speed_settings = self.speed_params[self.speed_mode]
        
        # Сброс амплитуд движения и параметров времени по умолчанию для текущей скорости
        self.x_move_amplitude = 0.0
        self.angle_move_amplitude = 0.0
        self.y_move_amplitude = 0.0
        self.period_time = list(current_speed_settings['period_time']) # Создаем копию списка

        # Управление движением вперед/назад по оси LY (левый стик по Y)
        if axes['ly'] > 0.3: 
            self.update_param = True
            self.x_move_amplitude = current_speed_settings['x_amp_base']
        elif axes['ly'] < -0.3:
            self.update_param = True
            self.x_move_amplitude = -current_speed_settings['x_amp_base']

        # Управление движением влево/вправо по оси LX (левый стик по X)
        if axes['lx'] > 0.3: 
            self.period_time[2] = 0.025 # Это значение для приземления ноги, оставляем его фиксированным для бокового шага
            self.update_param = True
            # ИЗМЕНЕНО: y_move_amplitude возвращено к фиксированному значению 0.015
            self.y_move_amplitude = 0.015 
        elif axes['lx'] < -0.3: 
            self.period_time[2] = 0.025
            self.update_param = True
            # ИЗМЕНЕНО: y_move_amplitude возвращено к фиксированному значению -0.015
            self.y_move_amplitude = -0.015 

        # Управление поворотом по оси RX (правый стик по X)
        if axes['rx'] > 0.3: 
            self.update_param = True
            # ИЗМЕНЕНО: angle_move_amplitude возвращено к фиксированному значению 8
            self.angle_move_amplitude = 8 
        elif axes['rx'] < -0.3: 
            self.update_param = True
            # ИЗМЕНЕНО: angle_move_amplitude возвращено к фиксированному значению -8
            self.angle_move_amplitude = -8 

        # Если параметры походки нуждаются в обновлении
        if self.update_param:
            self.gait_param = self.gait_manager.get_gait_param() 
            self.gait_param['init_z_offset'] = self.init_z_offset 
            self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude'] # Устанавливаем высоту подъема ноги
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            
        if self.status == 'stop' and self.update_param: 
            self.status = 'move' 
        elif self.status == 'move' and not self.update_param: 
            self.status = 'stop' 
            self.gait_manager.stop() 
        self.update_param = False

    def callback(self, axes):
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            # Высота тела управляется независимо от скорости
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
            if self.update_height and not self.update_param:
                self.gait_param = self.gait_manager.get_gait_param()
                self.gait_param['body_height'] = self.init_z_offset
                # Здесь также нужно обновить z_move_amplitude, если высота тела меняется
                if self.speed_mode != 0: # Только если не в режиме остановки
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else: # Если в режиме остановки, использовать дефолтное значение
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                self.gait_manager.update_param(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        pass

    # ====== ОБРАБОТЧИКИ КНОПОК ДЛЯ ПЕРЕКЛЮЧЕНИЯ СКОРОСТИ ======
    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = 1 # Устанавливаем Скорость 1
            rospy.loginfo("Speed Mode: 1 (Original)")
            self.board.set_buzzer(1000, 0.05, 0.02, 1) # Короткий звуковой сигнал
            # При смене режима скорости сбрасываем состояние движения
            self.gait_manager.stop()
            self.status = 'stop'

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = 2 # Устанавливаем Скорость 2
            rospy.loginfo("Speed Mode: 2 (From 2.png)")
            self.board.set_buzzer(1500, 0.05, 0.02, 1) # Короткий звуковой сигнал
            # При смене режима скорости сбрасываем состояние движения
            self.gait_manager.stop()
            self.status = 'stop'

    def triangle_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = 3 # Устанавливаем Скорость 3
            rospy.loginfo("Speed Mode: 3 (From 3.png)")
            self.board.set_buzzer(2000, 0.05, 0.02, 1) # Короткий звуковой сигнал
            # При смене режима скорости сбрасываем состояние движения
            self.gait_manager.stop()
            self.status = 'stop'

    def cross_callback(self, new_state): # Используем Cross (X) для остановки
        if new_state == ButtonState.Pressed:
            self.speed_mode = 0 # Устанавливаем режим остановки
            rospy.loginfo("Speed Mode: 0 (STOP)")
            self.board.set_buzzer(500, 0.1, 0.05, 2) # Двойной звуковой сигнал
            self.gait_manager.stop()
            self.status = 'stop'
    # ======================================================================

    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
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
                    # Убедимся, что z_move_amplitude обновляется здесь тоже
                    if self.speed_mode != 0:
                        self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                    else:
                        self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] # Используем дефолтное

                    self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
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
                if  hasattr(self, callback):
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
