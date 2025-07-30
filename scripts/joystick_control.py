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
    # Определим максимальную амплитуду движения вперед, чтобы избежать "перебора".
    # ЭТО ЗНАЧЕНИЕ НУЖНО НАСТРОИТЬ В ЗАВИСИМОСТИ ОТ ТОГО, ПРИ КАКОМ ЗНАЧЕНИИ ПРОИСХОДИТ ОШИБКА "ПЕРЕБОР".
    MAX_FORWARD_AMPLITUDE = 0.025 # Примерное значение, возможно, 0.02, 0.03 и т.д.
                                 # Пожалуйста, замените это значение на то, которое вызывает "перебор".
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)
        self.board = Board()
        # Исходные значения, будут переопределены режимами скорости
        self.period_time = [400, 0.2, 0.022]
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.time_stamp_ry = 0
        self.angle_move_amplitude = 0
        self.init_z_offset = 0.025
        self.count_stop = 0
        self.status = 'stop'
        self.update_height = False
        self.update_param = False
        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))

        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        # speed_mode: Отслеживает текущий режим скорости.
        # 0 - остановка, 1 - Скорость 1, 2 - Скорость 2, 3 - Скорость 3.
        self.speed_mode = 1 # Робот начинается с 1 скорости, как и запрошено

        # speed_params: Словарь, содержащий все параметры для каждого режима скорости.
        self.speed_params = {
            0: { # Остановка
                'period_time': [400, 0.2, 0.022],
                'x_amp_base': 0.0, # Амплитуда движения по X для остановки
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016
            },
            1: { # Скорость 1
                'period_time': [400, 0.2, 0.022],
                'x_amp_base': 0.01,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016
            },
            2: { # Скорость 2
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016
            },
            3: { # Скорость 3 (Максимальная скорость)
                # Важно: 'x_amp_base' для 3 скорости не должен превышать MAX_FORWARD_AMPLITUDE
                'period_time': [500.0, 0.22, 0.020],
                'x_amp_base': 0.025, # Установил, чтобы не превышало MAX_FORWARD_AMPLITUDE
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018,
                'init_x_offset': 0.0,
                'init_y_offset': -0.005,
                'step_fb_ratio': 0.030,
                'init_roll_offset': 0.0,
                'angle_move_amplitude': 0.0,
                'init_pitch_offset': 0.0,
                'z_swap_amplitude': 0.006,
                'arm_swing_gain': 2.0,
                'init_yaw_offset': 0.0,
                'pelvis_offset': 5.0,
                'hip_pitch_offset': 15.0
            }
        }

        time.sleep(0.2)

        # 机器人步态库调用
        # ЭТУ СТРОКУ ПЕРЕНЕСЛИ ВЫШЕ
        self.gait_manager = GaitManager()

        # Робот начинается с первой скорости, поэтому применяем параметры
        # ТЕПЕРЬ МОЖНО БЕЗОПАСНО ВЫЗЫВАТЬ _apply_speed_params()
        self._apply_speed_params()

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def _apply_speed_params(self):
        """Применяет параметры текущего режима скорости, соблюдая максимальную амплитуду."""
        params = self.speed_params[self.speed_mode]
        self.period_time = params['period_time']
        
        # Проверяем, чтобы базовая амплитуда не превышала MAX_FORWARD_AMPLITUDE
        if abs(params['x_amp_base']) > self.MAX_FORWARD_AMPLITUDE:
            rospy.logwarn(f"Предупреждение: Базовая амплитуда для скорости {self.speed_mode} ({params['x_amp_base']}) превышает MAX_FORWARD_AMPLITUDE ({self.MAX_FORWARD_AMPLITUDE}). Ограничиваем.")
            self.base_x_move_amplitude = self.MAX_FORWARD_AMPLITUDE * (1 if params['x_amp_base'] > 0 else -1)
        else:
            self.base_x_move_amplitude = params['x_amp_base']

        self.init_z_offset = params['init_z_offset']

        # Если робот останавливается (speed_mode == 0), то gait_manager.stop()
        if self.speed_mode == 0:
            self.gait_manager.stop()
            self.status = 'stop'
            rospy.loginfo("Робот остановлен (скорость 0).")
        else:
            # Применяем все параметры, если они есть в speed_params[self.speed_mode]
            current_gait_param = self.gait_manager.get_gait_param()
            for key, value in params.items():
                if key not in ['period_time', 'x_amp_base']: # Эти параметры обрабатываются отдельно
                    current_gait_param[key] = value
            self.gait_param = current_gait_param # Обновляем gait_param
            rospy.loginfo(f"Применена скорость: {self.speed_mode}, Базовая амплитуда X: {self.base_x_move_amplitude}")

    def axes_callback(self, axes):
        self.x_move_amplitude = 0.00
        self.angle_move_amplitude = 0.00
        self.y_move_amplitude = 0.00
        
        # Управление движением только если робот не остановлен (speed_mode > 0)
        if self.speed_mode > 0:
            if axes['ly'] > 0.3:
                self.update_param = True
                self.x_move_amplitude = self.base_x_move_amplitude
            elif axes['ly'] < -0.3:
                self.update_param = True
                self.x_move_amplitude = -self.base_x_move_amplitude

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
        else: # Если speed_mode == 0, робот должен стоять
            self.x_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0

        if self.update_param:
            self.gait_param = self.gait_manager.get_gait_param()
            self.gait_param['init_z_offset'] = self.init_z_offset
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            if self.status == 'stop':
                self.status = 'move'
        elif self.status == 'move' and not self.update_param: # Если джойстик в нейтральном положении и робот двигался
            self.status = 'stop'
            self.gait_manager.stop()
        self.update_param = False

    def callback(self, axes):
        # Логика изменения высоты тела (RY)
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
            if self.update_height and not self.update_param:
                self.gait_param = self.gait_manager.get_gait_param()
                self.gait_param['body_height'] = self.init_z_offset
                self.gait_manager.update_param(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode > 0: # Уменьшаем скорость, но не ниже 0
                self.speed_mode -= 1
                self._apply_speed_params()
                rospy.loginfo(f"Скорость уменьшена до: {self.speed_mode}")
            else:
                rospy.loginfo("Уже на минимальной скорости (0).")


    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode < 3: # Увеличиваем скорость, но не выше 3
                self.speed_mode += 1
                self._apply_speed_params()
                rospy.loginfo(f"Скорость увеличена до: {self.speed_mode}")
            else:
                rospy.loginfo("Уже на максимальной скорости (3).")

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        pass

    def triangle_callback(self, new_state):
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
        for key, value in axes.items(): # 轴的值被改变(the value of the axis is changed)
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
                new_state