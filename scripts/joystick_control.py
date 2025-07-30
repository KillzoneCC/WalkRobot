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
    # Если 'x_amp_base' для Скорости 3 вызывает "перебор", уменьшите его или увеличьте MAX_FORWARD_AMPLITUDE
    # исходя из тестов. Здесь я оставил 0.028, как в вашем примере для скорости 3.
    MAX_FORWARD_AMPLITUDE = 0.028 # Важно: Настройте это значение по результатам тестов.
                                 # Если 0.028 вызывает перебор, уменьшите его до 0.027 или ниже.

    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)
        self.board = Board()
        
        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        # speed_mode: Отслеживает текущий режим скорости.
        # 0 - остановка, 1 - Скорость 1, 2 - Скорость 2, 3 - Скорость 3.
        self.speed_mode = 1 # Робот **начинается с первой скорости**, как запрошено

        # speed_params: Словарь, содержащий все параметры для каждого режима скорости.
        # Для скоростей 0, 1, 2 оставляем только базовые параметры, чтобы избежать изменения позы
        # Для скорости 3 сохраняем все специфические параметры.
        self.speed_params = {
            0: { # Скорость 0 (Остановка)
                'period_time': [400, 0.2, 0.022], 
                'x_amp_base': 0.0, 
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
            3: { # Скорость 3 (Все специфические параметры для этой походки)
                'period_time': [500, 0.22, 0.022], 
                'x_amp_base': 0.028, 
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
        # ===================================================

        # Инициализация текущих параметров движения.
        self.period_time = list(self.speed_params[self.speed_mode]['period_time']) 
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        self.init_z_offset = self.speed_params[self.speed_mode]['init_z_offset'] 

        self.time_stamp_ry = 0
        self.count_stop = 0 
        self.status = 'stop' 
        self.update_height = False 
        self.update_param = False 
        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0 

        time.sleep(0.2) 

        # Инициализация менеджера походки робота
        self.gait_manager = GaitManager()
        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Применяем параметры начальной скорости (1) и гарантируем остановку
        self._apply_speed_params() 
        self.gait_manager.stop() 
        rospy.loginfo(f"JoystickController initialized. Starting in Speed Mode: {self.speed_mode}")

    def _apply_speed_params(self):
        """
        Применяет параметры текущего режима скорости.
        Учитывает MAX_FORWARD_AMPLITUDE для предотвращения "перебора".
        Игнорирует 'позиционные' параметры для скоростей 0, 1, 2, чтобы не менять позу.
        """
        params_key = self.speed_mode 
        current_speed_settings = self.speed_params[params_key]

        self.period_time = list(current_speed_settings['period_time'])
        
        requested_x_amp_base = current_speed_settings['x_amp_base']
        if abs(requested_x_amp_base) > self.MAX_FORWARD_AMPLITUDE:
            rospy.logwarn(f"ВНИМАНИЕ: Запрошенная базовая амплитуда X для скорости {self.speed_mode} ({requested_x_amp_base}) превышает MAX_FORWARD_AMPLITUDE ({self.MAX_FORWARD_AMPLITUDE}). Ограничиваем.")
            self.base_x_move_amplitude = self.MAX_FORWARD_AMPLITUDE * (1 if requested_x_amp_base > 0 else -1)
        else:
            self.base_x_move_amplitude = requested_x_amp_base

        self.init_z_offset = current_speed_settings['init_z_offset']

        if hasattr(self, 'gait_manager') and self.gait_manager is not None:
            self.gait_param = self.gait_manager.get_gait_param()
            self.gait_param['init_z_offset'] = self.init_z_offset
            self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']

            # Применяем специфические параметры только для скорости 3
            if self.speed_mode == 3:
                for key, value in current_speed_settings.items():
                    # Исключаем те, которые мы уже обработали или которые не относятся к gait_param напрямую
                    # А также исключаем параметры, которые могут изменить базовую позу (angle_move_amplitude здесь 0.0)
                    if key not in ['period_time', 'x_amp_base', 'init_z_offset', 'z_move_amplitude']:
                        self.gait_param[key] = value
            else:
                # Для скоростей 0, 1, 2 явно сбрасываем параметры, которые могут повлиять на позу,
                # если они были установлены ранее (например, при переходе с скорости 3)
                # Убедимся, что робот не меняет свою позу в режимах 0, 1, 2
                self.gait_param['init_x_offset'] = 0.0
                self.gait_param['init_y_offset'] = 0.0
                self.gait_param['step_fb_ratio'] = 0.0 # Сброс, если он влиял на позу
                self.gait_param['init_roll_offset'] = 0.0
                self.gait_param['init_pitch_offset'] = 0.0
                self.gait_param['z_swap_amplitude'] = 0.0 # Сброс, чтобы не влиял на позу стоя
                self.gait_param['arm_swing_gain'] = 0.0 # Сброс, чтобы не влиял на позу стоя
                self.gait_param['init_yaw_offset'] = 0.0
                self.gait_param['pelvis_offset'] = 0.0
                self.gait_param['hip_pitch_offset'] = 0.0

            if self.speed_mode == 0:
                self.gait_manager.stop()
                self.status = 'stop'
                self.x_move_amplitude = 0.0 
                self.y_move_amplitude = 0.0
                self.angle_move_amplitude = 0.0
                rospy.loginfo(f"Скорость переключена на: {self.speed_mode} (ОСТАНОВКА)")
            else:
                rospy.loginfo(f"Скорость переключена на: {self.speed_mode}, Базовая амплитуда X: {self.base_x_move_amplitude}")
                # Для скоростей 1, 2, 3 просто обновляем параметры, не запуская движение
                self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=0) 


    def axes_callback(self, axes):
        self.update_param = False 

        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            if self.status == 'move': 
                self.gait_manager.stop() 
                self.status = 'stop'
            return 
            
        else: # Если робот в режиме скорости 1, 2 или 3
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            self.period_time = list(self.speed_params[self.speed_mode]['period_time']) 

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
        
        if self.update_param:
            self.gait_param = self.gait_manager.get_gait_param() 
            self.gait_param['init_z_offset'] = self.init_z_offset 
            self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude'] 

            # Применяем специфические параметры gait_manager только для скорости 3, 
            # когда робот действительно начинает двигаться.
            if self.speed_mode == 3:
                current_speed_settings = self.speed_params[self.speed_mode]
                for key, value in current_speed_settings.items():
                    if key not in ['period_time', 'x_amp_base', 'init_z_offset', 'z_move_amplitude', 'angle_move_amplitude']:
                        self.gait_param[key] = value

            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            
            if self.status == 'stop': 
                self.status = 'move' 
        elif self.status == 'move': 
            if self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0:
                self.status = 'stop'
                self.gait_manager.stop() 


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
                
                if self.speed_mode != 0: 
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else: 
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] 

                # При изменении высоты, робот не должен двигаться по осям X, Y, Angle, 
                # поэтому передаем 0.0 для этих амплитуд.
                self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 

    def select_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode < 3:
                self.speed_mode += 1
            
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1) 
            self._apply_speed_params() 

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode > 0:
                self.speed_mode -= 1
            
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1) 
            self._apply_speed_params() 

    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state): 
        pass

    def circle_callback(self, new_state):
        pass

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
                    
                    if self.speed_mode != 0:
                        self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                    else:
                        self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] 

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