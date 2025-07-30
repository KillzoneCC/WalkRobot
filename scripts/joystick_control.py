#!/usr/bin/env python3
# encoding: utf:8
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
        
        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        # speed_mode: Отслеживает текущий режим скорости.
        # 0 - остановка, 1 - Скорость 1, 2 - Скорость 2, 3 - Скорость 3.
        self.speed_mode = 0 # Робот начинается с нулевой скорости, как и запрошено

        # speed_params: Словарь, содержащий все параметры для каждого режима скорости.
        self.speed_params = {
            1: { # Скорость 1
                'period_time': [400, 0.2, 0.022],  
                'x_amp_base': 0.01,
                'init_z_offset': 0.025, # Начальная высота тела для этой скорости
                'z_move_amplitude': 0.016  
            },
            2: { # Скорость 2
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025, # Начальная высота тела для этой скорости
                'z_move_amplitude': 0.016  
            },
            3: { # Скорость 3
                'period_time': [500.0, 0.22, 0.020], 
                'x_amp_base': 0.02,                   
                'init_z_offset': 0.025,               # Начальная высота тела для этой скорости
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
        self.period_time = list(self.speed_params[1]['period_time']) 
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        # Устанавливаем начальное смещение по Z (высоту тела) из параметров скорости 1
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
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Добавим начальную остановку робота при запуске и установку начальной позы
        self.gait_manager.stop()
        # Применяем начальную высоту тела
        initial_gait_param = self.gait_manager.get_gait_param()
        initial_gait_param['body_height'] = self.init_z_offset
        initial_gait_param['init_z_offset'] = self.init_z_offset # Убедимся, что init_z_offset тоже установлен
        initial_gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] # Используем базовую z_move_amplitude
        self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, initial_gait_param, step_num=0)
        
        rospy.loginfo("JoystickController initialized. Starting in Speed Mode: 0 (STOP) at initial pose.")


    # Метод для обработки входных данных с осей джойстика (движение, поворот)
    def axes_callback(self, axes):
        x_move_amplitude_changed = False
        y_move_amplitude_changed = False
        angle_move_amplitude_changed = False

        prev_x_amp = self.x_move_amplitude
        prev_y_amp = self.y_move_amplitude
        prev_angle_amp = self.angle_move_amplitude

        # Если робот находится в режиме остановки (speed_mode = 0),
        # сбрасываем все амплитуды движения в ноль.
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            if self.status == 'move':  
                self.gait_manager.stop() 
            self.status = 'stop'
            self.update_param = False 
            
        else: # Если робот в режиме скорости 1, 2 или 3
            current_speed_settings = self.speed_params[self.speed_mode]
            
            # Временно сбрасываем амплитуды, чтобы определить изменения
            temp_x_amp = 0.0
            temp_y_amp = 0.0
            temp_angle_amp = 0.0

            # Управление движением вперед/назад по оси LY (левый стик по Y)
            if axes['ly'] > 0.3: 
                temp_x_amp = current_speed_settings['x_amp_base'] 
            elif axes['ly'] < -0.3: 
                temp_x_amp = -current_speed_settings['x_amp_base']

            # Управление движением влево/вправо по оси LX (левый стик по X)
            if axes['lx'] > 0.3: 
                self.period_time[2] = 0.025 
                temp_y_amp = 0.015 
            elif axes['lx'] < -0.3: 
                self.period_time[2] = 0.025
                temp_y_amp = -0.015 

            # Управление поворотом по оси RX (правый стик по X)
            if axes['rx'] > 0.3: 
                temp_angle_amp = 8 
            elif axes['rx'] < -0.3: 
                temp_angle_amp = -8 
            
            # Проверяем, изменились ли амплитуды
            x_move_amplitude_changed = (self.x_move_amplitude != temp_x_amp)
            y_move_amplitude_changed = (self.y_move_amplitude != temp_y_amp)
            angle_move_amplitude_changed = (self.angle_move_amplitude != temp_angle_amp)

            # Обновляем фактические амплитуды
            self.x_move_amplitude = temp_x_amp
            self.y_move_amplitude = temp_y_amp
            self.angle_move_amplitude = temp_angle_amp

            # Определяем, нужно ли обновлять параметры
            if x_move_amplitude_changed or y_move_amplitude_changed or angle_move_amplitude_changed:
                self.update_param = True
            else:
                self.update_param = False


        # Если флаг update_param установлен (т.е. были изменения в движении или повороте)
        # ИЛИ если мы вышли из режима остановки (speed_mode > 0) и хотим начать движение
        # ИЛИ если мы изменили скорость, и робот был в движении, то надо обновить параметры
        if self.update_param or \
           (self.speed_mode > 0 and self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0)) or \
           (self.speed_mode > 0 and self.status == 'move' and (prev_x_amp != self.x_move_amplitude or prev_y_amp != self.y_move_amplitude or prev_angle_amp != self.angle_move_amplitude)):

            self.gait_param = self.gait_manager.get_gait_param() 
            
            # === ГЛАВНОЕ ИСПРАВЛЕНИЕ ЗДЕСЬ ===
            # Убеждаемся, что body_height всегда равен init_z_offset
            self.gait_param['init_z_offset'] = self.init_z_offset 
            self.gait_param['body_height'] = self.init_z_offset 
            # =================================

            # Устанавливаем высоту подъема ноги из параметров текущей скорости, если режим > 0
            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude'] 
                # Дополнительно обновляем другие параметры, если они существуют для текущей скорости
                self.gait_param['init_x_offset'] = self.speed_params[self.speed_mode].get('init_x_offset', 0.0)
                self.gait_param['init_y_offset'] = self.speed_params[self.speed_mode].get('init_y_offset', 0.0)
                self.gait_param['step_fb_ratio'] = self.speed_params[self.speed_mode].get('step_fb_ratio', 0.0)
                self.gait_param['init_roll_offset'] = self.speed_params[self.speed_mode].get('init_roll_offset', 0.0)
                self.gait_param['init_pitch_offset'] = self.speed_params[self.speed_mode].get('init_pitch_offset', 0.0)
                self.gait_param['z_swap_amplitude'] = self.speed_params[self.speed_mode].get('z_swap_amplitude', 0.0)
                self.gait_param['arm_swing_gain'] = self.speed_params[self.speed_mode].get('arm_swing_gain', 0.0)
                self.gait_param['init_yaw_offset'] = self.speed_params[self.speed_mode].get('init_yaw_offset', 0.0)
                self.gait_param['pelvis_offset'] = self.speed_params[self.speed_mode].get('pelvis_offset', 0.0)
                self.gait_param['hip_pitch_offset'] = self.speed_params[self.speed_mode].get('hip_pitch_offset', 0.0)

            else: # Если speed_mode == 0, используем z_move_amplitude от Скорости 1 как базовое
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']
                # init_z_offset и body_height уже установлены выше для всех случаев.

            # Отправляем обновленные параметры движения в GaitManager
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            
        # Логика для перехода между состоянием 'stop' и 'move'
        if self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0) and self.speed_mode > 0: 
            self.status = 'move' 
        elif self.status == 'move' and self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0 and self.speed_mode > 0:
            self.status = 'stop'
            self.gait_manager.stop()
        elif self.speed_mode == 0 and self.status == 'move': 
            self.gait_manager.stop()
            self.status = 'stop'

        self.update_param = False 


    # Общий метод обратного вызова для осей (используется для контроля высоты тела)
    def callback(self, axes):
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            
            # Проверяем, изменилась ли ось RY
            if self.last_axes['ry'] != axes['ry']:
                if axes['ry'] < -0.5: # Поднять робота
                    self.update_height = True
                    self.init_z_offset += 0.005
                    if self.init_z_offset > 0.06: 
                        self.update_height = False
                        self.init_z_offset = 0.06
                elif axes['ry'] > 0.5: # Опустить робота
                    self.update_height = True
                    self.init_z_offset += -0.005
                    if self.init_z_offset < 0.025: 
                        self.update_height = False
                        self.init_z_offset = 0.025
                elif abs(axes['ry']) < 0.1 and abs(self.last_axes['ry']) >= 0.5: # Когда стик отпустили
                    self.update_height = True # Отмечаем, что было изменение, чтобы параметры обновились

            
            # Если высота тела изменилась
            if self.update_height: 
                self.gait_param = self.gait_manager.get_gait_param()
                # === ГЛАВНОЕ ИСПРАВЛЕНИЕ ЗДЕСЬ ===
                # Убеждаемся, что body_height всегда равен init_z_offset
                self.gait_param['body_height'] = self.init_z_offset 
                self.gait_param['init_z_offset'] = self.init_z_offset # Тоже обновляем для согласованности
                # =================================

                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])
                
                if self.speed_mode != 0: 
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else: 
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']
                
                self.gait_param['init_x_offset'] = self.speed_params[self.speed_mode].get('init_x_offset', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['init_y_offset'] = self.speed_params[self.speed_mode].get('init_y_offset', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['step_fb_ratio'] = self.speed_params[self.speed_mode].get('step_fb_ratio', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['init_roll_offset'] = self.speed_params[self.speed_mode].get('init_roll_offset', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['init_pitch_offset'] = self.speed_params[self.speed_mode].get('init_pitch_offset', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['z_swap_amplitude'] = self.speed_params[self.speed_mode].get('z_swap_amplitude', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['arm_swing_gain'] = self.speed_params[self.speed_mode].get('arm_swing_gain', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['init_yaw_offset'] = self.speed_params[self.speed_mode].get('init_yaw_offset', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['pelvis_offset'] = self.speed_params[self.speed_mode].get('pelvis_offset', 0.0) if self.speed_mode != 0 else 0.0
                self.gait_param['hip_pitch_offset'] = self.speed_params[self.speed_mode].get('hip_pitch_offset', 0.0) if self.speed_mode != 0 else 0.0

                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 

    # Заглушка для SELECT (не используется)
    def select_callback(self, new_state):
        pass

    # Кнопка R1: Переключение скорости по возрастанию: 0 -> 1 -> 2 -> 3. На 3-й остаётся.
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode == 0:
                self.speed_mode = 1
            elif self.speed_mode == 1:
                self.speed_mode = 2
            elif self.speed_mode == 2:
                self.speed_mode = 3
            elif self.speed_mode == 3:
                self.speed_mode = 3 

            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1) 
            self.gait_manager.stop() 
            self.status = 'stop' 


    # Кнопка L1: Переключение скорости по убыванию: 3 -> 2 -> 1 -> 0
    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode == 3:
                self.speed_mode = 2
            elif self.speed_mode == 2:
                self.speed_mode = 1
            elif self.speed_mode == 1:
                self.speed_mode = 0 
            elif self.speed_mode == 0:
                self.speed_mode = 0 
                
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1) 
            self.gait_manager.stop() 
            self.status = 'stop' 


    # Заглушки для других кнопок
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

    # Метод обратного вызова для кнопки START (сброс высоты тела к начальной)
    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            self.gait_param = self.gait_manager.get_gait_param()
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                for i in range(t):
                    self.init_z_offset += 0.005 * abs(0.025 - self.init_z_offset) / (0.025 - self.init_z_offset)
                    
                    # === ГЛАВНОЕ ИСПРАВЛЕНИЕ ЗДЕСЬ ===
                    # Убеждаемся, что body_height всегда равен init_z_offset
                    self.gait_param['body_height'] = self.init_z_offset
                    self.gait_param['init_z_offset'] = self.init_z_offset
                    # =================================
                    
                    current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                    if self.speed_mode != 0:
                        self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                    else:
                        self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] 

                    self.gait_param['init_x_offset'] = self.speed_params[self.speed_mode].get('init_x_offset', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['init_y_offset'] = self.speed_params[self.speed_mode].get('init_y_offset', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['step_fb_ratio'] = self.speed_params[self.speed_mode].get('step_fb_ratio', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['init_roll_offset'] = self.speed_params[self.speed_mode].get('init_roll_offset', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['init_pitch_offset'] = self.speed_params[self.speed_mode].get('init_pitch_offset', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['z_swap_amplitude'] = self.speed_params[self.speed_mode].get('z_swap_amplitude', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['arm_swing_gain'] = self.speed_params[self.speed_mode].get('arm_swing_gain', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['init_yaw_offset'] = self.speed_params[self.speed_mode].get('init_yaw_offset', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['pelvis_offset'] = self.speed_params[self.speed_mode].get('pelvis_offset', 0.0) if self.speed_mode != 0 else 0.0
                    self.gait_param['hip_pitch_offset'] = self.speed_params[self.speed_mode].get('hip_pitch_offset', 0.0) if self.speed_mode != 0 else 0.0

                    self.gait_manager.update_param(current_period_time_for_update, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
                    time.sleep(0.05)

    # Заглушки для кнопок-крестовин (hat)
    def hat_xl_callback(self, new_state):
        pass

    def hat_xr_callback(self, new_state):
        pass

    def hat_yd_callback(self, new_state):
        pass

    def hat_yu_callback(self, new_state):
        pass

    # Основной метод обратного вызова для сообщений с джойстика
    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))
        
        # Обработка высоты тела (независимо от режима скорости)
        self.callback(axes)
        
        # Проверяем, изменились ли значения осей
        axes_to_check = ['lx', 'ly', 'rx'] # Исключаем 'ry' из этой проверки
        axes_changed = any(self.last_axes[key] != axes[key] for key in axes_to_check)
        
        # Если оси изменились, вызываем axes_callback для обработки движения
        if axes_changed or self.speed_mode != (self.last_speed_mode if hasattr(self, 'last_speed_mode') else 0): # Также обновляем, если изменился режим скорости
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
        self.last_speed_mode = self.speed_mode # Сохраняем текущий speed_mode для следующей итерации

if __name__ == "__main__":
    node = JoystickController() 
    try:
        rospy.spin() 
    except Exception as e:
        rospy.logerr(str(e))