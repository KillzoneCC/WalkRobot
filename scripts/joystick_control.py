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
        
        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        # speed_mode: Отслеживает текущий режим скорости.
        # 0 - остановка, 1 - Скорость 1, 2 - Скорость 2, 3 - Скорость 3.
        # Робот начинается со скорости 1
        self.speed_mode = 1 

        # speed_params: Словарь, содержащий все параметры для каждого режима скорости.
        self.speed_params = {
            1: { # Скорость 1 (По вашим точным указаниям)
                'period_time': [400, 0.2, 0.022],  
                'x_amp_base': 0.01,
                'init_z_offset': 0.025, # Базовая высота для скорости 1
                'z_move_amplitude': 0.016  
            },
            2: { # Скорость 2 
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016  
            },
            3: { # Скорость 3 
                'period_time': [500.0, 0.22, 0.020], 
                'x_amp_base': 0.02,                   
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

        self.period_time = list(self.speed_params[self.speed_mode]['period_time']) 
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        # Изначальная высота тела - теперь это ваша отправная точка для ручного управления.
        # Робот стартует с этой высоты и не будет "выпрямлять" её автоматически.
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

        self.gait_manager.stop()
        
        # === ВОЗВРАЩАЕМ ИНИЦИАЛИЗАЦИЮ: РОБОТ ВСТАЕТ НА СТАРТОВУЮ ВЫСОТУ И ИЗДАЕТ ЗВУК ===
        # Робот примет начальную позу (встанет) с высотой 0.025м.
        # Это не "выпрямление спины" в процессе ходьбы, а начальная установка позы.
        self.set_init_pose() 
        self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звук при запуске
        # ============================================================

        rospy.loginfo(f"JoystickController initialized. Starting in Speed Mode: {self.speed_mode}")

    # Функция для установки начальной позы робота (чтобы он встал)
    def set_init_pose(self):
        rospy.loginfo("Setting initial pose...")
        self.gait_manager.stop()
        time.sleep(0.5)
        
        # Устанавливаем высоту по умолчанию из параметров скорости 1
        # Это значение будет стартовой высотой робота.
        self.init_z_offset = self.speed_params[1]['init_z_offset'] 
        
        gait_param = self.gait_manager.get_gait_param()
        gait_param['body_height'] = self.init_z_offset
        gait_param['init_x_offset'] = self.speed_params[1].get('init_x_offset', 0.0)
        gait_param['init_y_offset'] = self.speed_params[1].get('init_y_offset', 0.0)
        gait_param['init_roll_offset'] = self.speed_params[1].get('init_roll_offset', 0.0)
        gait_param['init_pitch_offset'] = self.speed_params[1].get('init_pitch_offset', 0.0)
        gait_param['init_yaw_offset'] = self.speed_params[1].get('init_yaw_offset', 0.0)
        gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] 
        gait_param['pelvis_offset'] = self.speed_params[1].get('pelvis_offset', 0.0)
        gait_param['hip_pitch_offset'] = self.speed_params[1].get('hip_pitch_offset', 0.0)
        gait_param['arm_swing_gain'] = self.speed_params[1].get('arm_swing_gain', 0.0)
        
        self.gait_manager.update_param(self.speed_params[1]['period_time'], 0.0, 0.0, 0.0, gait_param, step_num=0)
        self.gait_manager.set_init_pose() # Команда для принятия начальной позы
        rospy.loginfo("Initial pose set.")
        self.status = 'stop' 

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
            # *** КЛЮЧЕВОЕ ИЗМЕНЕНИЕ: init_z_offset берется из текущего значения, а не из speed_params ***
            self.gait_param['init_z_offset'] = self.init_z_offset 
            
            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']  
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
                # *** КЛЮЧЕВОЕ ИЗМЕНЕНИЕ: 'body_height' берется из текущего init_z_offset ***
                self.gait_param['body_height'] = self.init_z_offset
                
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

    def select_callback(self, new_state):
        pass

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

    # Кнопка START: Теперь только издает звук и вызывает set_init_pose()
    # (без автоматического выпрямления спины с циклом)
    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            rospy.loginfo("Start button pressed. Setting initial pose.")
            # Если вам нужно, чтобы по START робот возвращался на "стандартную" высоту,
            # то set_init_pose() сделает это. Он не будет "выпрямлять спину" при ходьбе,
            # но эта кнопка будет выполнять функцию сброса позы.
            self.set_init_pose() 
            
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
        
        # Проверяем изменения на всех осях, кроме 'ry', которая обрабатывается отдельно для высоты
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