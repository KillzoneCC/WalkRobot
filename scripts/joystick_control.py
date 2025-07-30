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
        self.speed_params = {
            0: { # Скорость 0 (Остановка)
                'period_time': [400, 0.2, 0.022], 
                'x_amp_base': 0.0, # Амплитуда движения по X для остановки
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            1: { # Скорость 1 (По вашим точным указаниям)
                'period_time': [400, 0.2, 0.022], 
                'x_amp_base': 0.01,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            2: { # Скорость 2 (Как было согласовано)
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            3: { # Скорость 3 (Как было согласовано)
                'period_time': [500, 0.22, 0.022], 
                'x_amp_base': 0.028, # Важно: это значение может быть ограничено MAX_FORWARD_AMPLITUDE
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018, 
                # Дополнительные параметры для скорости 3
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
        self.period_time = list(self.speed_params[self.speed_mode]['period_time']) # Инициализируем с параметрами первой скорости
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        self.init_z_offset = self.speed_params[self.speed_mode]['init_z_offset'] # Высота тела по умолчанию для текущей скорости

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

        # Применяем параметры начальной скорости (1) и гарантируем остановку, если она не 0.
        self._apply_speed_params() 
        self.gait_manager.stop() # Гарантированная остановка при старте
        rospy.loginfo(f"JoystickController initialized. Starting in Speed Mode: {self.speed_mode}")


    def _apply_speed_params(self):
        """
        Применяет параметры текущего режима скорости.
        Учитывает MAX_FORWARD_AMPLITUDE для предотвращения "перебора".
        """
        # Если speed_mode = 0, берем параметры из speed_params[0].
        # Иначе, берем параметры из speed_params[self.speed_mode].
        params_key = self.speed_mode 
        current_speed_settings = self.speed_params[params_key]

        self.period_time = list(current_speed_settings['period_time'])
        
        # Проверяем и ограничиваем x_amp_base, чтобы избежать "перебора"
        requested_x_amp_base = current_speed_settings['x_amp_base']
        if abs(requested_x_amp_base) > self.MAX_FORWARD_AMPLITUDE:
            rospy.logwarn(f"ВНИМАНИЕ: Запрошенная базовая амплитуда X для скорости {self.speed_mode} ({requested_x_amp_base}) превышает MAX_FORWARD_AMPLITUDE ({self.MAX_FORWARD_AMPLITUDE}). Ограничиваем.")
            self.base_x_move_amplitude = self.MAX_FORWARD_AMPLITUDE * (1 if requested_x_amp_base > 0 else -1)
        else:
            self.base_x_move_amplitude = requested_x_amp_base

        self.init_z_offset = current_speed_settings['init_z_offset']

        # Получаем текущие параметры походки, чтобы обновить их
        if hasattr(self, 'gait_manager') and self.gait_manager is not None:
            self.gait_param = self.gait_manager.get_gait_param()
            self.gait_param['init_z_offset'] = self.init_z_offset
            self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']

            # Применяем все дополнительные параметры из speed_params для текущего режима скорости
            for key, value in current_speed_settings.items():
                if key not in ['period_time', 'x_amp_base', 'init_z_offset', 'z_move_amplitude', 'angle_move_amplitude']:
                    self.gait_param[key] = value
            
            # Если переключаемся на скорость 0, убедимся, что робот остановится.
            if self.speed_mode == 0:
                self.gait_manager.stop()
                self.status = 'stop'
                self.x_move_amplitude = 0.0 # Обнуляем амплитуды, чтобы избежать случайного движения
                self.y_move_amplitude = 0.0
                self.angle_move_amplitude = 0.0
                rospy.loginfo(f"Скорость переключена на: {self.speed_mode} (ОСТАНОВКА)")
            else:
                rospy.loginfo(f"Скорость переключена на: {self.speed_mode}, Базовая амплитуда X: {self.base_x_move_amplitude}")
                # Если робот был в состоянии "стоп" и перешел на скорость > 0, он не должен сразу начать движение.
                # Здесь мы просто обновляем его внутренние параметры для новой скорости.
                self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=0) 


    # Метод для обработки входных данных с осей джойстика (движение, поворот)
    def axes_callback(self, axes):
        # Сброс флага update_param в начале. Он будет установлен только если есть активное движение.
        self.update_param = False 

        # Если робот находится в режиме остановки (speed_mode = 0),
        # сбрасываем все амплитуды движения в ноль и гарантируем остановку.
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            if self.status == 'move': # Если робот был в движении, и переключили на 0 скорость
                self.gait_manager.stop() 
                self.status = 'stop'
            return # Выходим сразу, так как в режиме 0 движения быть не должно.
            
        else: # Если робот в режиме скорости 1, 2 или 3
            # Сбрасываем амплитуды движения (они будут установлены, если стики сдвинуты).
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            # Период времени обновляем на основе текущих настроек скорости
            self.period_time = list(self.speed_params[self.speed_mode]['period_time']) 

            # Управление движением вперед/назад по оси LY (левый стик по Y)
            if axes['ly'] > 0.3: # Движение вперед
                self.update_param = True
                self.x_move_amplitude = self.base_x_move_amplitude 
            elif axes['ly'] < -0.3: # Движение назад
                self.update_param = True
                self.x_move_amplitude = -self.base_x_move_amplitude

            # Управление движением влево/вправо по оси LX (левый стик по X)
            if axes['lx'] > 0.3: # Движение вправо
                self.period_time[2] = 0.025 
                self.update_param = True
                self.y_move_amplitude = 0.015 
            elif axes['lx'] < -0.3: # Движение влево
                self.period_time[2] = 0.025
                self.update_param = True
                self.y_move_amplitude = -0.015 

            # Управление поворотом по оси RX (правый стик по X)
            if axes['rx'] > 0.3: # Поворот вправо
                self.update_param = True
                self.angle_move_amplitude = 8 
            elif axes['rx'] < -0.3: # Поворот влево
                self.update_param = True
                self.angle_move_amplitude = -8 
        
        # Если флаг update_param установлен (т.е. были изменения в движении или повороте)
        if self.update_param:
            self.gait_param = self.gait_manager.get_gait_param() 
            self.gait_param['init_z_offset'] = self.init_z_offset 
            self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude'] 

            # Отправляем обновленные параметры движения в GaitManager
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            
            # Обновляем статус движения
            if self.status == 'stop': 
                self.status = 'move' 
        elif self.status == 'move': # Если робот был в движении, но стики вернулись в нейтральное положение
            if self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0:
                self.status = 'stop'
                self.gait_manager.stop() 


    # Общий метод обратного вызова для осей (используется для контроля высоты тела)
    def callback(self, axes):
        # Проверяем временную метку, чтобы не обновлять высоту слишком часто
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            # Управление высотой тела робота (ось RY - правый стик по Y)
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
                
                # Z-амплитуда шага должна быть взята из текущего режима скорости, если он не 0.
                if self.speed_mode != 0: 
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else: 
                    # Если в режиме остановки, используем значение по умолчанию для Z-амплитуды (Скорость 1)
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                # При изменении высоты, робот не должен двигаться по осям X, Y, Angle, 
                # поэтому передаем 0.0 для этих амплитуд.
                self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 # Задержка для плавного изменения высоты

    # Заглушка для SELECT (не используется)
    def select_callback(self, new_state):
        pass

    # Кнопка R1: Переключение скорости по возрастанию: 0 -> 1 -> 2 -> 3. На 3-й остаётся.
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Увеличиваем speed_mode, но не выше 3
            if self.speed_mode < 3:
                self.speed_mode += 1
            
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1) # Звук в зависимости от скорости
            self._apply_speed_params() # Применяем параметры новой скорости


    # Кнопка L1: Переключение скорости по убыванию: 3 -> 2 -> 1 -> 0
    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Уменьшаем speed_mode, но не ниже 0
            if self.speed_mode > 0:
                self.speed_mode -= 1
            
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1) # Звук в зависимости от скорости
            self._apply_speed_params() # Применяем параметры новой скорости


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
            # Вычисляем количество шагов для плавного возврата высоты к 0.025
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                for i in range(t):
                    # Постепенное изменение высоты
                    self.init_z_offset += 0.005 * abs(0.025 - self.init_z_offset) / (0.025 - self.init_z_offset)
                    self.gait_param['body_height'] = self.init_z_offset
                    
                    # Убедимся, что z_move_amplitude обновляется также
                    # Если в режиме остановки, используем значение по умолчанию для Z-амплитуды (Скорость 1)
                    if self.speed_mode != 0:
                        self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                    else:
                        self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] 

                    self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
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
        # Сопоставляем значения осей и кнопок с их именами
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
        # Axes_callback теперь сам обрабатывает логику для speed_mode 0.
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
        
        # Сохраняем текущие состояния для следующего цикла
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController() 
    try:
        rospy.spin() 
    except Exception as e:
        rospy.logerr(str(e))