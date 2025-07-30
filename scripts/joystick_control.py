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
        # Робот начинается со скорости 1
        self.speed_mode = 1 

        # speed_params: Словарь, содержащий все параметры для каждого режима скорости.
        # Параметры скоростей остаются ТОЧНО ТАКИМИ, как вы указали.
        self.speed_params = {
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
            3: { # Скорость 3 (Обновленные параметры в соответствии с вашими требованиями)
                'period_time': [500.0, 0.22, 0.020], # period_time, dsp_ratio, y_swap_amplitude
                'x_amp_base': 0.02,                   # ИЗМЕНЕНО: Исправлено на 0.02, чтобы соответствовать допустимому диапазону (-0.02~0.02)
                'init_z_offset': 0.025,               # init_z_offset (начальное смещение по Z, высота тела)
                'z_move_amplitude': 0.018,            # z_move_amplitude (амплитуда подъема ноги)
                # Дополнительные параметры из вашего списка, если они могут быть использованы:
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
        # Эти значения будут обновляться из self.speed_params в зависимости от self.speed_mode.
        # При старте (speed_mode = 1) амплитуды движения равны 0.
        self.period_time = list(self.speed_params[self.speed_mode]['period_time']) # Инициализируем базовыми значениями для скорости 1
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        # Изначальная высота тела - теперь это ваша отправная точка для ручного управления.
        self.init_z_offset = self.speed_params[1]['init_z_offset'] 

        self.time_stamp_ry = 0
        self.count_stop = 0
        self.status = 'stop' # Текущий статус движения робота: 'stop' или 'move'
        self.update_height = False # Флаг для обновления высоты тела - теперь используется
        self.update_param = False  # Флаг для обновления параметров походки
        # Словари для отслеживания предыдущих состояний осей и кнопок джойстика
        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0 # Неиспользуемая переменная режима

        time.sleep(0.2) # Небольшая задержка для инициализации

        # Инициализация менеджера походки робота
        self.gait_manager = GaitManager()
        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Добавим начальную остановку робота при запуске
        self.gait_manager.stop()
        
        # === ВОЗВРАЩАЕМ ИНИЦИАЛИЗАЦИЮ: РОБОТ ВСТАНЕТ И ИЗДАСТ ЗВУК ===
        self.set_init_pose() 
        self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звук при запуске
        # ============================================================

        # Сообщение о старте в режиме скорости 1
        rospy.loginfo(f"JoystickController initialized. Starting in Speed Mode: {self.speed_mode}")

    # Функция для установки начальной позы робота (чтобы он встал)
    def set_init_pose(self):
        rospy.loginfo("Setting initial pose...")
        self.gait_manager.stop()
        time.sleep(0.5)
        
        # Устанавливаем высоту по умолчанию из параметров скорости 1
        self.init_z_offset = self.speed_params[1]['init_z_offset'] 
        
        gait_param = self.gait_manager.get_gait_param()
        gait_param['body_height'] = self.init_z_offset
        gait_param['init_x_offset'] = self.speed_params[1].get('init_x_offset', 0.0)
        gait_param['init_y_offset'] = self.speed_params[1].get('init_y_offset', 0.0)
        gait_param['init_roll_offset'] = self.speed_params[1].get('init_roll_offset', 0.0)
        gait_param['init_pitch_offset'] = self.speed_params[1].get('init_pitch_offset', 0.0)
        gait_param['init_yaw_offset'] = self.speed_params[1].get('init_yaw_offset', 0.0)
        gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] # Z-амплитуда шага
        gait_param['pelvis_offset'] = self.speed_params[1].get('pelvis_offset', 0.0)
        gait_param['hip_pitch_offset'] = self.speed_params[1].get('hip_pitch_offset', 0.0)
        gait_param['arm_swing_gain'] = self.speed_params[1].get('arm_swing_gain', 0.0)
        
        # Обновляем gait_manager с этими параметрами и устанавливаем нулевые амплитуды движения
        self.gait_manager.update_param(self.speed_params[1]['period_time'], 0.0, 0.0, 0.0, gait_param, step_num=0)
        self.gait_manager.set_init_pose() # Команда для принятия начальной позы
        rospy.loginfo("Initial pose set.")
        self.status = 'stop' # После установки позы робот находится в состоянии "стоп"

    # Метод для обработки входных данных с осей джойстика (движение, поворот)
    def axes_callback(self, axes):
        # Если робот находится в режиме остановки (speed_mode = 0),
        # сбрасываем все амплитуды движения в ноль.
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            # Если робот был в движении и переключились на 0 скорость, останавливаем его.
            if self.status == 'move':  
                self.gait_manager.stop() # Останавливаем движение робота
            self.status = 'stop'
            self.update_param = False # Убеждаемся, что не обновляем параметры, если в режиме остановки
            
        else: # Если робот в режиме скорости 1, 2 или 3
            # Получаем параметры для текущего выбранного режима скорости
            current_speed_settings = self.speed_params[self.speed_mode]
            
            # Сброс амплитуд движения и параметров времени по умолчанию для текущей скорости
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            
            # Обновляем period_time, dsp_ratio, y_swap_amplitude из текущих настроек скорости
            self.period_time = list(current_speed_settings['period_time'])

            # Управление движением вперед/назад по оси LY (левый стик по Y)
            if axes['ly'] > 0.3: # Движение вперед
                self.update_param = True
                self.x_move_amplitude = current_speed_settings['x_amp_base'] # Берем базовую амплитуду из параметров текущей скорости
            elif axes['ly'] < -0.3: # Движение назад
                self.update_param = True
                self.x_move_amplitude = -current_speed_settings['x_amp_base']

            # Управление движением влево/вправо по оси LX (левый стик по X)
            if axes['lx'] > 0.3: # Движение вправо
                self.period_time[2] = 0.025 # Этот параметр отвечает за приземление ноги, фиксируем его для бокового шага
                self.update_param = True
                self.y_move_amplitude = 0.015 # y_move_amplitude зафиксировано на 0.015
            elif axes['lx'] < -0.3: # Движение влево
                self.period_time[2] = 0.025
                self.update_param = True
                self.y_move_amplitude = -0.015 # y_move_amplitude зафиксировано на -0.015

            # Управление поворотом по оси RX (правый стик по X)
            if axes['rx'] > 0.3: # Поворот вправо
                self.update_param = True
                self.angle_move_amplitude = 8 # angle_move_amplitude зафиксировано на 8 градусов
            elif axes['rx'] < -0.3: # Поворот влево
                self.update_param = True
                self.angle_move_amplitude = -8 # angle_move_amplitude зафиксировано на -8 градусов

        # Если флаг update_param установлен (т.е. были изменения в движении или повороте)
        # ИЛИ если мы вышли из режима остановки (speed_mode > 0) и хотим начать движение
        if self.update_param or (self.speed_mode > 0 and self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0)):
            self.gait_param = self.gait_manager.get_gait_param()  
            self.gait_param['init_z_offset'] = self.init_z_offset # Устанавливаем текущую высоту тела (теперь она контролируется вручную)
            
            # Устанавливаем высоту подъема ноги из параметров текущей скорости, если режим > 0
            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']  
                # Дополнительно обновляем другие параметры, если они существуют для текущей скорости
                self.gait_param['init_x_offset'] = self.speed_params[self.speed_mode].get('init_x_offset', 0.0)
                self.gait_param['init_y_offset'] = self.speed_params[self.speed_mode].get('init_y_offset', 0.0)
                self.gait_param['step_fb_ratio'] = self.speed_params[self.speed_mode].get('step_fb_ratio', 0.0)
                self.gait_param['init_roll_offset'] = self.speed_params[self.speed_mode].get('init_roll_offset', 0.0)
                # self.gait_param['angle_move_amplitude'] обновляется выше в зависимости от стика
                self.gait_param['init_pitch_offset'] = self.speed_params[self.speed_mode].get('init_pitch_offset', 0.0)
                self.gait_param['z_swap_amplitude'] = self.speed_params[self.speed_mode].get('z_swap_amplitude', 0.0)
                self.gait_param['arm_swing_gain'] = self.speed_params[self.speed_mode].get('arm_swing_gain', 0.0)
                self.gait_param['init_yaw_offset'] = self.speed_params[self.speed_mode].get('init_yaw_offset', 0.0)
                self.gait_param['pelvis_offset'] = self.speed_params[self.speed_mode].get('pelvis_offset', 0.0)
                self.gait_param['hip_pitch_offset'] = self.speed_params[self.speed_mode].get('hip_pitch_offset', 0.0)

            else: # Если speed_mode == 0, используем z_move_amplitude от Скорости 1 как базовое
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

            # Отправляем обновленные параметры движения в GaitManager
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            
        # Логика для перехода между состоянием 'stop' и 'move'
        if self.status == 'stop' and (self.update_param or (self.speed_mode > 0 and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0))):  
            self.status = 'move'  
        elif self.status == 'move' and not self.update_param and self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0 and self.speed_mode > 0:
            # Если робот был в движении, но все амплитуды стали 0 (и не режим 0) - значит он должен остановиться.
            self.status = 'stop'
            self.gait_manager.stop()
        elif self.speed_mode == 0 and self.status == 'move': # Если переключились на 0 скорость, и он двигался
            self.gait_manager.stop()
            self.status = 'stop'

        self.update_param = False # Сбрасываем флаг обновления параметров


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
                
                # Определяем period_time для передачи в update_param
                # Используем параметры из текущего speed_mode, если он активен,
                # иначе используем базовые параметры из speed_params[1]
                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])
                
                # Z-амплитуда шага должна быть взята из текущего режима скорости, если он не 0.
                if self.speed_mode != 0:  
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:  
                    # Если в режиме остановки, используем значение по умолчанию для Z-амплитуды (Скорость 1)
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']
                
                # Также передаем другие параметры, которые могут быть специфичны для скорости 3
                # Используем .get() с запасным значением, чтобы избежать KeyError, если параметра нет для данной скорости
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

                # Передаем обновленные параметры
                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 # Задержка для плавного изменения высоты

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
                # Если уже на 3-й скорости, остаемся на 3-й
                self.speed_mode = 3 

            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1) # Звук в зависимости от скорости
            self.gait_manager.stop() # Останавливаем текущее движение для плавного перехода
            self.status = 'stop' # Статус на "стоп", пока не начнутся новые команды движения


    # Кнопка L1: Переключение скорости по убыванию: 3 -> 2 -> 1 -> 0
    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.speed_mode == 3:
                self.speed_mode = 2
            elif self.speed_mode == 2:
                self.speed_mode = 1
            elif self.speed_mode == 1:
                self.speed_mode = 0 # После 1-й скорости переключаемся на 0 (остановку)
            elif self.speed_mode == 0:
                # Если уже на 0 скорости, остаемся на 0.
                self.speed_mode = 0 
                
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1) # Звук в зависимости от скорости
            self.gait_manager.stop() # Останавливаем текущее движение для плавного перехода
            self.status = 'stop' # Статус на "стоп", пока не начнутся новые команды движения


    # Заглушки для других кнопок
    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):  
        pass

    def circle_callback(self, new_state):
        pass

    def triangle_callback(self, new_state): # TRIANGLE теперь не используется для переключения скорости
        pass

    def cross_callback(self, new_state): # CROSS теперь не используется для переключения скорости
        pass

    # Метод обратного вызова для кнопки START. Теперь он не меняет высоту, только издаёт звук.
    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            rospy.loginfo("Start button pressed. Setting initial pose.")
            self.set_init_pose() # Возвращаем робота в исходную позу и высоту
            
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
        
        # Обработка высоты тела (теперь снова активна)
        self.callback(axes) 
        
        # Проверяем, изменились ли значения осей
        for key, value in axes.items():  
            if self.last_axes[key] != value: # Проверяем все оси
                axes_changed = True
        
        # Если оси изменились, вызываем axes_callback для обработки движения
        # В режиме скорости 0, axes_callback просто сбросит амплитуды и остановит, 
        # поэтому его можно вызывать всегда.
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))
        
        # Обработка состояний кнопок
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]: # Если значение кнопки изменилось
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released # Определяем состояние: нажата или отпущена
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal # Определяем состояние: удерживается или нормальное
            
            callback = "".join([key, '_callback']) # Формируем имя метода
            
            if new_state != ButtonState.Normal: # Если состояние изменилось (не нормальное)
                if hasattr(self, callback): # Проверяем, существует ли метод для этой кнопки
                    try:
                        getattr(self, callback)(new_state) # Вызываем метод
                    except Exception as e:
                        rospy.logerr(str(e))
        
        # Сохраняем текущие состояния для следующего цикла
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController() # Создаем экземпляр контроллера
    try:
        rospy.spin() # Запускаем основной цикл ROS
    except Exception as e:
        rospy.logerr(str(e))