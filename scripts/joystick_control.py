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
        self.speed_mode = 0 # Начинаем с остановки (нулевая скорость)

        # speed_params: Словарь, содержащий все параметры для каждого режима скорости.
        # Теперь Скорость 1 точно соответствует вашим последним указаниям.
        self.speed_params = {
            1: { # Скорость 1 (ТОЧНО ПО ВАШИМ УКАЗАНИЯМ!)
                'period_time': [400, 0.2, 0.022], 
                'x_amp_base': 0.01,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            2: { # Скорость 2 (Как было согласовано, по фото 2.png)
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016 
            },
            3: { # Скорость 3 (Как было согласовано, более быстрая)
                'period_time': [500, 0.22, 0.022],
                'x_amp_base': 0.028,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018 
            }
        }
        # ===================================================

        # Инициализация текущих параметров движения.
        # Они будут обновляться из self.speed_params в зависимости от self.speed_mode.
        # По умолчанию берем параметры Скорости 1 (они используются при переходе из 0 режима),
        # но робот начнет в режиме остановки (speed_mode = 0)
        self.period_time = list(self.speed_params[1]['period_time']) 
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        self.init_z_offset = self.speed_params[1]['init_z_offset'] # Высота тела по умолчанию

        self.time_stamp_ry = 0
        self.count_stop = 0
        self.status = 'stop' # Текущий статус движения робота: 'stop' или 'move'
        self.update_height = False # Флаг для обновления высоты тела
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

    # Метод для обработки входных данных с осей джойстика (движение, поворот)
    def axes_callback(self, axes):
        # Если робот находится в режиме остановки (speed_mode = 0),
        # сбрасываем все амплитуды движения в ноль и выходим.
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            # Сбрасываем period_time к базовым значениям (например, Скорость 1),
            # чтобы при выходе из режима остановки они не были пустыми.
            self.period_time = list(self.speed_params[1]['period_time']) 
            if self.status == 'move': # Если робот двигался и переключились на остановку
                self.gait_manager.stop() # Останавливаем движение робота
                self.status = 'stop'
            self.update_param = False # Убеждаемся, что не обновляем параметры, если в режиме остановки
            return # Выходим из функции

        # Получаем параметры для текущего выбранного режима скорости
        current_speed_settings = self.speed_params[self.speed_mode]
        
        # Сброс амплитуд движения и параметров времени по умолчанию для текущей скорости
        # Создаем копию списка period_time, чтобы можно было изменять его элемент ([2])
        self.x_move_amplitude = 0.0
        self.angle_move_amplitude = 0.0
        self.y_move_amplitude = 0.0
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
        if self.update_param:
            self.gait_param = self.gait_manager.get_gait_param() 
            self.gait_param['init_z_offset'] = self.init_z_offset # Устанавливаем текущую высоту тела
            # Устанавливаем высоту подъема ноги из параметров текущей скорости
            self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude'] 
            # Отправляем обновленные параметры движения в GaitManager
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            
        # Логика для перехода между состоянием 'stop' и 'move'
        if self.status == 'stop' and self.update_param: 
            self.status = 'move' 
        elif self.status == 'move' and not self.update_param: 
            self.status = 'stop' 
            self.gait_manager.stop() # Останавливаем робота, если нет активных команд движения
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
            if self.update_height and not self.update_param:
                self.gait_param = self.gait_manager.get_gait_param()
                self.gait_param['body_height'] = self.init_z_offset
                # Также обновляем z_move_amplitude, чтобы высота шага соответствовала текущей скорости,
                # если робот не в режиме остановки.
                if self.speed_mode != 0: 
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else: 
                    # Если в режиме остановки, используем значение по умолчанию для Z-амплитуды (Скорость 1)
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                self.gait_manager.update_param(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 # Задержка для плавного изменения высоты

    # Заглушка для SELECT (не используется)
    def select_callback(self, new_state):
        pass

    # ====== ОБРАБОТЧИКИ КНОПОК ДЛЯ ПЕРЕКЛЮЧЕНИЯ СКОРОСТИ ======
    # Кнопка L1: Переключение на Скорость 1
    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = 1 # Устанавливаем Скорость 1
            rospy.loginfo("Speed Mode: 1 (Original)")
            self.board.set_buzzer(1000, 0.05, 0.02, 1) # Короткий звуковой сигнал
            self.gait_manager.stop() # Останавливаем текущее движение для плавного перехода
            self.status = 'stop'

    # Кнопка R1: Переключение на Скорость 2
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = 2 # Устанавливаем Скорость 2
            rospy.loginfo("Speed Mode: 2")
            self.board.set_buzzer(1500, 0.05, 0.02, 1) # Короткий звуковой сигнал
            self.gait_manager.stop()
            self.status = 'stop'

    # Кнопка TRIANGLE: Переключение на Скорость 3
    def triangle_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = 3 # Устанавливаем Скорость 3
            rospy.loginfo("Speed Mode: 3 (Faster)")
            self.board.set_buzzer(2000, 0.05, 0.02, 1) # Короткий звуковой сигнал
            self.gait_manager.stop()
            self.status = 'stop'
            
    # Кнопка CROSS (X): Переключение на Нулевую скорость (ОСТАНОВКА)
    def cross_callback(self, new_state): 
        if new_state == ButtonState.Pressed:
            self.speed_mode = 0 # Устанавливаем режим остановки
            rospy.loginfo("Speed Mode: 0 (STOP)")
            self.board.set_buzzer(500, 0.1, 0.05, 2) # Двойной звуковой сигнал
            self.gait_manager.stop() # Полностью останавливаем робота
            self.status = 'stop'
    # ======================================================================

    # Заглушки для других кнопок (L2, R2, SQUARE, CIRCLE)
    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state): 
        pass

    def circle_callback(self, new_state):
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