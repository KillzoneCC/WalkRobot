#!/usr/bin/env python3
# encoding: utf-8

import time
import math # Импортируем модуль math для математических операций (например, atan2, degrees)
import rospy # Импортируем библиотеку ROS для Python
from ainex_sdk import Board # Импортируем класс Board для взаимодействия с аппаратной платой робота
from sensor_msgs.msg import Joy, Imu # Импортируем типы сообщений Joy (для джойстика) и Imu (для IMU сенсора)
from ainex_kinematics.gait_manager import GaitManager # Импортируем GaitManager для управления походкой робота
from ainex_kinematics.motion_manager import MotionManager # Импортируем MotionManager для выполнения предопределенных движений (действий)

# Определение карты осей джойстика. Каждая ось имеет свое имя.
AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
# Определение карты кнопок джойстика. Каждая кнопка имеет свое имя.
# Порядок соответствует индексу кнопки в массиве joy_msg.buttons.
# Например, 'cross' (крестик/A) соответствует индексу 0, 'circle' (круг/B) - индексу 1.
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

# Класс для определения состояний кнопки.
# Используется для отслеживания нажатия, удержания и отпускания кнопки.
class ButtonState():
    Normal = 0    # Обычное состояние (кнопка не нажата)
    Pressed = 1   # Кнопка только что была нажата
    Holding = 2   # Кнопка удерживается
    Released = 3  # Кнопка только что была отпущена

# Основной класс контроллера джойстика
class JoystickController:
    def __init__(self):
        # Инициализация ROS-узла. 'anonymous=True' делает узел уникальным при каждом запуске.
        rospy.init_node('joystick_control', anonymous=True)
        self.board = Board() # Создание экземпляра класса Board для управления платой робота

        # ====== ПЕРЕМЕННЫЕ ДЛЯ РЕЖИМОВ СКОРОСТИ ======
        self.speed_mode = 1 # Изначальный режим скорости. 0 - остановка, 1-3 - разные скорости.

        # Словарь, определяющий параметры движения для каждого режима скорости.
        self.speed_params = {
            1: { # Скорость 1 (медленная)
                'period_time': [400, 0.2, 0.022], # [период шага, фаза подъема, задержка шага]
                'x_amp_base': 0.01, # Базовая амплитуда движения по X (вперед/назад)
                'init_z_offset': 0.025, # Начальное смещение по Z (высота тела)
                'z_move_amplitude': 0.016 # Амплитуда движения по Z во время шага
            },
            2: { # Скорость 2 (средняя)
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016
            },
            3: { # Скорость 3 (быстрая)
                'period_time': [500.0, 0.22, 0.020],
                'x_amp_base': 0.02,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018
            }
        }

        # Инициализация параметров движения, используемых GaitManager
        self.period_time = list(self.speed_params[1]['period_time']) # Текущие параметры времени шага
        self.x_move_amplitude = 0 # Амплитуда движения по оси X (вперед/назад)
        self.y_move_amplitude = 0 # Амплитуда движения по оси Y (влево/вправо)
        self.angle_move_amplitude = 0 # Амплитуда поворота
        self.init_z_offset = self.speed_params[1]['init_z_offset'] # Текущее смещение по Z (высота)
        self.time_stamp_ry = 0 # Временная метка для контроля частоты изменения высоты
        self.count_stop = 0 # Счетчик для остановки (не используется активно в текущем коде)
        self.status = 'stop' # Текущий статус движения робота ('stop' или 'move')
        self.update_height = False # Флаг, указывающий, нужно ли обновить высоту тела
        self.update_param = False # Флаг, указывающий, нужно ли обновить параметры походки

        # Словари для хранения последнего состояния осей и кнопок джойстика
        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP)))
        self.mode = 0 # Дополнительный режим (не используется активно в текущем коде)

        time.sleep(0.2) # Небольшая задержка для инициализации

        # Инициализация менеджера походки робота. Отвечает за генерацию шагов.
        self.gait_manager = GaitManager()
        
        # === MotionManager для Get Up (функция подъема) ===
        # Инициализация менеджера движений. Отвечает за выполнение предопределенных действий.
        self.motion_manager = MotionManager()
        # Имена предопределенных действий для подъема робота, как они определены в app_controller.py
        self.lie_to_stand_action_name = 'lie_to_stand'     # Действие для подъема из положения "лицом вниз"
        self.recline_to_stand_action_name = 'recline_to_stand' # Действие для подъема из положения "лицом вверх"
        # =============================================

        # === Переменные для работы с IMU и состоянием падения ===
        # Текущее состояние робота, определяемое IMU.
        # Изначально 'stand_or_unknown', чтобы избежать случайного запуска подъема при старте.
        self.robot_state = 'stand_or_unknown'
        self.count_lie = 0       # Счетчик для состояния "лицом вниз" (prone)
        self.count_recline = 0   # Счетчик для состояния "лицом вверх" (supine)
        # Порог счетчика для подтверждения состояния падения.
        # Если счетчик превысит это значение, считается, что робот упал.
        self.FALL_COUNT_THRESHOLD = 50 
        
        # Подписка на топик ROS '/imu' для получения данных с IMU сенсора.
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.loginfo("Subscribed to IMU topic: /imu")
        # ===================================================================

        # Подписка на топик ROS '/joy' для получения данных с джойстика.
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Начальная остановка робота при запуске контроллера.
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Speed Mode: 0 (STOP)")
        rospy.loginfo("Controls: R1/L1 - Speed Up/Down, Circle (B) - Get Up (Auto-detect)")

    # === Callback функция для обработки данных IMU ===
    # Эта логика адаптирована из примера мобильного приложения, использующего `ay` (ускорение по Y)
    # и `az` (ускорение по Z) для определения положения робота.
    def imu_callback(self, msg: Imu):
        """
        Обрабатывает данные с IMU для определения состояния падения робота (лежит лицом вниз, лицом вверх, или стоит).
        """
        try:
            # Получаем компоненты линейного ускорения с IMU
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # Пороги для определения состояния (возможно, потребуется настройка под конкретного робота)
            ACCEL_THRESH = 7.0  # Порог ускорения для определения, что робот лежит
            ANGLE_THRESH = 30.0 # Порог угла для определения, что робот лежит относительно горизонтально
            COUNT_INCREMENT = 1 # Значение, на которое увеличивается счетчик
            COUNT_DECREMENT = 1 # Значение, на которое уменьшается счетчик

            # Вычисляем угол наклона относительно оси Z (вертикальной)
            if abs(az) > 1e-6: # Избегаем деления на ноль, если az очень близко к нулю
                angle_rad = math.atan2(abs(ay), abs(az))
                angle_deg = math.degrees(angle_rad)
            else:
                angle_deg = 90.0 # Если az почти 0, значит, робот, скорее всего, лежит на боку

            # Логика инкремента/декремента счетчиков на основе показаний IMU
            if angle_deg < ANGLE_THRESH: # Если робот лежит относительно горизонтально
                if az > ACCEL_THRESH: # Робот лежит "лицом вниз" (prone), так как Z-ускорение положительное
                    self.count_lie += COUNT_INCREMENT
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT) # Уменьшаем другой счетчик
                elif az < -ACCEL_THRESH: # Робот лежит "лицом вверх" (supine), так как Z-ускорение отрицательное
                    self.count_recline += COUNT_INCREMENT
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT) # Уменьшаем другой счетчик
                else:
                    # Если угол меньше порога, но az не соответствует четкому "лицом вниз/вверх",
                    # это может означать, что робот стоит или находится в промежуточном состоянии.
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
            else:
                # Если угол больше или равен порогу (робот лежит на боку, падает или находится в нестабильном положении)
                self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)

            # Сохраняем старое состояние для логирования изменений
            old_state = self.robot_state

            # Проверяем счетчики и обновляем состояние робота
            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'lie_to_stand' # Робот лежит лицом вниз, требуется подъем из этого положения
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'recline_to_stand' # Робот лежит лицом вверх, требуется подъем из этого положения
            else:
                self.robot_state = 'stand_or_unknown' # Робот стоит или его состояние не является явным падением

            # Логируем изменение состояния, если оно произошло
            if old_state != self.robot_state:
                rospy.loginfo(f"IMU detected robot state change: '{old_state}' -> '{self.robot_state}' (lie_count:{self.count_lie}, recline_count:{self.count_recline})")

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data in imu_callback: {e}")

    # Callback функция для обработки движения осями джойстика
    def axes_callback(self, axes):
        if self.speed_mode == 0: # Если режим скорости 0 (остановка)
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            if self.status == 'move':
                self.gait_manager.stop() # Если робот двигался, остановить его
            self.status = 'stop' # Установить статус на 'stop'
            self.update_param = False # Сбросить флаг обновления параметров
        else: # Если режим скорости > 0 (робот должен двигаться)
            current_speed_settings = self.speed_params[self.speed_mode] # Получить параметры для текущей скорости
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            self.period_time = list(current_speed_settings['period_time']) # Обновить время шага

            # Управление движением вперед/назад по оси LY (левый стик по Y)
            if axes['ly'] > 0.3: # Если левый стик вверх
                self.update_param = True
                self.x_move_amplitude = current_speed_settings['x_amp_base']
            elif axes['ly'] < -0.3: # Если левый стик вниз
                self.update_param = True
                self.x_move_amplitude = -current_speed_settings['x_amp_base']

            # Управление движением влево/вправо по оси LX (левый стик по X)
            if axes['lx'] > 0.3: # Если левый стик вправо
                self.period_time[2] = 0.025 # Коррекция задержки шага для бокового движения
                self.update_param = True
                self.y_move_amplitude = 0.015
            elif axes['lx'] < -0.3: # Если левый стик влево
                self.period_time[2] = 0.025 # Коррекция задержки шага
                self.update_param = True
                self.y_move_amplitude = -0.015

            # Управление поворотом по оси RX (правый стик по X)
            if axes['rx'] > 0.3: # Если правый стик вправо
                self.update_param = True
                self.angle_move_amplitude = 8 # Поворот вправо
            elif axes['rx'] < -0.3: # Если правый стик влево
                self.update_param = True
                self.angle_move_amplitude = -8 # Поворот влево

        # Обновление параметров походки, если есть изменения в движении или если робот начал двигаться из состояния "стоп"
        if self.update_param or (self.speed_mode > 0 and self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0)):
            self.gait_param = self.gait_manager.get_gait_param() # Получить текущие параметры походки
            self.gait_param['init_z_offset'] = self.init_z_offset # Установить высоту тела

            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']
            else:
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude'] # Используем базовую Z-амплитуду для остановки

            # Установить новые параметры шага в GaitManager
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)

        # Логика перехода между состояниями 'stop' и 'move'
        if self.status == 'stop' and (self.update_param or (self.speed_mode > 0 and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0))):
            self.status = 'move' # Начать движение
        elif self.status == 'move' and not self.update_param and self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0 and self.speed_mode > 0:
            self.status = 'stop' # Остановить движение, если все амплитуды нулевые и не требуется обновление
            self.gait_manager.stop()
        elif self.speed_mode == 0 and self.status == 'move':
            self.gait_manager.stop() # Принудительная остановка, если speed_mode 0
            self.status = 'stop'

        self.update_param = False # Сбросить флаг обновления параметров после обработки

    # Callback функция для обработки оси RY (правый стик по Y) для изменения высоты тела
    def callback(self, axes):
        if rospy.get_time() > self.time_stamp_ry: # Контроль частоты обновления, чтобы не слишком быстро менять высоту
            self.update_height = False
            if axes['ry'] < -0.5: # Если правый стик вверх (увеличение высоты)
                self.update_height = True
                self.init_z_offset += 0.005 # Увеличить высоту на 5 мм
                if self.init_z_offset > 0.06: # Ограничить максимальную высоту
                    self.update_height = False
                    self.init_z_offset = 0.06
            elif axes['ry'] > 0.5: # Если правый стик вниз (уменьшение высоты)
                self.update_height = True
                self.init_z_offset += -0.005 # Уменьшить высоту на 5 мм
                if self.init_z_offset < 0.025: # Ограничить минимальную высоту
                    self.update_height = False
                    self.init_z_offset = 0.025

            if self.update_height: # Если высота изменилась, обновить параметры походки
                self.gait_param = self.gait_manager.get_gait_param()
                self.gait_param['body_height'] = self.init_z_offset # Установить новую высоту тела

                # Определяем параметры времени для обновления (используем текущий speed_mode или базовый)
                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                if self.speed_mode != 0:
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                # Обновить параметры в GaitManager с текущими амплитудами движения (0,0,0, чтобы не было неожиданных шагов)
                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05 # Установить новую временную метку для следующего обновления

    def select_callback(self, new_state):
        pass # Функция для кнопки SELECT (не реализована)

    # Callback функция для кнопки R1 (увеличение скорости)
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed: # Только при нажатии кнопки
            self.speed_mode = min(self.speed_mode + 1, 3) # Увеличить режим скорости, максимум до 3
            rospy.loginfo(f"Speed Mode increased to: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1) # Звуковой сигнал
            self.gait_manager.stop() # Остановить текущее движение
            self.status = 'stop' # Установить статус на 'stop'

    # Callback функция для кнопки L1 (уменьшение скорости)
    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed: # Только при нажатии кнопки
            self.speed_mode = max(self.speed_mode - 1, 0) # Уменьшить режим скорости, минимум до 0
            rospy.loginfo(f"Speed Mode decreased to: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1) # Звуковой сигнал
            self.gait_manager.stop() # Остановить текущее движение
            self.status = 'stop' # Установить статус на 'stop'

    def l2_callback(self, new_state):
        pass # Функция для кнопки L2 (не реализована)

    def r2_callback(self, new_state):
        pass # Функция для кнопки R2 (не реализована)

    def square_callback(self, new_state):
        pass # Функция для кнопки SQUARE (не реализована)

    def triangle_callback(self, new_state):
        pass # Функция для кнопки TRIANGLE (не реализована)

    def cross_callback(self, new_state):
        pass # Функция для кнопки CROSS (крестик/A) (не реализована в этом файле)

    # === Обработчик кнопки Circle (Круг/B) - Авто-Get Up ===
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (Get Up) при нажатии кнопки Circle.
        Автоматически выбирает между lie_to_stand (лежа на животе)
        и recline_to_stand (лежа на спине) на основе состояния, определенного через IMU.
        Исключает активацию, если робот уже стоит.
        """
        if new_state == ButtonState.Pressed: # Только при нажатии кнопки
            rospy.loginfo(f"Circle (B) button pressed. Current IMU robot state: '{self.robot_state}'.")

            # !!! ДОБАВЛЕННАЯ ЛОГИКА ДЛЯ ИСКЛЮЧЕНИЯ ПОДЪЕМА СТОЯЩЕГО РОБОТА !!!
            if self.robot_state == 'stand_or_unknown':
                rospy.loginfo("Robot is already standing or state is unknown. Get Up action not needed.")
                self.board.set_buzzer(500, 0.05, 0.05, 2) # Короткий двойной зуммер, чтобы показать, что команда не выполнена
                return # Выходим из функции, если робот уже стоит или его положение неопределено
            # !!! КОНЕЦ ДОБАВЛЕННОЙ ЛОГИКИ !!!

            action_to_run = None # Переменная для хранения имени действия, которое нужно запустить
            log_msg = "" # Сообщение для логирования

            # Определяем, какое действие подъема нужно выполнить, на основе состояния IMU
            if self.robot_state == 'lie_to_stand':
                action_to_run = self.lie_to_stand_action_name # Робот лежит лицом вниз
                log_msg = "Robot state indicates lying on FRONT. Initiating lie_to_stand."
            elif self.robot_state == 'recline_to_stand':
                action_to_run = self.recline_to_stand_action_name # Робот лежит лицом вверх
                log_msg = "Robot state indicates lying on BACK. Initiating recline_to_stand."
            # Ветка 'else' для 'stand_or_unknown' была удалена, так как обрабатывается в начале функции
            # (return, если робот уже стоит).

            rospy.loginfo(log_msg) # Логируем выбранное действие
            self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звуковой сигнал перед выполнением действия

            try:
                if self.motion_manager is not None and action_to_run:
                    # Запускаем предопределенное действие подъема
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully.")
                    
                    # Сброс состояния робота и счетчиков IMU после отправки команды подъема.
                    # Предполагается, что после выполнения действия робот встанет.
                    self.robot_state = 'stand_or_unknown'
                    self.count_lie = 0
                    self.count_recline = 0
                    rospy.logdebug("Robot state set to 'stand_or_unknown' and IMU counters reset after Get Up command.")
                    
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")
                
    # Callback функция для кнопки Start (сброс высоты тела)
    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed: # Только при нажатии кнопки
            rospy.loginfo("Start button pressed. Resetting body height.")
            self.board.set_buzzer(1900, 0.1, 0.05, 1) # Звуковой сигнал
            self.gait_param = self.gait_manager.get_gait_param()
            # Вычисляем количество шагов для плавного изменения высоты до 0.025
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                for i in range(t):
                    # Постепенное изменение init_z_offset к 0.025
                    self.init_z_offset += 0.005 * abs(0.025 - self.init_z_offset) / (0.025 - self.init_z_offset)
                    self.gait_param['body_height'] = self.init_z_offset

                    # Определяем параметры времени для обновления (используем текущий speed_mode или базовый)
                    current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                    if self.speed_mode != 0:
                        self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                    else:
                        self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                    # Обновить параметры в GaitManager. step_num=1 означает, что это не первая походка.
                    self.gait_manager.update_param(current_period_time_for_update, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
                    time.sleep(0.05) # Небольшая задержка между шагами изменения высоты

    def hat_xl_callback(self, new_state):
        pass # Функция для HAT_XL (стрелки влево) (не реализована)

    def hat_xr_callback(self, new_state):
        pass # Функция для HAT_XR (стрелки вправо) (не реализована)

    def hat_yd_callback(self, new_state):
        pass # Функция для HAT_YD (стрелки вниз) (не реализована)

    def hat_yu_callback(self, new_state):
        pass # Функция для HAT_YU (стрелки вверх) (не реализована)

    # Главная callback функция для обработки сообщений Joy
    def joy_callback(self, joy_msg):
        # Преобразуем массив осей в словарь для удобства доступа по имени
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False # Флаг для отслеживания изменений осей
        # Преобразуем массив кнопок в словарь для удобства доступа по имени
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        self.callback(axes) # Вызов функции для обработки изменения высоты (RY ось)

        # Проверяем, изменились ли какие-либо оси (кроме RY, которая обрабатывается отдельно)
        for key, value in axes.items():
            if key != 'ry':
                if self.last_axes[key] != value:
                    axes_changed = True

        # Если оси изменились, вызываем axes_callback
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(f"Error in axes_callback: {e}")

        # Обработка состояний кнопок
        for key, value in buttons.items():
            new_state = ButtonState.Normal # Изначально считаем нормальное состояние
            if value != self.last_buttons[key]: # Если значение кнопки изменилось
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released # Нажата или отпущена
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal # Удерживается или нормальное

            callback = "".join([key, '_callback']) # Формируем имя callback-функции (например, 'circle_callback')

            # Если состояние кнопки не нормальное (т.е., была нажата, отпущена или удерживается)
            if new_state != ButtonState.Normal:
                if hasattr(self, callback): # Проверяем, существует ли такая callback-функция в классе
                    try:
                        getattr(self, callback)(new_state) # Вызываем соответствующую функцию
                    except Exception as e:
                        rospy.logerr(f"Error in {callback}: {e}")

        # Сохраняем текущее состояние кнопок и осей для следующего цикла
        self.last_buttons = buttons
        self.last_axes = axes

# Основная точка входа в программу
if __name__ == "__main__":
    node = JoystickController() # Создаем экземпляр контроллера
    try:
        rospy.spin() # Запускаем ROS-цикл, который будет обрабатывать сообщения
    except Exception as e:
        rospy.logerr(str(e)) # Логируем любые исключения, которые могут возникнуть