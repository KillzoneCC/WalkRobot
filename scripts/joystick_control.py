#!/usr/bin/env python3
# encoding: utf-8
import time
import math
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
# Предполагаем стандартный маппинг PlayStation: 1-'circle'(B), 3-'triangle'(Y), 4-'r1', 5-'l1'
# BUTTON_MAP индекс начинается с 0
BUTTON_MAP = 'cross', 'circle', 'triangle', 'square', 'r1', 'l1', 'r2', 'l2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

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
        self.speed_mode = 0 # Робот начинается с нулевой скорости

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
                'x_amp_base': 0.02, # Исправлено
                'init_z_offset': 0.025, # Начальная высота (по умолчанию)
                'z_move_amplitude': 0.018
            }
        }

        # Инициализация текущих параметров движения.
        self.period_time = list(self.speed_params[1]['period_time']) # Инициализируем базовыми значениями
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        # Высота тела по умолчанию, соответствует начальной позе
        self.init_z_offset = self.speed_params[1]['init_z_offset'] # 0.025
        self.time_stamp_ry = 0
        self.count_stop = 0
        self.status = 'stop' # Текущий статус движения робота: 'stop' или 'move'
        self.update_height = False # Флаг для обновления высоты тела
        self.update_param = False  # Флаг для обновления параметров походки

        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP))) # Исправлена синтаксическая ошибка
        self.mode = 0

        time.sleep(0.2)

        # Инициализация менеджера походки робота
        # GaitManager сам устанавливает робота в начальную позу
        self.gait_manager = GaitManager()
        
        # === ДОБАВЛЕНО: Инициализация MotionManager ===
        self.motion_manager = MotionManager()
        self.lie_to_stand_action_name = 'lie_to_stand'     # Подъем спереди (лицом вниз)
        self.recline_to_stand_action_name = 'recline_to_stand' # Подъем сзади (лицом вверх)
        # =============================================

        # === ДОБАВЛЕНО: Переменные для работы с IMU и состоянием падения ===
        # Состояние робота для Get Up
        self.fall_state = 'unknown' # 'lie', 'recline', 'stand', 'unknown'
        # Счетчики для фильтрации (как в мобильном приложении)
        self.count_lie = 0
        self.count_recline = 0
        # Порог счетчика для срабатывания
        self.FALL_COUNT_THRESHOLD = 15 # Может потребоваться настройка
        # Подписка на топик IMU
        # Согласно документации, основной топик это /imu
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.loginfo("Subscribed to IMU topic: /imu")
        # ===================================================================

        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Добавим начальную остановку робота при запуске
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Starting in Speed Mode: 0 (STOP)")
        rospy.loginfo("Controls: R1/L1 - Speed Up/Down, Circle (B) - Get Up (Auto-detect), Start - Reset Height")

    # === ДОБАВЛЕНО: Callback для данных IMU ===
    def imu_callback(self, msg: Imu):
        """
        Обрабатывает данные с IMU для определения состояния падения.
        Логика адаптирована для определения lie (лицом вниз) и recline (лицом вверх).
        """
        try:
            # Используем данные линейного ускорения из IMU
            # В системе координат робота: Z вверх/вниз по корпусу, Y влево/вправо, X вперёд/назад
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # Пороги для определения состояния (в m/s^2, примерно 9.81 m/s^2 = 1g)
            # Эти значения могут потребовать калибровки под конкретного робота
            STAND_THRESH = 7.0       # Если |Z| > этого и |Y| < порог, робот вертикален или лежит лицом вниз/вверх
            LIE_FACE_DOWN_THRESH = -7.0  # Если Z < этого и |Y| "спокойный", робот лицом вниз
            RECLINE_FACE_UP_THRESH = 7.0 # Если Z > этого и |Y| "спокойный", робот лицом вверх
            SIDEWAYS_THRESH = 4.0    # Если |Y| > этого, робот лежит на боку или в нестабильном положении

            # Предполагаем, что в состоянии покоя (stand или lie/recline) ускорение по X близко к 0.
            # Основное различие между положениями - знак и величина ускорения по Z и Y.

            # Сброс счетчиков, если робот в устойчивом положении
            if abs(az) > STAND_THRESH and abs(ay) < SIDEWAYS_THRESH:
                # Робот в устойчивом вертикальном или горизонтальном положении
                if az > RECLINE_FACE_UP_THRESH:
                    # Лежит на спине (лицом вверх)
                    # Z-ускорение положительное (направлено вверх, как гравитация в СК робота)
                    self.count_recline += 1
                    self.count_lie = max(0, self.count_lie - 1) # Сброс другого счетчика
                    # rospy.logdebug("IMU: Detected potential recline (face up)")
                elif az < LIE_FACE_DOWN_THRESH:
                    # Лежит на животе (лицом вниз)
                    # Z-ускорение отрицательное (направлено вниз)
                    self.count_lie += 1
                    self.count_recline = max(0, self.count_recline - 1) # Сброс другого счетчика
                    # rospy.logdebug("IMU: Detected potential lie (face down)")
                else:
                    # Вертикальное положение (стоит)
                    # Z ускорение между порогами, Y спокоен
                    self.count_lie = max(0, self.count_lie - 1)
                    self.count_recline = max(0, self.count_recline - 1)
                    # rospy.logdebug("IMU: Detected potential stand")
            else:
                # Робот лежит на боку или в нестабильном положении
                # Не изменяем счетчики или даже сбрасываем их медленно
                # rospy.logdebug(f"IMU: Detected side or unstable position. ay={ay:.2f}, az={az:.2f}")
                # Можно немного сбрасывать, чтобы избежать застревания
                self.count_lie = max(0, self.count_lie - 1)
                self.count_recline = max(0, self.count_recline - 1)

            # Обновляем состояние на основе счетчиков
            # Логируем только при изменении состояния для уменьшения логов
            old_state = self.fall_state
            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.fall_state = 'lie'
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.fall_state = 'recline'
            # else:
            #     # Если ни один счетчик не превысил порог, состояние может оставаться неизменным
            #     # или становиться 'stand'/'unknown' при сильном сбросе.
            #     # Пока оставим как есть. Можно добавить условие для 'stand'.

            if old_state != self.fall_state:
                 rospy.loginfo(f"Fall state updated by IMU: {old_state} -> {self.fall_state} (lie:{self.count_lie}, recl:{self.count_recline})")

            # rospy.logdebug(f"Counters: lie={self.count_lie}, recline={self.count_recline}, state={self.fall_state}")

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data in imu_callback: {e}")


    # ======================================================================

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
        else: # Если робот в режиме скорости 1, 2 или 3
            current_speed_settings = self.speed_params[self.speed_mode]

            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0

            # Обновляем period_time из текущих настроек скорости
            self.period_time = list(current_speed_settings['period_time'])

            # Управление движением вперед/назад по оси LY
            if axes['ly'] > 0.3: # Движение вперед
                self.update_param = True
                self.x_move_amplitude = current_speed_settings['x_amp_base']
            elif axes['ly'] < -0.3: # Движение назад
                self.update_param = True
                self.x_move_amplitude = -current_speed_settings['x_amp_base']

            # Управление движением влево/вправо по оси LX
            if axes['lx'] > 0.3: # Движение вправо
                self.period_time[2] = 0.025
                self.update_param = True
                self.y_move_amplitude = 0.015
            elif axes['lx'] < -0.3: # Движение влево
                self.period_time[2] = 0.025
                self.update_param = True
                self.y_move_amplitude = -0.015

            # Управление поворотом по оси RX
            if axes['rx'] > 0.3: # Поворот вправо
                self.update_param = True
                self.angle_move_amplitude = 8
            elif axes['rx'] < -0.3: # Поворот влево
                self.update_param = True
                self.angle_move_amplitude = -8

        # === Логика запуска движения ===
        # Запускаем движение, если:
        # 1. Есть команда обновления параметров (изменился стик) ИЛИ
        # 2. Робот стоял, перешел в режим движения (speed_mode > 0) и получает команду движения
        if self.update_param or (self.speed_mode > 0 and self.status == 'stop' and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0)):
            # === КЛЮЧЕВОЕ ИЗМЕНЕНИЕ ===
            # Получаем ТЕКУЩИЕ параметры от GaitManager (включая позу туловища)
            self.gait_param = self.gait_manager.get_gait_param()
            
            # Устанавливаем ТОЛЬКО текущую высоту тела и z_move_amplitude
            # НЕ копируем другие параметры позы туловища (init_x_offset, init_y_offset, init_pitch_offset и т.д.)
            # из speed_params. Они остаются такими, какими их установил GaitManager при инициализации.
            self.gait_param['init_z_offset'] = self.init_z_offset

            # Устанавливаем высоту подъема ноги из параметров текущей скорости
            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']
            else:
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

            # Отправляем обновленные параметры движения в GaitManager
            # self.gait_param содержит текущие параметры позы, установленные GaitManager,
            # которые НЕ перезаписываются.
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            # ==========================

        # Логика для перехода между состоянием 'stop' и 'move'
        if self.status == 'stop' and (self.update_param or (self.speed_mode > 0 and (self.x_move_amplitude != 0 or self.y_move_amplitude != 0 or self.angle_move_amplitude != 0))):
            self.status = 'move'
        elif self.status == 'move' and not self.update_param and self.x_move_amplitude == 0 and self.y_move_amplitude == 0 and self.angle_move_amplitude == 0 and self.speed_mode > 0:
            # Если робот был в движении, но все амплитуды стали 0 (и не режим 0) - останавливаем.
            self.status = 'stop'
            self.gait_manager.stop()
        elif self.speed_mode == 0 and self.status == 'move':
            # Если переключились на 0 скорость, и он двигался
            self.gait_manager.stop()
            self.status = 'stop'

        self.update_param = False # Сбрасываем флаг обновления параметров

    # === ИСПРАВЛЕНО: Восстановлен метод callback для управления высотой ===
    def callback(self, axes):
        # Проверяем временную метку, чтобы не обновлять высоту слишком часто
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            # Управление высотой тела робота (ось RY)
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

            # Если высота тела изменилась
            if self.update_height:
                # === КЛЮЧЕВОЕ ИЗМЕНЕНИЕ ===
                # Получаем ТЕКУЩИЕ параметры от GaitManager (включая позу туловища)
                self.gait_param = self.gait_manager.get_gait_param()
                
                # Используем body_height как в старом коде для update_param
                self.gait_param['body_height'] = self.init_z_offset

                # Определяем period_time для передачи в update_param
                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                # Z-амплитуда шага
                if self.speed_mode != 0:
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                # НЕ копируем другие параметры позы туловища из speed_params
                # Параметры позы остаются текущими, установленными GaitManager.

                # Передаем обновленные параметры (только высота и z_move_amplitude)
                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                # ==========================
                self.time_stamp_ry = rospy.get_time() + 0.05 # Задержка

    def select_callback(self, new_state):
        pass

    # === ИСПРАВЛЕНО: Обработчики кнопок R1 и L1 для изменения скорости ===
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Переключение скорости по возрастанию
            self.speed_mode = min(self.speed_mode + 1, 3) # Ограничиваем максимумом 3
            rospy.loginfo(f"Speed Mode increased to: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            # Переключение скорости по убыванию
            self.speed_mode = max(self.speed_mode - 1, 0) # Ограничиваем минимумом 0
            rospy.loginfo(f"Speed Mode decreased to: {self.speed_mode}")
            self.board.set_buzzer(1000 - self.speed_mode * 100, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'
    # =====================================================================

    def l2_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    # === ИЗМЕНЕНО: Обработчик кнопки Circle (предполагаемая кнопка "B") - Авто-Get Up ===
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (Get Up).
        Автоматически выбирает между lie_to_stand и recline_to_stand
        на основе состояния, определенного через IMU.
        """
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle (B) button pressed. Current detected fall state is '{self.fall_state}'.")

            action_to_run = None
            if self.fall_state == 'lie':
                action_to_run = self.lie_to_stand_action_name
                rospy.loginfo("Initiating lie_to_stand (face down -> stand).")
            elif self.fall_state == 'recline':
                action_to_run = self.recline_to_stand_action_name
                rospy.loginfo("Initiating recline_to_stand (face up -> stand).")
            else: # 'unknown' или 'stand'
                rospy.logwarn(f"Robot fall state is '{self.fall_state}'. Defaulting to lie_to_stand.")
                action_to_run = self.lie_to_stand_action_name

            self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звуковой сигнал

            # Запуск выбранного действия
            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' successfully initiated via MotionManager.")
                    # Сброс состояния после попытки подъема.
                    # Это предотвращает повторную попытку, если кнопка нажата снова,
                    # пока IMU не обновит состояние.
                    # Предполагаем, что после команды подъема робот больше не лежит.
                    # Можно сбросить в 'unknown' и дать IMU определить новое состояние,
                    # или установить в 'stand', если уверены, что действие сработает.
                    # Установка в 'stand' кажется более безопасной.
                    # self.fall_state = 'stand' # Или 'unknown'?
                    # self.count_lie = 0
                    # self.count_recline = 0
                    # rospy.loginfo("Fall state temporarily set to 'stand' after Get Up command.")
                else:
                    rospy.logwarn("MotionManager is not initialized or no action to run. Cannot perform get up action.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action for '{action_to_run}': {e}")
                
    # ======================================================================

    def triangle_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def start_callback(self, new_state):
        # Сброс высоты тела к начальной (как в старом коде)
        if new_state == ButtonState.Pressed:
            rospy.loginfo("Start button pressed. Resetting body height to default.")
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            # === КЛЮЧЕВОЕ ИЗМЕНЕНИЕ ===
            self.gait_param = self.gait_manager.get_gait_param() # Получаем текущие параметры
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

                    # НЕ копируем другие параметры позы туловища из speed_params
                    # Передача параметров (только высота и z_move_amplitude)
                    self.gait_manager.update_param(current_period_time_for_update, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
                    time.sleep(0.05)
            # ==========================

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

        # === ИСПРАВЛЕНО: Теперь метод callback существует ===
        # Обработка высоты тела (независимо от режима скорости)
        self.callback(axes)
        # ===================================================

        # Проверяем, изменились ли значения осей (кроме 'ry')
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
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal

            callback = "".join([key, '_callback'])

            if new_state != ButtonState.Normal:
                # Проверка на существование метода callback добавлена для безопасности
                if hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in {callback}: {str(e)}")
                else:
                    # Опционально: логировать, если callback для кнопки не найден
                    # rospy.logdebug(f"No callback defined for button '{key}'")
                    pass

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
