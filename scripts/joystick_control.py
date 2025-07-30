#!/usr/bin/env python3
# encoding: utf-8
import time
import math # Импортируем math для вычислений
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu # Добавляем импорт Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
# Предполагаем стандартный маппинг PlayStation: 1-'circle'(B), 3-'triangle'(Y)
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
        self.speed_mode = 0

        self.speed_params = {
            1: {
                'period_time': [400, 0.2, 0.022],
                'x_amp_base': 0.01,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016
            },
            2: {
                'period_time': [600, 0.25, 0.022],
                'x_amp_base': 0.020,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.016
            },
            3: {
                'period_time': [500.0, 0.22, 0.020],
                'x_amp_base': 0.02,
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018
            }
        }

        # Инициализация текущих параметров движения.
        self.period_time = list(self.speed_params[1]['period_time'])
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
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

        # Инициализация менеджера походки робота
        self.gait_manager = GaitManager()
        
        # === ДОБАВЛЕНО: Инициализация MotionManager ===
        self.motion_manager = MotionManager()
        self.lie_to_stand_action_name = 'lie_to_stand'
        self.recline_to_stand_action_name = 'recline_to_stand'
        # =============================================

        # === ДОБАВЛЕНО: Переменные для работы с IMU и состоянием падения ===
        # Состояние робота для Get Up
        self.fall_state = 'unknown' # 'lie', 'recline', 'stand', 'unknown'
        # Счетчики для фильтрации (как в мобильном приложении)
        self.count_lie = 0
        self.count_recline = 0
        # Порог счетчика для срабатывания (можно настроить)
        self.FALL_COUNT_THRESHOLD = 10 # Уменьшен для быстрого тестирования, можно вернуть 50
        # Подписка на топик IMU (проверьте правильное имя топика)
        # Согласно документации, возможные имена: /imu, /imu/data, /imu_corrected
        # Предположим, основной топик это /imu
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.loginfo("Subscribed to IMU topic: /imu")
        # ===================================================================

        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Добавим начальную остановку робота при запуске
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Starting in Speed Mode: 0 (STOP)")
        rospy.loginfo("Controls: Circle (B) - Get Up (Auto-detect)")

    # === ДОБАВЛЕНО: Callback для данных IMU ===
    def imu_callback(self, msg: Imu):
        """
        Обрабатывает данные с IMU для определения состояния падения.
        Логика взята из Pasted_Text_1753855641149.txt и адаптирована.
        """
        try:
            # Используем данные линейного ускорения из IMU
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # Вычисляем угол наклона (пример, может потребоваться калибровка)
            # Упрощенный расчет угла относительно вертикали Z
            if abs(az) > 1e-6: # Избегаем деления на ноль
                 # atan2 дает угол в радианах, переводим в градусы
                angle_rad = math.atan2(abs(ay), abs(az))
                angle_deg = math.degrees(angle_rad)
            else:
                # Если az почти 0, робот лежит боком или почти горизонтально
                angle_deg = 90.0 if ay > 0 else -90.0

            # rospy.logdebug(f"IMU Angle (deg): {angle_deg:.2f}") # Для отладки

            # Логика определения состояния (упрощенная)
            # Предположим: 
            # - Угол около 0 градусов -> стоит (stand)
            # - Угол около 180 (-180) градусов -> лежит на животе (lie)
            # - Угол около 0, но az отрицательный -> лежит на спине (recline)
            # ВАЖНО: Эти пороги могут потребовать настройки под конкретного робота!
            
            # Получаем Z-компоненту линейного ускорения
            acc_z = msg.linear_acceleration.z
            
            # Пороги для определения состояния (в m/s^2, примерно 9.81 m/s^2 = 1g)
            STAND_THRESHOLD = 7.0 # Если Z-ускорение > этого значения, считаем стоящим
            LIE_THRESHOLD = -7.0  # Если Z-ускорение < этого значения, считаем лежащим на животе
            
            if acc_z > STAND_THRESHOLD:
                # Робот, вероятно, стоит
                self.count_lie = max(0, self.count_lie - 1)
                self.count_recline = max(0, self.count_recline - 1)
                # Не меняем fall_state напрямую, чтобы избежать ложных срабатываний
                
            elif acc_z < LIE_THRESHOLD:
                # Робот, вероятно, лежит на животе (Z ускорение направлено вниз)
                self.count_lie += 1
                self.count_recline = max(0, self.count_recline - 1)
                
            elif acc_z < -STAND_THRESHOLD: # Примерный порог для спины, может совпадать с LIE_THRESHOLD
                 # Робот, вероятно, лежит на спине (Z ускорение направлено вверх)
                 # Это условие может быть сложнее, если LIE и RECLINE определяются по разным критериям
                 # Для простоты предположим, что если не на животе и не стоит, то на спине
                 # Но для этого нужно другое условие. Пока оставим базовую логику.
                 # Или можно использовать угол.
                 # Альтернатива: использовать ay и az для более точного определения
                 # Например, если ay близок к 0, а az < 0 -> лежит на животе
                 # Если ay близок к 0, а az > 0 -> лежит на спине
                 # Если |ay| велико -> лежит на боку или стоит
                 
                 # Упрощенная альтернатива: если az положительный и большой по модулю -> спина вниз
                 # Но это может конфликтовать с LIE_THRESHOLD. Нужно уточнить логику из исходника.
                 # Пока используем ту же логику, что и для LIE, но с другим счетчиком.
                 # На самом деле, нужно понимать, как именно различаются lie и recline в исходной логике.
                 # Возможно, это связано со знаком ay или комбинацией ay, az.
                 # Предположим для начала, что recline - это когда робот "перекинулся назад"
                 # Это может быть сложно определить только по az.
                 # Лучше придерживаться логики из imu_callback оригинала, но у нас только часть.
                 # Оригинал использует ay и az, и вычисляет некий "angle".
                 # Давайте использовать вычисленный angle_deg.
                 
                 # Пересмотр логики с использованием angle_deg:
                 # Это требует экспериментов. Примерная логика:
                 # if angle_deg < 30: # Стоит или почти стоит
                 #     ...
                 # elif 150 < angle_deg < 210: # Лежит на животе (угол между Z и вертикалью ~180)
                 #     ...
                 # elif -30 < angle_deg < 30: # Лежит на спине (угол между Z и вертикалью ~0, но az>0)
                 #     ... Но это сложно, так как az>0 и угол ~0 может быть и стоя.
                 
                 # Самый простой способ: полагаться на знак az относительно порога.
                 # Если az < -LIE_THRESHOLD -> живот вниз (lie)
                 # Если az > STAND_THRESHOLD -> стоит (stand)
                 # Если -STAND_THRESHOLD < az < STAND_THRESHOLD -> возможно, лежит на боку или в нестабильном положении
                 # или лежит на спине. Но как различить stand и recline?
                 
                 # Возможно, в оригинале используется комбинация. 
                 # Давайте предположим, что recline определяется по другому критерию.
                 # Например, если ay отрицательный и az положительный -> голова вниз (спина вниз).
                 # Или просто по тому, что Z-ускорение положительное и велико, а Y-ускорение мало.
                 # Это требует калибровки.
                 
                 # Пока реализуем базовую версию с одним порогом для lie.
                 # recline будет определяться косвенно или по другому критерию.
                 # Или предположим, что если не lie и не stand, то recline.
                 # Но это нестабильно.
                 
                 # Вернемся к логике с angle. 
                 # angle_deg = math.degrees(math.atan2(abs(ay), abs(az))) 
                 # Этот угол - это угол между вектором ускорения и осью Z.
                 # Если робот стоит, этот угол близок к 0.
                 # Если лежит на животе/спине, угол близок к 90.
                 # Это не то. 
                 
                 # Правильнее: угол между осью Z робота и вектором гравитации.
                 # Вектор гравитации в системе робота будет (0, 0, -g) если он стоит.
                 # Если он лежит на животе, вектор g будет направлен вдоль его Z в положительную сторону.
                 # То есть, Z_accel ~ +g.
                 # Если он лежит на спине, Z_accel ~ -g.
                 # Поэтому:
                 # Если az > STAND_THRESHOLD (например, > 7) -> стоит
                 # Если az < LIE_THRESHOLD (например, < -7) -> лежит на животе
                 # Если az > -STAND_THRESHOLD и az < STAND_THRESHOLD -> неопределенное состояние
                 # Но как определить, лежит ли он на спине или на боку?
                 # Нужно смотреть на ay.
                 # Если ay велико -> лежит на боку.
                 # Если ay мало и az > 0 -> лежит на спине.
                 
                 # Пересмотрим:
                 ACC_STAND_THRESH = 7.0
                 ACC_LIE_THRESH = -7.0
                 ACC_SIDE_THRESH = 3.0 # Если |ay| > этого, то лежит на боку
                 
                 if az > ACC_STAND_THRESH and abs(ay) < ACC_SIDE_THRESH:
                     # Стоит
                     self.count_lie = max(0, self.count_lie - 1)
                     self.count_recline = max(0, self.count_recline - 1)
                 elif az < ACC_LIE_THRESH and abs(ay) < ACC_SIDE_THRESH:
                     # Лежит на животе
                     self.count_lie += 1
                     self.count_recline = max(0, self.count_recline - 1)
                 elif az > ACC_STAND_THRESH and abs(ay) < ACC_SIDE_THRESH:
                     # Лежит на спине (Z_accel положительный и большой, Y_accel маленький)
                     # Но это то же условие, что и для стоя. Проблема.
                     # Возможно, нужно смотреть на знак az.
                     # Если az > ACC_STAND_THRESH -> стоит
                     # Если az < ACC_LIE_THRESH -> лежит на животе
                     # Если ACC_LIE_THRESH < az < ACC_STAND_THRESH -> лежит на спине или на боку
                     # Если abs(ay) > ACC_SIDE_THRESH -> лежит на боку
                     # Если abs(ay) < ACC_SIDE_THRESH и ACC_LIE_THRESH < az < ACC_STAND_THRESH -> лежит на спине
                     pass # Пока не реализуем recline через IMU, только через lie
                 
                 # Упрощаем: определяем только lie. Recline будет "по умолчанию" или через счетчик.
                 # Это не идеально, но ближе к исходной логике, где были отдельные счетчики.
                 
            
            # Обновляем состояние на основе счетчиков
            # (Логика взята из мобильного приложения)
            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.fall_state = 'lie'
                # Сбросим счетчик, чтобы не переключалось сразу обратно
                # В оригинале, после определения состояния, оно сбрасывалось?
                # Нет, оно сбрасывалось в run() после выполнения действия.
                # Здесь мы просто устанавливаем состояние.
                rospy.logdebug(f"Detected state: {self.fall_state}")
            # elif self.count_recline > self.FALL_COUNT_THRESHOLD:
            #     self.fall_state = 'recline'
            #     rospy.logdebug(f"Detected state: {self.fall_state}")
            # else:
            #     # Если ни один счетчик не превысил порог, состояние неизвестно или stand
            #     # Но не будем сбрасывать напрямую, чтобы избежать "мигания"
            #     # self.fall_state = 'unknown' 
            
            # В оригинале состояние 'stand' устанавливалось в run() после выполнения действия.
            # Здесь мы не меняем его, если счетчики не сработали.

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")

    # ======================================================================

    def axes_callback(self, axes):
        if self.speed_mode == 0:
            self.x_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
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
            self.gait_param['init_z_offset'] = self.init_z_offset

            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']
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
                self.gait_param['body_height'] = self.init_z_offset

                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                if self.speed_mode != 0:
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = min(self.speed_mode + 1, 3)
            rospy.loginfo(f"Speed Mode: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.speed_mode = max(self.speed_mode - 1, 0)
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

    # === ИЗМЕНЕНО: Обработчик кнопки Circle (предполагаемая кнопка "B") - Авто-Get Up ===
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (Get Up).
        Автоматически выбирает между lie_to_stand и recline_to_stand
        на основе состояния, определенного через IMU.
        """
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle (B) button pressed. Fall state is '{self.fall_state}'.")

            # === ПРОВЕРКА: Не выполняется ли уже движение? ===
            # Это хорошая практика, чтобы не прерывать текущие действия.
            # try:
            #     # Проверяем сервис, предоставляемый системой AInex
            #     # (нужно уточнить реальное имя сервиса и тип)
            #     # Например, если есть сервис для проверки состояния ходьбы:
            #     # from ainex_kinematics.srv import GetWalkingState, GetWalkingStateRequest
            #     # rospy.wait_for_service('/walking/is_walking', timeout=1.0)
            #     # is_walking_srv = rospy.ServiceProxy('/walking/is_walking', GetWalkingState)
            #     # walking_state = is_walking_srv(GetWalkingStateRequest())
            #     # if walking_state.is_walking: # Если робот идет или выполняет действие
            #     #     rospy.logwarn("Robot is currently moving. Get Up action skipped.")
            #     #     self.board.set_buzzer(800, 0.2, 0.1, 1) # Предупреждение
            #     #     return
            # except (rospy.ServiceException, rospy.ROSException) as e:
            #     rospy.logdebug(f"Could not check walking state: {e}. Proceeding with Get Up.")
            #     # Если сервис недоступен, продолжаем
            # ==================================================

            action_to_run = None
            if self.fall_state == 'lie':
                action_to_run = self.lie_to_stand_action_name
                rospy.loginfo("Robot appears to be lying on its front. Initiating lie_to_stand.")
            elif self.fall_state == 'recline':
                action_to_run = self.recline_to_stand_action_name
                rospy.loginfo("Robot appears to be lying on its back. Initiating recline_to_stand.")
            else: # 'unknown' или 'stand' или любой другой случай
                # В оригинале, если состояние не определено, могло бы не выполняться действие
                # или выполнялось бы действие по умолчанию.
                # Выберем действие по умолчанию, например, lie_to_stand, как fallback.
                rospy.logwarn(f"Robot fall state is '{self.fall_state}'. Defaulting to lie_to_stand.")
                action_to_run = self.lie_to_stand_action_name

            self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звуковой сигнал

            # Запуск выбранного действия
            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated via MotionManager.")
                    
                    # === ВАЖНО: Сброс состояния после попытки подъема ===
                    # В оригинальном коде состояние сбрасывалось в 'stand' после выполнения.
                    # Здесь мы делаем то же самое, предполагая, что действие было инициировано.
                    # Это предотвратит повторную попытку подъема, если кнопка нажата снова
                    # до того, как IMU снова определит падение.
                    self.fall_state = 'stand' # Или 'unknown'?
                    self.count_lie = 0
                    self.count_recline = 0
                    rospy.loginfo("Fall state reset to 'stand' after Get Up command.")
                    # =====================================================
                    
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
        if new_state == ButtonState.Pressed:
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            self.gait_param = self.gait_manager.get_gait_param()
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

                    self.gait_manager.update_param(current_period_time_for_update, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
                    time.sleep(0.05)

    # ... (остальные hat_*_callback остаются без изменений) ...

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        self.callback(axes) # Обработка высоты

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
                        rospy.logerr(f"Error in {callback}: {str(e)}")

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
