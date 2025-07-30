#!/usr/bin/env python3
# encoding: utf-8
import time
import math # Не забудьте импортировать math
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

# Используем BUTTON_MAP из проверенных рабочих примеров
AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
# BUTTON_MAP из Pasted_Text_1753855746643.txt, предполагая, что R1(индекс 7) и L1(индекс 6)
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
        self.speed_mode = 0 # Начинаем с остановки

        self.speed_params = {
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
            3: { # Скорость 3
                'period_time': [500.0, 0.22, 0.020],
                'x_amp_base': 0.02, # Исправлено с 0.03
                'init_z_offset': 0.025,
                'z_move_amplitude': 0.018
            }
        }

        # Инициализация параметров движения
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

        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        # Исправлена синтаксическая ошибка и соответствие длине BUTTON_MAP
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP))) 
        self.mode = 0

        time.sleep(0.2)

        # Инициализация менеджера походки робота
        self.gait_manager = GaitManager()
        
        # === MotionManager для Get Up ===
        self.motion_manager = MotionManager()
        # Имена действий из Pasted_Text_1753855641149.txt
        self.lie_to_stand_action_name = 'lie_to_stand'     
        self.recline_to_stand_action_name = 'recline_to_stand' 
        # =============================================

        # === Переменные для работы с IMU и состоянием падения ===
        # Состояние и счетчики из Pasted_Text_1753855641149.txt
        self.robot_state = 'stand' # Используем robot_state вместо fall_state для согласованности
        self.count_lie = 0
        self.count_recline = 0
        # Порог счетчика из примера мобильного приложения
        self.FALL_COUNT_THRESHOLD = 50 
        
        # Подписка на топик IMU
        # Из документации: /imu, /imu_corrected, /imu_raw
        # Пробуем основной топик
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.loginfo("Subscribed to IMU topic: /imu")
        # ===================================================================

        # Подписка на топик ROS '/joy'
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Начальная остановка робота
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Speed Mode: 0 (STOP)")
        rospy.loginfo("Controls: R1/L1 - Speed Up/Down, Circle (B) - Get Up (Auto-detect)")

    # === Callback для данных IMU, ЛОГИКА ИЗ МОБИЛЬНОГО ПРИЛОЖЕНИЯ ===
    def imu_callback(self, msg: Imu):
        """
        Обрабатывает данные с IMU для определения состояния падения.
        Логика адаптирована из Pasted_Text_1753855641149.txt (мобильное приложение).
        """
        try:
            # Используем Y и Z компоненты линейного ускорения как в примере
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # === ЛОГИКА ИЗ МОБИЛЬНОГО ПРИЛОЖЕНИЯ ===
            # Вычисляем угол наклона (пример из Pasted_Text_1753855641149.txt)
            # angle = math.atan2(abs(ay), abs(az)) * 180.0 / math.pi
            # Однако, в мобильном приложении проверка была angle < 30 И az < -7 или az > 7.
            # Это немного противоречиво, так как angle < 30 означает, что вектор ускорения
            # близок к одной из осей. Но давайте следовать исходной логике.
            
            # Упрощенная проверка, основанная на az:
            # Если az << 0, Z-ускорение направлено вниз -> робот лежит животом вниз.
            # Если az >> 0, Z-ускорение направлено вверх -> робот лежит спиной вверх.
            # Если |az| ~ 9.8 и |ay| велико -> робот лежит на боку.
            # Если |az| ~ 9.8 и |ay| мало -> робот лежит лицом вниз/вверх.
            # Если |az| << 9.8 -> робот не в горизонтальном положении (стоит, падает).

            # Пороги (настройка под вашего робота, значения из мобильного приложения)
            ACCEL_THRESH = 7.0 # Порог ускорения для определения горизонтального положения
            ANGLE_THRESH = 30.0 # Порог угла из мобильного приложения
            COUNT_INCREMENT = 1
            COUNT_DECREMENT = 1

            # Вычисляем угол (как в мобильном приложении)
            if abs(az) > 1e-6: # Избегаем деления на ноль
                angle_rad = math.atan2(abs(ay), abs(az))
                angle_deg = math.degrees(angle_rad)
            else:
                # Если az почти 0, робот лежит боком или почти горизонтально
                angle_deg = 90.0 if ay > 0 else -90.0

            # rospy.logdebug(f"IMU Raw: ay={ay:.2f}, az={az:.2f}, angle={angle_deg:.2f}") # Для отладки

            # Логика инкремента счетчиков, как в мобильном приложении
            # Проверяем, находится ли робот в горизонтальном положении и угол < 30
            if angle_deg < ANGLE_THRESH: 
                if az < -ACCEL_THRESH: # Лежит животом вниз
                     self.count_lie += COUNT_INCREMENT
                     self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                     # rospy.logdebug(f"IMU: Incrementing LIE count. lie={self.count_lie}, recl={self.count_recline}")
                elif az > ACCEL_THRESH: # Лежит спиной вверх
                     self.count_recline += COUNT_INCREMENT
                     self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                     # rospy.logdebug(f"IMU: Incrementing RECLINE count. lie={self.count_lie}, recl={self.count_recline}")
                else:
                    # Угол < 30, но az не соответствует лежанию лицом вниз/вверх
                    # (например, стоит вертикально или лежит на боку с малым ay)
                    # Медленно сбрасываем счетчики
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                    # rospy.logdebug(f"IMU: Decrementing counts (angle<30, az mid). lie={self.count_lie}, recl={self.count_recline}")
            else:
                 # Угол >= 30 (робот лежит на боку или в нестабильном положении)
                 # Медленно сбрасываем счетчики
                 self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                 self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                 # rospy.logdebug(f"IMU: Decrementing counts (angle>=30). lie={self.count_lie}, recl={self.count_recline}")

            # Сохраняем старое состояние для логирования изменений
            old_state = self.robot_state

            # Проверяем счетчики и обновляем состояние
            # Логика из примера: если счетчик > порога, состояние меняется
            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'lie_to_stand' # Состояние, требующее lie_to_stand
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'recline_to_stand' # Состояние, требующее recline_to_stand
            # else:
            #     # Состояние 'stand' устанавливается в circle_callback после попытки подъема
            #     # или может оставаться неизменным, если счетчики просто уменьшились.
            #     pass

            # Логируем изменение состояния
            if old_state != self.robot_state:
                 rospy.loginfo(f"IMU detected robot state change: {old_state} -> {self.robot_state} (lie:{self.count_lie}, recl:{self.count_recline})")

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data in imu_callback: {e}")


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
            # КЛЮЧЕВОЕ: Устанавливаем ТОЛЬКО init_z_offset и z_move_amplitude
            # Не копируем другие параметры позы из speed_params
            self.gait_param['init_z_offset'] = self.init_z_offset

            if self.speed_mode > 0:
                self.gait_param['z_move_amplitude'] = current_speed_settings['z_move_amplitude']
            else:
                self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)

        # Логика перехода между состояниями
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
        # Обработка высоты тела (ось RY)
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            if axes['ry'] < -0.5: # Поднять
                self.update_height = True
                self.init_z_offset += 0.005
                if self.init_z_offset > 0.06:
                    self.update_height = False
                    self.init_z_offset = 0.06
            elif axes['ry'] > 0.5: # Опустить
                self.update_height = True
                self.init_z_offset += -0.005
                if self.init_z_offset < 0.025:
                    self.update_height = False
                    self.init_z_offset = 0.025

            if self.update_height:
                self.gait_param = self.gait_manager.get_gait_param()
                # Используем body_height для update_param как в старом коде
                self.gait_param['body_height'] = self.init_z_offset

                current_period_time_for_update = list(self.speed_params[self.speed_mode]['period_time']) if self.speed_mode != 0 else list(self.speed_params[1]['period_time'])

                if self.speed_mode != 0:
                    self.gait_param['z_move_amplitude'] = self.speed_params[self.speed_mode]['z_move_amplitude']
                else:
                    self.gait_param['z_move_amplitude'] = self.speed_params[1]['z_move_amplitude']

                # КЛЮЧЕВОЕ: НЕ копируем другие параметры позы из speed_params
                self.gait_manager.update_param(current_period_time_for_update, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        pass

    # === Обработчики кнопок R1 и L1 для изменения скорости ===
    # Восстановлены из Pasted_Text_1753855746643.txt
    def r1_callback(self, new_state):
        # rospy.loginfo("R1 callback triggered") # Для отладки
        if new_state == ButtonState.Pressed:
            # Переключение скорости по возрастанию
            self.speed_mode = min(self.speed_mode + 1, 3) # Ограничиваем максимумом 3
            rospy.loginfo(f"Speed Mode increased to: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'

    def l1_callback(self, new_state):
        # rospy.loginfo("L1 callback triggered") # Для отладки
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
    def triangle_callback(self, new_state):
        pass
    def cross_callback(self, new_state):
        pass

    # === Обработчик кнопки Circle (B) - Авто-Get Up ===
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (Get Up).
        Автоматически выбирает между lie_to_stand и recline_to_stand
        на основе состояния, определенного через IMU.
        """
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle (B) button pressed. Current IMU robot state: '{self.robot_state}'.")

            action_to_run = None
            log_msg = ""
            
            # Определяем действие на основе состояния, определенного IMU
            if self.robot_state == 'lie_to_stand':
                action_to_run = self.lie_to_stand_action_name
                log_msg = "Robot state indicates lying on FRONT. Initiating lie_to_stand."
            elif self.robot_state == 'recline_to_stand':
                action_to_run = self.recline_to_stand_action_name
                log_msg = "Robot state indicates lying on BACK. Initiating recline_to_stand."
            else: # 'stand' или 'unknown'
                # fallback: как в мобильном приложении - выбираем lie_to_stand
                rospy.logwarn(f"Robot state is '{self.robot_state}'. Defaulting to lie_to_stand (front).")
                action_to_run = self.lie_to_stand_action_name
                log_msg = "Robot state unknown or stand. Defaulting to lie_to_stand (front)."

            rospy.loginfo(log_msg)
            self.board.set_buzzer(1500, 0.1, 0.05, 1)

            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully.")
                    
                    # === ВАЖНО: Сброс состояния и счетчиков ===
                    # После отправки команды подъема сбрасываем состояние и счетчики,
                    # чтобы IMU мог определить новое положение.
                    # Это предотвращает повторную попытку с тем же действием.
                    # Предполагаем, что робот больше не лежит в предыдущем положении.
                    # Можно установить в 'stand' и сбросить счетчики, или просто сбросить счетчики
                    # и позволить IMU определить новое состояние.
                    # Установка в 'stand' кажется более безопасной, чтобы не повторять действие.
                    self.robot_state = 'stand' 
                    self.count_lie = 0
                    self.count_recline = 0
                    rospy.logdebug("Robot state set to 'stand' and IMU counters reset after Get Up command.")
                    
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")
                
    # ======================================================================

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            rospy.loginfo("Start button pressed. Resetting body height.")
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

                    # КЛЮЧЕВОЕ: НЕ копируем другие параметры позы из speed_params
                    self.gait_manager.update_param(current_period_time_for_update, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
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

        self.callback(axes) # Обработка высоты

        for key, value in axes.items():
            if key != 'ry':
                if self.last_axes[key] != value:
                    axes_changed = True

        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(f"Error in axes_callback: {e}")

        # Обработка состояний кнопок
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal

            callback = "".join([key, '_callback'])

            if new_state != ButtonState.Normal:
                # Проверяем, существует ли метод callback
                if hasattr(self, callback):
                    try:
                        # rospy.logdebug(f"Calling {callback} with state {new_state}") # Для отладки
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in {callback}: {e}")
                # else:
                #     rospy.logdebug(f"No callback defined for button '{key}'") # Для отладки

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
