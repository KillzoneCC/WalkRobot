#!/usr/bin/env python3
# encoding: utf-8
import time
import math
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

# === УБЕДИТЕСЬ, ЧТО BUTTON_MAP СООТВЕТСТВУЕТ ВАШЕМУ ГЕЙМПАДУ ===
# Стандарт PlayStation: 0-cross, 1-circle, 2-triangle, 3-square, 4-R1, 5-L1, 6-R2, 7-L2
# Индекс в BUTTON_MAP соответствует индексу в joy_msg.buttons
BUTTON_MAP = 'cross', 'circle', 'triangle', 'square', 'r1', 'l1', 'r2', 'l2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''
AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'

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

        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        # Исправлена синтаксическая ошибка
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP))) 
        self.mode = 0

        time.sleep(0.2)

        # Инициализация менеджера походки робота
        self.gait_manager = GaitManager()
        
        # === MotionManager для Get Up ===
        self.motion_manager = MotionManager()
        self.lie_to_stand_action_name = 'lie_to_stand'     # Подъем спереди (лицом вниз)
        self.recline_to_stand_action_name = 'recline_to_stand' # Подъем сзади (лицом вверх)
        # =============================================

        # === Переменные для работы с IMU и состоянием падения ===
        self.fall_state = 'unknown' # 'lie', 'recline', 'stand', 'unknown'
        self.count_lie = 0
        self.count_recline = 0
        # Уменьшен порог для более быстрой реакции
        self.FALL_COUNT_THRESHOLD = 10 
        # Подписка на топик IMU
        # Проверьте правильное имя топика с помощью `rostopic list | grep imu`
        # Возможные имена: /imu, /imu/data, /imu_corrected
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.loginfo("Subscribed to IMU topic: /imu")
        # ===================================================================

        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Добавим начальную остановку робота при запуске
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Speed Mode: 0 (STOP)")
        rospy.loginfo("Controls: R1/L1 - Speed Up/Down, Circle (B) - Get Up (Auto-detect)")

    # === Callback для данных IMU ===
    def imu_callback(self, msg: Imu):
        """
        Обрабатывает данные с IMU для определения состояния падения.
        """
        try:
            # Используем Z и Y компоненты линейного ускорения
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # Пороги (может потребоваться калибровка под вашего робота)
            # Ускорение свободного падения g ~ 9.81 m/s^2
            STAND_THRESH = 7.0       # Если |Z| > этого и |Y| < порога, робот вертикален или лежит
            LIE_THRESH = -7.0        # Если Z < этого, робот лежит лицом вниз
            RECLINE_THRESH = 7.0     # Если Z > этого, робот лежит лицом вверх
            SIDEWAYS_THRESH = 4.0    # Если |Y| > этого, робот лежит на боку

            # Логика определения положения
            if abs(az) > STAND_THRESH and abs(ay) < SIDEWAYS_THRESH:
                # Робот в устойчивом положении (стоит, лицом вниз или вверх)
                if az > RECLINE_THRESH:
                    # Лежит на спине (лицом вверх)
                    self.count_recline += 2 # Быстрее накапливаем
                    self.count_lie = max(0, self.count_lie - 1)
                elif az < LIE_THRESH:
                    # Лежит на животе (лицом вниз)
                    self.count_lie += 2 # Быстрее накапливаем
                    self.count_recline = max(0, self.count_recline - 1)
                else:
                    # Вертикальное положение (стоит) или неопределенное
                    # Сбрасываем счетчики медленно
                    self.count_lie = max(0, self.count_lie - 1)
                    self.count_recline = max(0, self.count_recline - 1)
            else:
                # Робот лежит на боку или в нестабильном положении
                # Сбрасываем счетчики медленно
                self.count_lie = max(0, self.count_lie - 1)
                self.count_recline = max(0, self.count_recline - 1)

            # Обновляем состояние на основе счетчиков
            old_state = self.fall_state
            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.fall_state = 'lie'
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.fall_state = 'recline'
            # Не сбрасываем в 'stand' автоматически через IMU в этом коде.
            # Сброс будет происходить после команды Get Up или вручную.

            # Логируем изменение состояния
            if old_state != self.fall_state:
                 rospy.loginfo(f"IMU detected fall state: {old_state} -> {self.fall_state} (lie:{self.count_lie}, recl:{self.count_recline})")

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")

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

            # КЛЮЧЕВОЕ: НЕ копируем другие параметры позы из speed_params
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

    # === ИСПРАВЛЕНО: Обработчики кнопок R1 и L1 для изменения скорости ===
    # Убедитесь, что в BUTTON_MAP 'r1' и 'l1' находятся на правильных позициях!
    def r1_callback(self, new_state):
        # rospy.loginfo("R1 callback triggered") # Для отладки
        if new_state == ButtonState.Pressed:
            self.speed_mode = min(self.speed_mode + 1, 3)
            rospy.loginfo(f"Speed Mode increased: {self.speed_mode}")
            self.board.set_buzzer(1000 + self.speed_mode * 200, 0.05, 0.02, 1)
            self.gait_manager.stop()
            self.status = 'stop'

    def l1_callback(self, new_state):
        # rospy.loginfo("L1 callback triggered") # Для отладки
        if new_state == ButtonState.Pressed:
            self.speed_mode = max(self.speed_mode - 1, 0)
            rospy.loginfo(f"Speed Mode decreased: {self.speed_mode}")
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

    # === Обработчик кнопки Circle (B) - Авто-Get Up ===
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (Get Up).
        Автоматически выбирает между lie_to_stand и recline_to_stand
        на основе состояния, определенного через IMU.
        """
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle (B) button pressed. Detected fall state: '{self.fall_state}'.")

            action_to_run = None
            log_msg = ""
            if self.fall_state == 'lie':
                action_to_run = self.lie_to_stand_action_name
                log_msg = "Robot appears to be lying on its FRONT. Initiating lie_to_stand."
            elif self.fall_state == 'recline':
                action_to_run = self.recline_to_stand_action_name
                log_msg = "Robot appears to be lying on its BACK. Initiating recline_to_stand."
            else: # 'unknown' или 'stand'
                # fallback на lie_to_stand, как в мобильном приложении
                rospy.logwarn(f"Robot fall state is '{self.fall_state}'. Defaulting to lie_to_stand (front).")
                action_to_run = self.lie_to_stand_action_name
                log_msg = "Robot state unknown. Defaulting to lie_to_stand (front)."

            rospy.loginfo(log_msg)
            self.board.set_buzzer(1500, 0.1, 0.05, 1)

            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully.")
                    
                    # === ВАЖНО: Сброс состояния ===
                    # После команды подъема предполагаем, что робот больше не лежит.
                    # Сбрасываем состояние и счетчики, чтобы IMU мог определить новое положение.
                    # Это предотвращает повторную попытку с тем же действием, если кнопка
                    # нажата снова до того, как IMU обновит состояние.
                    self.fall_state = 'unknown' # Или 'stand', если уверены
                    self.count_lie = 0
                    self.count_recline = 0
                    rospy.logdebug("Fall state and counters reset after Get Up command.")
                    
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")
                
    # ======================================================================

    def triangle_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

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
                if hasattr(self, callback):
                    try:
                        # rospy.logdebug(f"Calling {callback} with state {new_state}") # Для отладки
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in {callback}: {e}")

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
