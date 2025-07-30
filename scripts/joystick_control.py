#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy
from ainex_kinematics.gait_manager import GaitManager
# === ДОБАВЛЕНО: Импорт MotionManager ===
from ainex_kinematics.motion_manager import MotionManager
# ======================================

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
                # Параметры позы туловища НЕ передаются в set_step/update_param напрямую из speed_params
                # Они задаются один раз при инициализации GaitManager
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
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0

        time.sleep(0.2)

        # Инициализация менеджера походки робота
        # GaitManager сам устанавливает робота в начальную позу
        self.gait_manager = GaitManager()
        # === ДОБАВЛЕНО: Инициализация MotionManager ===
        self.motion_manager = MotionManager()
        # Имена действий подъема (взяты из Pasted_Text_1753855641149.txt)
        self.lie_to_stand_action_name = 'lie_to_stand'     # Подъем спереди
        self.recline_to_stand_action_name = 'recline_to_stand' # Подъем сзади
        # =============================================

        # Подписка на топик ROS '/joy' для получения данных с джойстика
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        # Добавим начальную остановку робота при запуске
        self.gait_manager.stop()
        rospy.loginfo("JoystickController initialized. Starting in Speed Mode: 0 (STOP)")

    # ... (остальные методы остаются без изменений: axes_callback, callback, select_callback,
    #  r1_callback, l1_callback, l2_callback, r2_callback, square_callback,
    #  triangle_callback, cross_callback, start_callback, hat_*_callback) ...

    # === ДОБАВЛЕНО: Обработчик кнопки Circle (предполагаемая кнопка "B") ===
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (get up).
        Пытается определить, с какой стороны робот упал, и выбрать соответствующее действие.
        """
        # Проверяем, что кнопка была только что нажата (Pressed)
        if new_state == ButtonState.Pressed:
            rospy.loginfo("Circle (B) button pressed. Attempting to perform 'get up' action.")
            self.board.set_buzzer(1500, 0.1, 0.05, 1) # Звуковой сигнал

            # В простейшем случае можно запустить одно из действий.
            # В более сложной реализации (как в Pasted_Text_1753855641149.txt)
            # состояние робота ('stand', 'lie', 'recline') отслеживается с помощью IMU.
            # Здесь мы можем просто попробовать одно действие или запросить ввод/определение.

            # === Вариант 1: Запуск стандартного действия lie_to_stand ===
            # Это действие, вероятно, подходит для подъема со спины или живота в большинстве случаев.
            try:
                if self.motion_manager is not None:
                    # Запускаем действие подъема спереди
                    self.motion_manager.run_action(self.lie_to_stand_action_name)
                    rospy.loginfo(f"Action '{self.lie_to_stand_action_name}' initiated via MotionManager.")
                else:
                    rospy.logwarn("MotionManager is not initialized. Cannot perform get up action.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action for '{self.lie_to_stand_action_name}': {e}")
            # ===================================================================

            # === Вариант 2: (Расширенный) Попытка определить сторону падения ===
            # Это требует доступа к данным IMU или другого сенсора.
            # Ниже приведен концептуальный пример, как это *могло бы* выглядеть,
            # если бы у нас был доступ к углу наклона, например, через сервис или переменную.
            # ------------------------------------------------------------------------
            # try:
            #     # ПРЕДПОЛОЖЕНИЕ: Есть способ получить угол наклона робота (например, из IMU)
            #     # angle = self.get_robot_angle() # Эта функция должна быть реализована
            #
            #     # if angle is not None:
            #     #     if angle < 30: # Робот лежит на животе
            #     #         rospy.loginfo("Robot appears to be lying on its front. Initiating lie_to_stand.")
            #     #         self.motion_manager.run_action(self.lie_to_stand_action_name)
            #     #     elif angle > 150: # Робот лежит на спине
            #     #         rospy.loginfo("Robot appears to be lying on its back. Initiating recline_to_stand.")
            #     #         self.motion_manager.run_action(self.recline_to_stand_action_name)
            #     #     else:
            #     #         rospy.logwarn(f"Robot angle ({angle}) is ambiguous for get up action. Defaulting to lie_to_stand.")
            #     #         self.motion_manager.run_action(self.lie_to_stand_action_name)
            #     # else:
            #     #     rospy.logwarn("Could not determine robot angle. Defaulting to lie_to_stand.")
            #     #     self.motion_manager.run_action(self.lie_to_stand_action_name)
            # except Exception as e:
            #     rospy.logerr(f"Error determining robot state or calling MotionManager: {e}")
            #     # Fallback
            #     rospy.logwarn("Fallback: Initiating default lie_to_stand action.")
            #     try:
            #         self.motion_manager.run_action(self.lie_to_stand_action_name)
            #     except Exception as e2:
            #         rospy.logerr(f"Fallback action also failed: {e2}")
            # ------------------------------------------------------------------------

    # ======================================================================

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        # Обработка высоты тела (независимо от режима скорости)
        self.callback(axes)

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

