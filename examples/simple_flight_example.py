import rospy
import time
from threading import Thread
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped
import tf

# Класс для управления полетом дрона(Пример)
class FlightCommander:
    def __init__(self) -> None:
        # Инициализация начальных значений
        self.current_state = None  # текущее состояние дрона
        self.pose = None  # Текущее положение дрона
        self._setpoint = None  # целевое положение\скорость для дрона

        # Подписка на темы ROS для получения состояния и позиции дрона
        rospy.Subscriber('/mavros/state', State, self.state_callback_m_msgs)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.state_callback_g_msgs)
        
        # Публикация точек назначения для дрона
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # Инициализация потока для отправки точек назначения
        self.setpoint_thread = None
        self.stop_thread = False
        
        # Ожидание доступности сервисов ROS
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        
    # Запуск потока для отправки точек назначения
    def start_setpoint_thread(self):
        self.stop_thread = False
        self.setpoint_thread = Thread(target=self.send_setpoints_loop)
        self.setpoint_thread.start()

    # Остановка потока отправки точек назначения
    def stop_setpoint_thread(self):
        self.stop_thread = True
        if self.setpoint_thread:
            self.setpoint_thread.join()
            self.setpoint_thread = None

    # Цикл отправки целевого положения
    def send_setpoints_loop(self, target_rate: float=20):
        rate = rospy.Rate(target_rate)  
        while not self.stop_thread and not rospy.is_shutdown():
            if self._setpoint is None:
                continue
            if self.current_state.armed == False:
                self.arm_vehicle()
            self.setpoint_pub.publish(self._setpoint)
            rate.sleep()
        self._setpoint = None
    
    # Обратный вызов для обновления текущего состояния дрона
    def state_callback_m_msgs(self, msg):
        self.current_state = msg
        if msg.mode != "OFFBOARD" and self.setpoint_thread is not None:
            self.stop_setpoint_thread()
        
    # Обратный вызов для обновления текущей позиции дрона
    def state_callback_g_msgs(self, msg):
        self.pose = msg
        
    # Метод для посадки дрона
    def land_vehicle(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            land_response = land_service(0, 0, 0, 0, 0)
            if land_response.success:
                print("Landing command sent successfully!")
                return True
            else:
                print("Failed to send landing command.")
                return False
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
    
    # Метод для взлета дрона на определенную высоту
    def do_takeoff(self, altitude: float=1.0):
        while (self.current_state is None):
            pass
        if not self.current_state.armed:
            if self.arm_vehicle():
                print("Vehicle armed successfully!")
            else:
                print("Failed to arm the vehicle.")
                return
        self.set_position(0, 0, altitude, 0)
        if self.current_state.mode != "OFFBOARD":
            if self.change_mode("OFFBOARD"):
                print("Offboard mode set successfully!")
            else:
                print("Failed to set Offboard mode.")
                return
    
    # Метод для включения/выключения двигателей дрона
    def arm_vehicle(self, arm: bool=True):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_response = arm_service(arm)
            return arm_response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
    # Метод для изменения режима полета дрона
    def change_mode(self, mode: str):
        try:
            if mode == "OFFBOARD" and self.setpoint_thread is None:
                self.start_setpoint_thread()
                time.sleep(1)
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(0, mode)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
    # Метод для установки позиции дрона
    def set_position(self, x: float, y: float, z: float, yaw: float=0):
        if self.current_state:
            self.change_mode("OFFBOARD")
        q = tf.transformations.quaternion_from_euler(0, 0, yaw, axes="sxyz")
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.pose.position.x = x
        setpoint.pose.position.y = y
        setpoint.pose.position.z = z
        setpoint.pose.orientation.x = q[0]
        setpoint.pose.orientation.y = q[1]
        setpoint.pose.orientation.z = q[2]
        setpoint.pose.orientation.w = q[3]
        self._setpoint = setpoint


if __name__ == "__main__":
    # Пример использования
    rospy.init_node('test_flight_commander')
    commander = FlightCommander()
    commander.do_takeoff()
    commander.set_position(0, 0, 3, 0)
    time.sleep(5)
    commander.set_position(0, -3, 3, -1.57)
    time.sleep(10)
    commander.set_position(-5, -3, 3, 1.57)
    time.sleep(10)
    commander.land_vehicle()
