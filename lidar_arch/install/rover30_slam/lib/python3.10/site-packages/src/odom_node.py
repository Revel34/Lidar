import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations
from tf2_ros import TransformBroadcaster
from urdf_parser_py.urdf import Cylinder
from std_msgs.msg import String
from urdf_parser_py.urdf import URDF
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Vector3Stamped

class OdomFromJointStates(Node):
    """
    Node do odczytu odometrii dla robota 4WD z niezależnym skrętem kół.
    Uwzględnia kąty skrętu kół (wheel_turn_joint) i oblicza
    prędkości liniowe i kątowe w modelu Ackermanna.
    """

    def __init__(self):
        super().__init__('odom_from_joint_states')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.debug_pub = self.create_publisher(Vector3Stamped, '/odom_debug', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # Odczyt z robot_description

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Node teraz czeka na URDF z topicu /robot_description
        self.robot = None
        self.robot_description_sub = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos
        )
        # Odczyt JointState
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.last_time = self.get_clock().now()
        self.v = 0.0
        self.omega = 0.0

        self.create_timer(0.05, self.update_odom)

    # --- FUNKCJE POMOCNICZE ---

    def robot_description_callback(self, msg: String):
        self.get_logger().info(f"📩 Odebrano /robot_description ({len(msg.data)} bajtów)")
        if getattr(self, 'robot', None) is None:
            from urdf_parser_py.urdf import URDF
            self.robot = URDF.from_xml_string(msg.data)
            self.get_logger().info("✅ URDF poprawnie sparsowany")
    
            # Parametry kół i rozstaw dopiero teraz
            self.wheel_radius = self.get_wheel_radius()
            self.wheel_separation_y = self.get_wheel_separation_y()
            self.wheel_separation_x = self.get_wheel_separation_x()
            self.get_logger().info(
                f"wheel_radius: {self.wheel_radius}, sep_x: {self.wheel_separation_x}, sep_y: {self.wheel_separation_y}"
            )

    def get_wheel_radius(self):
        """Z URDF pobiera promień jednego z cylindrycznych kół wszystkie koła maja ten sam promień, według założenia"""
        wheel_names = ['right_front_wheel_link', 'left_front_wheel_link',
                       'right_rear_wheel_link', 'left_rear_wheel_link']
        for name in wheel_names:
            link = self.robot.link_map[name]
            if link.collision and link.collision.geometry:
                geom = link.collision.geometry
                if isinstance(geom, Cylinder):
                    return geom.radius
        return 0.1

    def get_wheel_separation_y(self):
        """Odległość między lewym a prawym kołem (dla obliczeń prędkości kątowej)"""
        try:
            front_left = self.robot.link_map['left_front_wheel_turn_link']
            front_right = self.robot.link_map['right_front_wheel_turn_link']
            return abs(front_left.origin.xyz[1] - front_right.origin.xyz[1])
        except KeyError:
            self.get_logger().warn("⚠️ Linki skrętu kół nie znalezione w URDF, używam domyślnej wartości 630 mm")
            return 0.630  # domyślna wartość w metrach
    def get_wheel_separation_x(self):
        """Odległość między przednimi a tylnymi kołami (opcjonalnie dla dokładnej kinematyki)"""
        try:
            front_left = self.robot.link_map['left_front_wheel_turn_link']
            rear_left = self.robot.link_map['left_rear_wheel_turn_link']
            return abs(front_left.origin.xyz[0] - rear_left.origin.xyz[0])
        except KeyError:
            self.get_logger().warn("⚠️ Linki skrętu kół nie znalezione w URDF, używam domyślnej wartości 842 mm")
            return 0.842  # domyślna wartość w metrach

    # --- CALLBACK DO ODCZYTU JOINT STATE ---
    def joint_states_callback(self, msg: JointState):
        if self.robot is None or not hasattr(self, 'wheel_radius'):
            self.get_logger().info("⏳ URDF jeszcze nie sparsowany, czekam na robot_description")
            return
        
        # DEBUG
        self.get_logger().info(f"Odebrano /joint_states, joints: {msg.name}")
    
        names = msg.name
        velocities = msg.velocity
        positions = msg.position  # kąty skrętu kół

        try:
            names = msg.name
            rf_vel = msg.velocity[names.index("right_front_wheel_joint")]
            lf_vel = msg.velocity[names.index("left_front_wheel_joint")]
            rr_vel = msg.velocity[names.index("right_rear_wheel_joint")]
            lr_vel = msg.velocity[names.index("left_rear_wheel_joint")]

            rf_turn = msg.position[names.index("right_front_steer_joint")]
            lf_turn = msg.position[names.index("left_front_steer_joint")]
            rr_turn = msg.position[names.index("right_rear_steer_joint")]
            lr_turn = msg.position[names.index("left_rear_steer_joint")]
        except ValueError:
            return

        # --- Prędkości kół ---
        v_rf = rf_vel * self.wheel_radius
        v_lf = lf_vel * self.wheel_radius
        v_rr = rr_vel * self.wheel_radius
        v_lr = lr_vel * self.wheel_radius

        # Średnie prędkości osi
        v_front = (v_rf + v_lf) / 2.0
        v_rear = (v_rr + v_lr) / 2.0
        self.v = (v_front + v_rear) / 2.0

        # Średnie kąty skrętu osi
        delta_f = (rf_turn + lf_turn) / 2.0
        delta_r = (rr_turn + lr_turn) / 2.0

        # --- Oblicz ruch w lokalnym układzie robota (dla 4WS / skrętnego napędu) ---
        # vx, vy – prędkości w lokalnym układzie bazowym (x do przodu, y w bok)
        vx_f = (v_rf * math.cos(rf_turn) + v_lf * math.cos(lf_turn)) / 2.0
        vy_f = (v_rf * math.sin(rf_turn) + v_lf * math.sin(lf_turn)) / 2.0
        vx_r = (v_rr * math.cos(rr_turn) + v_lr * math.cos(lr_turn)) / 2.0
        vy_r = (v_rr * math.sin(rr_turn) + v_lr * math.sin(lr_turn)) / 2.0

        # Średnie prędkości osi
        vx = (vx_f + vx_r) / 2.0
        vy = (vy_f + vy_r) / 2.0

        # Prędkość kątowa ω — przybliżenie różnicą między osiami
        omega = ((vy_f - vy_r) / self.wheel_separation_x) if abs(self.wheel_separation_x) > 1e-6 else 0.0

        # Zapisz do zmiennych klasy
        self.vx = vx
        self.vy = vy
        self.omega = omega

    # --- AKTUALIZACJA ODOMETRII ---
    def update_odom(self):
        # --- Nowy model integracji (dla vx, vy, omega) ---
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Transformacja z układu robota (vx, vy) do globalnego (x, y)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publikacja TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q_raw = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q_raw[0]
        t.transform.rotation.y = q_raw[1]
        t.transform.rotation.z = q_raw[2]
        t.transform.rotation.w = q_raw[3]
        self.tf_broadcaster.sendTransform(t)

        # Publikacja odometrii
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q_raw[0]
        odom.pose.pose.orientation.y = q_raw[1]
        odom.pose.pose.orientation.z = q_raw[2]
        odom.pose.pose.orientation.w = q_raw[3]

        # Tu używamy vx, vy zamiast jednego v
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom)

        # Publikacja debug
        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.vector.x = self.vx
        msg.vector.y = self.vy
        msg.vector.z = self.omega
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFromJointStates()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
