import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
import threading
import time

# Node를 상속받아 사용
class test_sub2(Node):

  def __init__(self):
    # Node 생성자 호출 , 노드이름 sub
    super().__init__('test_sub')

    # 통신상태가 원할하지 않을시 퍼블리시할 데이터를 버퍼에 10개까지 저장
    qos_profile = QoSProfile(depth = 10)

    self.toggle = True

    # publisher 생성
    # 메시지 타입 : Twist
    # API/cmd_vel 토픽 이름.
    # pub : API/cmd_vel (geometry_msgs/Twist) 1hz
    self.pub = self.create_publisher(
      Twist,
      'API/cmd_vel',
      qos_profile
    )

    # subscribe : feedback_vel (geometry_msgs/Twist)
    self.sub = self.create_subscription(
      Twist,
      'feedback_vel',
      self.sub_msg,
      qos_profile
    )
    self.service_client = self.create_client(
      SetBool,
      "API/follow_me_mode",
      )
    while not self.service_client.wait_for_service(timeout_sec=10):
      self.get_logger().warning('The service_oprator service not available.')

    self.service_request =  SetBool.Request()


    self.pub_thread = threading.Thread(target=self.pub_thread_function)
    self.pub_thread.start()

    self.client_thread = threading.Thread(target=self.client_thread_function)
    self.client_thread.start()


  def pub_thread_function(self):
     while rclpy.ok():
        try :
          self.pub_msg()
        finally:
          time.sleep(1)

  def client_thread_function(self):
     while rclpy.ok():
        try :
          self.send_request()
        finally:
          time.sleep(1)

  # 수신 메시지 출력 함수.
  def sub_msg(self,msg):
    self.get_logger().info('ROS2 feedback_vel linear.x={0}, angular.z={1}'.format(msg.linear.x,msg.angular.z))

  def pub_msg(self):
    twist_msg =Twist()
    twist_msg.linear.x = 0.5

    twist_msg.angular.z = random.uniform(-0.5,0.5)

    # twist_msg 발송.
    self.pub.publish(twist_msg)

  def send_request(self):
    self.service_request.data = self.toggle
    service_response = self.service_client.call_async(self.service_request)
    service_response.add_done_callback(self.handle_service_response)  # 콜백 추가

  def handle_service_response(self, service_response):
    try:
        response = service_response.result()
        self.get_logger().info(f'Received response: {response.success}, Received message: {response.message}')
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')

    self.toggle = not self.toggle


def main(args=None):
  rclpy.init(args=args)
  node = test_sub2()
  executor = MultiThreadedExecutor(num_threads = 2)
  executor.add_node(node)
  try:
    try :
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        executor.shutdown()
  finally:
    rclpy.shutdown()

if __name__ == '__main__':
  main()

