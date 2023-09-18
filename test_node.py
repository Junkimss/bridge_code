import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Node를 상속받아 사용
class test_sub2(Node):

  def __init__(self):
    # Node 생성자 호출 , 노드이름 sub
    super().__init__('test_sub')

    # 통신상태가 원할하지 않을시 퍼블리시할 데이터를 버퍼에 10개까지 저장
    qos_profile = QoSProfile(depth = 10)

  # publisher 생성
    # 메시지 타입 : Twist
    # cmd_vel 토픽 이름.
    # 메시지 출력
    self.sub = self.create_subscription(
      Twist,
      'API/cmd_vel',
      self.sub_msg,
      qos_profile
    )

  # 수신 메시지 출력 함수.
  def sub_msg(self,msg):
    self.get_logger().info('ROS2 API/cmd_vel linear.x={0}, linear.y={1}, linear.z={2}, angular.z={3},angular.z={4}, angular.z={5}'.format(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z))


def main(args=None):
  rclpy.init(args=args)
  node = test_sub2()
  try :
    rclpy.spin(node)
  except KeyboardInterrupt:
      node.get_logger().info('Keyboard Interrupt (SIGINT)')
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()

