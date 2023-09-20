import rclpy
from tpm_client_py import libRobotOP

def main():
    rclpy.init()

    tpm_client = libRobotOP.Lib()
    tpm_client.initROS()
    
    rclpy.spin(tpm_client)
    #tpm_client.axis_action(0, 0)
    #tpm_client.get_logger().info("==== test =====")

    rclpy.shutdown()

if __name__ == '__main__':
    main()