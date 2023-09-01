import rclpy
from tpm_client_py import libRos
from tpm_client_py import libRobotOP

def main():
    rclpy.init()

    #tpm_client = libRos.rosMnetLib()
    tpm_client = libRobotOP.Lib()

    tpm_client.initROS()
    rclpy.spin(tpm_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()