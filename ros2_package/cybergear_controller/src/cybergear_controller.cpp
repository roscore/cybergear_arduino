#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <serial/serial.h>

class CybergearController : public rclcpp::Node
{
public:
    CybergearController() : Node("cybergear_controller"), ser_("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000))
    {
        // ROS2 퍼블리셔, 서브스크라이버, 서비스 초기화 (추후 필요에 따라 추가)
    }

    void motorCommandCallback(const cybergear_controller::msg::MotorCommand::SharedPtr msg)
    {
        switch (msg->command_type) {
            case Communication_Type_MotionControl:
                Set_Motor_Parameter(msg->CAN_ID, msg->value, msg->value_type);
                break;
            // 필요한 경우 다른 command_type에 따른 처리를 추가
        }
    }

    void Float_to_Byte(float f, uint8_t byte[4]) {
        unsigned long longdata = *(unsigned long*)&f;       
        byte[0] = (longdata >> 24) & 0xFF;
        byte[1] = (longdata >> 16) & 0xFF;
        byte[2] = (longdata >> 8) & 0xFF;
        byte[3] = longdata & 0xFF;
    }

    void send_command(uint32_t id_num, uint8_t cmd_mode, uint8_t* tx_data) {
        uint8_t cdata[13] = {0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        cdata[1] = cmd_mode;
        cdata[4] = id_num;

        for (int i = 0; i < 8; i++) {
            cdata[5 + i] = tx_data[i];
        }

        uint8_t udata[16] = {0xAA, 1, 0, 0x08, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        for (int i = 0; i < 12; i++) {
            udata[4 + i] = cdata[i + 1];
        }

        ser_.write(udata, 16);
    }

    void Init_Motor(uint32_t Can_Id, float mode) {
        Enable_Motor(Can_Id);
        Set_Mode(Can_Id, mode);
    }

    void Enable_Motor(uint32_t CAN_ID) {
        uint8_t temp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_command(CAN_ID, 3, temp);
    }

    void Stop_Motor(uint32_t CAN_ID, uint8_t clear_error) {
        uint8_t temp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        temp[0] = clear_error;
        send_command(CAN_ID, 4, temp);
    }

    void Set_Motor_Parameter(uint32_t CAN_ID, uint16_t Index, float Value, char Value_type) {
        uint8_t Send_Data[8];
        Send_Data[0] = Index;
        Send_Data[1] = Index >> 8;
        Send_Data[2] = 0x00;
        Send_Data[3] = 0x00;

        if (Value_type == 'f') {
            uint8_t byte[4];
            Float_to_Byte(Value, byte);
            Send_Data[4] = byte[3];
            Send_Data[5] = byte[2];
            Send_Data[6] = byte[1];
            Send_Data[7] = byte[0];
        } else if (Value_type == 's') {
            Send_Data[4] = (uint8_t)Value;
            Send_Data[5] = 0x00;
            Send_Data[6] = 0x00;
            Send_Data[7] = 0x00;
        }
        send_command(CAN_ID, 18, Send_Data);
    }

    void Set_Mode(uint32_t CAN_ID, float Mode) {
        Set_Motor_Parameter(CAN_ID, 0x7005, Mode, 's');
    }

private:
    serial::Serial ser_;
    rclcpp::Subscription<cybergear_controller::msg::MotorCommand>::SharedPtr motor_command_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CybergearController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}