#include <mbed.h>

//-------------------------------------------------------------------------------------------------------------------------------
// define - Dynamixel Hex code table
//-------------------------------------------------------------------------------------------------------------------------------
// EEPROM AREA
#define EEPROM_MODEL_NUMBER_L 0x00
#define EEPROM_MODEL_NUMBER_H 0x01
#define EEPROM_VERSION 0x02
#define EEPROM_ID 0x03
#define EEPROM_BAUD_RATE 0x04
#define EEPROM_RETURN_DELAY_TIME 0x05
#define EEPROM_CW_ANGLE_LIMIT_L 0x06
#define EEPROM_CW_ANGLE_LIMIT_H 0x07
#define EEPROM_CCW_ANGLE_LIMIT_L 0x08
#define EEPROM_CCW_ANGLE_LIMIT_H 0x09
#define EEPROM_DRIVE_MODE 0x0A
#define EEPROM_LIMIT_TEMPERATURE 0x0B
#define EEPROM_LOW_LIMIT_VOLTAGE 0x0C
#define EEPROM_HIGN_LIMIT_VOLTAGE 0x0D
#define EEPROM_MAX_TORQUE_L 0x0E
#define EEPROM_MAX_TORQUE_H 0x0F
#define EEPROM_RETURN_LEVEL 0x10
#define EEPROM_ALARM_LED 0x11
#define EEPROM_ALARM_SHUTDOWN 0x12
#define EEPROM_MULTITURN_L 0x14
#define EEPROM_MULTITURN_H 0x15
#define EEPROM_RESOLUTION_DIV 0x16
// RAM AREA
#define RAM_TORQUE_ENABLE 0x18
#define RAM_LED 0x19
#define RAM_DERIVATIVE_GAIN 0x1A
#define RAM_INTERGRAL_GAIN 0x1B
#define RAM_PROPORTIONAL_GAIN 0x1C
#define RAM_GOAL_POSITION_L 0x1E
#define RAM_GOAL_POSITION_H 0x1F
#define RAM_GOAL_SPEED_L 0x20
#define RAM_GOAL_SPEED_H 0x21
#define RAM_TORQUE_LIMIT_L 0x22
#define RAM_TORQUE_LIMIT_H 0x23
#define RAM_PRESENT_POSITION_L 0x24
#define RAM_PRESENT_POSITION_H 0x25
#define RAM_PRESENT_SPEED_L 0x26
#define RAM_PRESENT_SPEED_H 0x27
#define RAM_PRESENT_LOAD_L 0x28
#define RAM_PRESENT_LOAD_H 0x29
#define RAM_PRESENT_VOLTAGE 0x2A
#define RAM_PRESENT_TEMPERATURE 0x2B
#define RAM_REGISTER 0x2C
#define RAM_MOVING 0x2E
#define RAM_LOCK 0x2F
#define RAM_PUNCH_L 0x30
#define RAM_PUNCH_H 0x31
#define RAM_SENSED_CURRENT_L 0x38
#define RAM_SENSED_CURRENT_H 0x39
#define RAM_CURRENT_L 0x44
#define RAM_CURRENT_H 0x45
#define RAM_TORQUE_CONTROL_MODE_ENABLE 0x46
#define RAM_GOAL_TORQUE_L 0x47
#define RAM_GOAL_TORQUE_H 0x48
#define RAM_GOAL_ACCELERATION 0x49

//-------------------------------------------------------------------------------------------------------------------------------
// Instruction commands Set
//-------------------------------------------------------------------------------------------------------------------------------
#define COMMAND_PING 0x01
#define COMMAND_READ_DATA 0x02
#define COMMAND_WRITE_DATA 0x03
#define COMMAND_REG_WRITE_DATA 0x04
#define COMMAND_ACTION 0x05
#define COMMAND_RESET 0x06
#define COMMAND_SYNC_WRITE 0x83

//-------------------------------------------------------------------------------------------------------------------------------
//Instruction packet lengths
#define READ_ONE_BYTE_LENGTH 0x01
#define READ_TWO_BYTE_LENGTH 0x02
#define RESET_LENGTH 0x02
#define PING_LENGTH 0x02
#define ACTION_LENGTH 0x02
#define SET_ID_LENGTH 0x04
#define SET_BD_LENGTH 0x04
#define SET_RETURN_LEVEL_LENGTH 0x04
#define READ_TEMP_LENGTH 0x04
#define READ_POS_LENGTH 0x04
#define READ_LOAD_LENGTH 0x04
#define READ_SPEED_LENGTH 0x04
#define READ_VOLT_LENGTH 0x04
#define READ_REGISTER_LENGTH 0x04
#define READ_MOVING_LENGTH 0x04
#define READ_LOCK_LENGTH 0x04
#define LED_LENGTH 0x04
#define SET_HOLDING_TORQUE_LENGTH 0x04
#define SET_MAX_TORQUE_LENGTH 0x05
#define SET_ALARM_LENGTH 0x04
#define READ_LOAD_LENGTH 0x04
#define SET_RETURN_LENGTH 0x04
#define WHEEL_LENGTH 0x05
#define SERVO_GOAL_LENGTH 0x07
#define SET_MODE_LENGTH 0x07
#define SET_PUNCH_LENGTH 0x04
#define SET_PID_LENGTH 0x06
#define SET_TEMP_LENGTH 0x04
#define SET_VOLT_LENGTH 0x05
#define SYNC_LOAD_LENGTH 0x0D
#define SYNC_DATA_LENGTH 0x02

//-------------------------------------------------------------------------------------------------------------------------------
// Specials
//-------------------------------------------------------------------------------------------------------------------------------

#define OFF 0x00
#define ON 0x01

#define SERVO 0x01
#define WHEEL 0x00

#define LEFT 0x00
#define RIGHT 0x01

#define NONE 0x00
#define READ 0x01
#define ALL 0x02

#define BROADCAST_ID 0xFE

#define HEADER 0xFF

#define STATUS_PACKET_TIMEOUT 50 // in millis()
#define STATUS_FRAME_BUFFER 5

class Dynamixel
{
  private:
	DigitalOut *dynamixelDi;
	Serial *dynamixelSerial;
	void transmitInstructionPacket(void);
	unsigned int readStatusPacket(void);
	void debugframe(void);
	void debugStatusframe(void);

  public:
	Dynamixel(PinName tx, PinName rx, int baud, PinName di);
	~Dynamixel(void);

	unsigned int reset(uint8_t ID);
	unsigned int ping(uint8_t ID);
	void setPosition(uint8_t ID, unsigned int Position, unsigned int Speed);
	unsigned int getPosition(uint8_t ID);
	unsigned int getTemperature(uint8_t ID);
};