/**
 * @file main.cpp
 * @brief 補充模組 05: MAVLink 2.0 微通訊引擎 (Serialization, Parser & QGC Handshake)
 * 
 * [對標 PX4 源碼]:
 * - src/modules/mavlink
 * - src/modules/mavlink/mavlink_receiver.cpp
 */

#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <cassert>

// MAVLink 2.0 基礎常數
constexpr uint8_t MAVLINK_STX_V2 = 0xFD;

// 常用 Message ID
constexpr uint32_t MAVLINK_MSG_ID_HEARTBEAT = 0;
constexpr uint32_t MAVLINK_MSG_ID_ATTITUDE  = 30;
constexpr uint32_t MAVLINK_MSG_ID_COMMAND_LONG = 76;
constexpr uint32_t MAVLINK_MSG_ID_COMMAND_ACK  = 77;

// CRC-16-MCRF4XX 演算法 (MAVLink 官方專用校驗算法)
uint16_t crc_accumulate(uint8_t data, uint16_t crcAccum) {
    uint8_t tmp = data ^ (uint8_t)(crcAccum & 0xff);
    tmp ^= (tmp << 4);
    crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    return crcAccum;
}

uint16_t crc_calculate(const uint8_t* pBuffer, size_t length) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc = crc_accumulate(pBuffer[i], crc);
    }
    return crc;
}

// MAVLink 2.0 封包實體
struct MavlinkPacket {
    uint8_t payload_len{0};
    uint8_t incompat_flags{0};
    uint8_t compat_flags{0};
    uint8_t seq{0};
    uint8_t sys_id{1};     // 飛機 System ID = 1
    uint8_t comp_id{1};    // Autopilot Component ID = 1
    uint32_t msg_id{0};    // 24-bit Message ID
    std::vector<uint8_t> payload;
    uint16_t checksum{0};
};

// 序列化為二進制訊框
std::vector<uint8_t> serializeMavlink2(const MavlinkPacket& pkt) {
    std::vector<uint8_t> frame;
    frame.push_back(MAVLINK_STX_V2);
    frame.push_back(pkt.payload_len);
    frame.push_back(pkt.incompat_flags);
    frame.push_back(pkt.compat_flags);
    frame.push_back(pkt.seq);
    frame.push_back(pkt.sys_id);
    frame.push_back(pkt.comp_id);
    frame.push_back(pkt.msg_id & 0xFF);
    frame.push_back((pkt.msg_id >> 8) & 0xFF);
    frame.push_back((pkt.msg_id >> 16) & 0xFF);

    frame.insert(frame.end(), pkt.payload.begin(), pkt.payload.end());

    // 計算 CRC (從 index 1 到 payload 結尾)
    uint16_t crc = crc_calculate(&frame[1], frame.size() - 1);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);

    return frame;
}

// 位元組流解析狀態機 (Byte Stream Parser FSM)
class MavlinkParser {
public:
    enum class State {
        WAIT_STX,
        READ_HEADER,
        READ_PAYLOAD,
        READ_CRC
    };

    bool parseByte(uint8_t byte, MavlinkPacket& outPacket) {
        switch (_state) {
            case State::WAIT_STX:
                if (byte == MAVLINK_STX_V2) {
                    _buffer.clear();
                    _buffer.push_back(byte);
                    _state = State::READ_HEADER;
                }
                break;

            case State::READ_HEADER:
                _buffer.push_back(byte);
                if (_buffer.size() == 10) { // STX(1) + LEN(1) + FLAGS(2) + SEQ(1) + SYS(1) + COMP(1) + MSGID(3)
                    _payloadLen = _buffer[1];
                    if (_payloadLen == 0) {
                        _state = State::READ_CRC;
                    } else {
                        _state = State::READ_PAYLOAD;
                    }
                }
                break;

            case State::READ_PAYLOAD:
                _buffer.push_back(byte);
                if (_buffer.size() == static_cast<size_t>(10 + _payloadLen)) {
                    _state = State::READ_CRC;
                }
                break;

            case State::READ_CRC:
                _buffer.push_back(byte);
                if (_buffer.size() == static_cast<size_t>(10 + _payloadLen + 2)) {
                    _state = State::WAIT_STX; // 回到初始態

                    // 驗證校驗碼
                    uint16_t calcCrc = crc_calculate(&_buffer[1], 9 + _payloadLen);
                    uint16_t recvCrc = _buffer[_buffer.size() - 2] | (_buffer[_buffer.size() - 1] << 8);

                    if (calcCrc == recvCrc) {
                        outPacket.payload_len = _payloadLen;
                        outPacket.sys_id = _buffer[5];
                        outPacket.comp_id = _buffer[6];
                        outPacket.msg_id = _buffer[7] | (_buffer[8] << 8) | (_buffer[9] << 16);
                        outPacket.payload.assign(_buffer.begin() + 10, _buffer.begin() + 10 + _payloadLen);
                        return true; // 成功解析一幀！
                    } else {
                        std::cerr << "[MAVLINK ERROR] CRC 校驗失敗！\n";
                    }
                }
                break;
        }
        return false;
    }

private:
    State _state{State::WAIT_STX};
    std::vector<uint8_t> _buffer;
    uint8_t _payloadLen{0};
};

int main() {
    std::cout << "=======================================================================\n";
    std::cout << " [補充模組 05]: MAVLink 2.0 微通訊引擎與 QGroundControl 握手互動       \n";
    std::cout << "=======================================================================\n";

    // 1. 模擬無人機發送 1Hz 心跳包 (HEARTBEAT)
    std::cout << "[場景 1]: 無人機序列化發送 HEARTBEAT 心跳封包...\n";
    MavlinkPacket hbPacket;
    hbPacket.msg_id = MAVLINK_MSG_ID_HEARTBEAT;
    hbPacket.payload_len = 4;
    // 假定 payload: [type=2 (Quadrotor), autopilot=12 (PX4), base_mode=128 (Armed), sys_status=4]
    hbPacket.payload = {2, 12, 128, 4};

    auto binaryFrame = serializeMavlink2(hbPacket);
    std::cout << " -> 生成二進制 MAVLink 2.0 幀，長度: " << binaryFrame.size() << " 位元組\n";
    std::cout << " -> STX 魔法位元組: 0x" << std::hex << (int)binaryFrame[0] << std::dec << " (預期 0xFD)\n";
    assert(binaryFrame[0] == 0xFD);

    // 2. 模擬地面站端逐位元組解析
    MavlinkParser parser;
    MavlinkPacket decodedPacket;
    bool parsedOk = false;

    for (uint8_t b : binaryFrame) {
        if (parser.parseByte(b, decodedPacket)) {
            parsedOk = true;
            break;
        }
    }
    assert(parsedOk);
    assert(decodedPacket.msg_id == MAVLINK_MSG_ID_HEARTBEAT);
    assert(decodedPacket.payload[0] == 2);
    std::cout << "✓ 地面站成功解碼 HEARTBEAT！識別到機型: Quadrotor, 飛控: PX4, 狀態: ARMED\n\n";

    // 3. 模擬地面站發送解鎖命令 (COMMAND_LONG: MAV_CMD_COMPONENT_ARM_DISARM = 400)
    std::cout << "[場景 2]: 地面站發送 COMMAND_LONG 解鎖請求 (CMD 400)...\n";
    MavlinkPacket cmdPacket;
    cmdPacket.msg_id = MAVLINK_MSG_ID_COMMAND_LONG;
    cmdPacket.payload_len = 6;
    // payload: [cmd_low, cmd_high, param1 (1.0=Arm), target_sys, target_comp, confirm]
    cmdPacket.payload = {0x90, 0x01, 1, 1, 1, 0}; // 0x0190 = 400 (ARM)

    auto cmdFrame = serializeMavlink2(cmdPacket);
    for (uint8_t b : cmdFrame) {
        if (parser.parseByte(b, decodedPacket)) {
            if (decodedPacket.msg_id == MAVLINK_MSG_ID_COMMAND_LONG) {
                uint16_t cmd = decodedPacket.payload[0] | (decodedPacket.payload[1] << 8);
                std::cout << " -> 飛控收到來自地面站命令 ID = " << cmd << " (MAV_CMD_COMPONENT_ARM_DISARM)\n";
                std::cout << " -> 轉交 Lab 07 Commander 處理... 執行解鎖成功！\n";

                // 回傳 COMMAND_ACK
                MavlinkPacket ackPacket;
                ackPacket.msg_id = MAVLINK_MSG_ID_COMMAND_ACK;
                ackPacket.payload_len = 3;
                ackPacket.payload = {0x90, 0x01, 0}; // 0 = MAV_RESULT_ACCEPTED
                auto ackFrame = serializeMavlink2(ackPacket);
                std::cout << " -> 飛控發送 COMMAND_ACK (MAV_RESULT_ACCEPTED) 至地面站！(長度 " 
                          << ackFrame.size() << " bytes)\n";
            }
        }
    }

    std::cout << "\n✓ 驗收成功: MAVLink 2.0 序列化、狀態機解析與 QGC 命令應答握手全通！\n";
    return 0;
}
