# 補充模組 05: MAVLink 2.0 微通訊引擎 (Ground Station Communication)

## 1. PX4 原始碼對照與設計背景

在真實無人機與地面站（QGroundControl / Mission Planner）或伴隨電腦（Raspberry Pi / Jetson running ROS 2 / MAVROS）通訊時，**MAVLink 2.0** 是業界統治性的二進制二進階通訊標準。
對照 PX4 源碼目錄：
- `src/modules/mavlink`
- `src/modules/mavlink/mavlink_receiver.cpp`
- `src/modules/mavlink/mavlink_messages.cpp`

### MAVLink 2.0 訊框格式 (Packet Format)
```text
 0      1      2      3      4      5      6      7  8  9     10...      +2 bytes
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬─────────┬──────────┬──────────┐
│ 0xFD │ LEN  │ INC  │ CMP  │ SEQ  │ SYS  │ COMP │ MSG_ID  │ PAYLOAD  │ CRC16    │
│(STX) │(長度)│(旗標)│(旗標)│(序號)│(系統)│(組件)│ (3位元組)│ (0~255)  │(校驗碼)  │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴─────────┴──────────┴──────────┘
```
- **0xFD**：MAVLink 2.0 的魔法起始字元（STX）。
- **CRC-16-MCRF4XX**：包含每種訊息特定的 `CRC_EXTRA` 種子，確保協議版本型別一致。

---

## 2. 核心互動場景

1. **心跳包定時廣播 (Heartbeat @ 1Hz)**：
   - 向地面站宣告飛機在線、飛控類型（PX4）、當前是否處於 `ARMED` 狀態。
2. **遙測串流發送 (Telemetry Streaming)**：
   - 定時發送 `ATTITUDE` (Msg ID 30) 讓 QGC 地面站儀表盤顯示即時人工地平線。
3. **指令握手響應 (Command & ACK Handshake)**：
   - 接收地面站發來的 `COMMAND_LONG`（例如 `MAV_CMD_COMPONENT_ARM_DISARM = 400`）。
   - 解析指令參數，與 Lab 07 Commander 狀態機聯動，並回傳 `COMMAND_ACK`（`MAV_RESULT_ACCEPTED`）。

---

## 3. 實作驗證重點

- [x] 成功將姿態數據打包序列化為符合 MAVLink 2.0 標準之二進制二進位訊框。
- [x] 透過字元級狀態機（Byte Stream FSM）精準反序列化並驗證 CRC16 校驗碼。
- [x] 模擬地面站發送解鎖指令，成功解碼並回覆 ACK 應答包。
