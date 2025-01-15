# AI 群體智慧機器人指揮中心

## 系統概述
本平台為一個整合性的機器人控制中心，可同時操控多台不同類型的機器人，實現群體智慧控制。

### 支援的機器人
1. AR Drone 2.0 無人機
   - 支援即時影像串流
   - 支援飛行控制
   - 支援姿態監控
2. TEMI 智慧服務機器人
   - 支援移動控制
   - 支援語音互動
   - 支援 IP 攝影機串流

## 系統需求
- Python 3.7+
- 網路環境：有線 + 無線
- MQTT Broker 服務
- OpenCV 支援
- Flask 網頁框架

## 安裝步驟
1. 安裝必要套件：
```bash
pip install -r requirements.txt
```

2. 確認網路設定：
   - 有線網路：用於 TEMI 機器人連線
   - 無線網路：用於 AR Drone 連線（192.168.1.1）

## 使用前準備

### AR Drone 設定
1. 將電腦連接至 AR Drone 的 Wi-Fi 網路
2. 確認可以 ping 通 192.168.1.1

### TEMI 機器人設定
1. 開啟 TEMI 的 MQTT App
2. 確認 TEMI 已連接到 Tailscale VPN
3. 開啟 IP Camera App 並記錄串流位址
4. 確認 MQTT 連線狀態

## 執行方式
1. 修改設定檔：
   - 確認 MQTT_HOST, MQTT_USERNAME, MQTT_PASSWORD
   - 設定正確的 IP Camera 串流位址

2. 啟動平台：
```bash
python robot_platform.py
```

3. 訪問控制介面：
   - 開啟瀏覽器訪問 http://[您的IP]:10000
   - 若在本機執行，可使用 http://localhost:10000

## 功能說明

### 無人機控制
- 起飛/降落
- 前後左右移動
- 高度調整
- 姿態穩定
- 即時影像串流
- 飛行數據監控

### TEMI 控制
- 移動控制（前進、後退、轉向）
- 語音指令
- 電池狀態監控
- 攝影機串流
- 導航點設定

## 故障排除
1. 無人機無法連線：
   - 確認 Wi-Fi 連接狀態
   - 重啟無人機

2. TEMI 連線問題：
   - 確認 MQTT App 運作狀態
   - 檢查 Tailscale 連線狀態
   - 確認 IP Camera App 設定

3. 影像串流問題：
   - 檢查網路頻寬
   - 確認 IP Camera 位址設定

## 注意事項
- 首次飛行請在開闊空間進行
- 確保電池電量充足
- 定期檢查網路連線狀態
- 保持 MQTT 服務穩定運作

