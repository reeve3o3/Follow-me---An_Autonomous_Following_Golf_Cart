# Follow Me - 跟隨球袋車
## 具有預測控制和自適應卡爾曼濾波的自主跟隨系統

---

## 成果展示

[![成果展示影片](https://img.youtube.com/vi/xwzQYBJqneQ/0.jpg)](https://youtu.be/xwzQYBJqneQ)

*點擊圖片觀看系統運作影片*

---

## 技術重點

本專案展示以下核心技術的應用：
- **自適應卡爾曼濾波** (Adaptive Kalman Filter) - 處理感測器噪聲與尖峰
- **預測性前饋控制** (Predictive Feedforward Control) - 延遲補償與運動預測
- **軟體定義脈衝調製** - 將二進制控制轉換為準比例控制
- **即時嵌入式系統開發** - 多執行緒架構與低延遲控制

---

## 專案概述

本專案聚焦於商用電動高爾夫球車自主跟隨系統的設計與實現。核心挑戰在於將僅具二進制輸入的消費級遙控器進行控制抽象化，並透過演算法克服其固有的非線性加速與系統延遲，最終將其改造為一穩健且響應平順的自主移動載具。

### 核心挑戰與解決方案

**1. 超寬頻 (UWB) 定位資料中的高頻雜訊和突發尖峰**
- 採用**自適應卡爾曼濾波器 (AKF)** 進行智能噪聲抑制
- 突變檢測機制：識別真實運動與傳感器誤差

**2. 從感測到驅動的固有系統延遲**
- 實現**預測性前饋控制器** 進行延遲補償
- 基於運動學模型預測目標未來位置，主動控制車輛

**3. 遙控器按鈕特性的平滑驅動問題**
- 設計**軟體定義脈衝調製層**
- 原始遙控器前進按鈕按一下會持續加速，需要轉換為可控的準比例控制

---

## 系統架構

```
┌─────────────────┐
│  UWB 傳感器模組  │ (原始位置數據: 角度、距離)
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────┐
│  自適應卡爾曼濾波器 (AKF)        │
│  - 狀態估計: [角度, 角速度]      │
│  - 動態噪聲協方差調整            │
│  - 突變檢測與自適應              │
└────────┬────────────────────────┘
         │ (濾波後角度)
         ▼
┌─────────────────────────────────┐
│  預測性前饋控制器                │
│  - 運動學預測模型                │
│  - 延遲補償 (0.2-0.5s)           │
│  - 角速度/加速度計算             │
└────────┬────────────────────────┘
         │ (預測位置 + 控制因子)
         ▼
┌─────────────────────────────────┐
│  自適應速度控制器                │
│  - 距離自適應                    │
│  - 收斂檢測                      │
│  - 動態調整控制強度              │
└────────┬────────────────────────┘
         │ (控制指令)
         ▼
┌─────────────────────────────────┐
│  軟體定義脈衝調製層              │
│  - GPIO 控制 (前進/左/右)        │
│  - 脈衝序列生成                  │
│  - 準比例速度控制                │
└────────┬────────────────────────┘
         │
         ▼
┌─────────────────┐
│  控制器          │ (實體車輛運動)
└─────────────────┘
```

---

## 核心算法設計

### 1. 控制抽象層：從遙控器按鈕到比例控制

**問題**：原裝遙控器的前進按鈕按一下後會持續前進並自動加速，無法直接控制速度。

**解決方案**：
- **硬體介面**：樹莓派 GPIO 直接連接遙控器按鈕觸點
- **軟體定義脈衝調製**：
  ```python
  def trigger_button(self, pin, duration=0.1):
      """觸發按鈕的函數（模擬按下按鈕）"""
      GPIO.output(pin, GPIO.HIGH)
      time.sleep(duration)
      GPIO.output(pin, GPIO.LOW)

  def forward_pulse(self, duration=0.05):
      """前進脈衝"""
      logger.debug("前進脈衝")
      self.trigger_button(self.FORWARD_PIN, duration)
  ```
- **效果**：將持續加速的按鈕轉換為準比例執行器，實現平滑恆速運動

### 2. 自適應卡爾曼濾波器 (AKF)

**目標**：處理 UWB 高頻噪聲和突發尖峰

**狀態向量**：`x = [角度, 角速度]`

**核心特性**：
1. **動態觀測噪聲調整**
   ```python
   if detect_angle_jump(measured_angle):
       R = R_base * 0.3  # 突變時降低噪聲，增加信任度
   else:
       R = R_base        # 正常情況標準噪聲
   ```

2. **突變檢測機制**
   - 計算當前測量值與歷史均值的偏差
   - 閾值檢測（默認 15°）
   - 區分真實運動與傳感器誤差

3. **角度規範化**
   - 處理角度跨越問題
   - 確保狀態向量一致性

**關鍵代碼片段**：
```python
def update(self, measured_angle):
    """更新步驟"""
    # 正規化測量角度
    measured_angle = self.normalize_angle(measured_angle)
    
    # 檢測角度突變
    is_jump = self.detect_angle_jump(measured_angle)
    
    # 根據是否突變調整觀測噪聲
    if is_jump:
        # 突變時增加信任度，降低噪聲
        self.R = self.R_base * 0.3
        logger.debug(f"檢測到角度突變: {measured_angle:.2f}°")
    else:
        # 正常情況下使用標準噪聲
        self.R = self.R_base.copy()
    
    # 計算卡爾曼增益
    innovation_cov = self.H @ self.P @ self.H.T + self.R
    K = self.P @ self.H.T @ np.linalg.inv(innovation_cov)
    
    # 更新狀態
    self.x = self.x + K.flatten() * innovation
    self.P = (I - K @ self.H) @ self.P
```

### 3. 預測性前饋控制器

**目標**：補償 0.2-0.5 秒的系統延遲

**運動學預測模型**：
```
角度_預測 = 角度_當前 + 角速度 * Δt + 0.5 * 角加速度 * Δt²
```

**工作流程**：
1. **角速度計算**（線性回歸）
   ```python
   def calculate_angular_velocity(self):
       """計算角速度"""
       if len(self.angle_history) < 2:
           return 0.0
       
       # 使用最近的幾個點計算角速度
       angles = list(self.angle_history)[-3:]
       times = list(self.time_history)[-3:]
       
       # 線性回歸計算角速度
       velocities = []
       for i in range(1, len(angles)):
           dt = times[i] - times[i-1]
           if dt > 0:
               dangle = angles[i] - angles[i-1]
               # 處理角度跨越
               if dangle > 90:
                   dangle -= 180
               elif dangle < -90:
                   dangle += 180
               velocities.append(dangle / dt)
       
       return np.mean(velocities) if velocities else 0.0
   ```

2. **角加速度計算**（二階差分）
3. **未來位置預測**
4. **主動控制決策**

**延遲自適應**：
- 動態估計系統延遲（基於數據間隔）
- 限制最大預測時間（0.5 秒）

### 4. 防止過度轉向的控制因子

**多維度控制調整**：

| 檢測條件 | 控制因子調整 | 說明 |
|---------|------------|------|
| 收斂檢測 | × 0.5 | 角度誤差正在減小 |
| 近距離 (< 2m) | × (距離/2.0) | 距離越近越謹慎 |
| 小角度誤差 (< 15°) | × 0.6 | 精細調整 |
| 高角速度 (> 20°/s) | × 0.4 | 防止過衝 |
| 預測改善 | × 0.7 | 趨勢向好時減少干預 |

```python
def calculate_control_factor(self, current_angle, distance):
    """計算控制因子，防止過度轉向"""
    # 基礎控制因子
    control_factor = 1.0
    
    # 1. 收斂檢測 - 如果正在收斂，減少控制強度
    if self.detect_convergence():
        control_factor *= 0.5
        logger.debug("檢測到收斂，降低控制強度")
    
    # 2. 距離因子 - 距離越近越謹慎
    if distance < 2.0:
        distance_factor = distance / 2.0
        control_factor *= distance_factor
    
    # 3. 角度因子 - 小角度誤差時更謹慎
    angle_abs = abs(current_angle)
    if angle_abs < 15:
        angle_factor = 0.6  # 小角度時降低強度
        control_factor *= angle_factor
    
    # 4. 角速度因子 - 如果轉向速度過快，減少控制
    angular_velocity = self.calculate_angular_velocity()
    if abs(angular_velocity) > 20:  # 角速度超過20度/秒
        velocity_factor = 0.4
        control_factor *= velocity_factor
    
    # 限制控制因子範圍
    control_factor = max(0.1, min(1.0, control_factor))
    
    return control_factor
```

---

## 技術棧與硬體規格

### 硬體配置
- **主控制器**: Raspberry Pi 4 Model B (4GB RAM)
- **定位系統**: UWB (Ultra-Wideband) 模組 DW1000
  - **型號**: DW1000
  - **尺寸**: 80×55mm
  - **天線**: PCB天線
  - **供電**: DC5V/1A
  - **工作頻段**: 6.5GHz
  - **工作溫度**: -20℃至65℃
  - **更新頻率**: 100Hz
  - **角度精度**: ±5°
  - **測距精度**: ±10cm
  - **測距距離**: 80m(穩定) / 150m(極限)
  - **定位角度**: 120°
  - **數據輸出**: TTL3.3V UART (/dev/ttyS0)
  - **波特率**: 115200
  - **資料位**: 8
  - **停止位**: 1
  - **數據格式**: 31 字節/數據包
  - **數據包結構**:
    - 起始/結束標記: 0x2A / 0x23
    - 角度數據: 1 字節 + 3 字節符號指示器
    - 距離數據: 4 字節（小端序，Little-Endian）
- **執行器**: Volt Electric Golf Push Cart 馬達控制器
- **控制介面**: GPIO 直接控制（Pin 18/23/24/25）

### 軟體架構
- **程式語言**: Python 3.x
- **核心函式庫**:
  - `numpy`: 矩陣運算和卡爾曼濾波
  - `RPi.GPIO`: GPIO 控制
  - `pyserial`: 串口通訊
  - `threading`: 多執行緒數據處理
- **控制算法**:
  - 自適應卡爾曼濾波器 (Adaptive Kalman Filter)
  - 預測性前饋控制 (Predictive Feedforward Control)
  - 軟體定義脈衝調製 (Software-Defined Pulse Modulation)

---

## 系統參數

| 參數 | 數值 |
|-----|------|
| UWB 更新頻率 | 100Hz |
| UWB 角度精度 | ±5° |
| UWB 測距精度 | ±10cm |
| 控制迴圈頻率 | 10 Hz |
| 卡爾曼濾波器突變閾值 | 15° |
| 系統延遲估計 | 0.2-0.5s |
| GPIO 脈衝持續時間 | 50ms (0.05s) |

---

## 專案結構

```
Fellow-me/
├── README.md                     # 專案說明文件
├── LICENSE                       # 授權聲明
│
├── docs/                         # 技術文件與原始碼目錄
│   ├── algorithm_details.md      # 算法詳細說明
│   └── follow me-跟隨球袋車.txt  # 完整系統控制程式
│
└── config/                       # 配置檔案目錄
    ├── requirements.txt          # Python 依賴套件
    └── README.md                 # 配置說明
```
