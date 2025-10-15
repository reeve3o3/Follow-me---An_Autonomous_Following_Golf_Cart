# 算法詳細說明 - 自適應卡爾曼濾波器與預測性前饋控制

## 概述

本文件提供 Follow Me 自動跟隨系統中使用的核心算法的深入技術細節。

## 1. 自適應卡爾曼濾波器 (Adaptive Kalman Filter, AKF)

### 1.1 數學基礎

自適應卡爾曼濾波器估計狀態向量：

```
x = [θ, ω]ᵀ
```

其中：
- `θ` = 角度（度）
- `ω` = 角速度（度/秒）

### 1.2 狀態空間模型

**預測步驟 (Prediction Step)：**

```
x̂ₖ|ₖ₋₁ = Fₖ xₖ₋₁
Pₖ|ₖ₋₁ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ
```

其中狀態轉移矩陣：

```
Fₖ = [1  dt]  (角度 = 舊角度 + 角速度×dt)
     [0   1]  (角速度 = 舊角速度)
```

**更新步驟 (Update Step)：**

```
Kₖ = Pₖ|ₖ₋₁ Hᵀ (H Pₖ|ₖ₋₁ Hᵀ + Rₖ)⁻¹
x̂ₖ = x̂ₖ|ₖ₋₁ + Kₖ(zₖ - H x̂ₖ|ₖ₋₁)
Pₖ = (I - Kₖ H) Pₖ|ₖ₋₁
```

其中觀測矩陣（UWB 只測量角度）：

```
H = [1  0]  (只提取狀態向量中的角度部分)
```

### 1.3 自適應機制

關鍵創新是**觀測噪聲協方差 (R) 的動態調整**：

```python
if detect_angle_jump(measured_angle):
    R = R_base * 0.3  # 降低噪聲，增加信任度
else:
    R = R_base        # 標準噪聲
```

**突變檢測算法：**

程式碼實現：
```python
def detect_angle_jump(self, measured_angle):
    if len(self.angle_history) < 2:
        return False
    
    # 計算與歷史角度的差異
    recent_angles = list(self.angle_history)[-3:]
    avg_recent = np.mean(recent_angles)
    
    # 檢查是否有大幅度突變
    angle_diff = abs(measured_angle - avg_recent)
    
    return angle_diff > self.innovation_threshold
```

其中：
- `measured_angle` = 當前測量值
- `recent_angles` = 最近 3 個測量樣本
- `innovation_threshold` = 15.0°

### 1.4 參數設定

從程式碼中的實際參數：

**狀態轉移矩陣 F：**
```python
self.F = np.array([[1.0, 0.0],
                  [0.0, 1.0]])
# 在 predict 時更新: self.F[0, 1] = dt
```

**過程噪聲協方差 Q：**
```python
self.Q = np.array([[0.1, 0.0],
                  [0.0, 0.5]])
```

**觀測噪聲協方差 R：**
```python
self.R_base = np.array([[1.0]])
# 自適應調整: 突變時 R = R_base * 0.3
```

**突變檢測閾值：**
```python
self.innovation_threshold = 15.0  # 度
```

## 2. 預測性前饋控制器 (Predictive Feedforward Controller)

### 2.1 運動學預測模型

控制器使用**恆定加速度運動學模型**預測未來角度：

```
θ(t + Δt) = θ(t) + ω(t)·Δt + ½·α(t)·Δt²
```

其中：
- `θ(t)` = 當前角度
- `ω(t)` = 角速度
- `α(t)` = 角加速度
- `Δt` = 預測範圍（估計延遲）

### 2.2 角速度估計

**計算方法（平均多個瞬時速度）：**

程式碼實現：
```python
def calculate_angular_velocity(self):
    # 使用最近的幾個點計算角速度
    angles = list(self.angle_history)[-3:]
    times = list(self.time_history)[-3:]
    
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

數學表達：
```
ω = mean[(θᵢ - θᵢ₋₁) / (tᵢ - tᵢ₋₁)]  for i = 1 to n
```

其中 n 為最近 2-3 個樣本的速度計算結果

### 2.3 角加速度估計

**計算方法（角速度的平均變化率）：**

程式碼實現：
```python
def calculate_angular_acceleration(self):
    # 計算連續的角速度
    velocities = []
    vel_times = []
    for i in range(1, len(angles)):
        dt = times[i] - times[i-1]
        if dt > 0:
            dangle = angles[i] - angles[i-1]
            # 處理角度跨越
            velocities.append(dangle / dt)
            vel_times.append((times[i] + times[i-1]) / 2)
    
    # 計算角加速度
    accelerations = []
    for i in range(1, len(velocities)):
        dt = vel_times[i] - vel_times[i-1]
        if dt > 0:
            dvel = velocities[i] - velocities[i-1]
            accelerations.append(dvel / dt)
    
    return np.mean(accelerations) if accelerations else 0.0
```

數學表達：
```
α = mean[(ωᵢ - ωᵢ₋₁) / (tᵢ - tᵢ₋₁)]
```

其中 `ω` 是從角度計算出的瞬時角速度

### 2.4 延遲補償

**自適應延遲估計：**

```
τ_estimated = 1.5 × mean(Δt_measurements)
```

系統持續監控數據到達間隔並相應調整預測範圍。

**預測範圍約束：**

```
Δt_pred = min(τ_estimated, τ_max)
```

其中 `τ_max = 0.5s`（安全限制）

### 2.5 控制因子計算

控制因子根據多個啟發式方法動態調整控制強度：

```
f_control = f_convergence × f_distance × f_angle × f_velocity × f_prediction
```

其中每個因子範圍為 [0.1, 1.0]：

| 因子 | 條件 | 數值 |
|--------|-----------|-------|
| `f_convergence` | 收斂至目標 | 0.5 |
| `f_distance` | `d < 2m` | `d / 2` |
| `f_angle` | `|θ| < 15°` | 0.6 |
| `f_velocity` | `|ω| > 20°/s` | 0.4 |
| `f_prediction` | 改善趨勢 | 0.7 |

## 3. 脈衝調製控制

### 3.1 準比例速度控制

系統通過**脈衝間隔調製**將二進制開/關控制轉換為比例速度：

```
T_interval = T_base / f_control
```

其中：
- `T_base` = 操作類型的基礎間隔
- `f_control` = 來自預測控制器的控制因子

### 3.2 基礎間隔

**前進運動：**

| 距離區間 | 基礎間隔 |
|--------------|---------------|
| `d > 2.5m` | 0.02s（快速） |
| `1.5m < d < 2.5m` | 0.08s（慢速） |
| `d < 1.5m` | 停止 |

**轉向：**

| 角度大小 | 基礎間隔 |
|----------------|---------------|
| `|θ| > 45°` | 0.05s（激進） |
| `20° < |θ| < 45°` | 0.08s（正常） |
| `|θ| < 20°` | 0.15s（精細） |

### 3.3 脈衝持續時間

所有控制脈衝使用固定持續時間 `50ms`，足以觸發馬達控制器同時避免持續啟動。

## 4. 系統整合

### 4.1 數據流

```
UWB 傳感器 (100 Hz 硬體規格)
    ↓
UWB 讀取執行緒 (持續讀取，10ms 延遲)
    ↓
自適應卡爾曼濾波器 (每次讀取時濾波)
    ↓
主控制迴圈 (10 Hz)
    ↓
預測控制器 (計算控制因子)
    ↓
自適應速度控制器 (計算脈衝間隔)
    ↓
脈衝調製層 (GPIO 控制)
    ↓
馬達控制器 (車輛運動)
```

### 4.2 時序特性

根據程式碼中的實際參數：

| 組件 | 參數 | 說明 |
|-----------|------|------|
| UWB 硬體更新頻率 | 100 Hz | UWB 模組規格 |
| UWB 讀取執行緒延遲 | 10 ms | `time.sleep(0.01)` |
| 主控制迴圈 | 10 Hz | `time.sleep(0.1)` |
| GPIO 脈衝持續時間 | 50 ms | `duration=0.05` |
| 數據逾時檢查 | 1.0 s | `timeout=1.0` |

### 4.3 執行緒架構

- **主執行緒**：控制迴圈（10 Hz）
- **UWB 執行緒**：串口數據讀取（持續）
- **GPIO 控制**：同步（由主執行緒觸發）

## 5. 調整指南

### 5.1 卡爾曼濾波器調整

**過程噪聲協方差 (Q)：**

```
Q = [q_angle    0     ]
    [0       q_velocity]
```

- 增加 `q_angle` 以獲得更快的角度追蹤
- 增加 `q_velocity` 以獲得更好的速度估計
- 預設值：`q_angle = 0.1`, `q_velocity = 0.5`

**觀測噪聲協方差 (R)：**

```
R_base = [r_angle]
```

- 增加 `r_angle` 以獲得更多平滑
- 減少 `r_angle` 以獲得更快響應
- 預設值：`r_angle = 1.0`

**尖峰檢測閾值：**

- 增加以降低尖峰檢測靈敏度
- 減少以獲得更積極的濾波
- 預設值：`15°`

### 5.2 預測控制調整

**延遲估計：**

- 根據觀察到的系統響應進行調整
- 測量實際的控制到響應延遲
- 預設值：`0.2s`

**預測範圍：**

- 應匹配或略微超過系統延遲
- 限制以防止過度預測
- 預設值：`0.5s`（最大值）

### 5.3 控制因子調整

在 `calculate_control_factor()` 中調整權重：

```python
# 當前預設值：
f_convergence = 0.5
f_angle_small = 0.6
f_velocity_high = 0.4
f_prediction = 0.7
```

- 減少因子以獲得更積極的控制
- 增加因子以獲得更保守的控制

---

**文件版本**：1.0  
**最後更新**：2025 年 10 月
