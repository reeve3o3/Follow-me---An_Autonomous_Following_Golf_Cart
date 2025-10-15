# 配置檔案目錄

此目錄包含系統配置和依賴管理檔案。

## 檔案說明

### requirements.txt
Python 依賴套件列表

**核心依賴**：
- `numpy` - 數學運算和矩陣操作
- `pyserial` - 串口通訊
- `RPi.GPIO` - Raspberry Pi GPIO 控制

**選配依賴**：
- `matplotlib` - 數據視覺化（調試用）
- `pandas` - 數據分析和記錄

## 安裝方式

```bash
pip install -r config/requirements.txt
```
