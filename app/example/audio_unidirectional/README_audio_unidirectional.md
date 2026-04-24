# 音訊單向傳輸範例（audio_unidirectional）

本文件說明範例的目的、架構、建置與燒錄方式、執行流程（Coordinator/Node）、SAC 管線配置、按鍵/LED 行為、統計監控與常見問題。內容以實作導向為主，約 4 頁可讀。

---

## 1. 範例目的與成果

- 目標：展示使用 SPARK 無線（SWC）與音訊核心（SAC）建立「單向」立體聲 48 kHz 低延遲音訊串流。
- 角色劃分：
  - Coordinator（Coord）：從 I2S/SAI 取得音訊，經處理後透過無線傳送。
  - Node：接收無線音訊，經處理後送往 I2S/SAI 播放。
- 音質與韌體防護：
  - 正常模式使用 24-bit；連線品質不佳時自動降為 16-bit（Fallback）。
  - 具時鐘漂移補償（CDC）與 Underflow 靜音保護，減少破音與爆音。

---

## 2. 專案架構

- App（本範例所在）：`app/example/audio_unidirectional/`
  - `audio_unidirectional_coord.c`：Coordinator 應用層流程與回呼。
  - `audio_unidirectional_node.c`：Node 應用層流程與回呼。
- Backend：`backend/quasar_backend/`
  - 封裝 BSP 與第三方元件，提供 I2S、無線端點、LED/Button、定時器等介面（facade）。
- BSP（Quasar）：`bsp/quasar/`
  - 板級驅動（GPIO、LED、RGB、Button、Audio、Timer、Radio SPI 等）。
- Core（Audio/SWC）：`core/audio`, `core/wireless`
  - SAC：音訊管線、處理 Stage、統計與記憶體池。
  - SWC：無線連線、封包時槽、連線回呼、連線品質指標。

---

## 3. 建置與燒錄（快速指引）

> 以下為 VS Code 工作區常見作法；實際依您環境與 Presets 為準。

- 建置：
  - 推薦使用工作區任務或 CMake Presets（`CMakePresets.json`）。
- 燒錄：
  - 可使用 VS Code 任務「`[DFU/STM32] Flash binary`」，其底層呼叫 `script/dfu-util.py`。
- PowerShell（自備 pandoc/dfu 驅動僅供參考）：
  ```powershell
  # 產物在 build/ 對應目錄；依實際目標板與 Preset 而定
  # 燒錄任務
  # VS Code: Terminal → Run Task → [DFU/STM32] Flash binary
  ```

---

## 4. 執行流程總覽

### 4.1 Coordinator（I2S → SWC）

流程摘要：
1. 進入配對模式、建立 SWC 連線。
2. 啟動 I2S DMA RX，透過回呼將音訊餵入 SAC Producer。
3. 由 App 定時器觸發 SAC Process 與 Consume，經 Packing 與（必要時）Fallback 後交給 SWC 發送。

關鍵回呼（節錄）：
```c
// I2S DMA RX 完成：產生音訊
static void i2s_rx_audio_complete_callback(void)
{
    sac_pipeline_produce(sac_pipeline, &sac_status);
    facade_audio_process_timer_trigger(); // 觸發處理
}

// App 定時器：處理與消費
static void audio_process_callback(void)
{
    if (sac_pipeline_get_producer_buffer_load(sac_pipeline, &sac_status) > 0)
        sac_pipeline_process(sac_pipeline, &sac_status);

    if (sac_pipeline_get_consumer_buffer_load(sac_pipeline, &sac_status) > 0)
        sac_pipeline_consume(sac_pipeline, &sac_status);
}
```

### 4.2 Node（SWC → I2S）

流程摘要：
1. 完成配對，接收 SWC 音訊封包。
2. 由 SWC 回呼/機制定期將封包推入 SAC Producer。
3. App 定時器觸發 SAC Process（Unpack、Volume、CDC、Mute），DMA TX 完成回呼驅動下一筆音訊。

關鍵回呼（節錄）：
```c
// I2S DMA TX 完成：消費音訊
static void i2s_tx_audio_complete_callback(void)
{
    sac_pipeline_consume(sac_pipeline, &sac_status);
}

// App 定時器：處理（Unpack/Volume/CDC/Mute）
static void audio_process_callback(void)
{
    sac_pipeline_process(sac_pipeline, &sac_status);
}
```

---

## 5. SAC 管線配置（重點）

本範例以單一路徑為主，每端一條管線：

### 5.1 Coordinator 管線（Producer: I2S → Consumer: SWC）

- Producer：I2S（無封裝、即時產生）。
- Processing：
  - Fallback（根據 link margin 切換位元深度）
  - Packing（32→24 bit，正常模式）
  - Packing Fallback（32→16 bit，降質模式）
- Consumer：SWC（需封裝、延遲消費以對齊時槽）。

ASCII 流程：
```
[I2S Producer] → [Fallback] → [Pack24 / Pack16] → [SWC Consumer]
```

### 5.2 Node 管線（Producer: SWC → Consumer: I2S）

- Producer：SWC（封裝資料解出）。
- Processing：
  - Fallback（同步判定）
  - Unpack24（24→32 bit，正常模式）
  - Unpack16（16→32 bit，降質模式）
  - Volume（數位音量）
  - CDC（時鐘漂移補償）
  - Mute on Underflow（防爆音）
- Consumer：I2S（無封裝、即時消費）。

ASCII 流程：
```
[SWC Producer] → [Fallback] → [Unpack24/Unpack16] → [Volume] → [CDC] → [Mute] → [I2S Consumer]
```

關鍵參數建議：
- `do_initial_buffering`：Coord=true（預先緩衝），Node=false。
- Fallback：`link_margin_threshold ≈ 12 dB`。
- CDC（Node 端）：`resampling_length ≈ 48`，隊列目標 ≈ 3 封包。

---

## 6. 配對、按鍵與 LED

- 進入配對：按下用戶按鍵（如 `BUTTON_USER_1`，實際見 BSP 映射）。
- LED 指示（範例常見邏輯）：
  - 未配對/配對中：閃爍指示（若為 AV IND 無 codec，使用 PA1 LED）。
  - 配對成功：常亮/顏色改變。
  - Fallback 狀態：可用 RGB/LED 反映（由 backend facade 統一控制）。

> 實際腳位與行為以 `backend/quasar_backend` 與 `bsp/quasar` 的定義為準。

---

## 7. 統計與診斷

常用查詢：
```c
uint32_t pl = sac_pipeline_get_producer_buffer_load(sac_pipeline, &sac_status);
uint32_t cl = sac_pipeline_get_consumer_buffer_load(sac_pipeline, &sac_status);
uint32_t ov = sac_pipeline_get_consumer_buffer_overflow_count(sac_pipeline, &sac_status);
uint32_t uf = sac_pipeline_get_consumer_buffer_underflow_count(sac_pipeline, &sac_status);
```

判讀指南：
- Producer Overflow > 0：輸入過快；增大 Producer queue 或降延遲需求。
- Consumer Underflow > 0：處理/輸出跟不上；開啟 CDC 或加大延遲（Node）。
- Fallback 頻繁：連線品質不佳；調整天線、距離、干擾源。

---

## 8. 常見問題（FAQ）

- Q1：為何一開始容易 Underflow？
  - A：Coordinator 預先緩衝不足。設 `do_initial_buffering=true` 並調整緩衝門檻。

- Q2：24/16-bit 何時切換？
  - A：由 Fallback 依 `link_margin_threshold` 自動判斷；門檻可按場域調整。

- Q3：Node 端出現長期抖動？
  - A：啟用 CDC 並確認 `resampling_length`、目標隊列深度設定合理。

- Q4：如何快速定位記憶體破壞？
  - A：初始化前後填充 Memory Pool 並加 Guard Bytes；異常時立即報錯。

---

## 9. 延伸閱讀與建議

- SAC 進階章節：「第三章：SPARK Audio Core (SAC) 詳解」（同工作區檔案）。
- Backend 封裝與板級差異：`backend/quasar_backend/` 與 `bsp/quasar/`。
- 無線配置與連線機制：`core/wireless/` 相關設定與回呼。

---

（完）