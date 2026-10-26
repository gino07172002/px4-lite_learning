# 模擬展示素材

| 檔案 | 用途 | 規格 | 容量 |
| --- | --- | --- | --- |
| `px4-lite-comparison.mp4` | 完整原速影片 | H.264、1854×812、30 fps、約 26.8 秒、AAC 音訊 | 約 0.60 MB |
| `px4-lite-comparison.gif` | README 自動循環預覽 | 960×420、10 fps、兩倍速、無音訊 | 約 2.06 MB |

來源為 `D:\px4-lite-simu.mp4`（約 20.3 MB）。原始影片不需加入儲存庫。

根目錄 README 使用可點擊的 GIF 預覽，連到儲存庫中的完整 MP4。將 README 與這兩個素材檔一起推送即可顯示預覽。

若希望 README 直接顯示 GitHub 的影片播放器：

1. 在 GitHub 網頁編輯根目錄 `README.md`。
2. 將壓縮後的 `px4-lite-comparison.mp4` 拖入編輯區，等待附件上傳完成。
3. 保留 GitHub 產生的 `https://github.com/user-attachments/assets/...` 網址，獨立放一行，可取代 GIF 預覽那一行。
4. 用 Preview 確認播放器後儲存變更。

影片低於免費方案的 10 MB 附件限制，詳見 [GitHub 附件說明](https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/attaching-files)。

## 重新產生

在儲存庫根目錄執行（需要 FFmpeg）：

```powershell
ffmpeg -i D:\px4-lite-simu.mp4 -map 0:v:0 -map '0:a:0?' -c:v libx264 -preset slow -crf 26 -pix_fmt yuv420p -c:a aac -b:a 64k -movflags +faststart -map_metadata -1 -y docs/media/px4-lite-comparison.mp4

ffmpeg -i D:\px4-lite-simu.mp4 -filter_complex "[0:v]setpts=0.5*PTS,fps=10,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=sierra2_4a:diff_mode=rectangle" -an -loop 0 -y docs/media/px4-lite-comparison.gif
```
