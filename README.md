# terrainshare_new

## RealSense D435,D415で動作確認をしています。

### https://dev.intelrealsense.com/docs
### こちら参考にしています

## WindowsのAnaconda環境にて動作確認済み。

conda activate something
python terrainshare_new.py
python terrainshare_april_tag.py

・terrainshare_new.py
現行の最新版。emriverに対するマッピング、PC側での点群表示などの機能を搭載している。
安定動作(6時間以上の耐久を確認)

・terrainshare_april_tag.py
RealSenseでAprilTagを読むためのコード。
高橋かずひとさんの
https://github.com/Kazuhito00/AprilTag-Detection-Python-Sample
こちらのリポジトリをRealSense化した。
タグの中心の距離を取るようになっている

・ros2_april_tag_realsense.py
ROS2とRealSenseでARタグをするプログラム。
基本的に上をROS2化しただけ。今後tf出力などもやる予定。

今後もwebサイト追加などいろいろとやることがあるので応援よろしくお願いします！
