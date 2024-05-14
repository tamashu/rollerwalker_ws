# About Rollerwalker
- 駆動力を持たない受動車輪を用いた脚・車輪ハイブリッド型移動ロボット
- 脚型移動ロボットの脚先に受動車輪と足首切り替え機構を付与することでハイブリッド化
- ローラースケートと同じ原理で移動 **（ローラーウォーク）**
- モデルは東工大のTITAN-Ⅷの脚先に受動車輪を付加することで作成
- 高さ0.25m，左右幅0.6m，前後0.5m，全質量24kg
  
## 参考文献
1. 遠藤玄，　広瀬茂男：“ローラーウォーカーに関する研究　-基本的運動の生成と自立推進実験-”，日本ロボット学会誌，18巻，8号，pp.1159-1165，2000.
2. [遠藤玄，　広瀬茂男：“ローラーウォーカーに関する研究　-脚軌道による推進特性の適応的調節-", 日本ロボット学会誌, 26巻, 6号, pp.691-698, 2008.](https://web.archive.org/web/20190504171340id_/https://www.jstage.jst.go.jp/article/jrsj1983/26/6/26_6_691/_pdf)
3. ロボ學,"脚車輪複合移動体　ローラーウォーカー",[https://robogaku.jp/history/locomotion/L-2000-2.html](https://robogaku.jp/history/locomotion/L-2000-2.html),最終閲覧日：2024/05/14.

# About Program
- 動作環境：Ubuntu20.04 ros-noetic
- 必要パッケージ
```shell-session
sudo apt install ros-noetic-gazebo-ros
sudo apt install ros-noetic-gazebo-ros-control 
sudo apt install ros-noetic-ros-control
sudo apt install ros-noetic-ros-controllers 
```
- ビルド
```shell-session
  cd rollerwalker_ws/
  catkin build
  source devel/setup.bash
```

- 直進動作の軌道
```shell-session
  roslaunch straight_forwad straight_forward.launch 
```



https://github.com/tamashu/rollerwalker_ws/assets/110759396/835bf609-bb99-471e-81de-875921183c92

