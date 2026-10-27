# 土と葉の模擬観測・給水量AI・じょうろ角度制御

## 現在できること

1. 4個のプランターを巡回する。
2. 土壌水分と葉のしおれ度を模擬センサートピックから取得する。
3. 学習済みのローカルAIで目標給水量を計算する。
4. 給水が必要な鉢に低速で近づき、停止する。
5. じょうろを傾け、Gazebo関節から取得した角度と仮想流量の積算値で給水量を調整する。
6. 目標量に達したらじょうろを直立に戻し、元の巡回経路へ戻る。
7. 給水した分だけ模擬の土壌水分・葉のしおれ度を更新し、土の色・葉の色と垂れ具合・札の色にも反映する。
8. 次の巡回では更新後の状態を読み、必要量を減らす／給水を見送る。

**土と葉はユーザーが選択した模擬センサー方式です。カメラ画像認識ではありません。** 角度はGazeboの実際の関節位置から取得します。流量・mL・植物の回復は独自の仮想モデルで、実際の水流や植物生理を再現したものではありません。

## PowerShellで実行

Gazeboで標準Burgerが起動しており、ほかの走行ノードが停止している状態で実行します。初回のビルドと導入は完了しています。

```powershell
docker exec -it -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && python3 /home/ubuntu/pai_final_project/run_precision_demo.py --reset-plants'
```

`--reset-plants` はデモ用の乾いた状態を再設定し、じょうろも仮想750 mLに補給します。その際、現在位置を維持してロボットだけを作業フォルダの関節付きモデルから再生成します。元の標準SDFとプランター配置は変更しません。カメラは全体表示に戻ります。

約2分20秒で1周します（PC負荷と開始位置で変動）。PowerShellには土壌水分・葉のしおれ度・目標mL・測定角度・指令角度・流量・積算量・給水結果が日本語で表示されます。

給水後の状態を引き継いで、もう一度観察する場合はリセットを省略します。

```powershell
docker exec -it -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && python3 /home/ubuntu/pai_final_project/run_precision_demo.py'
```

2周続ける場合は `--laps 2` を追加します。初期状態からの2周試験では、1周目に鉢1・3へ給水し、2周目は全鉢の給水を見送りました。所要時間は約3分23秒なので、提出動画は1周の実演に短い説明を添える構成が適しています。

## 停止

別のPowerShellで次を実行します。

```powershell
docker exec -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && ros2 service call /pai_patrol/stop std_srvs/srv/Trigger {}'
```

ロボットを止め、じょうろを直立に戻してから終了します。途中までの仮想給水量も植物状態に反映します。角度指令が0.8秒途絶えるとGazebo側の関節制御も目標を直立へ戻します。

## 観測・判断

`precision_plants.py` が `/pai_plants/state` に土壌水分・葉のしおれ度等を0.2秒ごとに配信します。初期値は `simulated_plants.json`、現在値は `plant_scene/live_plants.json` です。土壌水分は教材内の0～100の模擬尺度で、実センサーの体積含水率ではありません。

AIは4入力（土壌水分、葉のしおれ度、気温、湿度）、隠れ層16個のReLUニューラルネットです。1,200件で学習、200件で検証、300件で最終評価しました。人工ラベルの生成式は次です。

```text
目標mL = clip(4 × (55 − 土壌水分) + 0.45 × 葉のしおれ度
              + 0.6 × (気温 − 25) − 0.1 × (湿度 − 50), 0, 180)
```

実行時は保存済みの重みで推論します。土壌水分55以上なら追加給水を見送り、最大180仮想mLに制限します。濡れた土で葉がしおれる場合に、さらに水を与えることを避けるための制限です。植物の病気等を診断する機能ではありません。

この式は教材用に作成したもので農学的な根拠はありません。最終評価の平均絶対誤差約0.016 mLは、人工式を再現できた程度を示す数値です。実植物に必要な給水量の精度を意味しません。

## 角度・流量の制御

標準Burgerのファイルをコピーした `precision_plugin/burger_precision.sdf` に、軽量のじょうろリンクとY軸回転関節を追加しています。C++プラグインは関節位置を読み、速度モーターを使って目標角度へ動かします。表示だけだった旧版とは異なり、関節の測定値には指令値との差・応答遅れがあります。

`/pai_watering/joint_states` は標準JointState形式の角度・角速度です。`/pai_watering/can_state` は測定角度、指令、仮想流量、積算量、残量を配信します。

```text
仮想流量[mL/シミュレーション秒]
    = 1.2 × max(測定角度[度] − 20, 0) × sqrt(仮想残量 / 750)
```

この流量をGazeboの時間で積分します。Python側は目標量と積算量の差から次の目標角度を決め、残量が減った場合の流量変化にも対応します。上限角度は50度です。終了時は直立に戻るまでの流出も計上します。目標との許容差は±5仮想mLです。

水滴は測定角度・流量から描画する説明用表示です。水滴の衝突計算や流体シミュレーションは行っていません。仮想給水の植物への反映は、鉢のそばで制御している対象のIDへ送っています。

## 状態の更新

4仮想mLで土壌水分を1ポイント上げ、2仮想mLで葉のしおれ度を1ポイント下げます。給水イベントはセッションごとの積算値で送るため、同じ通知の重複で二重加算しません。変化は土の色、葉の色と姿勢、札の色へ反映します。

これは動作結果を次の判断に接続するための簡易モデルです。蒸散、根の吸収、時間をかけた葉の回復、鉢の大きさ別の水収支等はモデル化していません。

## 接近と保護

巡回地点から対象鉢の中心約0.50 mの位置まで、上限0.055 m/sで接近し、静止して給水します。その後は巡回地点へ戻ります。通常の前方停止距離0.35 mに対し、既知の鉢の接近範囲では0.14 m（再開0.19 m）を使い、低速で位置を合わせます。土・葉・角度データが途絶えた場合は給水を終了し、直立を指示します。

ロボットの元SDF、既存散水アーム関連ファイル、Docker設定、既存パッケージは変更していません。実行中のロボットはコピーした関節付きモデルへ差し替えています。変更前のコードは `backups/before_precision_20261026T104714.zip` に保存しました。

## 記録・再試験

通常実行の `logs/patrol_*.jsonl` に判断・目標量・結果、対応する `*_precision.csv` に角度・流量・積算量を保存します。CSVの `elapsed_s` は実時間、`sim_time` はGazebo時間です。

```powershell
# 独立したROS通信範囲での16テスト
docker exec -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && cd /home/ubuntu/pai_final_project && python3 -m unittest -v test_patrol test_watering_ai test_precision'

# 2周の実走行試験（状態と仮想残量を初期化）
docker exec -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && python3 /home/ubuntu/pai_final_project/verify_precision_demo.py'

# 給水中の停止試験
docker exec -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && python3 /home/ubuntu/pai_final_project/verify_precision_demo.py --laps 1 --stop-during-watering'
```

旧版の `patrol.py --watering-demo` も残していますが、固定の模擬値から要否を判断して待機する旧動作です。精密給水には `run_precision_demo.py` を使用してください。

再ビルドが必要な場合は、既存コンテナの開発環境で以下を実行します。

```powershell
docker exec -u ubuntu pai_ros2 bash -lc 'source /opt/ros/humble/setup.bash && cmake -S /home/ubuntu/pai_final_project/precision_plugin -B /home/ubuntu/pai_final_project/precision_plugin/build -DCMAKE_BUILD_TYPE=Release && cmake --build /home/ubuntu/pai_final_project/precision_plugin/build -j2'
```
