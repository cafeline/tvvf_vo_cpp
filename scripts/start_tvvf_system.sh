#!/bin/bash

# TVVF-VO Global Field Navigation System 起動スクリプト

echo "=== TVVF-VO Global Field Navigation System 起動スクリプト ==="

# ROSワークスペースのセットアップ
source /opt/ros/humble/setup.bash
source /home/rosuser/raspicat_ws/install/setup.bash

echo "1. Gazeboシミュレーションを起動中..."
ros2 launch raspicat_gazebo raspicat_with_iscas_museum.launch.py &
GAZEBO_PID=$!

echo "2. シミュレーション起動を待機中..."
sleep 5

echo "3. モーターパワーを有効化中..."
ros2 service call /motor_power std_srvs/SetBool '{data: true}'

echo "4. マップサーバーを起動中..."
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/rosuser/raspicat_ws/src/raspicat_slam_navigation/raspicat_slam/config/maps/iscas_museum_map.yaml \
  -p use_sim_time:=true &
MAP_SERVER_PID=$!

echo "5. ライフサイクルマネージャーでマップサーバーを起動中..."
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=true \
  -p autostart:=true \
  -p node_names:=[map_server] &
LIFECYCLE_PID=$!

sleep 3

echo "6. EMCLを起動中（自己位置推定）..."
ros2 run emcl2 emcl2_node --ros-args \
  -p use_sim_time:=true \
  -r scan:=/scan &
EMCL_PID=$!

echo "7. Costmap Generatorを起動中..."
ros2 run costmap_generator costmap_generator_node --ros-args \
  -p use_sim_time:=true \
  -p wall_clearance_distance:=0.3 \
  -p dynamic_obstacle_radius:=0.15 \
  -p publish_frequency:=10.0 &
COSTMAP_PID=$!

echo "8. Obstacle Trackerを起動中..."
ros2 run obstacle_tracker obstacle_tracker --ros-args \
  -p use_sim_time:=true &
OBSTACLE_PID=$!

echo "9. TVVF-VOナビゲーションシステムを起動中..."
ros2 launch tvvf_vo_c tvvf_vo_global_field.launch.py use_sim_time:=true &
TVVF_PID=$!

echo "10. /cmd_vel監視を開始..."
sleep 5

# マップはマップサーバーから提供されるため、手動パブリッシュは不要

sleep 2

echo "11. ゴール地点を設定中..."
ros2 topic pub -1 /clicked_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'map'}, point: {x: 6.0, y: -5.0, z: 0.0}}"

echo ""
echo "=== システム起動完了 ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Map Server PID: $MAP_SERVER_PID"
echo "Lifecycle Manager PID: $LIFECYCLE_PID"
echo "EMCL PID: $EMCL_PID"
echo "Costmap Generator PID: $COSTMAP_PID"
echo "Obstacle Tracker PID: $OBSTACLE_PID"
echo "TVVF-VO PID: $TVVF_PID"
echo ""

echo "11. /cmd_velトピックを監視中..."
echo "-----------------------------------"
ros2 topic echo /cmd_vel --once &
CMD_VEL_MONITOR=$!

# 10秒間cmd_velを監視
for i in {1..10}; do
    echo "監視中... ($i/10秒)"
    ros2 topic echo /cmd_vel --once 2>/dev/null && echo "✓ /cmd_vel 出力確認！"
    sleep 1
done

echo ""
echo "システムを停止するには Ctrl+C を押してください"

# シグナルハンドリング
trap 'echo "システムを停止中..."; kill $GAZEBO_PID $MAP_SERVER_PID $LIFECYCLE_PID $EMCL_PID $COSTMAP_PID $OBSTACLE_PID $TVVF_PID $CMD_VEL_MONITOR 2>/dev/null; exit' INT

# プロセスが終了するまで待機
wait