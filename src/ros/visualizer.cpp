#include "tvvf_vo_c/ros/visualizer.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c {

TVVFVOVisualizer::TVVFVOVisualizer(rclcpp::Node& node, const std::string& global_frame)
    : node_(node), global_frame_(global_frame) {
}

visualization_msgs::msg::MarkerArray TVVFVOVisualizer::create_vector_field_markers(
    const std::vector<Position>& positions,
    const std::vector<Force>& forces,
    int marker_id) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    if (positions.size() != forces.size()) {
        RCLCPP_WARN(node_.get_logger(), "Vector field positions and forces size mismatch");
        return marker_array;
    }

    // ベクトル場の矢印マーカー
    for (size_t i = 0; i < positions.size(); ++i) {
        if (forces[i].magnitude() < 1e-3) continue;  // 小さい力は描画しない

        Position start = positions[i];
        Position end = Position(
            positions[i].x + forces[i].fx * 0.2,  // スケール調整
            positions[i].y + forces[i].fy * 0.2
        );

        // 力の大きさに応じて色を変更
        double force_magnitude = forces[i].magnitude();
        std_msgs::msg::ColorRGBA color;
        if (force_magnitude < 1.0) {
            color = create_color(0.0, 1.0, 0.0, 0.6);  // 緑（小さい力）
        } else if (force_magnitude < 3.0) {
            color = create_color(1.0, 1.0, 0.0, 0.7);  // 黄（中程度の力）
        } else {
            color = create_color(1.0, 0.0, 0.0, 0.8);  // 赤（大きい力）
        }

        auto arrow_marker = create_arrow_marker(start, end, color, marker_id + i, 0.03);
        arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム
        marker_array.markers.push_back(arrow_marker);
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray TVVFVOVisualizer::create_obstacle_markers(
    const std::vector<DynamicObstacle>& obstacles,
    int marker_id) {
    
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obstacle = obstacles[i];

        // 障害物の円柱マーカー
        std_msgs::msg::ColorRGBA color;
        if (obstacle.velocity.magnitude() < 0.1) {
            color = create_color(0.8, 0.0, 0.0, 0.7);  // 赤（静的障害物）
        } else {
            color = create_color(1.0, 0.5, 0.0, 0.8);  // オレンジ（動的障害物）
        }

        auto cylinder_marker = create_cylinder_marker(
            obstacle.position, obstacle.radius, 0.2, color, marker_id + i);
        cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム
        marker_array.markers.push_back(cylinder_marker);

        // 動的障害物の速度ベクトル
        if (obstacle.velocity.magnitude() > 0.1) {
            Position start = obstacle.position;
            Position end = Position(
                obstacle.position.x + obstacle.velocity.vx * 2.0,
                obstacle.position.y + obstacle.velocity.vy * 2.0
            );

            auto velocity_arrow = create_arrow_marker(
                start, end, create_color(0.0, 0.0, 1.0, 0.8), 
                marker_id + obstacles.size() + i, 0.02);
            velocity_arrow.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム
            marker_array.markers.push_back(velocity_arrow);
        }
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray TVVFVOVisualizer::create_vo_cone_markers(
    const RobotState& robot_state,
    const std::vector<DynamicObstacle>& obstacles,
    double time_horizon,
    int marker_id) {
    
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < obstacles.size(); ++i) {
        const auto& obstacle = obstacles[i];

        // VO円錐の計算
        auto cone_vertices = compute_vo_cone_vertices(
            robot_state.position, obstacle.position, obstacle.radius, time_horizon);

        if (cone_vertices.size() >= 3) {
            // 円錐の線分マーカー
            std_msgs::msg::ColorRGBA cone_color = create_color(1.0, 0.0, 1.0, 0.4);  // マゼンタ
            auto cone_marker = create_line_marker(cone_vertices, cone_color, marker_id + i, 0.01);
            cone_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            
            // 円錐の線分を定義
            cone_marker.points.clear();
            for (size_t j = 1; j < cone_vertices.size(); ++j) {
                // ロボットから各頂点への線
                geometry_msgs::msg::Point p1, p2;
                p1.x = robot_state.position.x;
                p1.y = robot_state.position.y;
                p1.z = 0.1;
                p2.x = cone_vertices[j].x;
                p2.y = cone_vertices[j].y;
                p2.z = 0.1;
                
                cone_marker.points.push_back(p1);
                cone_marker.points.push_back(p2);
            }
            
            cone_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム
            marker_array.markers.push_back(cone_marker);
        }
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray TVVFVOVisualizer::create_path_markers(
    const Path& path,
    int marker_id) {
    
    visualization_msgs::msg::MarkerArray marker_array;

    if (path.points.empty()) {
        return marker_array;
    }

    // 経路点をPositionに変換
    std::vector<Position> path_positions;
    for (const auto& point : path.points) {
        path_positions.push_back(point.position);
    }

    // 経路の線分マーカー
    std_msgs::msg::ColorRGBA path_color = create_color(0.0, 0.8, 1.0, 0.9);  // シアン
    auto path_marker = create_line_marker(path_positions, path_color, marker_id, 0.05);
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム
    marker_array.markers.push_back(path_marker);

    // 経路点の球マーカー
    for (size_t i = 0; i < path.points.size(); i += 3) {  // 間引き表示
        auto point_marker = visualization_msgs::msg::Marker();
        point_marker.header.frame_id = global_frame_;
        point_marker.header.stamp = node_.get_clock()->now();
        point_marker.id = marker_id + 1000 + i;
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;

        point_marker.pose.position.x = path.points[i].position.x;
        point_marker.pose.position.y = path.points[i].position.y;
        point_marker.pose.position.z = 0.05;

        point_marker.scale.x = 0.08;
        point_marker.scale.y = 0.08;
        point_marker.scale.z = 0.08;

        point_marker.color = create_color(0.0, 1.0, 1.0, 0.8);
        point_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

        marker_array.markers.push_back(point_marker);
    }

    return marker_array;
}

visualization_msgs::msg::Marker TVVFVOVisualizer::create_goal_marker(
    const Goal& goal,
    int marker_id) {
    
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_.get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal.position.x;
    marker.pose.position.y = goal.position.y;
    marker.pose.position.z = 0.0;

    marker.scale.x = goal.tolerance * 2;
    marker.scale.y = goal.tolerance * 2;
    marker.scale.z = 0.05;

    marker.color = create_color(0.0, 1.0, 0.0, 0.6);  // 緑
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

    return marker;
}

visualization_msgs::msg::Marker TVVFVOVisualizer::create_trajectory_marker(
    const std::vector<Position>& robot_positions,
    int marker_id) {
    
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_.get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    for (const auto& pos : robot_positions) {
        geometry_msgs::msg::Point point;
        point.x = pos.x;
        point.y = pos.y;
        point.z = 0.02;
        marker.points.push_back(point);
    }

    marker.scale.x = 0.02;  // 線の太さ
    marker.color = create_color(0.5, 0.0, 0.5, 0.8);  // 紫
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

    return marker;
}

visualization_msgs::msg::MarkerArray TVVFVOVisualizer::create_delete_all_markers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    auto delete_marker = visualization_msgs::msg::Marker();
    delete_marker.header.frame_id = global_frame_;
    delete_marker.header.stamp = node_.get_clock()->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.id = 0;
    
    marker_array.markers.push_back(delete_marker);
    return marker_array;
}

std_msgs::msg::ColorRGBA TVVFVOVisualizer::create_color(double r, double g, double b, double a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

visualization_msgs::msg::Marker TVVFVOVisualizer::create_arrow_marker(
    const Position& start,
    const Position& end,
    const std_msgs::msg::ColorRGBA& color,
    int marker_id,
    double scale) {
    
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_.get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = start.x;
    start_point.y = start.y;
    start_point.z = 0.1;
    end_point.x = end.x;
    end_point.y = end.y;
    end_point.z = 0.1;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    marker.scale.x = scale;      // 軸の太さ
    marker.scale.y = scale * 2;  // 矢印の幅
    marker.scale.z = scale * 2;  // 矢印の高さ

    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

    return marker;
}

visualization_msgs::msg::Marker TVVFVOVisualizer::create_cylinder_marker(
    const Position& center,
    double radius,
    double height,
    const std_msgs::msg::ColorRGBA& color,
    int marker_id) {
    
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_.get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = height / 2.0;

    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = height;

    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

    return marker;
}

visualization_msgs::msg::Marker TVVFVOVisualizer::create_line_marker(
    const std::vector<Position>& points,
    const std_msgs::msg::ColorRGBA& color,
    int marker_id,
    double line_width) {
    
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_.get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    for (const auto& pos : points) {
        geometry_msgs::msg::Point point;
        point.x = pos.x;
        point.y = pos.y;
        point.z = 0.05;
        marker.points.push_back(point);
    }

    marker.scale.x = line_width;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

    return marker;
}

std::vector<Position> TVVFVOVisualizer::compute_vo_cone_vertices(
    const Position& robot_pos,
    const Position& obstacle_pos,
    double obstacle_radius,
    double time_horizon) {
    
    std::vector<Position> vertices;

    // ロボットから障害物への方向ベクトル
    double dx = obstacle_pos.x - robot_pos.x;
    double dy = obstacle_pos.y - robot_pos.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < obstacle_radius) {
        return vertices;  // ロボットが障害物内にある場合
    }

    // 障害物に対する接線角度を計算
    double tangent_angle = std::asin(obstacle_radius / distance);
    double obstacle_angle = std::atan2(dy, dx);

    // 円錐の頂点計算（時間ホライズン分だけ延長）
    double extension_length = time_horizon * 2.0;  // 可視化用の長さ

    vertices.push_back(robot_pos);  // 円錐の原点

    // 左の接線
    double left_angle = obstacle_angle + tangent_angle;
    vertices.push_back(Position(
        robot_pos.x + extension_length * std::cos(left_angle),
        robot_pos.y + extension_length * std::sin(left_angle)
    ));

    // 右の接線
    double right_angle = obstacle_angle - tangent_angle;
    vertices.push_back(Position(
        robot_pos.x + extension_length * std::cos(right_angle),
        robot_pos.y + extension_length * std::sin(right_angle)
    ));

    return vertices;
}

} // namespace tvvf_vo_c