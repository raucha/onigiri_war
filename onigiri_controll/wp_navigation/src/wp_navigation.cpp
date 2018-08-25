#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_broadcaster.h>

#define GAIN_CHASE 0.01
#define PNT_START_CHASE 16

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose {
  double x;
  double y;
  double yaw;
};

// MyPose way_point[] = {{1.3, 0.0, 0.0},     {1.3, 0.0, -0.75}, {2.1, -0.910, -0.75},
//                       {2.1, -0.910, 0.75}, {3.0, 0.0, 0.75},  {999, 999, 999}};
MyPose way_point[] = {
  {0.061, -0.007, -0.847},
  {0.696, -0.791, -0.926},
  // {0.736, -0.751, -0.926},
  {2.050, -2, 0.928},
  {2.352, -1.577, 0.922},
  {2.644, -1.312, 0.796},

  // {2.780, -1.335, 0.939},
  // {2.780, -1.235, 0.939},
  // {3.578, 0.0502, 2.647},
  {2.165, 1,  2.3},
  {4.054, 0, -2.25},
  {0.5, 0.04, -0.9},
  // {4.024, 0, 3.104},
  // {0.061, -0.007, -0.847},
  // {0.736, -0.751, 0.926},
  // {2.012, -2.010, 0.928},
  // {2.780, -1.235, 0.939},
  // {3.578, 0.0502, 2.647},
  // {3.519, -0.553, 0.672},
  {999, 999, 999}};

static const std::string OPENCV_WINDOW = "Image window";

class RoboCtrl {
 private:
  enum EState {
    STATE_IDLE = 0,
    STATE_WAYPOINT = 1,
    STATE_CHASE = 2,
  };
  ros::NodeHandle node;
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber Laser_sub_;
  ros::Publisher twist_pub_;

  // ROS時間
  ros::Time push_time_;
  // ROS変化秒
  ros::Duration under_time_;

  cv::Mat hsv;
  cv::Mat mask;
  cv::Mat image;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  EState m_state;

  // ロボット制御用
  double m_diffPos;
  double m_frontspeed;
  double m_turnspeed;
  ros::Time m_timechasestr;

  // ウェイポイント制御用
  int m_destPnt;
  bool m_isSent;
  MoveBaseClient ac;

 public:
  RoboCtrl() : it_(node), ac("move_base", true) {
    ros::NodeHandle node;
    //購読するtopic名
    //		odom_sub_ = node.subscribe("odom", 1, &RoboCtrl::odomCallback,
    // this);
    image_sub_ = it_.subscribe("/red_bot/camera/image_raw", 1, &RoboCtrl::imageCb, this);

    //配布するtopic名
    // twist_pub_ =
    // node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",
    // 1);
    twist_pub_ = node.advertise<geometry_msgs::Twist>("/red_bot/cmd_vel", 1);

    //内部関数初期化
    m_frontspeed = 0.5;
    m_turnspeed = 0.0;
    m_diffPos = 0.0;

    // まず最初はウェイポイント
    m_state = STATE_WAYPOINT;

    initWaypoint();
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~RoboCtrl() { cv::destroyWindow(OPENCV_WINDOW); }

  void initWaypoint() {
    // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
    // この例では５秒間待つ(ブロックされる)
    while (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("The server comes up");
    m_isSent = false;
    m_destPnt = 0;
  }

  void moveRobo() {
    //速度データ型宣言
    geometry_msgs::Twist twist;

    checkWaypoint();

    // STATE_CHASEの場合 P制御で追いかける
    if (m_state == STATE_CHASE) {
      ros::Duration tick = ros::Time::now() - m_timechasestr;
      double tickdbl = tick.toSec();
      m_frontspeed = 0.08;
      m_turnspeed = m_diffPos * GAIN_CHASE;
      ROS_INFO("CHASE PHASE(1) %f", tickdbl);
    //   if (tickdbl <= 8.0) {
    //     m_frontspeed = 0.1;
    //     m_turnspeed = m_diffPos * GAIN_CHASE;
    //     ROS_INFO("CHASE PHASE(1) %f", tickdbl);
    //   } else if (tickdbl <= 16.0) {
    //     m_frontspeed = 0.0;
    //     m_turnspeed = m_diffPos * GAIN_CHASE;
    //     ROS_INFO("CHASE PHASE(2) %f", tickdbl);
    //   } else if (tickdbl > 16.0)
    //     m_frontspeed = 0.0;
    //   m_turnspeed = -m_diffPos * GAIN_CHASE;
    //   ROS_INFO("CHASE PHASE(3) %f", tickdbl);
    }

    // ウェイポイント終わったらSTATE_IDLEにしてその場で回る
    if (m_state == STATE_IDLE) {
      m_frontspeed = 0.0;
      // m_turnspeed = 0.0;
      m_turnspeed = 0.5;
    }

    ROS_INFO("NOW %d", m_state);
    ROS_INFO("diff %f", m_diffPos);

    // ROS速度データに内部関数値を代入
    if (m_state == STATE_CHASE || m_state == STATE_IDLE) {
      twist.linear.x = m_frontspeed;
      twist.angular.z = m_turnspeed;
      twist_pub_.publish(twist);
    }
  }

  void checkWaypoint() {
    // ウェイポイント送信済みか？
    if (m_state != STATE_WAYPOINT) {
      return;
    }
    if (m_isSent) {
      // ウェイポイント送信済みなら到着結果の確認をする
      // 追跡モード中はm_isSentはfalseなので確認しない
      bool isSucceeded = ac.waitForResult(ros::Duration(0.5));
      // 結果を見て、成功ならSucceeded、失敗ならFailedと表示
      actionlib::SimpleClientGoalState state = ac.getState();

      // 到着済みか?
      if (isSucceeded) {
        // 到着済みなら次の点を送信する
        m_isSent = false;
        ROS_INFO("WP Reached: No.%d (%s)", m_destPnt + 1, state.toString().c_str());
        ros::Duration(1.0).sleep();
        sendWaypoint(++m_destPnt);
      } else {
        // 到着してないなら何もしない
        ROS_INFO("WP not reached: No.%d (%s)", m_destPnt + 1, state.toString().c_str());
      }
    } else {
      // ウェイポイント送信済みでない場合
      // STATE_WAYPOINTのときウェイポイント送信(初回時やSTATE_CHASEから復帰時)
      // STATE_CHASE, STATE_IDLEでは何もしない)
      sendWaypoint(m_destPnt);
    }
  }

  void sendWaypoint(int n_ptr) {
    // STATE_WAYPOINTの場合のみ呼ばれる
    // ウェイポイントの更新 or 初回時 or STATE_CHASEからの復帰
    move_base_msgs::MoveBaseGoal goal;
    // map(地図)座標系
    goal.target_pose.header.frame_id = "map";
    // 現在時刻
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = way_point[n_ptr].x;
    goal.target_pose.pose.position.y = way_point[n_ptr].y;

    // 最終ウェイポイントに達したらSTATE_IDLEに遷移して抜ける。
    if (goal.target_pose.pose.position.x == 999) {
      m_state = STATE_IDLE;
      return;
    }

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[n_ptr].yaw);

    ROS_INFO("Sending goal: No.%d", n_ptr + 1);
    ROS_INFO("X=%f Y=%f th=%f", way_point[n_ptr].x, way_point[n_ptr].y, way_point[n_ptr].yaw);

    // サーバーにgoalを送信
    ac.sendGoal(goal);

    m_isSent = true;
  }

  void cancelWayPoint() {
    // ウェイポイントのキャンセル。STATE_CHASEへの移行時に呼ばれる
    ac.cancelAllGoals();
    ROS_INFO("WAYPOINT CANCELED. START CHASING!!");

    // STATE_WAYPOINT復帰時にウェイポイントを送り直すようにしておく
    m_isSent = false;
  }

  void odomCallback(const nav_msgs::Odometry& odom) { return; }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    const int IMG_CENTER = 200;
    const int range = 10;
    static EState laststate = m_state;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(60 - range, 100, 100), cv::Scalar(60 + range, 255, 255),
                mask);  // 色検出でマスク画像の作成
    // cv::bitwise_and(cv_ptr->image,mask,image);

    cv::Moments mu = cv::moments(mask, false);
    cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

    double area = mu.m00;
    int x = mu.m10 / mu.m00;
    int y = mu.m01 / mu.m00;
    ROS_INFO("AREA = %f", area);

    if (area > 100) {
      // if ((x >= 0) && (x <= 640) && (area > 20000) && (m_destPnt >= PNT_START_CHASE)) {
      // 敵が見つかったら追跡する
      m_diffPos = -(x - IMG_CENTER);

      // STATE_CHASEに入る前の状態を保存
      switch (m_state) {
        case STATE_WAYPOINT:
          cancelWayPoint();
        case STATE_IDLE:
          laststate = m_state;
          m_timechasestr = ros::Time::now();
          break;
        default:
          break;
      }
      // STATE_CHASEに遷移
      if(m_state == STATE_IDLE ||m_state == STATE_WAYPOINT){
        m_state = STATE_CHASE;
      }
    } else {
      // 敵を見失う or そもそも敵を見つけていない場合
      m_diffPos = 0;
      // STATE_CHASEに入る前の状態を戻す
      if (m_state == STATE_CHASE) {
      // if (m_state == STATE_CHASE) {
        m_state = laststate;
      }
    }

    // ROS_INFO("obj x=%d y=%d",x,y);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, mask);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv) {
  // ROSのノード初期化
  ros::init(argc, argv, "robo_ctrl");
  RoboCtrl robo_ctrl;
  ros::Rate r(20);

  while (ros::ok()) {
    robo_ctrl.moveRobo();
    ros::spinOnce();
    r.sleep();
  }
}
