#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <thread>
#include <chrono>

class FlightCommander {
public:
    FlightCommander(): spinner_(2) {
        // Подписка на целевые координаты
        target_pose_sub_ = nh_.subscribe("/vehicle/desPose", 10, &FlightCommander::poseCallback, this);    
        // Подписываемся на состояние ЛА
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &FlightCommander::state_cb, this);
        // Подписываемся текущее положение ЛА
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &FlightCommander::pose_cb, this);
        // Инициализируем publisher для целевого состояния ЛА
        local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        // Инициализируем таймер для отправки целевого состояния
        //offboard_timer_ = nh_.createTimer(ros::Duration(0.05), &FlightCommander::offboard_timer_cb, this); // 20 Hz
        //offboard_timer_.stop(); // Останавливаем таймер до перехода в режим OFFBOARD
        // (AsyncSpinner - это класс, который используется для асинхронного выполнения коллбеков (callback) из очереди сообщений
        // аналогично ros::spin() или ros::spinOnce()
        // Запускаем асинхронный спинер
        spinner_.start();
    }

    void run() {
        // Ожидаем подключения к автопилоту
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        // Запускаем режим offboard
        offboard_enable(true);

        // Запускаем режим АРМ
        if (!current_state_.armed)
            arm_vehicle(true);

          
        //auto start_time = ros::Time::now();
        

        //Запускаем полет
        while (ros::ok()) {
              rate.sleep();
              calculate_velocitys();
              local_pos_pub_.publish(setpoint_); 
              
        }     
 
        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_pose_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_pos_pub_;
    ros::Timer offboard_timer_;
   
    double desired_position_x = 0;
    double desired_position_y = 0;
    double desired_position_z = 0;
    double current_position_x = 0;
    double current_position_y = 0;
    double current_position_z = 0;

    // Коэффициенты П-регулятора для осей
    double kp_position_x = 1; 
    double kp_position_y = 1;
    double kp_position_z = 1;
     

    
    ros::Rate rate = ros::Rate(20.0);
    mavros_msgs::State current_state_;
    mavros_msgs::PositionTarget setpoint_;
    geometry_msgs::PoseStamped current_pose_;
    ros::AsyncSpinner spinner_;

    //  Установка целевоой позиции
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Обработка полученных координат
       
        desired_position_x = msg->pose.position.x;
        desired_position_y = msg->pose.position.y;
        desired_position_z = msg->pose.position.z;

    }
    //  Получение текущего состояния дрона
    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    //  Получение текущих координат дрона
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }


    void calculate_velocitys() {
        // Устанавливаем систему координат в рамках которой будет отправляться команда
        setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // Установим битовую маску где покажем что должен выполнить автопилот, полет в точку или набор заданной скорости, ускорения, угла угловой скорости.
        setpoint_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
            | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        
        // РАСЧЕТ скоростей       
        double position_error_x = desired_position_x - current_pose_.pose.position.x;
        double position_error_y = desired_position_y - current_pose_.pose.position.y;
        double position_error_z = desired_position_z - current_pose_.pose.position.z;
        

        // Применение П-регулятора для линейной скорости
        double target_velocity_x = kp_position_x * position_error_x;
        double target_velocity_y = kp_position_y * position_error_y;
        double target_velocity_z = kp_position_z * position_error_z;
 

        setpoint_.velocity.x = target_velocity_x;
        setpoint_.velocity.y = target_velocity_y;
        setpoint_.velocity.z = target_velocity_z;

              
       
    }
    
    // Арм аппарата
    bool arm_vehicle(bool arm = true) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        
        ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        return arming_client.call(arm_cmd) && arm_cmd.response.success;
    }

    // Смена режима полета
    bool change_mode(const std::string& mode) {
        mavros_msgs::SetMode sm;
        sm.request.custom_mode = mode;

        ros::ServiceClient client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        return client.call(sm) && sm.response.mode_sent;
    }
    
    // Активация автоматического режима полета
    void offboard_enable(bool enable) {
        //if (!enable) {
        //    offboard_timer_.stop();
       //     return;
       // }

        // Запускаем таймер отправки целевого положения
        //offboard_timer_.start();
        // Немного ждем пока сообщения начнут приходить на автопилот
        ros::Rate rate(1.0);
        rate.sleep();
        // Переключение режима на OFFBOARD
        change_mode("OFFBOARD");
    }

    // Таймер отправки целевого положения
    //void offboard_timer_cb(const ros::TimerEvent&) {
    //    setpoint_.header.stamp = ros::Time::now();
        //local_pos_pub_.publish(setpoint_);
    //}

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "flight_commander_node");
    FlightCommander commander;
    commander.run();
    return 0;
}




