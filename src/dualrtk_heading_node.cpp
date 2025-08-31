\
    #include <ros/ros.h>
    #include <sensor_msgs/NavSatFix.h>
    #include <std_msgs/Float64.h>
    #include <std_msgs/Bool.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <tf/transform_broadcaster.h>
    #include <cmath>
    #include <string>

    // This node adapts the logic from the user's fix2tfpose.cpp (dual-antenna yaw),
    // but makes it self-contained (no external geo_pos_conv.hpp), adds /heading output,
    // parameterizes topics/frames, and disables TF by default to avoid clashes.

    struct ECEF { double x, y, z; };
    struct ENU  { double E, N, U; };

    static constexpr double WGS84_A = 6378137.0;           // semi-major axis
    static constexpr double WGS84_F = 1.0 / 298.257223563; // flattening
    static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

    static double deg2rad(double d){ return d * M_PI / 180.0; }

    static ECEF lla_to_ecef(double lat_deg, double lon_deg, double h){
      double lat = deg2rad(lat_deg);
      double lon = deg2rad(lon_deg);
      double sin_lat = std::sin(lat), cos_lat = std::cos(lat);
      double sin_lon = std::sin(lon), cos_lon = std::cos(lon);
      double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat*sin_lat);
      double x = (N + h) * cos_lat * cos_lon;
      double y = (N + h) * cos_lat * sin_lon;
      double z = (N * (1.0 - WGS84_E2) + h) * sin_lat;
      return {x,y,z};
    }

    struct ENUOrigin {
      double lat0_deg{0}, lon0_deg{0}, h0{0};
      ECEF ecef0{0,0,0};
      double sin_lat0{0}, cos_lat0{1}, sin_lon0{0}, cos_lon0{1};
      bool initialized{false};
      void set(double lat_deg, double lon_deg, double h){
        lat0_deg = lat_deg; lon0_deg = lon_deg; h0 = h;
        ecef0 = lla_to_ecef(lat_deg, lon_deg, h);
        double lat0 = deg2rad(lat_deg), lon0 = deg2rad(lon_deg);
        sin_lat0 = std::sin(lat0); cos_lat0 = std::cos(lat0);
        sin_lon0 = std::sin(lon0); cos_lon0 = std::cos(lon0);
        initialized = true;
      }
      ENU ecef_to_enu(const ECEF& ecef) const {
        // dx in ECEF
        double dx = ecef.x - ecef0.x;
        double dy = ecef.y - ecef0.y;
        double dz = ecef.z - ecef0.z;
        // ENU rotation
        double E = -sin_lon0*dx + cos_lon0*dy;
        double N = -sin_lat0*cos_lon0*dx - sin_lat0*sin_lon0*dy + cos_lat0*dz;
        double U =  cos_lat0*cos_lon0*dx + cos_lat0*sin_lon0*dy + sin_lat0*dz;
        return {E,N,U};
      }
    };

    class DualRTKHeadingNode {
    public:
      DualRTKHeadingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh){
        // Parameters (match the repo's YAML/README)
        pnh.param("use_first_fix_as_origin", use_first_fix_as_origin_, true);
        pnh.param("ref_lat", ref_lat_, 0.0);
        pnh.param("ref_lon", ref_lon_, 0.0);
        pnh.param("ref_alt", ref_alt_, 0.0);
        pnh.param("yaw_offset_deg", yaw_offset_deg_, 0.0);
        pnh.param("fix_front_topic", fix_front_topic_, std::string("/fix"));
        pnh.param("fix_back_topic",  fix_back_topic_,  std::string("/fix_2"));
        pnh.param("max_sync_dt", max_sync_dt_, 0.20);
        pnh.param("publish_heading_pose", publish_heading_pose_, true);
        pnh.param("heading_pose_frame_id", heading_pose_frame_id_, std::string("map"));
        pnh.param("heading_pose_base_frame", heading_pose_base_frame_, std::string("base_link"));
        pnh.param("heading_pose_position", heading_pose_position_, std::string("mid"));
        pnh.param("publish_tf", publish_tf_, false);

        sub_front_ = nh.subscribe(fix_front_topic_, 20, &DualRTKHeadingNode::frontCb, this);
        sub_back_  = nh.subscribe(fix_back_topic_,  20, &DualRTKHeadingNode::backCb, this);

        pub_heading_ = nh.advertise<std_msgs::Float64>("/heading", 10);
        pub_len_     = nh.advertise<std_msgs::Float64>("/baseline_length", 10);
        pub_valid_   = nh.advertise<std_msgs::Bool>("/gnss_stat", 10);
        if(publish_heading_pose_){
          pub_pose_  = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
        }

        if(!use_first_fix_as_origin_){
          origin_.set(ref_lat_, ref_lon_, ref_alt_);
          ROS_INFO("Using fixed ENU origin: lat=%.8f lon=%.8f alt=%.3f", ref_lat_, ref_lon_, ref_alt_);
        } else {
          ROS_INFO("Will use FIRST received fix as ENU origin.");
        }
      }

    private:
      void frontCb(const sensor_msgs::NavSatFixConstPtr& msg){
        last_front_ = *msg;
        has_front_ = true;
        maybeSetOriginFrom(*msg);
        computeIfReady();
      }
      void backCb(const sensor_msgs::NavSatFixConstPtr& msg){
        last_back_ = *msg;
        has_back_ = true;
        maybeSetOriginFrom(*msg);
        computeIfReady();
      }
      void maybeSetOriginFrom(const sensor_msgs::NavSatFix& msg){
        if(use_first_fix_as_origin_ && !origin_.initialized){
          origin_.set(msg.latitude, msg.longitude, msg.altitude);
          ROS_INFO("ENU origin set by first fix: lat=%.8f lon=%.8f alt=%.3f", msg.latitude, msg.longitude, msg.altitude);
        }
      }
      bool recentPair(const ros::Time& t1, const ros::Time& t2) const {
        return std::fabs((t1 - t2).toSec()) <= max_sync_dt_;
      }
      void computeIfReady(){
        if(!(has_front_ && has_back_ && origin_.initialized)) return;
        if(!recentPair(last_front_.header.stamp, last_back_.header.stamp)) return;

        // Convert to ENU
        ECEF f_ecef = lla_to_ecef(last_front_.latitude, last_front_.longitude, last_front_.altitude);
        ECEF b_ecef = lla_to_ecef(last_back_.latitude,  last_back_.longitude,  last_back_.altitude);
        ENU  f = origin_.ecef_to_enu(f_ecef);
        ENU  b = origin_.ecef_to_enu(b_ecef);

        // Yaw from baseline (front - back)
        const double dE = f.E - b.E;
        const double dN = f.N - b.N;
        double yaw = std::atan2(dN, dE) + yaw_offset_deg_ * M_PI/180.0;
        while(yaw >  M_PI) yaw -= 2*M_PI;
        while(yaw < -M_PI) yaw += 2*M_PI;

        // Baseline length
        const double baseline = std::sqrt((f.E-b.E)*(f.E-b.E) + (f.N-b.N)*(f.N-b.N) + (f.U-b.U)*(f.U-b.U));

        std_msgs::Float64 h; h.data = yaw;            pub_heading_.publish(h);
        std_msgs::Float64 L; L.data = baseline;       pub_len_.publish(L);
        std_msgs::Bool ok; ok.data = std::isfinite(baseline) && baseline > 0.05; pub_valid_.publish(ok);
        if(!ok.data) return;

        if(publish_heading_pose_ || publish_tf_){
          geometry_msgs::PoseStamped ps;
          ps.header.frame_id = heading_pose_frame_id_;
          ps.header.stamp = last_front_.header.stamp;

          if(heading_pose_position_ == "front"){ ps.pose.position.x = f.E; ps.pose.position.y = f.N; ps.pose.position.z = f.U; }
          else if(heading_pose_position_ == "back"){ ps.pose.position.x = b.E; ps.pose.position.y = b.N; ps.pose.position.z = b.U; }
          else if(heading_pose_position_ == "mid"){ ps.pose.position.x = 0.5*(f.E + b.E); ps.pose.position.y = 0.5*(f.N + b.N); ps.pose.position.z = 0.5*(f.U + b.U); }
          else /* zero */ { ps.pose.position.x = 0; ps.pose.position.y = 0; ps.pose.position.z = 0; }

          tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
          tf::quaternionTFToMsg(q, ps.pose.orientation);

          if(publish_heading_pose_) pub_pose_.publish(ps);

          if(publish_tf_){
            static tf::TransformBroadcaster br;
            tf::Transform T;
            T.setOrigin(tf::Vector3(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
            T.setRotation(q);
            br.sendTransform(tf::StampedTransform(T, ps.header.stamp, heading_pose_frame_id_, heading_pose_base_frame_));
          }
        }
      }

      // params
      bool use_first_fix_as_origin_{true};
      double ref_lat_{0}, ref_lon_{0}, ref_alt_{0};
      double yaw_offset_deg_{0};
      std::string fix_front_topic_{"/fix"};
      std::string fix_back_topic_{"/fix_2"};
      double max_sync_dt_{0.2};
      bool publish_heading_pose_{true};
      std::string heading_pose_frame_id_{"map"};
      std::string heading_pose_base_frame_{"base_link"};
      std::string heading_pose_position_{"mid"};
      bool publish_tf_{false};

      // state
      ros::Subscriber sub_front_, sub_back_;
      ros::Publisher pub_heading_, pub_len_, pub_valid_, pub_pose_;
      sensor_msgs::NavSatFix last_front_, last_back_;
      bool has_front_{false}, has_back_{false};
      ENUOrigin origin_;
    };

    int main(int argc, char** argv){
      ros::init(argc, argv, "dualrtk_heading_node");
      ros::NodeHandle nh;
      ros::NodeHandle pnh("~");
      DualRTKHeadingNode node(nh, pnh);
      ros::spin();
      return 0;
    }
