// Copyright (c) 2023, Czech Technical University in Prague
// Copyright (c) 2019, paplhjak
// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/exception.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber.hpp>


using namespace std::chrono_literals;

namespace point_cloud_transport
{
class Republisher2 : public rclcpp::Node
{
public:
  //! Constructor
  explicit Republisher2(const rclcpp::NodeOptions & options);

private:
  void initialize();

  std::shared_ptr<point_cloud_transport::PointCloudTransport> pct;
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_{false};
  point_cloud_transport::Subscriber sub;
  std::shared_ptr<point_cloud_transport::PublisherPlugin> pub;
  std::shared_ptr<point_cloud_transport::Publisher> simple_pub;
};

Republisher2::Republisher2(const rclcpp::NodeOptions & options)
: Node("draco_to_raw", options)
{
  // Initialize Republisher2component after construction
  // shared_from_this can't be used in the constructor
  this->timer_ = create_wall_timer(
    1ms, [this]() {
      if (initialized_) {
        timer_->cancel();
      } else {
        this->initialize();
        initialized_ = true;
      }
    });
}

void Republisher2::initialize()
{
  std::string in_topic = "/livox/lidar";
  std::string out_topic = "/livox/lidar_repub";
  std::string in_transport = "draco";
  std::string out_transport = "raw";

  pct = std::make_shared<point_cloud_transport::PointCloudTransport>(this->shared_from_this());

  auto qos_override_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });
  rclcpp::SubscriptionOptions sub_options;
  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = qos_override_options;
  sub_options.qos_overriding_options = qos_override_options;

  if (out_transport.empty()) {
    // Use all available transports for output
    this->simple_pub =
      std::make_shared<point_cloud_transport::Publisher>(
      pct->advertise(
        out_topic,
        rmw_qos_profile_default));

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "out topic1: " << this->simple_pub->getTopic());

    // Use Publisher::publish as the subscriber callback
    typedef void (point_cloud_transport::Publisher::* PublishMemFn)(
      const sensor_msgs::msg::
      PointCloud2::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::Publisher::publish;

    const point_cloud_transport::TransportHints hint(in_transport);
    this->sub = pct->subscribe(
      in_topic, static_cast<uint32_t>(1),
      pub_mem_fn, this->simple_pub, &hint);
  } else {
    // Load transport plugin
    typedef point_cloud_transport::PublisherPlugin Plugin;
    auto loader = pct->getPublisherLoader();
    std::string lookup_name = Plugin::getLookupName(out_transport);
    RCLCPP_INFO(this->get_logger(), "Loading %s publisher", lookup_name.c_str());

    auto instance = loader->createUniqueInstance(lookup_name);
    // DO NOT use instance after this line
    this->pub = std::move(instance);
    pub->advertise(this->shared_from_this(), out_topic);

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "out topic2: " << this->pub->getTopic());

    // Use PublisherPlugin::publish as the subscriber callback
    typedef void (point_cloud_transport::PublisherPlugin::* PublishMemFn)(
      const sensor_msgs::msg::
      PointCloud2::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::PublisherPlugin::publish;

    RCLCPP_INFO(this->get_logger(), "Loading %s subscriber", in_topic.c_str());

    const point_cloud_transport::TransportHints hint(in_transport);
    this->sub = pct->subscribe(
      in_topic, static_cast<uint32_t>(1),
      pub_mem_fn, pub, &hint);
  }
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "in topic: " << this->sub.getTopic());
}
}  // namespace point_cloud_transport

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<point_cloud_transport::Republisher2>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}